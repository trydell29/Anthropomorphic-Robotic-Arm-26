"""
ARA vision teleop -- host side.

Camera -> MediaPipe Pose (forearm axis) + MediaPipe Hand (21 landmarks, run on a
wrist-anchored crop) -> 12 axis values -> UDP to the arm's ESP32.

Implements the host half of docs/PROTOCOL.md v0.1.

  ARA,<seq>,<a0>..<a11>\\n   to 192.168.4.1:4210

Why both models:
  Pose supplies the elbow, which is the only way to know where the forearm is
  pointing. Without it, wrist bend and forearm roll can only be measured as
  deviation from a held calibration pose, and the system cannot tell "I bent my
  wrist" from "I rotated my whole arm."

Why the ROI crop:
  Pose needs the torso in frame, which makes the hand small. Running
  HandLandmarker on a crop around the wrist restores pixel density, which the
  splay channels need -- they have the least angular travel and the least
  tolerance for landmark noise.

Why detection runs on the un-mirrored frame:
  MediaPipe labels pose landmarks anatomically, from what it sees. Mirroring the
  frame makes a right arm look like a left one, so landmarks 14/16 stop being
  your right elbow/wrist and hand chirality inverts along with it. All pose/hand
  detection above happens on the raw frame; only the displayed preview is
  flipped afterward, purely for a natural mirror-like view. This removes an
  entire class of silent sign errors while still looking normal on screen.

Keys:  c = calibrate    s = toggle sending    ESC = quit

Models (not committed -- see README):
  hand_landmarker.task
  pose_landmarker_lite.task

  curl -o pose_landmarker_lite.task \\
    https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task
"""

import math
import socket
import time

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# ===========================================================================
# CONFIG
# ===========================================================================

HAND_MODEL = "hand_landmarker.task"
POSE_MODEL = "pose_landmarker_lite.task"   # lite is plenty: only elbow+wrist are used

CAM_INDEX = 0
CAM_W, CAM_H = 1280, 720

ESP_ADDR = ("192.168.4.1", 4210)
SEND_HZ = 40
SEND_ON_START = False          # press 's' once tracking looks right

RIGHT_HAND = True              # which arm is being tracked

# --- Pose cost control ---
# The forearm moves slowly compared to fingers, so Pose does not need every
# frame and does not need full resolution. Landmarks are normalized 0-1, so
# downscaling costs nothing in the coordinate math.
POSE_EVERY = 8                 # run Pose every Nth frame
POSE_SCALE = 0.4               # downscale factor for the Pose input

# --- ROI crop ---
ROI_SCALE = 0.42               # crop side as a fraction of frame height
ROI_SMOOTH = 0.5               # EMA on the crop center, stops the box jittering
ROI_MISS_LIMIT = 8             # frames without a hand before falling back to full frame

# --- Finger curl calibration (degrees at the PIP joint) ---
CURL_STRAIGHT = 172.0
CURL_CURLED = 50.0

# --- Splay ---
# Splay is the weakest channel: small travel, and it degrades as a finger closes
# and its ray foreshortens. Freeze it once the finger curls past this.
SPLAY_CURL_GATE = 110.0        # servo units (0-180); below this, splay is frozen
SPLAY_RANGE_DEG = 12.0         # +/- deviation from neutral mapped to full travel

# --- Wrist / roll ---
WRIST_RANGE_DEG = 60.0
ROLL_LIMIT_DEG = 80.0          # must match AXIS_MIN/MAX[11] in firmware
WRIST_DEADBAND = 4.0
ROLL_DEADBAND = 6.0

# Roll is ill-conditioned when the hand points near the forearm axis or straight
# at the camera. Freeze rather than track garbage.
FREEZE_ALIGN_DEG = 25.0        # hand-to-forearm angle below which roll freezes

# --- Thumb opposition ---
OPP_RANGE_DEG = 35.0

SMOOTH = 0.35                  # per-channel EMA, 0 = raw

# ===========================================================================
# LANDMARK INDICES
# ===========================================================================

WRIST, THUMB_CMC, THUMB_MCP, THUMB_IP, THUMB_TIP = 0, 1, 2, 3, 4
INDEX_MCP, INDEX_PIP, INDEX_TIP = 5, 6, 8
MIDDLE_MCP, MIDDLE_PIP, MIDDLE_TIP = 9, 10, 12
RING_MCP, RING_PIP, RING_TIP = 13, 14, 16
PINKY_MCP, PINKY_PIP, PINKY_TIP = 17, 18, 20

# (mcp, pip, tip) per finger, in axis order 0..4
CURL_TRIPLETS = [
    (THUMB_CMC, THUMB_MCP, THUMB_TIP),
    (INDEX_MCP, INDEX_PIP, INDEX_TIP),
    (MIDDLE_MCP, MIDDLE_PIP, MIDDLE_TIP),
    (RING_MCP, RING_PIP, RING_TIP),
    (PINKY_MCP, PINKY_PIP, PINKY_TIP),
]

# splay measured for index, middle, ring, pinky -> axes 6..9
SPLAY_MCPS = [INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP]
SPLAY_TIPS = [INDEX_PIP, MIDDLE_PIP, RING_PIP, PINKY_PIP]

POSE_R_ELBOW, POSE_R_WRIST = 14, 16
POSE_L_ELBOW, POSE_L_WRIST = 13, 15

HAND_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
]

N_AXES = 12

# ===========================================================================
# MATH
# ===========================================================================


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else np.zeros_like(v)


def angle_between(a, b):
    """Unsigned angle between two vectors, degrees."""
    c = float(np.dot(unit(a), unit(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def angle_at(a, b, c):
    """Unsigned angle at vertex b, degrees."""
    return angle_between(a - b, c - b)


def signed_angle_about(axis, v_from, v_to):
    """Angle from v_from to v_to measured about `axis`, right-handed, degrees.

    Both vectors are projected into the plane perpendicular to axis first, so
    only the component of rotation about that axis is measured.
    """
    ax = unit(axis)
    a = unit(v_from - ax * np.dot(v_from, ax))
    b = unit(v_to - ax * np.dot(v_to, ax))
    if np.linalg.norm(a) < 1e-6 or np.linalg.norm(b) < 1e-6:
        return 0.0
    s = float(np.dot(np.cross(a, b), ax))
    c = float(np.dot(a, b))
    return math.degrees(math.atan2(s, c))


def remap(x, lo, hi, out_lo, out_hi):
    if hi == lo:
        return out_lo
    t = (x - lo) / (hi - lo)
    t = max(0.0, min(1.0, t))
    return out_lo + t * (out_hi - out_lo)


def deadband(x, band):
    if abs(x) < band:
        return 0.0
    return x - math.copysign(band, x)


def palm_frame(w):
    """Orthonormal frame from hand world landmarks.

    x = long axis, wrist -> middle MCP (distal)
    z = palm normal
    y = lateral, completes the right-handed set
    """
    x = unit(w[MIDDLE_MCP] - w[WRIST])
    across = w[INDEX_MCP] - w[PINKY_MCP]
    z = unit(np.cross(x, across))
    if not RIGHT_HAND:
        z = -z                      # chirality flips the palm normal
    y = unit(np.cross(z, x))
    return x, y, z


# ===========================================================================
# CHANNEL EXTRACTION
# ===========================================================================


class Calibration:
    """Neutral references captured from a held flat pose."""

    def __init__(self):
        self.ok = False
        self.wrist_bend = 0.0
        self.roll_ref = None        # palm normal, in the forearm-perp plane
        self.splay = [0.0] * 4
        self.opp = 0.0


def extract(world, forearm, calib):
    """Compute raw (uncalibrated) geometric quantities from one frame.

    world   : (21,3) hand world landmarks, metric, camera-aligned orientation
    forearm : unit vector elbow -> wrist, or None if Pose lost the arm
    """
    hx, hy, hz = palm_frame(world)

    # --- finger curls (axes 0..4) ---
    curls = []
    for a, b, c in CURL_TRIPLETS:
        ang = angle_at(world[a], world[b], world[c])
        curls.append(remap(ang, CURL_CURLED, CURL_STRAIGHT, 0.0, 180.0))

    # --- splay (axes 6..9) ---
    # Angle of each finger's proximal ray about the palm normal, relative to the
    # hand's long axis. Measured in the palm plane, so curl does not rotate it --
    # but foreshortening still degrades it, hence the gate applied by the caller.
    splay_raw = []
    for mcp, pip in zip(SPLAY_MCPS, SPLAY_TIPS):
        ray = world[pip] - world[mcp]
        splay_raw.append(signed_angle_about(hz, hx, ray))

    # --- thumb opposition (axis 10) ---
    # Thumb rotation out of the palm plane: how far the thumb ray has swung
    # toward the palm about the hand's long axis.
    thumb_ray = world[THUMB_MCP] - world[THUMB_CMC]
    opp_raw = signed_angle_about(hx, hy, thumb_ray)

    # --- wrist bend + roll (axes 5, 11) --- need the forearm
    wrist_raw = None
    roll_raw = None
    align = None
    if forearm is not None:
        align = angle_between(forearm, hx)

        # Flexion/extension: rotation of the hand's long axis away from the
        # forearm, about the hand's lateral axis. Signed by construction.
        wrist_raw = signed_angle_about(hy, forearm, hx)

        # Pronation/supination: roll of the palm normal about the forearm axis.
        roll_raw = None if calib.roll_ref is None else \
            signed_angle_about(forearm, calib.roll_ref, hz)

    return {
        "curls": curls,
        "splay": splay_raw,
        "opp": opp_raw,
        "wrist": wrist_raw,
        "roll": roll_raw,
        "align": align,
        "frame": (hx, hy, hz),
    }


def to_channels(raw, calib, last, frozen_flags):
    """Map raw geometry to the 12 protocol channels."""
    ch = [0.0] * N_AXES

    # 0..4 finger flexion
    for i in range(5):
        ch[i] = raw["curls"][i]

    # 6..9 splay, gated on curl
    for i in range(4):
        finger_axis = i + 1                     # index..pinky are axes 1..4
        if raw["curls"][finger_axis] < SPLAY_CURL_GATE:
            ch[6 + i] = last[6 + i]             # frozen: too curled to trust
            frozen_flags["splay"] = True
        else:
            d = raw["splay"][i] - calib.splay[i]
            ch[6 + i] = remap(d, -SPLAY_RANGE_DEG, SPLAY_RANGE_DEG, 0.0, 180.0)

    # 10 thumb opposition
    d = raw["opp"] - calib.opp
    ch[10] = remap(d, -OPP_RANGE_DEG, OPP_RANGE_DEG, 0.0, 180.0)

    # 5 wrist flexion/extension
    if raw["wrist"] is None:
        ch[5] = last[5]
        frozen_flags["wrist"] = True
    else:
        d = deadband(raw["wrist"] - calib.wrist_bend, WRIST_DEADBAND)
        ch[5] = remap(d, -WRIST_RANGE_DEG, WRIST_RANGE_DEG, 0.0, 180.0)

    # 11 elbow roll -- signed degrees, not 0-180. See PROTOCOL section 2.
    if raw["roll"] is None or (raw["align"] is not None
                               and raw["align"] < FREEZE_ALIGN_DEG):
        ch[11] = last[11]
        frozen_flags["roll"] = True
    else:
        d = deadband(raw["roll"], ROLL_DEADBAND)
        ch[11] = max(-ROLL_LIMIT_DEG, min(ROLL_LIMIT_DEG, d))

    return ch


# ===========================================================================
# ROI
# ===========================================================================


class Roi:
    """Wrist-anchored crop, so the hand model sees pixels instead of a smudge."""

    def __init__(self):
        self.cx = None
        self.cy = None
        self.miss = 0

    def update_center(self, cx, cy):
        if self.cx is None:
            self.cx, self.cy = cx, cy
        else:
            self.cx = ROI_SMOOTH * self.cx + (1 - ROI_SMOOTH) * cx
            self.cy = ROI_SMOOTH * self.cy + (1 - ROI_SMOOTH) * cy

    def box(self, w, h):
        """Return (x0, y0, side) or None to use the whole frame."""
        if self.cx is None or self.miss > ROI_MISS_LIMIT:
            return None
        side = int(ROI_SCALE * h)
        x0 = int(self.cx - side / 2)
        y0 = int(self.cy - side / 2)
        x0 = max(0, min(w - side, x0))
        y0 = max(0, min(h - side, y0))
        return x0, y0, side


# ===========================================================================
# MAIN
# ===========================================================================


def main():
    hand = vision.HandLandmarker.create_from_options(
        vision.HandLandmarkerOptions(
            base_options=python.BaseOptions(model_asset_path=HAND_MODEL),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_tracking_confidence=0.5))

    pose = vision.PoseLandmarker.create_from_options(
        vision.PoseLandmarkerOptions(
            base_options=python.BaseOptions(model_asset_path=POSE_MODEL),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1))

    cap = cv2.VideoCapture(CAM_INDEX)
    # MJPG first, then size. USB webcams default to raw YUYV, which saturates USB
    # bandwidth at 720p and throttles the capture before MediaPipe sees anything.
    # This matters far more on a Pi than on a laptop.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    print("capture: %dx%d @ %.0f fps requested" % (
        cap.get(cv2.CAP_PROP_FRAME_WIDTH),
        cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
        cap.get(cv2.CAP_PROP_FPS)))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sending = SEND_ON_START

    calib = Calibration()
    roi = Roi()
    smoothed = None
    last = [90.0] * N_AXES
    last[0:5] = [180.0] * 5
    last[11] = 0.0
    seq = 0
    last_send = 0.0
    t0 = time.time()
    fps_t, fps_n, fps = time.time(), 0, 0.0

    frame_n = 0
    last_pose_res = None

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        # NOTE: deliberately NOT mirrored. See module docstring.
        h, w = frame.shape[:2]
        ts = int((time.time() - t0) * 1000)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # ---- Pose, decimated and downscaled ----
        frame_n += 1
        if frame_n % POSE_EVERY == 0:
            small = cv2.resize(rgb, None, fx=POSE_SCALE, fy=POSE_SCALE)
            last_pose_res = pose.detect_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB,
                         data=np.ascontiguousarray(small)), ts)
        pose_res = last_pose_res

        forearm = None
        pose_wrist_px = None
        if pose_res and pose_res.pose_landmarks:
            pl = pose_res.pose_landmarks[0]
            ei, wi = (POSE_R_ELBOW, POSE_R_WRIST) if RIGHT_HAND else \
                     (POSE_L_ELBOW, POSE_L_WRIST)
            e, wr = pl[ei], pl[wi]
            if e.visibility > 0.5 and wr.visibility > 0.5:
                # Aspect-correct x so angles are not skewed by the frame ratio.
                ar = w / h
                ev = np.array([e.x * ar, e.y, e.z])
                wv = np.array([wr.x * ar, wr.y, wr.z])
                forearm = unit(wv - ev)
                pose_wrist_px = (wr.x * w, wr.y * h)
                roi.update_center(*pose_wrist_px)

        # ---- Hand on the ROI crop ----
        box = roi.box(w, h)
        if box:
            x0, y0, side = box
            sub = rgb[y0:y0 + side, x0:x0 + side]
        else:
            x0, y0, side = 0, 0, None
            sub = rgb

        hand_res = hand.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(sub)),
            ts)

        frozen = {"splay": False, "wrist": False, "roll": False}
        have_hand = bool(hand_res.hand_landmarks)

        if have_hand:
            roi.miss = 0
            world = np.array([[p.x, p.y, p.z]
                              for p in hand_res.hand_world_landmarks[0]])

            # draw skeleton, mapping crop coords back to the full frame
            lm = hand_res.hand_landmarks[0]
            s = side if side else max(w, h)
            pts = []
            for p in lm:
                if side:
                    pts.append((int(x0 + p.x * s), int(y0 + p.y * s)))
                else:
                    pts.append((int(p.x * w), int(p.y * h)))
            for a, b in HAND_EDGES:
                cv2.line(frame, pts[a], pts[b], (0, 220, 120), 2)
            for p in pts:
                cv2.circle(frame, p, 3, (255, 255, 255), -1)

            raw = extract(world, forearm, calib)

            if calib.ok:
                ch = to_channels(raw, calib, last, frozen)
                last = ch
                smoothed = np.array(ch) if smoothed is None else \
                    SMOOTH * smoothed + (1 - SMOOTH) * np.array(ch)

                now = time.time()
                if sending and now - last_send >= 1.0 / SEND_HZ:
                    seq = (seq + 1) & 0xFFFF
                    vals = ",".join(str(int(round(v))) for v in smoothed)
                    sock.sendto(f"ARA,{seq},{vals}\n".encode(), ESP_ADDR)
                    last_send = now
        else:
            roi.miss += 1
            smoothed = None      # stop sending; ESP32 watchdog holds position

        # ---- overlay ----
        if box:
            cv2.rectangle(frame, (x0, y0), (x0 + side, y0 + side),
                          (90, 90, 200), 1)

        names = ["thmb", "indx", "midl", "ring", "pnky", "WRST",
                 "iSpl", "mSpl", "rSpl", "pSpl", "tOpp", "ROLL"]
        y = 26
        if smoothed is not None:
            for n, v in zip(names, smoothed):
                cv2.putText(frame, f"{n} {int(v):4d}", (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                y += 20

        status = []
        if not calib.ok:
            status.append(("HOLD FLAT + NEUTRAL, PRESS 'c'", (0, 165, 255)))
        if forearm is None:
            status.append(("NO FOREARM (pose lost)", (0, 0, 255)))
        if not have_hand:
            status.append(("NO HAND", (0, 0, 255)))
        if frozen["roll"]:
            status.append(("ROLL FROZEN", (0, 140, 255)))
        if frozen["splay"]:
            status.append(("SPLAY FROZEN", (0, 140, 255)))
        status.append((f"SEND {'ON' if sending else 'OFF'}",
                       (0, 220, 0) if sending else (140, 140, 140)))
        sy = h - 20 - 22 * (len(status) - 1)
        for text, col in status:
            cv2.putText(frame, text, (10, sy), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, col, 2)
            sy += 22

        fps_n += 1
        if time.time() - fps_t >= 1.0:
            fps, fps_n, fps_t = fps_n / (time.time() - fps_t), 0, time.time()
        cv2.putText(frame, f"{fps:.1f} fps", (w - 110, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # Mirror for display only -- detection above runs on the unflipped
        # frame so MediaPipe's anatomical left/right labeling stays correct.
        cv2.imshow("ARA teleop  [c] calibrate  [s] send  [ESC] quit",
                   cv2.flip(frame, 1))

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
        if k == ord('s'):
            sending = not sending
        if k == ord('c') and have_hand:
            world = np.array([[p.x, p.y, p.z]
                              for p in hand_res.hand_world_landmarks[0]])
            r = extract(world, forearm, calib)
            hx, hy, hz = r["frame"]
            calib.splay = list(r["splay"])
            calib.opp = r["opp"]
            if forearm is not None:
                calib.wrist_bend = r["wrist"]
                # store the palm normal projected into the forearm-perp plane
                f = unit(forearm)
                calib.roll_ref = unit(hz - f * np.dot(hz, f))
                calib.ok = True
                print("calibrated")
            else:
                print("calibration needs the forearm -- step back so the "
                      "elbow is in frame")

    cap.release()
    cv2.destroyAllWindows()
    hand.close()
    pose.close()


if __name__ == "__main__":
    main()