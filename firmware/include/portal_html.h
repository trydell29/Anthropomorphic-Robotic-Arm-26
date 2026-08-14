// Portal UI, served from flash at "/".
//
// Kept in its own header so main.cpp stays readable. Single source of truth --
// do not keep a second copy of this markup anywhere.
//
// Constraints that shaped it:
//   - No internet. The arm runs on its own softAP, so there are no web fonts,
//     no CDN, no icon library. System font stacks only.
//   - The firmware clamps every value. That normally FIGHTS limit-finding, so
//     the panel flags a clamp instead of silently swallowing your input.
//   - Sliders are populated FROM /state, never pushed TO it on load. See
//     PROTOCOL.md section 5.2 -- otherwise the first slider you touch snaps.

#pragma once
#include <pgmspace.h>

static const char PORTAL_HTML[] PROGMEM = R"RAWHTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ARA bench</title>
<style>
:root{
  --panel:#E8E6E1; --field:#F6F5F2; --ink:#1C1E22; --dim:#6B6E74;
  --rule:#C9C6BF; --live:#B86B14; --ok:#3F6B45; --bad:#9E332B; --cv:#2F5D7C;
}
*{box-sizing:border-box}
html{-webkit-text-size-adjust:100%}
body{
  margin:0;padding:0 0 3rem;background:var(--panel);color:var(--ink);
  font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif;
}
.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
.wrap{max-width:860px;margin:0 auto;padding:0 1rem}

/* ---- header ---- */
header{
  position:sticky;top:0;z-index:5;background:var(--panel);
  border-bottom:1px solid var(--rule);padding:.85rem 0 .7rem;
}
.hrow{display:flex;align-items:baseline;gap:.75rem;flex-wrap:wrap}
h1{
  margin:0;font:600 15px/1 ui-monospace,SFMono-Regular,Menlo,monospace;
  letter-spacing:.22em;text-transform:uppercase;
}
.sub{color:var(--dim);font-size:12px;letter-spacing:.04em}
.spacer{flex:1}

.seg{display:flex;border:1px solid var(--ink);border-radius:2px;overflow:hidden}
.seg button{
  font:600 11px/1 ui-monospace,Menlo,monospace;letter-spacing:.14em;
  padding:.5rem .9rem;border:0;background:transparent;color:var(--ink);
  cursor:pointer;
}
.seg button[aria-pressed="true"]{background:var(--ink);color:var(--field)}
.seg button:focus-visible{outline:2px solid var(--live);outline-offset:-2px}

/* ---- status strip ---- */
.status{
  display:flex;gap:1.15rem;flex-wrap:wrap;margin-top:.55rem;
  font-family:ui-monospace,Menlo,monospace;font-size:11px;letter-spacing:.06em;
  color:var(--dim);
}
.status b{font-weight:600;color:var(--ink)}
.dot{display:inline-block;width:7px;height:7px;border-radius:50%;
     background:var(--dim);margin-right:.35rem;vertical-align:1px}
.dot.on{background:var(--ok)} .dot.off{background:var(--bad)}
.dot.warn{background:var(--live)}

/* ---- sections ---- */
section{margin-top:1.6rem}
.shead{
  display:flex;align-items:baseline;gap:.6rem;
  border-bottom:1px solid var(--rule);padding-bottom:.3rem;margin-bottom:.2rem;
}
.shead h2{
  margin:0;font:600 11px/1 ui-monospace,Menlo,monospace;
  letter-spacing:.18em;text-transform:uppercase;
}
.shead .note{font-size:11px;color:var(--dim)}

/* ---- axis rows: the instrument ---- */
.axis{
  display:grid;align-items:center;gap:.6rem;padding:.42rem 0;
  grid-template-columns:2ch 12ch 1fr 6ch 8ch 5ch;
  border-bottom:1px solid rgba(201,198,191,.5);
}
.axis .idx{font-family:ui-monospace,Menlo,monospace;font-size:11px;color:var(--dim)}
.axis .nm{font-size:12.5px;letter-spacing:.01em}
.axis input[type=range]{width:100%;margin:0;accent-color:var(--ink);cursor:pointer}
.axis .val{
  font-family:ui-monospace,Menlo,monospace;font-size:13px;text-align:right;
  font-variant-numeric:tabular-nums;
}
.axis .us{
  font-family:ui-monospace,Menlo,monospace;font-size:11px;color:var(--dim);
  text-align:right;font-variant-numeric:tabular-nums;
}
.axis .val.clamped{color:var(--bad);font-weight:600}
.axis.dis{opacity:.42}
.axis.dis input{cursor:not-allowed}

/* ---- bench jog ---- */
.jogrow{
  display:flex;align-items:center;gap:.5rem;flex-wrap:wrap;padding:.5rem 0;
  border-bottom:1px solid rgba(201,198,191,.5);
}
.jogrow .idx{font-family:ui-monospace,Menlo,monospace;font-size:11px;color:var(--dim)}
.jogrow .nm{font-size:12.5px;min-width:8ch}
.jogrow input[type=number]{
  width:5.5rem;font:12px ui-monospace,Menlo,monospace;padding:.35rem .4rem;
  border:1px solid var(--rule);border-radius:2px;background:var(--field);color:var(--ink);
}
.jogrow .st{margin-left:auto;font:600 11px/1 ui-monospace,Menlo,monospace;
            letter-spacing:.06em;color:var(--dim)}
.jogrow .st.on{color:var(--live)}
.jogrow.dis{opacity:.42}
.jogrow.dis button,.jogrow.dis input{pointer-events:none}

.mark{
  font:600 10px/1 ui-monospace,Menlo,monospace;letter-spacing:.08em;
  border:1px solid var(--rule);background:var(--field);color:var(--dim);
  border-radius:2px;padding:.3rem .25rem;cursor:pointer;
}
.mark:hover:not(:disabled){border-color:var(--ink);color:var(--ink)}
.mark:disabled{opacity:.35;cursor:not-allowed}
.mark.has{border-color:var(--live);color:var(--live)}

/* ---- buttons ---- */
.btns{display:flex;gap:.4rem;flex-wrap:wrap;margin-top:.7rem}
button.b{
  font:500 12px/1 system-ui,sans-serif;padding:.5rem .8rem;cursor:pointer;
  background:var(--field);border:1px solid var(--rule);border-radius:2px;
  color:var(--ink);
}
button.b:hover:not(:disabled){border-color:var(--ink)}
button.b:disabled{opacity:.4;cursor:not-allowed}
button.b.danger{border-color:var(--bad);color:var(--bad)}
button.b:focus-visible{outline:2px solid var(--live);outline-offset:1px}

/* ---- limits capture ---- */
#limitOut{
  width:100%;margin-top:.6rem;min-height:7.5rem;padding:.6rem;
  font-family:ui-monospace,Menlo,monospace;font-size:11px;line-height:1.5;
  background:var(--field);border:1px solid var(--rule);border-radius:2px;
  color:var(--ink);resize:vertical;
}
.hint{font-size:12px;color:var(--dim);margin:.5rem 0 0}
.empty{padding:1.1rem 0;color:var(--dim);font-size:12.5px}

@media (max-width:560px){
  .axis{grid-template-columns:2ch 1fr 5ch 4ch;row-gap:.3rem}
  .axis input[type=range]{grid-column:1/-1}
  .axis .us{display:none}
}
@media (prefers-reduced-motion:reduce){*{transition:none!important}}
</style>
</head>
<body>
<div class="wrap">

<header>
  <div class="hrow">
    <h1>ARA</h1>
    <span class="sub mono" id="modeLabel">connecting</span>
    <span class="spacer"></span>
    <div class="seg" role="group" aria-label="Control mode">
      <button id="mMan" aria-pressed="false" onclick="setMode('MANUAL')">Manual</button>
      <button id="mCv"  aria-pressed="false" onclick="askCv()">Vision</button>
    </div>
  </div>
  <div class="status">
    <span><span class="dot" id="dLink"></span>link</span>
    <span><span class="dot" id="dPca"></span>servo board</span>
    <span><span class="dot" id="dUdp"></span>vision feed</span>
    <span id="sRx">rx <b>0</b></span>
    <span id="sBad">malformed <b>0</b></span>
    <span id="sMot"></span>
  </div>
</header>

<section>
  <div class="shead"><h2>Program pose</h2>
    <span class="note">MANUAL mode only &mdash; steppers are not touched, zero them by hand first</span></div>
  <div class="btns">
    <button class="b" id="btnStart" onclick="startPose()">Start</button>
  </div>
  <p class="hint" id="poseNote">loading&hellip;</p>
</section>

<section>
  <div class="shead"><h2>Grasps</h2>
    <span class="note">recall is a motion, not a jump &mdash; steppers take seconds</span></div>
  <div class="btns" id="presets"><span class="empty">loading</span></div>
</section>

<section>
  <div class="shead"><h2>Hand</h2><span class="note">servos on the PCA9685</span>
    <label style="margin-left:auto;display:flex;align-items:center;gap:.4rem;
                  font-size:11px;color:var(--dim);cursor:pointer">
      <input type="checkbox" id="rawTest" onchange="onRawToggle(this.checked)">
      full 0&ndash;180 sweep (ignore this axis's assumed limits)
    </label>
  </div>
  <div id="handAxes"><div class="empty">loading</div></div>
  <p class="hint">A slider only sends the first time you move it &mdash; untouched
     servos get no signal at all and stay wherever they physically are.</p>
</section>

<section>
  <div class="shead"><h2>Elbow</h2><span class="note">open-loop steppers &mdash; zero drifts silently</span></div>
  <div id="armAxes"></div>
  <div class="btns">
    <button class="b danger" onclick="zeroAxis(11)">Set roll zero here</button>
    <button class="b danger" onclick="zeroAxis(12)">Set extension zero here</button>
  </div>
  <p class="hint">Re-zero before every measurement session. A skipped step offsets
     every preset afterward with no warning anywhere in the system.</p>
</section>

<section>
  <div class="shead"><h2>Bench jog</h2>
    <span class="note">continuous spin for wiring/direction checks &mdash; MANUAL mode only</span></div>
  <div id="jogAxes"></div>
  <p class="hint">There is <b>no position limit</b> while jogging &mdash; the joint
     keeps turning until you press <b>stop</b>. Start at low Hz. Switching to
     Vision force-stops any active jog.</p>
</section>

<section>
  <div class="shead"><h2>Limit capture</h2><span class="note">for the TBD table in main.cpp</span></div>
  <p class="hint">Drive a joint to the edge of its safe travel, then press
     <b>lo</b> or <b>hi</b> on that row to record where you stopped. When every
     axis is marked, paste the result over <span class="mono">AXIS_MIN</span> and
     <span class="mono">AXIS_MAX</span>.</p>
  <div class="btns">
    <button class="b" onclick="buildLimits()">Build arrays</button>
    <button class="b" onclick="copyLimits()">Copy</button>
    <button class="b" onclick="clearLimits()">Clear marks</button>
  </div>
  <textarea id="limitOut" spellcheck="false" readonly
            placeholder="Marks you record appear here as C++ you can paste."></textarea>
</section>

</div>
<script>
const N=13, SERVO_N=11;
let st=null, dragging=-1, sendT={}, lo={}, hi={}, lastSent={}, rawTest=false;

const el=id=>document.getElementById(id);
const usFor=d=>Math.round(500+Math.max(0,Math.min(180,d))/180*2000);
const isServo=i=>i<SERVO_N;

async function poll(){
  try{
    const r=await fetch('/state',{cache:'no-store'});
    st=await r.json(); el('dLink').className='dot on'; render();
  }catch(e){ el('dLink').className='dot off'; el('modeLabel').textContent='no link'; }
}

function render(){
  if(!st) return;
  const cv = st.mode==='CV';
  el('mMan').setAttribute('aria-pressed', !cv);
  el('mCv').setAttribute('aria-pressed', cv);

  let label = cv ? 'vision' : 'manual';
  if(st.switching) label='lowering elbow';
  else if(st.ramping) label='easing in';
  else if(cv && st.stale) label='vision feed lost \u2014 holding';
  el('modeLabel').textContent=label;

  el('dPca').className='dot '+(st.pca?'on':'off');
  el('dUdp').className='dot '+(!cv?'':(st.stale?'off':'on'));
  el('sRx').innerHTML='rx <b>'+st.rx+'</b>';
  el('sBad').innerHTML='malformed <b>'+st.malformed+'</b>';
  const jogging = st.roll_jogging || st.ext_jogging;
  el('sMot').textContent = jogging ? 'jogging' : (st.roll_running||st.ext_running) ? 'moving' : '';

  el('btnStart').disabled = cv;
  el('poseNote').textContent = poseNoteText();

  if(el('presets').firstElementChild.tagName!=='BUTTON') drawPresets();
  if(!el('handAxes').querySelector('.axis')) drawAxes();
  syncAxes();
  if(!el('jogAxes').querySelector('.jogrow')) drawJog();
  syncJog();
}

function drawPresets(){
  el('presets').innerHTML = st.presets.map(p=>
    `<button class="b" onclick="preset('${p}')">${p}</button>`).join('');
}

function servoBounds(i){
  // rawTest bypasses this axis's assumed AXIS_MIN/AXIS_MAX -- see /axis raw=1.
  // Otherwise mirror the firmware's actual clamp so the slider can't ask for
  // something clampAxis() will just silently refuse.
  return rawTest ? [0,180] : [st.min[i], st.max[i]];
}

function row(i){
  const stepper=!isServo(i);
  const [min,max] = stepper ? [st.min[i], st.max[i]] : servoBounds(i);
  const chLabel = isServo(i) ? String(st.pca_ch[i]).padStart(2,'0') : '--';
  return `<div class="axis" id="ax${i}">
    <span class="idx mono" title="PCA9685 channel">${chLabel}</span>
    <span class="nm">${st.names[i].replace(/_/g,' ')}</span>
    <input type="range" min="${min}" max="${max}" step="1" value="0"
           id="sl${i}" aria-label="${st.names[i]}"
           oninput="drag(${i},this.value)"
           onpointerdown="dragging=${i}" onpointerup="dragging=-1"
           onpointercancel="dragging=-1">
    <span class="val mono" id="v${i}">--</span>
    <span class="us mono" id="u${i}">${stepper?'deg':''}</span>
    <span style="display:flex;gap:2px">
      <button class="mark" id="lo${i}" onclick="mark(${i},'lo')">lo</button>
      <button class="mark" id="hi${i}" onclick="mark(${i},'hi')">hi</button>
    </span>
  </div>`;
}

function onRawToggle(v){
  rawTest=v;
  for(let i=0;i<SERVO_N;i++){
    const [min,max]=servoBounds(i), sl=el('sl'+i);
    if(!sl) continue;
    sl.min=min; sl.max=max;
  }
  syncAxes();
}

function drawAxes(){
  let h='',a='';
  for(let i=0;i<N;i++){ (isServo(i)?h+=row(i):a+=row(i)); }
  el('handAxes').innerHTML=h; el('armAxes').innerHTML=a;
}

function syncAxes(){
  const cv = st.mode==='CV';
  for(let i=0;i<N;i++){
    const v=st.axes[i], sl=el('sl'+i), vv=el('v'+i);
    if(i!==dragging){ sl.value=Math.round(v); }
    vv.textContent = isServo(i)? Math.round(v) : (v>0?'+':'')+Math.round(v);
    if(isServo(i)) el('u'+i).textContent = usFor(v)+'\u00B5s';

    // Show when the firmware clamp actually bit. Without this the panel
    // silently lies about what you commanded.
    const sent=lastSent[i];
    vv.className = 'val mono' + (sent!==undefined && Math.abs(sent-v)>0.6 ? ' clamped':'');

    const locked = cv && i!==12;
    sl.disabled=locked;
    el('ax'+i).className='axis'+(locked?' dis':'');
    el('lo'+i).disabled=locked; el('hi'+i).disabled=locked;
    el('lo'+i).className='mark'+(lo[i]!==undefined?' has':'');
    el('hi'+i).className='mark'+(hi[i]!==undefined?' has':'');
  }
}

function drag(i,v){
  el('v'+i).textContent=v;
  if(isServo(i)) el('u'+i).textContent=usFor(+v)+'\u00B5s';
  clearTimeout(sendT[i]);
  const raw = rawTest && isServo(i) ? '&raw=1' : '';
  sendT[i]=setTimeout(()=>post('/axis','idx='+i+'&value='+v+raw,()=>{lastSent[i]=+v;}),45);
}

function post(path,body,then){
  fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body})
    .then(r=>{ if(r.ok) return r.json(); throw 0; })
    .then(j=>{ st=j; if(then)then(); render(); })
    .catch(()=>{});
}

function setMode(m){ lastSent={}; post('/mode','mode='+m); }
function askCv(){
  if(st && st.mode==='CV') return;
  if(confirm('Switching to vision lowers the elbow to its zero over several seconds. Anything the hand is holding goes with it.\n\nContinue?')) setMode('CV');
}
function preset(n){ lastSent={}; post('/preset','name='+encodeURIComponent(n)); }
function startPose(){ lastSent={}; post('/neutral',''); }
function poseNoteText(){
  const parts=[];
  for(let i=0;i<SERVO_N;i++) parts.push(st.names[i].replace(/_/g,' ')+' '+Math.round(st.neutral[i])+'\u00B0');
  return parts.join(', ');
}
function zeroAxis(i){
  const n=st?st.names[i].replace(/_/g,' '):'axis '+i;
  if(confirm('Declare the current physical position of '+n+' to be zero?\n\nEvery preset is measured from here.')) post('/zero','axis='+i);
}

const JOG_MIN=100, JOG_MAX=8000;

function jogRow(i){
  return `<div class="jogrow" id="jax${i}">
    <span class="idx mono">${String(i).padStart(2,'0')}</span>
    <span class="nm">${st.names[i].replace(/_/g,' ')}</span>
    <input type="number" id="jspd${i}" value="500" min="${JOG_MIN}" max="${JOG_MAX}"
           step="50" aria-label="jog speed, Hz">
    <span class="mono" style="font-size:11px;color:var(--dim)">Hz</span>
    <button class="b" onclick="jog(${i},-1)">run &#9664;</button>
    <button class="b" onclick="jog(${i},1)">run &#9654;</button>
    <button class="b danger" onclick="jogStop(${i})">stop</button>
    <span class="st mono" id="jst${i}"></span>
  </div>`;
}

function drawJog(){
  el('jogAxes').innerHTML = jogRow(11) + jogRow(12);
}

function syncJog(){
  const dis = st.mode==='CV';
  [11,12].forEach(i=>{
    const jogging = i===11 ? st.roll_jogging : st.ext_jogging;
    el('jax'+i).className = 'jogrow'+(dis?' dis':'');
    el('jax'+i).querySelectorAll('button,input').forEach(x=>x.disabled=dis);
    const s=el('jst'+i);
    s.textContent = jogging ? 'JOGGING' : '';
    s.className = 'st mono'+(jogging?' on':'');
  });
}

function jogSpeed(i){
  const v = +el('jspd'+i).value;
  return Math.max(JOG_MIN, Math.min(JOG_MAX, v||JOG_MIN));
}
function jog(i,dir){ post('/jog','axis='+i+'&dir='+dir+'&speed='+jogSpeed(i)); }
function jogStop(i){ post('/jog','axis='+i+'&stop=1'); }

function mark(i,which){
  const v=Math.round(st.axes[i]);
  (which==='lo'?lo:hi)[i]=v; syncAxes();
}
function clearLimits(){ lo={};hi={}; el('limitOut').value=''; syncAxes(); }

function buildLimits(){
  const mins=[],maxs=[]; let missing=[];
  for(let i=0;i<N;i++){
    if(lo[i]===undefined||hi[i]===undefined) missing.push(i);
    mins.push(lo[i]!==undefined?lo[i]:'TBD');
    maxs.push(hi[i]!==undefined?hi[i]:'TBD');
  }
  let out='';
  if(missing.length) out+='// still unmarked: axes '+missing.join(', ')+'\n';
  out+='static const float AXIS_MIN[AXIS_COUNT] = {\n  '+mins.join(', ')+'\n};\n';
  out+='static const float AXIS_MAX[AXIS_COUNT] = {\n  '+maxs.join(', ')+'\n};';
  el('limitOut').value=out;
}
async function copyLimits(){
  const t=el('limitOut'); if(!t.value) buildLimits();
  try{ await navigator.clipboard.writeText(t.value); }
  catch(e){ t.removeAttribute('readonly'); t.select(); t.setAttribute('readonly',''); }
}

poll(); setInterval(poll,600);
</script>
</body>
</html>)RAWHTML";