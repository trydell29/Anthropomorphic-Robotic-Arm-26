// Copy this file to wifi_secrets.h and fill in your own network.
//
//     cp include/wifi_secrets.example.h include/wifi_secrets.h
//
// wifi_secrets.h is gitignored. Never commit real credentials -- this repo is
// public, and git history is permanent even if you delete the file later.
//
// This is optional and development-only. Without it the board runs on its
// softAP alone, which is how the arm actually operates in the field. With it,
// the board ALSO joins your network so your laptop keeps its internet while
// you work, and the board is reachable at http://ara.local
//
// 2.4 GHz only. The ESP32 has no 5 GHz radio, so if your router publishes one
// SSID for both bands you may need the 2.4 GHz-specific name.

#pragma once

#define STA_SSID "ara26"
#define STA_PASS "armin"