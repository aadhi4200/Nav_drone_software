<div align="center">
<img width="1200" height="475" alt="GHBanner" src="https://ai.google.dev/static/site-assets/images/share-ais-513315318.png" />
</div>

# Run and deploy your AI Studio app

This contains everything you need to run your app locally.

View your app in AI Studio: https://ai.studio/apps/9ffbfceb-35ae-42d4-a785-2b6d0cf9872a

## Run Locally

**Prerequisites:**  Node.js


1. Install dependencies:
   `npm install`
2. Set the `GEMINI_API_KEY` in [.env.local](.env.local) to your Gemini API key
3. Run the app:
   `npm run dev`




┌─────────────────────────────────────────────────┐
│  UAV  SkyNav Avionics Systems        ○ SIM MODE │ ← header badge
│                                   PORTAL SECURE  │
├──────────────────────────┬──────────────────────┤
│                          │  Mission Control      │
│   MapLibre Map           │  [IDLE] [○ SIM MODE]  │
│   (click to set dest)    │                       │
│                          │  [Launch Mission]     │
│                          │  [Emergency Override] │
├──────────────────────────┴──────────────────────┤
│   Telemetry Log Stream                          │
└─────────────────────────────────────────────────┘

npm install

npm run dev

# Open the app directly in a new browser tab
# Type this URL directly in your browser address bar:
http://localhost:5173

pip install fastapi uvicorn

# Run the bridge
cd ~/drone_ws/src/autonomous_drone_ros2
python backend/main.py



---

## Desktop app (Electron, Windows `.exe`)

The dashboard can run as a native Windows app instead of a browser tab. The Electron app is a
**UI shell only** — the FastAPI/ROS 2 backend still runs on the Ubuntu machine / companion
computer (Pi), and the app talks to it over the LAN exactly like the browser version.

### Dev (hot reload)

```bash
npm run electron:dev   # starts Vite on :3000 and opens the Electron window against it
```

### Build the Windows installer

```bash
npm run electron:build   # vite build + electron-builder → release/Drone Mission Dashboard Setup <version>.exe
```

Build on a Windows machine (or a Windows VM / GitHub Actions windows runner). Cross-compiling
from Ubuntu via Wine is possible but fragile — not the supported path here.

### Pointing the app at the backend

The backend address is **configurable at runtime** — click the host chip in the top-right of the
header (next to "PORTAL SECURE") and enter the Ubuntu machine's / Pi's LAN address (e.g.
`192.168.1.42` — port defaults to 8000). The value persists across app restarts (localStorage).
Priority: saved value → `VITE_API_URL` from `.env` at build time → `http://localhost:8000`.

### Notes

- The installer is **unsigned**, so Windows SmartScreen will show an "unrecognized app" warning
  on first run — click "More info → Run anyway". Code-signing needs a paid certificate and is a
  separate concern.
- `assets/icon.ico` is a generated placeholder (top-down quad glyph) — replace with a real icon
  whenever branding matters.
- The backend must be reachable on port 8000 from the Windows machine (check Ubuntu's firewall
  and that uvicorn binds `0.0.0.0`, which `backend/main.py` already does).
