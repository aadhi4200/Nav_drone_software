# CLAUDE.md — Electron Windows `.exe` Packaging for Drone Dashboard

> Task spec for Claude Code. Goal: wrap the existing React/TypeScript drone dashboard in
> Electron and produce a distributable Windows `.exe`, **without** changing any of the
> dashboard's existing functionality, ROS 2/backend architecture, or the WebSocket/REST logic
> already built out per the project's main `CLAUDE.md`.

---

## 0. Source repo (ground truth — read before touching anything)

| Repo | Role |
|---|---|
| `https://github.com/aadhi4200/Drone-A-to-B-waypoint-Nav-system.git` | React + TypeScript + MapLibre frontend (`src/App.tsx`, `src/components/MapPane.tsx`, `src/api.ts`, `src/types.ts`) and the FastAPI + rclpy bridge (`backend/main.py`). |

**Check the build tool before doing anything else** — inspect `package.json` `scripts` and
`devDependencies` to confirm whether this project uses **Vite** (`vite build`, output in `dist/`)
or **Create React App** (`react-scripts build`, output in `build/`). Everything below assumes
Vite unless you find otherwise; if it's CRA, swap the build command and output directory
accordingly and note that change explicitly in your summary.

---

## 1. Critical architectural constraint — read this first

**The Electron app is a UI shell only.** It does **not** bundle or run the FastAPI/rclpy backend.
The backend (`backend/main.py`) requires `rclpy` and a live ROS 2 environment, which only exists
on the Ubuntu dev machine / companion computer (Raspberry Pi 5) — it cannot run inside a packaged
Windows `.exe`. Do not attempt to bundle Python, ROS 2, or the backend into the Electron build.

The packaged app connects to the backend over the network (`http://<backend-host>:8000` and
`ws://<backend-host>:8000/ws/system-status`), exactly like the browser version does today — the
only difference is it now runs in a native window instead of a browser tab.

**Implication:** the backend base URL must be **configurable at runtime**, not hardcoded to
`localhost`. If `src/api.ts` (or wherever the API base URL lives) currently hardcodes
`http://localhost:8000`, this is a bug for this task and must be fixed as part of it — a Windows
laptop running the packaged `.exe` needs to reach the Ubuntu machine's IP on the LAN, not its own
loopback address.

---

## 2. What to build

### 2.1 Electron shell

- Add `electron/main.js` (or `electron/main.ts` if the repo is already TS-first for tooling
  scripts) as the Electron main process entry point. Responsibilities:
  - Create a single `BrowserWindow` (suggest 1400×900 default, resizable) with
    `contextIsolation: true`, `nodeIntegration: false` — do not weaken these for convenience,
    there's no reason this app needs raw Node access from the renderer.
  - In production, load the built static output (`dist/index.html` for Vite) via `loadFile`.
  - In dev, load `http://localhost:5173` (or whatever the Vite dev server port is) via `loadURL`,
    gated behind an `NODE_ENV`/`ELECTRON_START_URL` check so the same `main.js` works for both
    `npm run electron:dev` and the packaged build without manual edits.
- Add a minimal `electron/preload.js` using `contextBridge` if any main-process APIs are needed
  later (e.g. reading a saved backend-URL config from disk) — don't add IPC surface area that
  isn't actually used yet, keep this empty/minimal for the first pass.

### 2.2 Backend URL configuration (required, not optional)

Do not ship this with a hardcoded backend IP. Implement one of the following (recommend the
first for simplicity):

- **(a) Settings screen (recommended):** a small in-app settings panel (new component, e.g.
  `BackendSettings.tsx`) where the operator enters/edits the backend host:port once. Persist it
  — since `localStorage`/browser storage behaves normally inside Electron's renderer (this is
  *not* the Claude-artifact sandbox restriction, this is a real Electron app), `localStorage` is
  fine here. Read this value wherever `src/api.ts` currently constructs request URLs and wherever
  the WebSocket hook (Feature 3/7/9 from the main `CLAUDE.md`) opens its connection.
- **(b) Env var at build time:** bake a default backend URL in via `.env` (`VITE_API_BASE_URL`)
  for convenience, but still allow runtime override via (a) — a fixed build-time-only URL means
  rebuilding the `.exe` every time the Pi's IP changes, which is worse UX than a settings field.
- Show the currently-configured backend address somewhere persistently visible in the UI (status
  bar, per the existing status-bar pattern from the main project spec) so it's never ambiguous
  which machine the dashboard thinks it's talking to.

### 2.3 Build tooling

Add to `package.json`:

```json
{
  "main": "electron/main.js",
  "scripts": {
    "electron:dev": "concurrently \"vite\" \"wait-on tcp:5173 && electron .\"",
    "electron:build": "vite build && electron-builder --win"
  },
  "build": {
    "appId": "com.aadhi.dronedashboard",
    "productName": "Drone Mission Dashboard",
    "files": ["dist/**/*", "electron/**/*"],
    "win": {
      "target": "nsis",
      "icon": "assets/icon.ico"
    },
    "nsis": {
      "oneClick": false,
      "allowToChangeInstallationDirectory": true
    }
  }
}
```

Install dev dependencies: `electron`, `electron-builder`, `concurrently`, `wait-on`.

Adjust the `files`/output-directory references if the repo turns out to be CRA (`build/` instead
of `dist/`) per section 0.

### 2.4 App icon

Add a placeholder `assets/icon.ico` (256×256 minimum, multi-resolution `.ico`) if one doesn't
already exist in the repo — `electron-builder` will fail or fall back to a generic icon without
it. Generating a polished icon is out of scope for this task; a simple placeholder is fine,
flag it as a follow-up.

---

## 3. Explicitly out of scope for this task

- **No changes to `backend/main.py`** or any ROS 2 node — this task only touches the frontend
  repo's packaging, not the robotics stack.
- **No bundling of Python/ROS 2/rclpy into the Electron app.** If you find yourself trying to
  make this work, stop — it's the wrong direction. The backend stays on Ubuntu/the Pi.
- **No code-signing setup.** The unsigned `.exe` will trigger Windows SmartScreen's "unrecognized
  app" warning — that's expected and acceptable for now. Note it in the README, don't try to
  solve it (requires a paid cert, separate concern).
- **No macOS/Linux packaging targets** in this pass — `--win` only. `electron-builder` supports
  multi-platform later if needed, but don't add the complexity now.
- **No changes to the existing WebSocket reconnect-with-backoff logic** beyond what's needed to
  make the backend URL configurable (section 2.2) — the existing "losing the socket must never
  affect the drone, only the UI" principle from the main project spec still applies unchanged.

---

## 4. Build environment note (cross-compilation caveat)

The dev machine is Ubuntu 22.04. `electron-builder --win` *can* cross-compile a Windows target
from Linux using Wine, but this is known to be fragile (missing Wine deps, code-signing tool
failures even when signing is disabled, etc.). If `npm run electron:build` fails with Wine-related
errors on the Ubuntu machine:

1. Try `sudo apt install wine` (or the specific missing dependency named in the error) first.
2. If it's still unreliable after one or two fix attempts, **don't sink further time into it** —
   document the failure and recommend running `npm run electron:build` directly on a Windows
   machine or a Windows VM/GitHub Actions Windows runner instead. This is a normal, expected
   fallback for this toolchain, not a sign something is broken in the app code.

---

## 5. Acceptance criteria

- [ ] `npm run electron:dev` launches the dashboard in a native Electron window, hot-reloading
      against the Vite dev server, with full existing functionality (map, waypoints, telemetry,
      WebSocket status) intact and unchanged.
- [ ] `npm run electron:build` produces a Windows installer (`.exe` via NSIS) in the
      `electron-builder` output directory, without errors (or with a documented Wine-related
      fallback per section 4).
- [ ] The packaged app does **not** hardcode `localhost` as the backend address — a settings
      screen or equivalent lets the operator point it at the Ubuntu machine/Pi's actual LAN IP,
      and this is persisted across app restarts.
- [ ] The currently-configured backend address is visible somewhere in the UI at all times.
- [ ] Installing and running the packaged `.exe` on a separate Windows machine (not the dev box)
      successfully connects to a backend running on the Ubuntu machine over LAN — verified by an
      actual cross-machine test, not just "it builds."
- [ ] No Python, ROS 2, or backend code has been bundled into the Electron app or its build
      output.
- [ ] `contextIsolation: true` and `nodeIntegration: false` remain set in the `BrowserWindow`
      config — verify this wasn't loosened to work around some unrelated bug during development.

---

## 6. Open questions to resolve before/during implementation

Flag these back rather than guessing silently:

1. **Vite or CRA?** (section 0) — changes build command and output path.
2. **Fixed backend IP vs. per-session settings field** (section 2.2) — recommended default is a
   settings field with a sensible placeholder, but confirm this matches how the dashboard will
   actually be used (e.g. is the Pi's IP static once deployed, in which case baking a default
   *and* allowing override might be the better UX).
3. Does an `assets/icon.ico` already exist anywhere in the repo, or does one need to be created
   from scratch (even as a placeholder)?
