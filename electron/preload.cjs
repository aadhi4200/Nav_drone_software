// Intentionally minimal. The renderer needs no main-process APIs today —
// backend-URL config lives in localStorage (see src/backendUrl.ts). If a
// main-process capability is ever needed, expose it here via contextBridge
// rather than loosening contextIsolation/nodeIntegration.
