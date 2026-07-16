// Electron main process — UI shell only. The FastAPI/rclpy backend runs on the
// Ubuntu machine / companion computer and is reached over the network; nothing
// backend-related is bundled here (see CLAUDE.md section 1).
//
// .cjs because package.json has "type": "module" — Electron's main process
// loads this with require() semantics.
const { app, BrowserWindow, shell } = require('electron');
const path = require('path');

function createWindow() {
  const win = new BrowserWindow({
    width: 1400,
    height: 900,
    backgroundColor: '#0a0a0c',
    webPreferences: {
      // Security posture required by the task spec — do not loosen.
      contextIsolation: true,
      nodeIntegration: false,
      preload: path.join(__dirname, 'preload.cjs'),
    },
  });
  win.setMenuBarVisibility(false);

  // Dev: `npm run electron:dev` sets ELECTRON_START_URL to the Vite dev
  // server. Prod (packaged): load the built static bundle from dist/.
  const startUrl = process.env.ELECTRON_START_URL;
  if (startUrl) {
    win.loadURL(startUrl);
  } else {
    win.loadFile(path.join(__dirname, '..', 'dist', 'index.html'));
  }

  // Any target="_blank"/window.open goes to the system browser, not a new
  // Electron window.
  win.webContents.setWindowOpenHandler(({ url }) => {
    shell.openExternal(url);
    return { action: 'deny' };
  });
}

app.whenReady().then(() => {
  createWindow();
  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});
