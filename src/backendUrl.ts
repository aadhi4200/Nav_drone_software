// Runtime-configurable backend base URL (Electron packaging task, CLAUDE.md §2.2).
// Priority: operator-saved value (localStorage) → VITE_API_URL build-time env →
// http://localhost:8000. api.ts reads this once at module load; BackendSettings
// reloads the app on save so a change takes effect everywhere (fetch, WebSocket,
// camera stream) at once.

const STORAGE_KEY = 'drone_backend_url';

export const DEFAULT_BACKEND_URL =
  (import.meta as any).env?.VITE_API_URL || 'http://localhost:8000';

/** "192.168.1.42", "pi5:8000", "http://host:8000" → "http://host:port" (or null if unparseable). */
export function normalizeBackendUrl(raw: string): string | null {
  let v = raw.trim().replace(/\/+$/, '');
  if (!v) return null;
  if (!/^https?:\/\//i.test(v)) v = `http://${v}`;
  try {
    const u = new URL(v);
    if (!u.hostname) return null;
    const port = u.port || '8000';
    return `${u.protocol}//${u.hostname}:${port}`;
  } catch {
    return null;
  }
}

export function getSavedBackendUrl(): string | null {
  try {
    return localStorage.getItem(STORAGE_KEY);
  } catch {
    return null;
  }
}

export function saveBackendUrl(url: string): void {
  localStorage.setItem(STORAGE_KEY, url);
}

export function clearSavedBackendUrl(): void {
  localStorage.removeItem(STORAGE_KEY);
}

export function resolveBackendUrl(): string {
  return getSavedBackendUrl() || DEFAULT_BACKEND_URL;
}
