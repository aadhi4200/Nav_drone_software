import React, { useState } from 'react';
import { createPortal } from 'react-dom';
import { Server, X, RotateCcw } from 'lucide-react';
import {
  DEFAULT_BACKEND_URL, normalizeBackendUrl,
  getSavedBackendUrl, saveBackendUrl, clearSavedBackendUrl,
} from '../backendUrl';
import { API_BASE } from '../api';

interface BackendSettingsProps {
  open: boolean;
  onClose: () => void;
}

// Backend address settings (Electron packaging task, CLAUDE.md §2.2). The
// packaged .exe runs on a Windows laptop while the FastAPI/ROS2 backend runs
// on the Ubuntu machine / Pi — so "localhost" is only right on the dev box.
// Saving reloads the app: api.ts reads the URL once at module load, and a
// reload is the one moment every consumer (REST, WebSocket, camera stream)
// re-resolves it together.
export default function BackendSettings({ open, onClose }: BackendSettingsProps) {
  const [value, setValue] = useState(getSavedBackendUrl() ?? '');
  const [error, setError] = useState<string | null>(null);

  if (!open) return null;

  const handleSave = () => {
    const normalized = normalizeBackendUrl(value);
    if (!normalized) {
      setError('Enter a host, host:port, or full http:// URL.');
      return;
    }
    saveBackendUrl(normalized);
    window.location.reload();
  };

  const handleReset = () => {
    clearSavedBackendUrl();
    window.location.reload();
  };

  return createPortal(
    <div
      className="fixed inset-0 z-[1000] flex items-center justify-center bg-black/70 backdrop-blur-sm"
      onClick={onClose}
    >
      <div
        className="bg-[#141417] border border-white/10 rounded-2xl p-5 w-[380px] space-y-4 shadow-2xl"
        onClick={e => e.stopPropagation()}
      >
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-2">
            <Server className="w-4 h-4 text-[#5996FF]" />
            <h3 className="font-semibold text-white tracking-wide text-xs uppercase font-display">Backend Connection</h3>
          </div>
          <button onClick={onClose} className="text-[#7c7c84] hover:text-white cursor-pointer" title="Close">
            <X className="w-4 h-4" />
          </button>
        </div>

        <div className="bg-[#0a0a0c]/90 rounded-xl border border-white/10 p-3 space-y-1.5 text-[10.5px] font-mono">
          <div className="flex justify-between">
            <span className="text-[#7c7c84] uppercase">Currently using</span>
            <span className="text-white font-bold">{API_BASE}</span>
          </div>
          <div className="flex justify-between">
            <span className="text-[#7c7c84] uppercase">Default</span>
            <span className="text-[#9a9aa2]">{DEFAULT_BACKEND_URL}</span>
          </div>
        </div>

        <div className="space-y-1.5">
          <label className="text-[9.5px] text-[#7c7c84] font-mono uppercase tracking-wide">
            Backend host / URL (Ubuntu machine or Pi on your LAN)
          </label>
          <input
            type="text"
            value={value}
            onChange={e => { setValue(e.target.value); setError(null); }}
            onKeyDown={e => { if (e.key === 'Enter') handleSave(); }}
            placeholder="e.g. 192.168.1.42  or  pi5.local:8000"
            className="w-full bg-[#0a0a0c] border border-white/10 rounded-lg px-3 py-2 text-[11.5px] font-mono text-white placeholder-[#5c5c64] focus:border-[#5996FF]/60 focus:outline-none"
            autoFocus
          />
          {error && <p className="text-[9.5px] text-red-400 font-mono">{error}</p>}
          <p className="text-[9px] text-[#7c7c84] font-mono leading-relaxed">
            Port defaults to 8000 if omitted. Saving reconnects the dashboard
            (REST, live WebSocket and camera stream) to the new address.
          </p>
        </div>

        <div className="flex items-center gap-2">
          <button
            onClick={handleSave}
            className="flex-1 py-2 rounded-xl text-xs font-bold font-sans bg-[#5996FF] text-black hover:bg-[#ffdd55] transition-all cursor-pointer"
          >
            Save & Reconnect
          </button>
          <button
            onClick={handleReset}
            disabled={getSavedBackendUrl() === null}
            title="Forget the saved address and go back to the default"
            className="flex items-center gap-1 py-2 px-3 rounded-xl text-[10px] font-bold font-mono uppercase text-[#9a9aa2] hover:text-white bg-[#0a0a0c] border border-white/10 disabled:opacity-40 disabled:cursor-not-allowed cursor-pointer"
          >
            <RotateCcw className="w-3 h-3" /> Default
          </button>
        </div>
      </div>
    </div>,
    document.body
  );
}
