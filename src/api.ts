// api.ts — talks to backend/main.py (FastAPI + ROS2 bridge, default http://localhost:8000)

import { resolveBackendUrl } from './backendUrl';

// Read once at module load. The operator can repoint this at another machine
// via the Backend settings panel, which saves to localStorage and reloads the
// app — so every consumer (fetch, WebSocket hook, camera <img>) picks up the
// new address together without needing to be individually reactive.
export const API_BASE = resolveBackendUrl();
export const WS_BASE = API_BASE.replace(/^http/, 'ws');

export interface UploadWaypoint {
  lat: number;
  lon: number;
  alt?: number;
  label?: string;
  marker_id?: number;
}

export interface MissionStatus {
  mission_state: string;
  drone_status?: string;
  lat: number;
  lon: number;
  altitude: number;
  heading?: number;
  battery?: number;
  flight_mode?: string;
}

// ApiError carries the backend's structured 503 gate-rejection reasons
// (see backend/main.py's _require_all_clear) so the UI can show *why*.
export class ApiError extends Error {
  status: number;
  detail: unknown;
  constructor(status: number, detail: unknown) {
    super(typeof detail === 'string' ? detail : JSON.stringify(detail));
    this.status = status;
    this.detail = detail;
  }
}

async function post(path: string, body?: unknown) {
  const res = await fetch(`${API_BASE}${path}`, {
    method: 'POST',
    headers: body !== undefined ? { 'Content-Type': 'application/json' } : undefined,
    body: body !== undefined ? JSON.stringify(body) : undefined,
  });
  if (!res.ok) {
    const detail = await res.json().catch(() => res.statusText);
    throw new ApiError(res.status, (detail as any)?.detail ?? detail);
  }
  return res.json();
}

async function get(path: string) {
  const res = await fetch(`${API_BASE}${path}`);
  if (!res.ok) {
    const detail = await res.json().catch(() => res.statusText);
    throw new ApiError(res.status, (detail as any)?.detail ?? detail);
  }
  return res.json();
}

export async function uploadWaypoints(
  waypoints: UploadWaypoint[], speedMs?: number,
  landMode?: 'aruco' | 'gps', waitS?: number,
) {
  return post('/mission/upload', {
    waypoints, speed_ms: speedMs, land_mode: landMode, wait_s: waitS,
  });
}

// ── Feature 1: runtime ArUco marker generation ───────────────────────────
export interface MarkerGenerateResponse {
  status: string;
  model_name: string;
  marker_id: number;
  texture_path: string;
}
export async function generateMarker(label: string, lat: number, lon: number, markerId?: number) {
  return post('/markers/generate', { label, lat, lon, marker_id: markerId }) as Promise<MarkerGenerateResponse>;
}

// ── Feature 2 (3.3): laptop-geolocation-driven SITL home ─────────────────
export async function setHome(lat: number, lon: number) {
  return post('/system/set-home', { lat, lon }) as Promise<{ status: string; relaunch_needed: boolean }>;
}
export interface HomeSyncState { lat: number | null; lon: number | null; synced_at: string | null; }
export async function getHome(): Promise<HomeSyncState> {
  return get('/system/home');
}

// ── Feature 4: sim/hardware mode ─────────────────────────────────────────
export async function getMode(): Promise<{ mode: 'sim' | 'hardware' }> {
  return get('/system/mode');
}
export async function setMode(mode: 'sim' | 'hardware') {
  return post('/system/mode', { mode });
}

// ── Feature 11: hardware profile + range estimate ────────────────────────
import type { DroneProfile, RangeEstimate } from './types';
export async function getProfile(): Promise<{ profile: DroneProfile; estimate: RangeEstimate }> {
  return get('/system/profile');
}
export async function setProfile(profile: DroneProfile): Promise<{ profile: DroneProfile; estimate: RangeEstimate }> {
  return post('/system/profile', profile);
}

// ── Feature 10: travel log ────────────────────────────────────────────────
export async function getTravelLog(missionId: number) {
  return get(`/missions/${missionId}/travel-log`);
}

// ── Geofence (QGC-style polygon fence, enforced by PX4 itself) ───────────
export interface GeofenceVertex { lat: number; lon: number; }
export interface GeofenceState {
  vertices: GeofenceVertex[];
  action: string | null;
  set_at: string | null;
  pushed_to_px4: boolean;
}
export async function getGeofence(): Promise<GeofenceState> {
  return get('/geofence');
}
export async function setGeofence(vertices: GeofenceVertex[], action: string = 'return') {
  return post('/geofence', { vertices, action }) as Promise<{
    status: string; vertex_count: number; action: string; pushed_to_px4: boolean;
  }>;
}
export async function clearGeofence() {
  const res = await fetch(`${API_BASE}/geofence`, { method: 'DELETE' });
  if (!res.ok) {
    const detail = await res.json().catch(() => res.statusText);
    throw new ApiError(res.status, (detail as any)?.detail ?? detail);
  }
  return res.json() as Promise<{ status: string; cleared_on_px4: boolean }>;
}

export async function startMission() {
  return post('/mission/start');
}

export async function abortMission() {
  return post('/mission/abort');
}

export async function returnHome() {
  return post('/mission/return-home');
}

export async function resetMission() {
  return post('/mission/reset');
}

export async function armDrone() {
  return post('/drone/arm');
}

export async function disarmDrone() {
  return post('/drone/disarm');
}

export async function takeoffDrone() {
  return post('/drone/takeoff');
}

export async function landDrone() {
  return post('/drone/land');
}

export type ManualNudgeCmd =
  | 'FWD' | 'BACK' | 'LEFT' | 'RIGHT' | 'UP' | 'DOWN' | 'YAW_LEFT' | 'YAW_RIGHT' | 'HOLD';

export async function manualNudge(cmd: ManualNudgeCmd) {
  return post(`/drone/manual/${cmd}`);
}

export async function getMissionStatus(): Promise<MissionStatus> {
  const res = await fetch(`${API_BASE}/mission/status`);
  if (!res.ok) throw new Error(`GET /mission/status failed: ${res.status}`);
  return res.json();
}
