import React, { useState, useEffect, useRef } from 'react';
import { uploadWaypoints, abortMission, returnHome, resetMission, getMissionStatus, armDrone, disarmDrone, takeoffDrone, landDrone, manualNudge, ManualNudgeCmd, generateMarker, setHome, getHome, getMode, getTravelLog, getGeofence, setGeofence, clearGeofence, ApiError, API_BASE } from './api';
import BackendSettings from './components/BackendSettings';
// NOTE: We import our api functions but rename the local startMission
// to avoid conflict with the imported one
import { startMission as ros2Start } from './api';
import {
  ShieldAlert, Zap, Radio, AlertTriangle,
  MapPin, Download, Check, Copy, KeyRound
} from 'lucide-react';
import { FlightState, LatLng, BatteryState, SignalState, ClimateState, SensorOrientation, TelemetryLog, Obstacle, MissionWaypoint } from './types';
import MapPane from './components/MapPane';
import MapErrorBoundary from './components/MapErrorBoundary';
import FlightControlPanel from './components/FlightControlPanel';
import SensorReadout from './components/SensorReadout';
import TelemetryTerminal, { TelemetryLogStream, TelemetryInsights } from './components/TelemetryTerminal';
import CameraFeed from './components/CameraFeed';
import IMUGraph from './components/IMUGraph';
import FlightTestBench from './components/FlightTestBench';
import ConnectivityBanner from './components/ConnectivityBanner';
import WaypointList from './components/WaypointList';
import DroneProfilePanel from './components/DroneProfilePanel';
import { useSystemStatusSocket } from './hooks/useSystemStatusSocket';

const API_KEY =
  process.env.GOOGLE_MAPS_PLATFORM_KEY ||
  (import.meta as any).env?.VITE_GOOGLE_MAPS_PLATFORM_KEY ||
  (globalThis as any).GOOGLE_MAPS_PLATFORM_KEY ||
  '';
const hasValidKey = Boolean(API_KEY) && API_KEY !== 'YOUR_API_KEY' && API_KEY !== '';

const TARGET_ALTITUDE_M = 2.5;
// Keep in sync with backend/main.py's ABORT_ALTITUDE_M -- must clear
// RTH_ALTITUDE (drone_interfaces/constants.py, 7.0m) with margin, confirmed
// live 2026-07-10: with the old 3.0m value, every return-home self-aborted
// via this exact check mid-climb before ever reaching home.
const ABORT_ALTITUDE_M  = 10.0;

const STATIC_OBSTACLES: Obstacle[] = [
  { id: "obs_crane_1",  lat: 9.969200, lng: 76.244800, radiusMeters: 45, heightMeters: 35, type: "Harbour Gantry Crane" },
  { id: "obs_mast_2",   lat: 9.961500, lng: 76.236000, radiusMeters: 35, heightMeters: 42, type: "Navigational Beacon Mast" },
  { id: "obs_tower_3",  lat: 9.974000, lng: 76.233500, radiusMeters: 40, heightMeters: 48, type: "High-Voltage Power Pylon" },
];

// ── Last-known real position, cached across page loads ──────────────────
// startLoc/dronePos/destLoc used to always start from a hardcoded
// Kochi-area coordinate, then get corrected asynchronously once
// getHome()/geolocation resolved -- meaning the wrong city briefly (or, if
// both of those ever fail, indefinitely) showed on every load, confirmed
// live 2026-07-11. useState's initializer must be synchronous, so it can't
// literally await geo access -- but it CAN synchronously read the last
// value that a *previous* successful sync already wrote to localStorage,
// which is genuinely "derived from this PC's geo access", just persisted
// across reloads instead of re-derived from scratch. First-ever run with
// no cache yet still falls back to the literal default once; every load
// after any successful sync starts from the real place instead.
const LAST_KNOWN_POSITION_KEY = 'drone_last_known_position';

function readCachedPosition(): LatLng | null {
  try {
    const raw = localStorage.getItem(LAST_KNOWN_POSITION_KEY);
    if (!raw) return null;
    const parsed = JSON.parse(raw);
    if (typeof parsed.lat === 'number' && typeof parsed.lng === 'number') return parsed;
  } catch {
    // Corrupt/inaccessible localStorage -- fall through to the hardcoded default.
  }
  return null;
}

function writeCachedPosition(lat: number, lng: number) {
  try {
    localStorage.setItem(LAST_KNOWN_POSITION_KEY, JSON.stringify({ lat, lng }));
  } catch {
    // Storage full/disabled -- non-fatal, just means next load won't have it cached.
  }
}

export default function App() {
  // ── Core location state ─────────────────────────────
  const [startLoc,   setStartLoc]   = useState<LatLng>(() => readCachedPosition() ?? { lat: 9.965800, lng: 76.242100 });
  // No default destination: the green dest marker/dashed path must not appear
  // until the operator actually picks one (map click or manual entry).
  const [destLoc,    setDestLoc]    = useState<LatLng | null>(null);
  const [activePage, setActivePage] = useState<'mission' | 'testbench'>('mission');
  const [showBackendSettings, setShowBackendSettings] = useState(false);
  // Host:port only — the full URL is in the chip tooltip and settings panel.
  const backendHost = (() => { try { return new URL(API_BASE).host; } catch { return API_BASE; } })();
  const [dronePos,   setDronePos]   = useState(() => {
    const cached = readCachedPosition();
    return cached ? { ...cached, heading: 0 } : { lat: 9.965800, lng: 76.242100, heading: 0 };
  });
  const [obstacles,  setObstacles]  = useState<Obstacle[]>(STATIC_OBSTACLES);

  // ── Path state ──────────────────────────────────────
  const [directPath,       setDirectPath]       = useState<LatLng[]>([]);
  const [plannedPath,      setPlannedPath]      = useState<LatLng[]>([]);
  const [currentPathIndex, setCurrentPathIndex] = useState(0);

  // ── Flight state ────────────────────────────────────
  const [flightState,    setFlightState]    = useState<FlightState>(FlightState.IDLE);
  const [logs,           setLogs]           = useState<TelemetryLog[]>([]);
  const [missionTimeSec, setMissionTimeSec] = useState<number>(0);

  // ── GPS sync ────────────────────────────────────────
  const [gpsSyncStatus, setGpsSyncStatus] = useState<'idle' | 'locating' | 'success' | 'error'>('idle');
  const [gpsSyncError,  setGpsSyncError]  = useState<string | null>(null);

  // ── ROS2 connection state (NEW) ─────────────────────
  const [ros2Connected, setRos2Connected] = useState<boolean>(false);
  const [droneStatus,   setDroneStatus]   = useState<string>('DISCONNECTED');

  // ── Mission stops (B/C/D...), speed, traveled trail (Features 1/2/5/8/9) ──
  const [waypoints, setWaypoints] = useState<MissionWaypoint[]>([]);
  const [speedMs, setSpeedMs] = useState<number>(3.0);
  // Landing choice at each stop: ArUco precision landing vs plain GPS
  // AUTO.LAND (no marker), plus the adjustable ground wait before the
  // next takeoff.
  const [landMode, setLandMode] = useState<'aruco' | 'gps'>('aruco');
  const [waitS, setWaitS] = useState<number>(5);
  const [traveledPath, setTraveledPath] = useState<LatLng[]>([]);
  const nextStopLetter = useRef<number>(0); // 0 -> 'B', 1 -> 'C', ...
  const MAX_TRAVELED_POINTS = 2000;

  // ── Home sync staleness display (section 3.3.5) ─────
  const [homeLastSyncedAt, setHomeLastSyncedAt] = useState<string | null>(null);

  // ── Sim/hardware mode (Feature 4) ───────────────────
  const [mode, setModeState] = useState<'sim' | 'hardware'>('sim');
  useEffect(() => { getMode().then(({ mode }) => setModeState(mode)).catch(() => {}); }, []);

  // ── Geofence (QGC-style): committed polygon + in-progress draft ─────
  const [geofence, setGeofenceState] = useState<LatLng[]>([]);
  const [fenceDraft, setFenceDraft] = useState<LatLng[]>([]);
  const handleFenceVertex = (loc: LatLng) => setFenceDraft(prev => [...prev, loc]);

  const handleFinishFence = async () => {
    if (fenceDraft.length < 3) return;
    try {
      const res = await setGeofence(fenceDraft.map(p => ({ lat: p.lat, lon: p.lng })), 'return');
      setGeofenceState(fenceDraft);
      setFenceDraft([]);
      addNewLogEntry(flightState,
        `GEOFENCE: set (${res.vertex_count} vertices, breach action RETURN)` +
        (res.pushed_to_px4 ? ' — enforced by PX4.' : ' — will push to PX4 on connect.'));
    } catch (e) {
      const msg = e instanceof ApiError ? String(e.detail) : 'backend unreachable';
      addNewLogEntry(flightState, `GEOFENCE: rejected — ${msg}`);
    }
  };

  const handleClearFence = async () => {
    setFenceDraft([]);
    if (geofence.length === 0) return;
    try {
      await clearGeofence();
      setGeofenceState([]);
      addNewLogEntry(flightState, 'GEOFENCE: cleared (PX4 fence wiped, breach action off).');
    } catch {
      addNewLogEntry(flightState, 'GEOFENCE: clear failed — backend unreachable, fence unchanged.');
    }
  };

  // ── Live WebSocket push: node/preflight status, IMU, position ───────
  const { connected: wsConnected, nodeStatus, imu, position: wsPosition, missionState: wsMissionState } = useSystemStatusSocket();

  useEffect(() => {
    getGeofence()
      .then(g => {
        setGeofenceState((g?.vertices ?? []).map(v => ({ lat: v.lat, lng: v.lon })));
      })
      .catch(() => {});
  }, [wsConnected]);

  // ── Simulation ──────────────────────────────────────
  const [simulationWindSpeed, setSimulationWindSpeed] = useState<number>(8.5);

  const [battery, setBattery] = useState<BatteryState>({
    percentage: 100, voltage: 16.8,
    cellVoltages: [4.20, 4.20, 4.20, 4.20],
    temperatureCelsius: 24.5, healthPercent: 98.6, dischargeRateAmps: 0.15
  });

  const [signal, setSignal] = useState<SignalState>({
    strengthDbm: -48, qualityPercent: 100,
    encryptionKey: "5f8a92b4c7d6e1f0a38b9d7c6e5a4f3b2c1d0e9f8a7b6c5d4e3f2a1b0c9d8e7f",
    isEncrypted: true, protocol: "AES-256-GCM"
  });

  const [climate, setClimate] = useState<ClimateState>({
    ambientTempC: 18.2, batteryTempC: 24.5,
    heaterActive: false, coolerActive: false,
    windSpeedKnots: 8.5, windDirectionDegrees: 140,
    airDensityKgM3: 1.204, thermalThrottling: false
  });

  const [sensors, setSensors] = useState<SensorOrientation>({
    pitch: 0, roll: 0, yaw: 0,
    gpsAccuracyMothers: 0.02, barometerAltitudeM: 0.0,
    lidarDistanceM: 50.0, obstacleAvoidanceActive: false, obstacleType: null
  });

  const mainTickerInterval = useRef<NodeJS.Timeout | null>(null);
  const logTickerDivider   = useRef(0);
  const degToMeter         = 111000;
  const didInitialGpsSync  = useRef(false);

  // ── GPS sync ────────────────────────────────────────
  const applySyncedLocation = (userLat: number, userLng: number) => {
    setStartLoc({ lat: userLat, lng: userLng });
    setDronePos({ lat: userLat, lng: userLng, heading: 0 });
    writeCachedPosition(userLat, userLng);
    setDestLoc({ lat: userLat + 0.0075, lng: userLng + 0.0065 });
    setObstacles([
      { id: "local_antenna_1", lat: userLat+0.0034, lng: userLng+0.0028, radiusMeters: 45, heightMeters: 35, type: "RF Antenna Tower" },
      { id: "local_grid_2",    lat: userLat+0.0015, lng: userLng+0.0048, radiusMeters: 38, heightMeters: 42, type: "Electrical Grid Substation" },
      { id: "local_pylon_3",   lat: userLat+0.0049, lng: userLng-0.0035, radiusMeters: 48, heightMeters: 48, type: "Transmission Pylon Zone" },
    ]);
    setGpsSyncStatus('success');
    addNewLogEntry(FlightState.IDLE, `GPS LOCK SUCCESS: Lat:${userLat.toFixed(6)} Lng:${userLng.toFixed(6)}`);

    // Sync drone home to wherever the operator actually is (SITL-only —
    // real hardware already gets true home from GPS at boot). This is
    // one action from the operator's point of view: syncing location
    // IS setting home, not two separate steps.
    const syncedAt = new Date().toISOString();
    setHomeLastSyncedAt(syncedAt);
    setHome(userLat, userLng).then((res) => {
      if (res.relaunch_needed) {
        addNewLogEntry(FlightState.IDLE,
          "WARNING: SITL is already running with a different home — relaunch the sim for the synced location to take effect.");
      }
    }).catch(() => {
      addNewLogEntry(FlightState.IDLE, "ROS2: Backend unreachable — home sync not persisted.");
    });
  };

  const syncLaptopLocation = () => {
    if (!('geolocation' in navigator)) {
      setGpsSyncStatus('error');
      setGpsSyncError("Browser does not support geolocation.");
      return;
    }
    setGpsSyncStatus('locating');
    setGpsSyncError(null);

    const handleError = (error: GeolocationPositionError) => {
      setGpsSyncStatus('error');
      const msgs: Record<number, string> = {
        1: "Access Refused. Try opening in a separate tab.",
        2: "Position unavailable.",
        3: "GPS timeout.",
      };
      const msg = msgs[error.code] || error.message;
      setGpsSyncError(msg);
      addNewLogEntry(FlightState.IDLE, `GPS LOCK FAILED: ${msg}`);
    };

    // Try a real GPS-chip-level fix first (best accuracy) with a bounded
    // wait; only fall back to WiFi/IP-based positioning (faster, but can be
    // off by hundreds of meters — confirmed live 2026-07-11, drifted ~850m
    // from the high-accuracy reading) if the precise fix doesn't land in
    // time. This gets the best of both: precision when a GPS fix is
    // actually available, reliability (no more indefinite timeout) when
    // it isn't.
    navigator.geolocation.getCurrentPosition(
      (position) => applySyncedLocation(position.coords.latitude, position.coords.longitude),
      () => {
        addNewLogEntry(FlightState.IDLE, "GPS: High-accuracy fix unavailable — retrying with network-based location...");
        navigator.geolocation.getCurrentPosition(
          (position) => applySyncedLocation(position.coords.latitude, position.coords.longitude),
          handleError,
          { enableHighAccuracy: false, timeout: 20000, maximumAge: 60000 }
        );
      },
      { enableHighAccuracy: true, timeout: 10000, maximumAge: 0 }
    );
  };

  useEffect(() => {
    // Guard against React 18 StrictMode's dev-only double-invoke of mount
    // effects, which would otherwise fire the geolocation request twice.
    if (didInitialGpsSync.current) return;
    didInitialGpsSync.current = true;

    // Hydrate from the backend's PERSISTED home first -- startLoc/dronePos
    // are just in-memory React state that otherwise reset to a hardcoded
    // placeholder (Kochi-area coordinates) on every page load, even though
    // a real synced home already exists in the database from a previous
    // session. Confirmed live 2026-07-11: operator was in Trivandrum with
    // a correctly-persisted Trivandrum home, but the page still showed the
    // drone at the Kochi default because nothing ever read it back. Live
    // geolocation below can still refine this further if it succeeds --
    // this is a fallback for "already synced, not asking again", not a
    // replacement for it.
    getHome().then((home) => {
      if (home.lat != null && home.lon != null) {
        setStartLoc({ lat: home.lat, lng: home.lon });
        setDronePos({ lat: home.lat, lng: home.lon, heading: 0 });
        writeCachedPosition(home.lat, home.lon);
        if (home.synced_at) setHomeLastSyncedAt(home.synced_at);
      }
    }).catch(() => {
      // Backend unreachable at mount -- fall through to the hardcoded
      // placeholder via the geolocation call below, same as before.
    }).finally(() => {
      syncLaptopLocation();
    });
  }, []);
  // Only the quick single-destination flow, not the multi-stop waypoint
  // builder -- once the operator has added real waypoints via the map,
  // destLoc's own marker/path is stale and confusing (they can end up in
  // completely different places, since destLoc defaults to a hardcoded
  // placeholder and is never synced to the waypoints array). startMission()
  // already ignores destLoc once waypoints.length > 0; the map/path should
  // reflect that same priority instead of showing both at once.
  useEffect(() => {
    if (waypoints.length > 0) return;
    if (startLoc && destLoc) calculateFlightPath();
  }, [startLoc, destLoc, obstacles, waypoints.length]);

  // ── Continuous laptop GPS tracking ───────────────────
  // Keeps dronePos following the real device location while on the ground.
  // Stops deferring here once ROS2 telemetry takes over, or once a mission
  // is actually EN_ROUTE — at that point dronePos is owned by real MAVROS
  // telemetry or the simulated flight-path ticker, not raw device GPS.
  useEffect(() => {
    if (!('geolocation' in navigator)) return;
    if (ros2Connected) return;
    if (flightState !== FlightState.IDLE && flightState !== FlightState.PLANNING) return;

    const watchId = navigator.geolocation.watchPosition(
      (position) => {
        setDronePos(prev => ({
          lat:     position.coords.latitude,
          lng:     position.coords.longitude,
          heading: prev.heading,
        }));
      },
      () => { /* ignore transient watch errors — last known dronePos stands */ },
      // Same GPS-chip-vs-WiFi-fix reasoning as syncLaptopLocation() above --
      // enableHighAccuracy:true stalls indefinitely on hardware with no GPS.
      { enableHighAccuracy: false, maximumAge: 2000 }
    );

    return () => navigator.geolocation.clearWatch(watchId);
  }, [ros2Connected, flightState]);

  // ── Simulation tick ─────────────────────────────────
  // Only drives dronePos when ROS2 isn't connected — otherwise this fake
  // movement fights the real MAVROS-fed position from the status poll below.
  useEffect(() => {
    if (!ros2Connected && (flightState === FlightState.EN_ROUTE || flightState === FlightState.EMERGENCY_LANDING)) {
      mainTickerInterval.current = setInterval(handleSimulationTick, 150);
    } else {
      if (mainTickerInterval.current) {
        clearInterval(mainTickerInterval.current);
        mainTickerInterval.current = null;
      }
    }
    return () => { if (mainTickerInterval.current) clearInterval(mainTickerInterval.current); };
  }, [flightState, plannedPath, currentPathIndex, ros2Connected]);

  // ── ROS2 status polling (NEW) ───────────────────────
  useEffect(() => {
    const interval = setInterval(async () => {
      try {
        const status = await getMissionStatus();
        setRos2Connected(true);
        setDroneStatus(status.drone_status || 'DISCONNECTED');

        // Update drone position + heading from real MAVROS telemetry.
        // Only when the WebSocket push (Feature 8/9) isn't live — once it is,
        // that's the position source and this REST poll is purely a fallback.
        if (!wsConnected && typeof status.lat === 'number' && typeof status.lon === 'number') {
          setDronePos(prev => ({
            lat:     status.lat,
            lng:     status.lon,
            heading: typeof status.heading === 'number' ? status.heading : prev.heading,
          }));
        }

        // Update altitude from ROS2 — only when the WebSocket push isn't live;
        // once it is, wsPosition.altitude is the source and this is a fallback.
        if (!wsConnected && typeof status.altitude === 'number') {
          setSensors(prev => ({ ...prev, barometerAltitudeM: status.altitude }));
        }

        // Sync mission state from ROS2 — same WS-primary/REST-fallback split
        // as position/altitude above (wsMissionState effect handles the live case).
        if (!wsConnected) {
          if (status.mission_state === "MISSION_COMPLETE" && flightState === FlightState.EN_ROUTE) {
            setFlightState(FlightState.LANDED_SAFE);
            addNewLogEntry(FlightState.LANDED_SAFE, "ROS2: Mission complete — drone landed.");
          }
          if (status.mission_state === "MISSION_ABORT" && flightState !== FlightState.EMERGENCY_LANDING) {
            setFlightState(FlightState.EMERGENCY_LANDING);
            addNewLogEntry(FlightState.EMERGENCY_LANDING, "ROS2: Mission aborted by system.");
          }

          // Safety: abort if altitude overshoots the 2.5m target by ~0.5m.
          // The backend enforces this abort authoritatively regardless of the
          // dashboard; this is a REST-fallback client-side trip for when the
          // WebSocket (which already reflects the backend's own abort) is down.
          if (typeof status.altitude === 'number' &&
              status.altitude >= ABORT_ALTITUDE_M &&
              flightState === FlightState.EN_ROUTE) {
            addNewLogEntry(FlightState.EMERGENCY_LANDING,
              `SAFETY: Altitude ${status.altitude.toFixed(1)}m exceeded ${ABORT_ALTITUDE_M}m limit — aborting.`);
            setFlightState(FlightState.EMERGENCY_LANDING);
            try {
              await abortMission();
            } catch {
              // Backend unreachable — local state already reflects the abort
            }
          }
        }

      } catch {
        // Backend not reachable — simulation mode
        setRos2Connected(false);
        setDroneStatus('DISCONNECTED');
      }
    }, 500);
    return () => clearInterval(interval);
  }, [flightState, wsConnected]);

  // ── WebSocket position push (Features 8/9) — native rate, not throttled ──
  useEffect(() => {
    if (!wsPosition || !wsConnected) return;
    // (0,0) is the no-GPS-fix-yet sentinel (same convention the backend's
    // REST /mission/status now nulls out) — before PX4 has a lock, MAVROS
    // can still emit NavSatFix zeros, which put the marker at Null Island.
    if (wsPosition.lat === 0 && wsPosition.lon === 0) return;
    setDronePos({ lat: wsPosition.lat, lng: wsPosition.lon, heading: wsPosition.heading });
    setSensors(prev => ({ ...prev, barometerAltitudeM: wsPosition.altitude }));
  }, [wsPosition, wsConnected, flightState]);

  // ── Traveled trail (the red line) ──────────────────────────────────
  // Fed from dronePos itself, not from the WebSocket message, so the line
  // still draws when the socket is down and REST polling is the position
  // source. Points are deduped to >=0.5m spacing.
  const lastTrailPointRef = useRef<LatLng | null>(null);
  useEffect(() => {
    if (flightState !== FlightState.EN_ROUTE) return;
    const last = lastTrailPointRef.current;
    if (last) {
      const dLat = (dronePos.lat - last.lat) * 111320;
      const dLng = (dronePos.lng - last.lng) * 111320 * Math.cos((dronePos.lat * Math.PI) / 180);
      if (Math.hypot(dLat, dLng) < 0.5) return;
    }
    lastTrailPointRef.current = { lat: dronePos.lat, lng: dronePos.lng };
    setTraveledPath(prev => {
      const next = [...prev, { lat: dronePos.lat, lng: dronePos.lng }];
      return next.length > MAX_TRAVELED_POINTS ? next.slice(next.length - MAX_TRAVELED_POINTS) : next;
    });
  }, [dronePos, flightState]);

  // ── Auto-ready after a mission ends ────────────────────────────────
  // Once the drone is on the ground after MISSION_COMPLETE (or an abort),
  // reset the ROS2 state machine and re-open the Launch button — the
  // operator shouldn't have to hunt for a reset to fly again. The trail
  // stays on screen until the next launch clears it.
  useEffect(() => {
    if (flightState !== FlightState.LANDED_SAFE && flightState !== FlightState.EMERGENCY_LANDING) return;
    if (droneStatus !== 'LANDED' && droneStatus !== 'CONNECTED') return;
    const timer = setTimeout(async () => {
      try {
        await resetMission();
      } catch { /* backend unreachable — local re-arm only */ }
      lastTrailPointRef.current = null;
      setFlightState(FlightState.IDLE);
      addNewLogEntry(FlightState.IDLE, 'Mission closed out — ready to launch the next one.');
    }, 4000);
    return () => clearTimeout(timer);
  }, [flightState, droneStatus]);

  // ── Trail restore after mid-flight reconnect ────────────────────────
  // If the page loads (or the backend comes back) while a mission is already
  // airborne, the in-memory trail is empty/gapped — rebuild the flown segment
  // from the persisted travel log so the map shows the whole route, not just
  // what happened after reconnect. Purely cosmetic: never blocks anything.
  const seededMissionRef = useRef<number | null>(null);
  const hydratedRef = useRef(false); // active-mission hydration runs once per page load
  useEffect(() => {
    if (!wsConnected) return;
    (async () => {
      try {
        const status = await getMissionStatus();

        // ── Active-mission hydration (page reopened mid-flight) ──────────
        // A fresh tab boots with an empty planner and IDLE status even though
        // the drone is still flying. Rebuild the mission stops and running
        // status from the backend's stored upload so the UI resumes the
        // mission instead of looking brand new.
        const st = (status as any).mission_state as string | undefined;
        const missionAirborne = st !== undefined &&
          !['IDLE', 'MISSION_COMPLETE', 'MISSION_ABORT', 'DISCONNECTED'].includes(st);
        if (missionAirborne && !hydratedRef.current) {
          hydratedRef.current = true;
          const wps: any[] = (status as any).waypoints ?? [];
          const restored: MissionWaypoint[] = wps
            .filter(w => w.label && w.label !== 'A')
            .map(w => ({
              label: w.label, lat: w.lat, lng: w.lon,
              alt: w.alt ?? TARGET_ALTITUDE_M,
              markerId: w.marker_id ?? undefined,
            }));
          if (restored.length > 0) {
            setWaypoints(restored);
            // Future stops must not reuse a restored label.
            nextStopLetter.current = Math.max(
              ...restored.map(w => w.label.charCodeAt(0) - 'B'.charCodeAt(0) + 1));
            const first = wps[0];
            if (first?.land_mode === 'aruco' || first?.land_mode === 'gps') setLandMode(first.land_mode);
            if (typeof first?.wait_s === 'number') setWaitS(first.wait_s);
          }
          if (typeof (status as any).speed_ms === 'number') setSpeedMs((status as any).speed_ms);
          setFlightState(FlightState.EN_ROUTE);
          addNewLogEntry(FlightState.EN_ROUTE,
            `RESUME: Reconnected to active mission (${st}) — ${restored.length} stop(s) restored, planning locked.`);
        }

        const mid: number | null = (status as any).mission_id ?? null;
        if (mid == null || seededMissionRef.current === mid) return;
        seededMissionRef.current = mid;
        const log = await getTravelLog(mid);
        const flown: LatLng[] = (log?.path ?? []).map((p: any) => ({ lat: p.lat, lng: p.lon }));
        if (flown.length === 0) return;
        // Prepend: DB points predate anything the live WS has appended since.
        setTraveledPath(prev => {
          const merged = [...flown, ...prev];
          return merged.length > MAX_TRAVELED_POINTS ? merged.slice(merged.length - MAX_TRAVELED_POINTS) : merged;
        });
      } catch { /* trail restore is best-effort — ignore */ }
    })();
  }, [wsConnected]);

  // ── WebSocket IMU push (Feature 6) — drives the live attitude readout too ──
  useEffect(() => {
    if (!imu || !wsConnected) return;
    setSensors(prev => ({ ...prev, pitch: imu.pitch, roll: imu.roll }));
  }, [imu, wsConnected]);

  // ── WebSocket mission-state push (Feature 7) — WS-primary, REST poll above
  // only takes over when the socket is down ──
  useEffect(() => {
    if (!wsMissionState || !wsConnected) return;
    const state = wsMissionState.mission_state;
    if (state === "MISSION_COMPLETE" && flightState === FlightState.EN_ROUTE) {
      setFlightState(FlightState.LANDED_SAFE);
      addNewLogEntry(FlightState.LANDED_SAFE, "ROS2: Mission complete — drone landed.");
    }
    if (state === "MISSION_ABORT" && flightState !== FlightState.EMERGENCY_LANDING) {
      setFlightState(FlightState.EMERGENCY_LANDING);
      addNewLogEntry(FlightState.EMERGENCY_LANDING, "ROS2: Mission aborted by system.");
    }
  }, [wsMissionState, wsConnected, flightState]);

  // ── Path calculation ────────────────────────────────
  const calculateFlightPath = () => {
    if (!startLoc || !destLoc) return;
    setFlightState(FlightState.PLANNING);
    setDirectPath([startLoc, destLoc]);

    // Simple obstacle avoidance — add intermediate waypoints
    const waypoints: LatLng[] = [startLoc];
    for (const obs of obstacles) {
      const midLat = (startLoc.lat + destLoc.lat) / 2;
      const midLng = (startLoc.lng + destLoc.lng) / 2;
      const dLat = obs.lat - midLat;
      const dLng = obs.lng - midLng;
      const dist = Math.sqrt(dLat*dLat + dLng*dLng) * degToMeter;
      if (dist < obs.radiusMeters * 3) {
        const perpLat = midLat + dLng * 0.002;
        const perpLng = midLng - dLat * 0.002;
        waypoints.push({ lat: perpLat, lng: perpLng });
      }
    }
    waypoints.push(destLoc);
    setPlannedPath(waypoints);
    setCurrentPathIndex(0);
    setFlightState(FlightState.IDLE);
  };

  // ── Mission start ───────────────────────────────────
  const startMission = async () => {
    if (!destLoc && waypoints.length === 0) return;
    if (flightState !== FlightState.IDLE && flightState !== FlightState.PLANNING) return;

    setFlightState(FlightState.EN_ROUTE);
    setMissionTimeSec(0);
    setCurrentPathIndex(0);
    setTraveledPath([]);
    addNewLogEntry(FlightState.EN_ROUTE, "MISSION START: Trajectory execution initiated.");

    // ── Send to ROS2 via FastAPI ──────────────────────
    // Multi-stop mission if the operator built one via the map; otherwise
    // fall back to the single quick-destination flow (destLoc) unchanged.
    const uploadList = waypoints.length > 0
      ? waypoints.map(w => ({ lat: w.lat, lon: w.lng, alt: w.alt, label: w.label, marker_id: w.markerId }))
      : [{ lat: destLoc!.lat, lon: destLoc!.lng, alt: TARGET_ALTITUDE_M, label: "B" }];

    try {
      await uploadWaypoints(uploadList, speedMs, landMode, waitS);
      await ros2Start();
      addNewLogEntry(FlightState.EN_ROUTE, `ROS2: ${uploadList.length} waypoint(s) sent, max speed ${speedMs} m/s.`);
    } catch (e) {
      if (e instanceof ApiError) {
        setFlightState(FlightState.IDLE);
        addNewLogEntry(FlightState.IDLE, `ROS2: Mission rejected (${e.message}) — not ready.`);
      } else {
        addNewLogEntry(FlightState.EN_ROUTE,
          "ROS2: Backend not reachable — running in simulation mode only.");
      }
    }
  };

  // ── Arm drone ────────────────────────────────────────
  const armDroneHandler = async () => {
    try {
      await armDrone();
      addNewLogEntry(flightState, "ROS2: ARM command sent to drone.");
    } catch (e) {
      if (e instanceof ApiError) {
        addNewLogEntry(flightState, `ROS2: Arm rejected — ${e.message}`);
      } else {
        addNewLogEntry(flightState, "ROS2: Backend not reachable — cannot arm.");
      }
    }
  };

  const disarmDroneHandler = async () => {
    try {
      await disarmDrone();
      addNewLogEntry(flightState, "ROS2: DISARM command sent to drone.");
    } catch (e) {
      if (e instanceof ApiError) {
        addNewLogEntry(flightState, `ROS2: Disarm rejected — ${e.message}`);
      } else {
        addNewLogEntry(flightState, "ROS2: Backend not reachable — cannot disarm.");
      }
    }
  };

  const takeoffDroneHandler = async () => {
    try {
      await takeoffDrone();
      addNewLogEntry(flightState, "ROS2: TAKEOFF command sent (manual bench control).");
    } catch (e) {
      addNewLogEntry(flightState, e instanceof ApiError
        ? `ROS2: Takeoff rejected — ${e.message}`
        : "ROS2: Backend not reachable — cannot take off.");
    }
  };

  const landDroneHandler = async () => {
    try {
      await landDrone();
      addNewLogEntry(flightState, "ROS2: LAND command sent (manual bench control).");
    } catch (e) {
      addNewLogEntry(flightState, e instanceof ApiError
        ? `ROS2: Land rejected — ${e.message}`
        : "ROS2: Backend not reachable — cannot land.");
    }
  };

  // Manual directional nudges are fired rapidly while a Test Bench button
  // is held -- deliberately silent on success (no log spam per nudge) and
  // only logs a REJECTION, so a genuine gate failure (mission active, not
  // armed) is still visible without flooding the telemetry log.
  const manualNudgeHandler = async (cmd: ManualNudgeCmd) => {
    try {
      await manualNudge(cmd);
    } catch (e) {
      if (e instanceof ApiError) {
        addNewLogEntry(flightState, `ROS2: Manual ${cmd} rejected — ${e.message}`);
      }
    }
  };

  // ── Mission stops (map-click-driven, Feature 1/2) ────
  // A mission is "active" from launch until the ROS state machine returns to
  // rest. Stops added mid-flight were silently ignored by the drone (the
  // waypoint upload already happened at launch), so planning is locked.
  // wsMissionState is a message object — the state string lives in its
  // .mission_state field. When the backend state is known it wins; the local
  // flightState is only the fallback while the socket is down.
  const rosMissionState = wsMissionState?.mission_state;
  const missionActive = rosMissionState !== undefined
    ? !['IDLE', 'MISSION_COMPLETE', 'MISSION_ABORT'].includes(rosMissionState)
    : flightState === FlightState.EN_ROUTE;

  const addWaypointFromMap = (loc: LatLng) => {
    if (missionActive) {
      addNewLogEntry(flightState, 'Waypoints are locked while a mission is active — new stop ignored. Wait for the mission to finish (or reset) to plan again.');
      return;
    }
    const label = String.fromCharCode('B'.charCodeAt(0) + nextStopLetter.current);
    nextStopLetter.current += 1;
    setWaypoints(prev => {
      // Clear any stale quick-flow destination path on the *first* real
      // waypoint -- otherwise its dashed line lingers, pointing at
      // destLoc's (often unrelated) location alongside the new waypoint
      // markers.
      if (prev.length === 0) {
        setDirectPath([]);
        setPlannedPath([]);
      }
      return [...prev, { label, lat: loc.lat, lng: loc.lng, alt: TARGET_ALTITUDE_M, markerStatus: 'idle' }];
    });
  };

  const updateWaypointAlt = (label: string, alt: number) => {
    setWaypoints(prev => prev.map(w => (w.label === label ? { ...w, alt } : w)));
  };

  const removeWaypoint = (label: string) => {
    setWaypoints(prev => prev.filter(w => w.label !== label));
  };

  const clearAllWaypoints = () => {
    if (waypoints.length === 0) return;
    setWaypoints([]);
    addNewLogEntry(flightState, 'Mission stops reset — click the map to plan a new route.');
  };

  const generateWaypointMarker = async (label: string) => {
    const wp = waypoints.find(w => w.label === label);
    if (!wp) return;
    setWaypoints(prev => prev.map(w => (w.label === label ? { ...w, markerStatus: 'generating' } : w)));
    try {
      const res = await generateMarker(label, wp.lat, wp.lng, wp.markerId);
      setWaypoints(prev => prev.map(w => (w.label === label
        ? { ...w, markerId: res.marker_id, markerStatus: 'spawned' } : w)));
      addNewLogEntry(flightState, `ArUco marker #${res.marker_id} spawned for stop ${label}.`);
    } catch (e) {
      setWaypoints(prev => prev.map(w => (w.label === label ? { ...w, markerStatus: 'error' } : w)));
      addNewLogEntry(flightState, e instanceof ApiError
        ? `Marker generation failed for ${label}: ${e.message}`
        : `Marker generation failed for ${label}: backend unreachable.`);
    }
  };

  // ── Emergency override ──────────────────────────────
  const triggerEmergencyOverride = async () => {
    if (flightState !== FlightState.EN_ROUTE) return;
    setFlightState(FlightState.EMERGENCY_LANDING);
    addNewLogEntry(FlightState.EMERGENCY_LANDING, "EMERGENCY OVERRIDE: Forced descent initiated.");

    try {
      await abortMission();
      addNewLogEntry(FlightState.EMERGENCY_LANDING, "ROS2: ABORT sent to drone.");
    } catch {
      addNewLogEntry(FlightState.EMERGENCY_LANDING, "ROS2: Backend not reachable — local emergency only.");
    }
  };

  // Return Home — distinct from Emergency Override: flies back to the
  // recorded home position first, then lands, rather than landing in
  // place. Reuses mission_manager's own RTH path (same one the failsafe
  // monitor uses for comms/node loss), just operator-triggered.
  const triggerReturnHome = async () => {
    if (flightState !== FlightState.EN_ROUTE) return;
    addNewLogEntry(flightState, "RETURN HOME: Operator requested — flying back to home position.");

    try {
      await returnHome();
      addNewLogEntry(flightState, "ROS2: RTH sent to drone.");
    } catch {
      addNewLogEntry(flightState, "ROS2: Backend not reachable — return-home not sent.");
    }
  };

  // ── Reset ───────────────────────────────────────────
  const resetSystem = async () => {
    if (mainTickerInterval.current) clearInterval(mainTickerInterval.current);
    setFlightState(FlightState.IDLE);
    setDronePos({ lat: startLoc.lat, lng: startLoc.lng, heading: 0 });
    setMissionTimeSec(0);
    setCurrentPathIndex(0);
    setTraveledPath([]);
    setSensors(prev => ({ ...prev, pitch: 0, roll: 0, yaw: 0, barometerAltitudeM: 0, obstacleAvoidanceActive: false }));
    addNewLogEntry(FlightState.IDLE, "SYSTEM RESET: All telemetry cleared. Drone returned to base.");

    // This used to be purely a local UI reset -- mission_manager's own
    // state machine never transitioned MISSION_COMPLETE/MISSION_ABORT back
    // to IDLE on its own, so every mission after the first was silently
    // ignored ROS2-side even though the website looked ready again. Also
    // send the real reset so a second mission can actually launch.
    try {
      await resetMission();
      addNewLogEntry(FlightState.IDLE, "ROS2: Mission state machine reset to IDLE.");
    } catch {
      addNewLogEntry(FlightState.IDLE, "ROS2: Backend not reachable — local reset only.");
    }
  };

  // ── Toggle heater / cooler ──────────────────────────
  const toggleHeater = () => setClimate(p => ({ ...p, heaterActive: !p.heaterActive }));
  const toggleCooler = () => setClimate(p => ({ ...p, coolerActive: !p.coolerActive }));

  // ── Simulation tick logic ───────────────────────────
  const handleSimulationTick = () => {
    if (flightState === FlightState.EN_ROUTE) {
      if (plannedPath.length < 2 || currentPathIndex >= plannedPath.length - 1) return;

      const target = plannedPath[currentPathIndex + 1];
      setDronePos(prev => {
        const dLat = target.lat - prev.lat;
        const dLng = target.lng - prev.lng;
        const dist  = Math.sqrt(dLat*dLat + dLng*dLng);
        const speed = 0.0001;

        if (dist < speed * 1.5) {
          const nextIdx = currentPathIndex + 1;
          setCurrentPathIndex(nextIdx);
          if (nextIdx >= plannedPath.length - 1) {
            setFlightState(FlightState.LANDED_SAFE);
            addNewLogEntry(FlightState.LANDED_SAFE, "LANDING COMPLETE: Destination reached.");
          }
          return { lat: target.lat, lng: target.lng, heading: prev.heading };
        }

        const heading = Math.atan2(dLng, dLat) * (180 / Math.PI);
        return {
          lat: prev.lat + (dLat / dist) * speed,
          lng: prev.lng + (dLng / dist) * speed,
          heading,
        };
      });

      setSensors(prev => ({
        ...prev,
        pitch:              Math.sin(Date.now() * 0.003) * 3,
        roll:               Math.sin(Date.now() * 0.002) * 2,
        barometerAltitudeM: 45 + Math.sin(Date.now() * 0.001) * 2,
        lidarDistanceM:     Math.max(5, 50 - Math.sin(Date.now() * 0.001) * 10),
      }));

      setBattery(prev => ({ ...prev, percentage: Math.max(0, prev.percentage - 0.003) }));
      setMissionTimeSec(prev => prev + 1);

      logTickerDivider.current += 1;
      if (logTickerDivider.current >= 20) {
        logTickerDivider.current = 0;
        addNewLogEntry(FlightState.EN_ROUTE, `EN_ROUTE: Telemetry ingest active.`);
      }
    }

    if (flightState === FlightState.EMERGENCY_LANDING) {
      setSensors(prev => {
        const nextAlt = Math.max(0, prev.barometerAltitudeM - 1.8);
        if (nextAlt === 0) setFlightState(FlightState.LANDED_SAFE);
        return { ...prev, barometerAltitudeM: nextAlt, pitch: prev.pitch * 0.7, roll: prev.roll * 0.7 };
      });
    }
  };

  const formatDuration = (s: number) =>
    `${Math.floor(s/60).toString().padStart(2,'0')}:${(s%60).toString().padStart(2,'0')}`;

  // ── Log entry ───────────────────────────────────────
  const addNewLogEntry = (state: FlightState, detail: string) => {
    const formattedDur = missionTimeSec > 0 ? formatDuration(missionTimeSec) : undefined;
    const frame: TelemetryLog = {
      id:                        `UAV-COOPS-${Date.now().toString().slice(-5)}`,
      timestamp:                 new Date().toISOString(),
      flightState:               state,
      latitude:                  dronePos.lat,
      longitude:                 dronePos.lng,
      altitudeMeters:            sensors.barometerAltitudeM,
      headingDegrees:            dronePos.heading,
      batteryPercentage:         battery.percentage,
      batteryTempCelsius:        climate.batteryTempC,
      signalStrengthDbm:         signal.strengthDbm,
      encryptionActive:          signal.isEncrypted,
      pitchDegrees:              sensors.pitch,
      rollDegrees:               sensors.roll,
      yawDegrees:                dronePos.heading,
      lidarDistanceMeters:       sensors.lidarDistanceM,
      obstacleAvoidanceActive:   sensors.obstacleAvoidanceActive,
      emergencyLandingActive:    state === FlightState.EMERGENCY_LANDING,
      ambientTemperatureCelsius: climate.ambientTempC,
      climateHeaterActive:       climate.heaterActive,
      climateCoolerActive:       climate.coolerActive,
      windSpeedKnots:            simulationWindSpeed,
      flightDuration:            formattedDur,
      detail:                    formattedDur ? `${detail} (Elapsed: ${formattedDur})` : detail,
    };
    setLogs(prev => [...prev, frame]);
  };

  // ── Render ───────────────────────────────────────────
  return (
    <div className="min-h-screen bg-[#0a0a0c] text-white font-sans selection:bg-[#5996FF] selection:text-black pb-14 relative overflow-hidden">

      {/* Background grid */}
      <div className="absolute inset-0 z-0 opacity-15 pointer-events-none">
        <svg width="100%" height="100%" xmlns="http://www.w3.org/2000/svg">
          <defs>
            <pattern id="grid" width="40" height="40" patternUnits="userSpaceOnUse">
              <path d="M 40 0 L 0 0 0 40" fill="none" stroke="#334155" strokeWidth="1" />
            </pattern>
          </defs>
          <rect width="100%" height="100%" fill="url(#grid)" />
        </svg>
      </div>

      {/* Header */}
      <header className="border-b border-white/10 bg-[#0a0a0c]/90 sticky top-0 z-40 backdrop-blur-md">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 h-16 flex items-center justify-between">
          <div className="flex items-center space-x-3">
            <div className="w-8 h-8 bg-[#5996FF] rounded flex items-center justify-center font-bold text-black shadow-[0_0_15px_rgba(89,150,255,0.4)]">
              UAV
            </div>
            <div>
              <h1 className="text-sm font-bold tracking-tight text-white uppercase flex items-center">
                SkyNav Avionics Systems
                <span className="text-[#5996FF] text-[9px] font-mono ml-2 bg-[#141417] px-2 py-0.5 rounded border border-white/10">v4.2.0-STABLE</span>
              </h1>
              <p className="text-[10px] text-[#9a9aa2] font-mono">Autonomous Drone Mission Control · LiDAR · ROS2 FastAPI Bridge</p>
            </div>
          </div>
          <div className="flex items-center space-x-2">
            {/* Page switcher: Mission Control vs Flight Test Bench */}
            <div className="flex items-center bg-[#141417] border border-white/10 rounded-full p-0.5 mr-1">
              <button
                onClick={() => setActivePage('mission')}
                className={`text-[10px] font-mono uppercase tracking-wide px-3 py-1 rounded-full transition-all ${
                  activePage === 'mission' ? 'bg-[#5996FF] text-black font-bold' : 'text-[#9a9aa2] hover:text-white'
                }`}
              >
                Mission
              </button>
              <button
                onClick={() => setActivePage('testbench')}
                className={`text-[10px] font-mono uppercase tracking-wide px-3 py-1 rounded-full transition-all ${
                  activePage === 'testbench' ? 'bg-[#1ebcbd] text-black font-bold' : 'text-[#9a9aa2] hover:text-white'
                }`}
              >
                Test Bench
              </button>
            </div>
            {/* ROS2 connection indicator in header */}
            <span className={`text-[10px] font-mono px-2.5 py-1 rounded-full border flex items-center ${
              ros2Connected
                ? 'bg-emerald-500/10 border-emerald-500/20 text-emerald-400'
                : 'bg-[#141417] border-white/10 text-[#9a9aa2]'
            }`}>
              <span className={`w-1.5 h-1.5 rounded-full mr-2 ${ros2Connected ? 'bg-emerald-500 animate-pulse' : 'bg-slate-500'}`} />
              {ros2Connected ? 'ROS2 CONNECTED' : 'SIM MODE'}
            </span>
            <span className="text-[10px] font-mono shrink-0 px-2.5 py-1 bg-[#141417] border border-white/10 text-[#5996FF] rounded-full flex items-center">
              <span className="w-1.5 h-1.5 rounded-full bg-emerald-500 mr-2 animate-pulse" />
              PORTAL SECURE
            </span>
            {/* Configured backend address — always visible so it's never
                ambiguous which machine the dashboard is talking to (Electron
                packaging task §2.2). Click to change it. */}
            <button
              onClick={() => setShowBackendSettings(true)}
              title={`Backend: ${API_BASE} — click to change`}
              className="text-[10px] font-mono shrink-0 px-2.5 py-1 bg-[#141417] border border-white/10 text-[#9a9aa2] hover:text-white hover:border-[#5996FF]/40 rounded-full flex items-center transition-all cursor-pointer"
            >
              <span className={`w-1.5 h-1.5 rounded-full mr-2 ${wsConnected ? 'bg-emerald-500' : 'bg-red-500'}`} />
              {backendHost}
            </button>
          </div>
        </div>
      </header>

      {/* Main layout */}
      <main className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 mt-6 space-y-7 relative z-10">

        <ConnectivityBanner nodeStatus={nodeStatus} wsConnected={wsConnected} />

        {activePage === 'testbench' ? (
          <FlightTestBench
            imu={imu}
            position={wsPosition}
            nodeStatus={nodeStatus}
            wsConnected={wsConnected}
            droneStatus={droneStatus}
            onArm={armDroneHandler}
            onDisarm={disarmDroneHandler}
            onTakeoff={takeoffDroneHandler}
            onLand={landDroneHandler}
            onManualNudge={manualNudgeHandler}
          />
        ) : (
        <>
        <section className="grid grid-cols-1 lg:grid-cols-12 gap-6">

          <div className="lg:col-span-8 flex flex-col gap-6">
            <MapErrorBoundary>
              <MapPane
                apiKey={API_KEY}
                hasValidKey={hasValidKey}
                startLoc={startLoc}
                // Suppress the quick-flow destination marker once real
                // waypoints exist -- otherwise it renders at its own
                // (unrelated, often stale) location alongside the
                // waypoint-array markers ("B", "C"...), which is exactly
                // the confusing overlap this is fixing.
                destLoc={waypoints.length > 0 ? null : destLoc}
                dronePos={dronePos}
                obstacles={obstacles}
                flightState={flightState}
                onSetStartLoc={setStartLoc}
                onSetDestLoc={setDestLoc}
                onAddWaypoint={addWaypointFromMap}
                planningLocked={missionActive}
                waypoints={waypoints}
                traveledPath={traveledPath}
                geofence={geofence}
                fenceDraft={fenceDraft}
                onFenceVertex={handleFenceVertex}
                onFinishFence={handleFinishFence}
                onClearFence={handleClearFence}
                plannedPath={plannedPath}
                directPath={directPath}
                avoidanceActive={sensors.obstacleAvoidanceActive}
                emergencyLandingActive={flightState === FlightState.EMERGENCY_LANDING}
              />
            </MapErrorBoundary>
            <CameraFeed
              dronePos={dronePos}
              flightState={flightState}
              destLoc={destLoc}
              obstacles={obstacles}
              sensors={sensors}
              ros2Connected={ros2Connected}
            />
            <TelemetryLogStream logs={logs} onClearLogs={() => setLogs([])} />
          </div>

          <div className="lg:col-span-4 flex flex-col gap-6">
            <FlightControlPanel
              startLoc={startLoc}
              destLoc={destLoc}
              dronePos={dronePos}
              flightState={flightState}
              battery={battery}
              signal={signal}
              onSetStartLoc={setStartLoc}
              onSetDestLoc={setDestLoc}
              onPlanPath={calculateFlightPath}
              onLaunchMission={startMission}
              onEmergencyOverride={triggerEmergencyOverride}
              onReturnHome={triggerReturnHome}
              onResetDrone={resetSystem}
              activePathLength={plannedPath.length}
              gpsSyncStatus={gpsSyncStatus}
              gpsSyncError={gpsSyncError}
              homeLastSyncedAt={homeLastSyncedAt}
              onSyncLaptopLocation={syncLaptopLocation}
              missionTimeSec={missionTimeSec}
              ros2Connected={ros2Connected}
              droneStatus={droneStatus}
              onArmDrone={armDroneHandler}
              allClear={nodeStatus ? nodeStatus.all_clear : !wsConnected}
              waypointCount={waypoints.length}
              mode={mode}
            />
            <WaypointList
              waypoints={waypoints}
              onUpdateAlt={updateWaypointAlt}
              onRemove={removeWaypoint}
              onClearAll={clearAllWaypoints}
              missionActive={missionActive}
              onGenerateMarker={generateWaypointMarker}
              speedMs={speedMs}
              onSetSpeedMs={setSpeedMs}
              landMode={landMode}
              onSetLandMode={setLandMode}
              waitS={waitS}
              onSetWaitS={setWaitS}
              mode={mode}
              abortAltitudeM={ABORT_ALTITUDE_M}
              disabled={wsConnected && nodeStatus ? !nodeStatus.all_clear : false}
            />
            <DroneProfilePanel mode={mode} onModeChange={setModeState} />
            <TelemetryInsights logs={logs} />
          </div>

        </section>

        <section>
          <IMUGraph imu={imu} ros2Connected={ros2Connected} />
        </section>

        <section>
          <SensorReadout
            sensors={sensors}
            climate={climate}
            signal={signal}
            speedKmh={flightState === FlightState.EN_ROUTE ? (32.4 + Math.sin(Date.now() * 0.005) * 1.5) : 0}
            altitudeM={sensors.barometerAltitudeM}
            onToggleHeater={toggleHeater}
            onToggleCooler={toggleCooler}
            onSetEncryptionKey={(k) => setSignal(p => ({ ...p, encryptionKey: k, isEncrypted: k !== '' }))}
            onSimulateObstacleAlert={() => {
              setSensors(prev => ({ ...prev, lidarDistanceM: 8.5, obstacleAvoidanceActive: true, obstacleType: "Dynamic Substation Flare" }));
              addNewLogEntry(FlightState.EN_ROUTE, "LIDAR ALERT: Unmapped hazard detected!");
            }}
            simulationWindSpeed={simulationWindSpeed}
            onChangeWindSpeed={setSimulationWindSpeed}
          />
        </section>
        </>
        )}

      </main>

      <BackendSettings open={showBackendSettings} onClose={() => setShowBackendSettings(false)} />
    </div>
  );
}
