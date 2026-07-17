#!/usr/bin/env python3
"""
backend/main.py — FastAPI + rclpy bridge between the website and the ROS2
mission stack (see ~/drone_ws2/src/autonomous_drone_ros2 for the ROS2 side).
"""
import asyncio
import json
import math
import os
import threading
import time
from typing import List, Optional

import cv2
import numpy as np
import rclpy
import uvicorn
from cv_bridge import CvBridge
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from mavros_msgs.msg import HomePosition, State, Waypoint as MavWaypoint
from mavros_msgs.srv import WaypointClear, WaypointPush
from nav_msgs.msg import Odometry
from pydantic import BaseModel
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Image, Imu, NavSatFix
from std_msgs.msg import Int32, String, Float64

import db
import range_estimate
from drone_interfaces.aruco_marker import write_pad_model_everywhere
from drone_interfaces.constants import (ARUCO_ID_AUTO_START,
                                          BATTERY_HEARTBEAT_STALE_S,
                                          MAVROS_STATE_STALE_S,
                                          NODE_HEARTBEAT_STALE_S,
                                          RTH_ALTITUDE,
                                          TOPIC_MISSION_SAFETY_EVENT)
from drone_interfaces.geo import gps_distance_m, gps_to_local
from drone_interfaces.gz_spawn import find_world_name, spawn_model

from contextlib import asynccontextmanager


@asynccontextmanager
async def _lifespan(_app: FastAPI):
    if ros_node:
        ros_node.loop = asyncio.get_running_loop()
    yield


app = FastAPI(lifespan=_lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000",
                   "http://127.0.0.1:3000",
                   "http://localhost:5173",
                   "http://127.0.0.1:5173"],
    allow_methods=["*"],
    allow_headers=["*"],
)

ros_node = None

# ── Altitude safety limits ────────────────────────
TARGET_ALTITUDE_M = 2.5
# Must clear RTH_ALTITUDE (drone_interfaces/constants.py, currently 7.0m) with
# margin -- confirmed live 2026-07-10 that RTH_ALTITUDE > ABORT_ALTITUDE_M
# made every return-home (manual button and the automatic failsafe path
# alike) self-abort via this exact check mid-climb, before ever reaching
# home. 10.0m = 7.0m RTH climb + ~3m margin for transient setpoint overshoot
# (observed up to ~8.3m actual against a 7.0m target in that same test).
ABORT_ALTITUDE_M    = 20.0  # RTH_ALTITUDE(10m) + WSL EKF z-drift (3-5m observed 2026-07-13) brushed the old 15m ceiling mid-RTH
ALT_ABORT_DEBOUNCE_S = 0.3  # must read over the ceiling continuously for
                            # this long before the safety abort fires  # was 10.0 -- RTH cruises at 7m and WSL EKF
                           # altitude drift (+2m seen live 2026-07-13 after a
                           # 30s search spin) ate the 3m margin and false-
                           # aborted a return-home; 15m keeps the net well
                           # above RTH_ALTITUDE + drift while still catching
                           # a real runaway climb
DEFAULT_MAX_SPEED_MS = 3.0
HOME_MISMATCH_THRESHOLD_M = 1000.0

REPO_MODELS_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "autonomous_drone_ros2",
    "simulation", "gazebo", "models"))
LAST_SYNCED_HOME_FILE = os.path.expanduser("~/drone_ws2/.last_synced_home")

MONITORED_NODES = {
    "drone_base": "/drone_base/status",
    "waypoint_navigator": "/waypoint_nav/status",
    "aruco_landing": "/aruco_landing/status",
    "vision_node": "/vision_node/heartbeat",
    "camera_node": "/camera_node/heartbeat",
    "mission_manager": "/mission/status",
}

AIRBORNE_MISSION_STATES = {"PREFLIGHT", "TAKEOFF", "GOTO_WAYPOINT", "ARUCO_LAND",
                            "WAIT_ON_GROUND", "INTER_TAKEOFF", "RETURN_HOME", "HOME_LAND",
                            "PX4_FAILSAFE"}


# ── Data models ───────────────────────────────────
class Waypoint(BaseModel):
    lat:   float
    lon:   float
    alt:   float = TARGET_ALTITUDE_M
    label: str   = "B"
    marker_id: Optional[int] = None


class MissionUpload(BaseModel):
    waypoints: List[Waypoint]
    speed_ms: Optional[float] = None
    land_mode: Optional[str] = "aruco"   # "aruco" = vision landing | "gps" = plain AUTO.LAND, no marker
    wait_s: Optional[float] = None       # ground wait (s) before the next takeoff


class MarkerGenerateRequest(BaseModel):
    label: str
    marker_id: Optional[int] = None
    lat: float
    lon: float


class SetHomeRequest(BaseModel):
    lat: float
    lon: float


class ModeRequest(BaseModel):
    mode: str  # "sim" | "hardware"


class GeofencePoint(BaseModel):
    lat: float
    lon: float


class GeofenceRequest(BaseModel):
    vertices: List[GeofencePoint]
    action: str = "return"  # QGC-style breach action: warn | hold | return | land


class DroneProfileRequest(BaseModel):
    motor_kv: Optional[float] = None
    esc_amp: Optional[float] = None
    battery_mah: Optional[float] = None
    cells: Optional[int] = None
    num_motors: Optional[int] = 4
    auw_grams: Optional[float] = None
    efficiency_factor: Optional[float] = None
    cruise_speed_ms: Optional[float] = None


# ── Connectivity gate (Feature 9) — authoritative, not just UI ───────────
def _gate():
    if ros_node is None:
        return False, ["ros2_bridge_not_connected"]
    return ros_node.all_clear()[:2]


def _require_all_clear():
    ok, reasons = _gate()
    if not ok:
        raise HTTPException(status_code=503, detail={"reasons": reasons})


def _route_distance_m(home_lat, home_lon, waypoints: List[Waypoint]) -> float:
    if not waypoints:
        return 0.0
    total = gps_distance_m(home_lat, home_lon, waypoints[0].lat, waypoints[0].lon)
    for a, b in zip(waypoints, waypoints[1:]):
        total += gps_distance_m(a.lat, a.lon, b.lat, b.lon)
    total += gps_distance_m(waypoints[-1].lat, waypoints[-1].lon, home_lat, home_lon)
    return total


# ── Geofence (QGC-style) ──────────────────────────
# The fence lives in PX4 itself (uploaded like QGC does, as
# NAV_FENCE_POLYGON_VERTEX_INCLUSION mission items + GF_ACTION), so breach
# enforcement works even if MAVROS/backend/website all die mid-flight.
GF_ACTION_CODES = {"warn": 1, "hold": 2, "return": 3, "land": 5}  # PX4 GF_ACTION values
NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001  # MAV_CMD id, same item type QGC uploads


def point_in_polygon(lat: float, lon: float, vertices) -> bool:
    """Ray-casting on raw lat/lon degrees — accurate at mission scale
    (hundreds of metres), where treating the earth as flat is fine."""
    inside = False
    n = len(vertices)
    for i in range(n):
        la1, lo1 = vertices[i]["lat"], vertices[i]["lon"]
        la2, lo2 = vertices[(i + 1) % n]["lat"], vertices[(i + 1) % n]["lon"]
        if (la1 > lat) != (la2 > lat):
            if lon < (lo2 - lo1) * (lat - la1) / (la2 - la1) + lo1:
                inside = not inside
    return inside


# ── Existing + extended endpoints ─────────────────
@app.post("/mission/upload")
def upload(mission: MissionUpload):
    # Validate the INCOMING waypoints against the geofence BEFORE the gate:
    # _require_all_clear()'s geofence_valid looks at the PREVIOUSLY stored
    # waypoints, so (a) a fence-violating payload must be rejected here by
    # name, and (b) a bad earlier upload must never block replacing it with
    # a good one (found live 2026-07-12: the gate rejected a valid upload
    # because the *prior* mission's waypoint was outside the fence).
    fence = db.get_config("geofence")
    fence_verts = (fence or {}).get("vertices") or []
    for w in mission.waypoints:
        if fence_verts and not point_in_polygon(w.lat, w.lon, fence_verts):
            raise HTTPException(400, f"Waypoint {w.label} ({w.lat:.6f},{w.lon:.6f}) "
                                      "is outside the geofence — rejected.")
        if w.alt > ABORT_ALTITUDE_M:
            raise HTTPException(400, f"Waypoint {w.label} altitude {w.alt}m exceeds "
                                      f"ABORT_ALTITUDE_M={ABORT_ALTITUDE_M}m — rejected, not clamped.")

    # The incoming payload is now known-good — replace the stored waypoints
    # before gating so geofence_valid judges THIS mission, not the last one.
    if mission.land_mode not in (None, "aruco", "gps"):
        raise HTTPException(400, f"land_mode must be 'aruco' or 'gps', got {mission.land_mode!r}")
    if mission.wait_s is not None and not (0 <= mission.wait_s <= 120):
        raise HTTPException(400, f"wait_s must be 0-120 seconds, got {mission.wait_s}")

    if ros_node:
        # land_mode/wait_s ride on each waypoint dict so the published
        # /mission/waypoints payload stays a plain list (mission_manager and
        # waypoint_navigator both parse it as one).
        ros_node.uploaded_waypoints = [
            {"lat": w.lat, "lon": w.lon, "alt": w.alt, "label": w.label,
             "marker_id": w.marker_id,
             "land_mode": mission.land_mode or "aruco",
             "wait_s": mission.wait_s} for w in mission.waypoints]
        ros_node.last_speed_ms = mission.speed_ms
    _require_all_clear()

    if ros_node and ros_node.home_lat is not None:
        route_m = _route_distance_m(ros_node.home_lat, ros_node.home_lon, mission.waypoints)
        profile = db.get_profile()
        if profile:
            est = range_estimate.estimate(profile)
            if est["range_m"] is not None and route_m > est["range_m"]:
                raise HTTPException(
                    400, f"Planned route ({route_m:.0f}m) exceeds the estimated safe "
                         f"range ({est['range_m']:.0f}m) for the configured drone profile.")

    if ros_node:
        msg = String(); msg.data = json.dumps(ros_node.uploaded_waypoints)
        ros_node.waypoints_pub.publish(msg)
        if mission.speed_ms:
            ros_node.set_max_speed(mission.speed_ms)
    return {"status": "ok", "count": len(mission.waypoints)}


@app.post("/mission/start")
def start():
    _require_all_clear()
    if ros_node:
        ros_node.alt_abort_triggered = False
        ros_node._alt_over_since   = None
        ros_node.mission_state = "IDLE"
        # Re-publish the stored waypoints right before START: the one-shot
        # publish in /mission/upload can be lost to DDS discovery when the
        # upload lands seconds after node boot (seen live 2026-07-14 — both
        # subscribers silently flew the seeded DEFAULT_B instead).
        if getattr(ros_node, "uploaded_waypoints", None):
            wmsg = String(); wmsg.data = json.dumps(ros_node.uploaded_waypoints)
            ros_node.waypoints_pub.publish(wmsg)
        msg = String(); msg.data = "START"
        ros_node.cmd_pub.publish(msg)
    return {"status": "ok"}


@app.post("/mission/abort")
def abort():
    if ros_node:
        msg = String(); msg.data = "ABORT"
        ros_node.cmd_pub.publish(msg)
    return {"status": "ok"}


@app.post("/mission/reset")
def reset():
    """mission_manager never transitions MISSION_COMPLETE/MISSION_ABORT back
    to IDLE on its own -- confirmed live 2026-07-10: every /mission/start
    after a drone's first-ever mission was silently ignored (the node's own
    "state == IDLE" guard never matched again), which looks exactly like
    "not taking off" even though the whole stack is healthy. The frontend's
    "Reset System" button previously only cleared local UI state; this
    endpoint is what actually resets the ROS2-side state machine so a
    second mission can launch without restarting the node stack.
    """
    if ros_node:
        msg = String(); msg.data = "RESET"
        ros_node.cmd_pub.publish(msg)
        # Drop the cached plan too: /mission/start re-publishes
        # uploaded_waypoints right before START, and /mission/status echoes
        # them for page-reload hydration — after a completed mission both
        # would resurrect the previous waypoints (mission_manager and
        # waypoint_navigator already cleared theirs on COMPLETE/ABORT).
        ros_node.uploaded_waypoints = []
        ros_node.last_speed_ms = None
    return {"status": "ok"}


@app.post("/mission/return-home")
def return_home():
    """Operator-triggered RTH — reuses mission_manager's existing RTH:<reason>
    handling (the same path failsafe_monitor uses for comms/node-loss), just
    with an explicit reason distinguishing a manual request from an automatic
    failsafe one in the safety-event log. Not gated by all_clear, same as
    /mission/abort — an operator needs to be able to call the drone home
    precisely when connectivity is degraded, not only when it's perfect.
    mission_manager._trigger_rth() itself no-ops if the mission isn't
    currently airborne, so this is safe to call at any time.
    """
    if ros_node:
        msg = String(); msg.data = "RTH:MANUAL"
        ros_node.cmd_pub.publish(msg)
    return {"status": "ok"}


@app.post("/drone/arm")
def arm_drone():
    _require_all_clear()
    if ros_node:
        msg = String(); msg.data = "ARM"
        ros_node.base_cmd_pub.publish(msg)
    return {"status": "ok", "command": "ARM"}


@app.post("/drone/disarm")
def disarm_drone():
    if ros_node:
        msg = String(); msg.data = "DISARM"
        ros_node.base_cmd_pub.publish(msg)
    return {"status": "ok", "command": "DISARM"}


@app.post("/drone/takeoff")
def drone_takeoff():
    """Manual bench takeoff (Flight Test Bench) -- outside any autonomous
    mission. TAKEOFF on /drone_base/command arms AND sets OFFBOARD itself
    (see drone_base_node._arm_and_offboard), so this doesn't require a
    separate prior /drone/arm call, same as mission_manager's own PREFLIGHT
    -> TAKEOFF transition.
    """
    if not ros_node:
        raise HTTPException(503, "ROS bridge not connected")
    if not ros_node.mavros_connected:
        raise HTTPException(503, "MAVROS not connected.")
    if ros_node.mission_state not in ("IDLE", "MISSION_COMPLETE", "MISSION_ABORT"):
        raise HTTPException(409, "A mission is active — stop/reset it before manual takeoff.")
    msg = String(); msg.data = "TAKEOFF"
    ros_node.base_cmd_pub.publish(msg)
    return {"status": "ok", "command": "TAKEOFF"}


@app.post("/drone/land")
def drone_land():
    """Manual bench land -- ends a manual-control test flight in place."""
    if not ros_node:
        raise HTTPException(503, "ROS bridge not connected")
    if ros_node.drone_status not in ("AIRBORNE", "LANDING"):
        raise HTTPException(409, "Drone is not airborne.")
    msg = String(); msg.data = "LAND"
    ros_node.base_cmd_pub.publish(msg)
    return {"status": "ok", "command": "LAND"}


MANUAL_NUDGE_CMDS = {"FWD", "BACK", "LEFT", "RIGHT", "UP", "DOWN", "YAW_LEFT", "YAW_RIGHT", "HOLD"}


@app.post("/drone/manual/{cmd}")
def manual_nudge(cmd: str):
    """Flight Test Bench directional pad -- small position/yaw nudges for
    bench/real-drone attitude and control-response testing.

    Deliberately gated narrower than _require_all_clear(): that gate also
    demands GPS lock and a matching geofence, which would block exactly the
    indoor/tripod bench testing this exists for. Manual control only needs
    the mission state machine to be idle (so it can never race a running
    autonomous mission for setpoint ownership) and the vehicle to already
    be armed (arming itself already requires MAVROS to be connected).
    """
    cmd = cmd.upper()
    if cmd not in MANUAL_NUDGE_CMDS:
        raise HTTPException(400, f"Unknown manual command: {cmd}")
    if not ros_node:
        raise HTTPException(503, "ROS bridge not connected")
    if ros_node.mission_state not in ("IDLE", "MISSION_COMPLETE", "MISSION_ABORT"):
        raise HTTPException(409, "A mission is active — stop/reset it before using manual control.")
    if ros_node.drone_status not in ("ARMED", "AIRBORNE", "LANDING"):
        raise HTTPException(409, "Drone is not armed — arm (and take off) before manual control.")
    msg = String(); msg.data = cmd
    ros_node.manual_nudge_pub.publish(msg)
    return {"status": "ok", "cmd": cmd}


@app.get("/mission/status")
def status():
    if ros_node:
        # current_lat/current_lon default to 0.0 until a real NavSatFix
        # arrives via _gps_cb -- reporting that sentinel as if it were a
        # real fix put the dashboard's drone marker at Null Island (0,0)
        # whenever the backend was up but MAVROS/PX4 wasn't, confirmed live
        # 2026-07-11. Same 0.0-means-no-fix-yet convention already used by
        # gps_lock below; the REST payload just wasn't honoring it.
        has_fix = ros_node.current_lat != 0.0 or ros_node.current_lon != 0.0
        return {
            "mission_state": ros_node.mission_state,
            "drone_status":  ros_node.drone_status,
            "lat":           ros_node.current_lat if has_fix else None,
            "lon":           ros_node.current_lon if has_fix else None,
            "altitude":      ros_node.altitude,
            "heading":       ros_node.heading,
            "battery":       ros_node.battery_pct,
            "flight_mode":   ros_node.flight_mode,
            # non-null only while a mission is being recorded — lets a
            # reconnecting dashboard restore the flown trail from the DB
            "mission_id":    ros_node.current_mission_id,
            # the last uploaded plan — lets a page opened mid-flight rebuild
            # the mission stops instead of booting into an empty planner
            "waypoints":     ros_node.uploaded_waypoints,
            "speed_ms":      ros_node.last_speed_ms,
        }
    return {"mission_state": "DISCONNECTED"}


# ── Feature 1: runtime ArUco marker generation + spawn ────────────────────
@app.post("/markers/generate")
def generate_marker(req: MarkerGenerateRequest):
    _require_all_clear()
    if ros_node is None or ros_node.mode == "hardware":
        raise HTTPException(503, "Marker generation is sim-only (mode=hardware).")
    if ros_node.home_lat is None:
        raise HTTPException(503, "Home GPS not yet locked — cannot compute a spawn pose.")

    marker_id = req.marker_id
    if marker_id is None:
        marker_id = ros_node.marker_assignments.get(req.label)
        if marker_id is None:
            marker_id = ros_node.next_auto_marker_id
            ros_node.next_auto_marker_id += 1
    ros_node.marker_assignments[req.label] = marker_id

    model_name = f"aruco_pad_{req.label}"
    result = write_pad_model_everywhere(marker_id, REPO_MODELS_ROOT, model_name)

    north, east = gps_to_local(ros_node.home_lat, ros_node.home_lon, req.lat, req.lon)
    sdf_path = result.get("px4_sdf_path", result["sdf_path"])
    world_name = find_world_name()
    ok, message = spawn_model(world_name, model_name, sdf_path, east, north, 0.001)
    if not ok:
        raise HTTPException(500, f"gz spawn failed: {message}")

    return {"status": "ok", "model_name": model_name, "marker_id": marker_id,
            "texture_path": result["texture_path"]}


# ── Feature 2 (3.3): laptop-geolocation-driven SITL home ──────────────────
@app.post("/system/set-home")
def set_home(req: SetHomeRequest):
    db.set_home(req.lat, req.lon)
    os.makedirs(os.path.dirname(LAST_SYNCED_HOME_FILE), exist_ok=True)
    with open(LAST_SYNCED_HOME_FILE, "w") as f:
        f.write(f"{req.lat},{req.lon}")

    # SITL's home is baked in at PX4 launch time from this same file (see
    # launch_full_sim.sh Stage 1) — syncing after PX4 is already up changes
    # what's on disk but not the running instance, so the home_position_match
    # gate will keep failing until the sim is relaunched.
    relaunch_needed = (
        ros_node is not None
        and ros_node.mavros_connected
        and ros_node.home_lat is not None
        and gps_distance_m(ros_node.home_lat, ros_node.home_lon, req.lat, req.lon) > HOME_MISMATCH_THRESHOLD_M
    )
    return {"status": "ok", "relaunch_needed": relaunch_needed}


@app.get("/system/home")
def get_home():
    home = db.get_home()
    if home is None:
        return {"lat": None, "lon": None, "synced_at": None}
    return home


# ── Geofence endpoints (QGC-style polygon fence) ───────────────────────────
@app.get("/geofence")
def get_geofence():
    fence = db.get_config("geofence") or {}
    return {
        "vertices": fence.get("vertices", []),
        "action": fence.get("action"),
        "set_at": fence.get("set_at"),
        "pushed_to_px4": bool(ros_node and ros_node.geofence_push_confirmed
                              and fence.get("vertices")),
    }


@app.post("/geofence")
def set_geofence(req: GeofenceRequest):
    # Deliberately NOT gated by _require_all_clear: drawing a fence is safety
    # config, and must be settable before the stack is fully up (same policy
    # as /system/set-home).
    if len(req.vertices) < 3:
        raise HTTPException(400, "A geofence polygon needs at least 3 vertices")
    if req.action not in GF_ACTION_CODES:
        raise HTTPException(400, f"action must be one of {sorted(GF_ACTION_CODES)}")
    for v in req.vertices:
        if not (-90.0 <= v.lat <= 90.0 and -180.0 <= v.lon <= 180.0):
            raise HTTPException(400, f"vertex out of range: {v.lat},{v.lon}")
    verts = [{"lat": v.lat, "lon": v.lon} for v in req.vertices]
    # Home must be INSIDE the fence — a "return" breach action with home
    # outside would command the drone to fly out through its own fence.
    if ros_node and ros_node.home_lat is not None and \
            not point_in_polygon(ros_node.home_lat, ros_node.home_lon, verts):
        raise HTTPException(400, "Fence must contain the home position")
    db.set_config("geofence", {"vertices": verts, "action": req.action,
                               "set_at": db._now()})
    pushed = ros_node.push_geofence_to_px4(verts, req.action) if ros_node else False
    if ros_node:
        # If the push didn't land (PX4 not up yet), the retry timer re-pushes
        # on the next MAVROS connect — the fence is never silently dropped.
        ros_node.geofence_push_confirmed = pushed
    return {"status": "ok", "vertex_count": len(verts),
            "action": req.action, "pushed_to_px4": pushed}


@app.delete("/geofence")
def clear_geofence():
    db.set_config("geofence", None)
    cleared = False
    if ros_node:
        cleared = ros_node.clear_geofence_on_px4()
        ros_node._set_mavros_param("GF_ACTION", integer=0)
        ros_node.geofence_push_confirmed = True  # nothing left to re-push
    return {"status": "ok", "cleared_on_px4": cleared}


# ── Feature 4: sim/hardware toggle ────────────────────────────────────────
@app.get("/system/mode")
def get_mode():
    return {"mode": db.get_config("mode", "sim")}


@app.post("/system/mode")
def set_mode(req: ModeRequest):
    if req.mode not in ("sim", "hardware"):
        raise HTTPException(400, "mode must be 'sim' or 'hardware'")
    db.set_config("mode", req.mode)
    if ros_node:
        ros_node.mode = req.mode
    return {"status": "ok", "mode": req.mode}


# ── Feature 11: hardware profile + range estimate ─────────────────────────
@app.get("/system/profile")
def get_profile():
    profile = db.get_profile() or {}
    return {"profile": profile, "estimate": range_estimate.estimate(profile)}


@app.post("/system/profile")
def set_profile(req: DroneProfileRequest):
    db.set_profile(req.model_dump())
    return get_profile()


# ── Feature 10: persisted travel log ──────────────────────────────────────
@app.get("/missions/{mission_id}/travel-log")
def travel_log(mission_id: int):
    log = db.get_travel_log(mission_id)
    if log is None:
        raise HTTPException(404, "mission not found")
    return log


# ── Camera stream endpoints (unchanged) ───────────
@app.get("/camera/stream")
async def camera_stream():
    """
    MJPEG stream — React <img src="/camera/stream"> displays it live.
    No websocket needed — works like a regular image tag.
    """
    def generate_frames():
        while True:
            if ros_node is None or ros_node.latest_frame is None:
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(
                    blank, "Waiting for camera...", (160, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 200), 2)
                _, buffer = cv2.imencode(".jpg", blank)
            else:
                _, buffer = cv2.imencode(
                    ".jpg", ros_node.latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])

            frame_bytes = buffer.tobytes()
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + frame_bytes +
                b"\r\n"
            )
            import time as _time; _time.sleep(0.033)

    return StreamingResponse(
        generate_frames(),
        media_type="multipart/x-mixed-replace;boundary=frame"
    )


@app.get("/camera/snapshot")
async def camera_snapshot():
    """Single JPEG snapshot — for testing."""
    if ros_node is None or ros_node.latest_frame is None:
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        _, buffer = cv2.imencode(".jpg", blank)
    else:
        _, buffer = cv2.imencode(".jpg", ros_node.latest_frame)

    return StreamingResponse(iter([buffer.tobytes()]), media_type="image/jpeg")


# ── Feature 3/6/7/8/9: WebSocket push (node status, IMU, position) ────────
@app.websocket("/ws/system-status")
async def system_status_ws(websocket: WebSocket):
    await websocket.accept()
    if ros_node:
        ros_node.ws_clients.add(websocket)
        await websocket.send_json(ros_node.build_node_status_payload())
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        if ros_node:
            ros_node.ws_clients.discard(websocket)


async def _broadcast(payload: dict):
    if ros_node is None:
        return
    dead = []
    for ws in list(ros_node.ws_clients):
        try:
            await ws.send_json(payload)
        except Exception:
            dead.append(ws)
    for ws in dead:
        ros_node.ws_clients.discard(ws)


def push_from_ros_thread(payload: dict):
    """Call from an rclpy callback (a different thread than uvicorn's asyncio
    loop) to push a WebSocket message. Must hop threads via
    run_coroutine_threadsafe — a naive cross-thread await would just block."""
    if ros_node is None or ros_node.loop is None:
        return
    try:
        asyncio.run_coroutine_threadsafe(_broadcast(payload), ros_node.loop)
    except RuntimeError:
        pass  # event loop closed (uvicorn shutting down) — never fatal for the ROS side


# ── Updated BridgeNode ────────────────────────────
class BridgeNode(Node):
    def __init__(self):
        super().__init__("mission_api_node")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ── Publishers ────────────────────────────
        self.waypoints_pub = self.create_publisher(String, "/mission/waypoints", 10)
        self.cmd_pub = self.create_publisher(String, "/mission/command", 10)
        self.base_cmd_pub = self.create_publisher(String, "/drone_base/command", 10)
        self.manual_nudge_pub = self.create_publisher(String, "/manual/nudge", 10)

        # ── Subscribers: mission/base/camera (existing) ──
        self.create_subscription(String,    "/mission/status",               self._mission_cb, 10)
        self.create_subscription(Float64, "/mission/ground_z", self._ground_z_cb, 10)
        self.create_subscription(String,    "/drone_base/status",            self._base_cb,    10)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._gps_cb,     sensor_qos)
        self.create_subscription(Odometry,  "/mavros/local_position/odom",    self._odom_cb,    sensor_qos)
        self.create_subscription(Image,     "/camera/image_raw",              self._camera_cb,  sensor_qos)
        self.create_subscription(State,     "/mavros/state",                  self._state_cb,   sensor_qos)

        # ── Subscribers: new for connectivity gate / IMU / home ──
        self.create_subscription(HomePosition, "/mavros/home_position/home", self._home_cb, sensor_qos)
        self.create_subscription(Imu,          "/mavros/imu/data",           self._imu_cb,  sensor_qos)
        self.create_subscription(BatteryState, "/mavros/battery",           self._battery_cb, sensor_qos)
        self.create_subscription(String, "/waypoint_nav/status",  self._make_heartbeat_cb("waypoint_navigator"), 10)
        self.create_subscription(String, "/aruco_landing/status", self._make_heartbeat_cb("aruco_landing"), 10)
        self.create_subscription(String, "/vision_node/heartbeat", self._make_heartbeat_cb("vision_node"), 10)
        self.create_subscription(String, "/camera_node/heartbeat", self._make_heartbeat_cb("camera_node"), 10)
        self.create_subscription(String, TOPIC_MISSION_SAFETY_EVENT, self._safety_event_cb, 10)

        # NOTE: not mavros_msgs/srv/ParamSet — confirmed 2026-07-10 that
        # /mavros/param/set now serves ParamSetV2 in this MAVROS version, a
        # type mismatch a ParamSet-typed client can never discover (its
        # service_is_ready() just stays permanently False, no error). The
        # standard ROS2 parameter service works reliably (verified live,
        # including a real arm succeeding once NAV_DLL_ACT was set this way).
        self.param_set_client = self.create_client(SetParameters, "/mavros/param/set_parameters")
        # Geofence upload — same waypoint-protocol services QGC uses for fences
        self.geofence_push_client = self.create_client(WaypointPush, "/mavros/geofence/push")
        self.geofence_clear_client = self.create_client(WaypointClear, "/mavros/geofence/clear")

        # ── Internal state ─────────────────────────
        self.mission_state = "IDLE"
        self.drone_status  = "DISCONNECTED"
        self.current_lat   = 0.0
        self.current_lon   = 0.0
        self.altitude      = 0.0
        self.heading       = 0.0
        self.battery_pct   = 100.0
        self.flight_mode   = "UNKNOWN"
        self.mavros_connected = False
        self.nav_dll_act_confirmed = False
        self._nav_dll_act_timer = None
        self.geofence_push_confirmed = False
        self._geofence_timer = None
        self.alt_abort_triggered = False
        self._alt_over_since   = None
        self.ground_z = 0.0  # EKF z-drift snapshot broadcast by mission_manager at each takeoff
        self.home_lat = self.home_lon = None

        self.latest_frame  = None
        self.bridge        = CvBridge()

        # Connectivity gate bookkeeping (Feature 9)
        self.node_last_seen = {name: None for name in MONITORED_NODES}
        self.node_last_seen["drone_base"] = time.monotonic()  # first /drone_base/status may lag briefly
        self._last_gate_key = None

        # MAVROS-derived liveness (Feature 9 fix): mavros_connected/gps_lock/
        # battery_ok were being read as one-shot cached booleans that only
        # ever moved forward — once MAVROS said "connected" they stayed
        # true forever, even after MAVROS/PX4 died, so the gate would
        # report ALL_CLEAR against a dead stack. Track last-message time for
        # each, same staleness pattern as node_last_seen above.
        self.mavros_state_last_seen = None
        self.gps_last_seen = None
        self.battery_last_seen = None

        # Marker generation (Feature 1)
        self.marker_assignments = {}
        self.next_auto_marker_id = ARUCO_ID_AUTO_START

        # Mode + uploaded waypoints (Features 4/5, 9's geofence check)
        self.mode = db.get_config("mode", "sim")
        self.uploaded_waypoints = []
        self.last_speed_ms = None

        # WebSocket bookkeeping — loop is set from the FastAPI startup hook
        self.ws_clients = set()
        self.loop = None

        # Travel log (Feature 10)
        self.current_mission_id = None
        # True once the first /mission/status message arrives. Lets _mission_cb
        # tell "backend booted while a mission was already airborne" (adopt the
        # open DB row) apart from "watched a mission actually start" (new row).
        self._mission_state_synced = False

        self.create_timer(0.5, self._push_node_status)
        self.create_timer(1.0, self._log_travel_point)

        self.get_logger().info("BridgeNode ready — camera stream on /camera/stream")

    # ── Existing callbacks ─────────────────────────
    def _mission_cb(self, msg):
        prev = self.mission_state
        first_since_boot = not self._mission_state_synced
        self._mission_state_synced = True
        self.mission_state = msg.data
        self.node_last_seen["mission_manager"] = time.monotonic()
        if prev != msg.data:
            push_from_ros_thread({"type": "mission_state", "mission_state": msg.data})
        if prev not in AIRBORNE_MISSION_STATES and msg.data in AIRBORNE_MISSION_STATES:
            # If this backend just started and the mission is ALREADY airborne,
            # a previous backend instance died mid-flight — re-attach to its
            # open mission row instead of splitting the travel log in two.
            adopted = db.get_open_mission() if first_since_boot else None
            if adopted is not None:
                self.current_mission_id = adopted
                self.get_logger().warn(
                    f"Backend restarted mid-flight — adopted open mission {adopted}, "
                    "travel log continues in the same row.")
            else:
                self.current_mission_id = db.start_mission(self.home_lat, self.home_lon)
        elif prev in AIRBORNE_MISSION_STATES and msg.data not in AIRBORNE_MISSION_STATES:
            if self.current_mission_id is not None:
                outcome = "COMPLETE" if msg.data == "MISSION_COMPLETE" else "ABORTED_LANDED"
                db.end_mission(self.current_mission_id, outcome)
                self.current_mission_id = None

    def _base_cb(self, msg):
        self.drone_status = msg.data
        self.node_last_seen["drone_base"] = time.monotonic()

    def _state_cb(self, msg):
        prev_connected = self.mavros_connected
        self.flight_mode = msg.mode
        self.mavros_connected = msg.connected
        self.mavros_state_last_seen = time.monotonic()
        if msg.connected and not prev_connected:
            self.nav_dll_act_confirmed = False
            if self._nav_dll_act_timer is None:
                self._nav_dll_act_timer = self.create_timer(3.0, self._disable_gcs_link_failsafe)
            # A (re)connect may be a fresh PX4 process with no fence loaded —
            # re-push the stored fence until confirmed (same retry pattern as
            # the NAV_DLL_ACT timer above).
            self.geofence_push_confirmed = False
            if self._geofence_timer is None:
                self._geofence_timer = self.create_timer(3.0, self._repush_geofence)
        elif not msg.connected:
            # Reconnect later may land on a fresh PX4 process (param not
            # guaranteed persisted) — re-arm the retry loop next connect.
            self.nav_dll_act_confirmed = False

    def _disable_gcs_link_failsafe(self):
        """This project has no human-operated GCS (QGroundControl) — the
        website + MAVROS + companion nodes are the only link. PX4's
        NAV_DLL_ACT defaults to a nonzero "data link loss" failsafe action
        that requires a GCS heartbeat to arm at all (verified: with the
        default value, /mavros/cmd/arming fails every time with "Arming
        denied: Resolve system health failures first" — see
        rcAndDataLinkCheck.cpp's gcs_connection_required check).

        Runs on a retry timer, not a one-shot attempt: the param-set client
        may not have finished its service-discovery handshake yet at the
        moment MAVROS first reports connected (confirmed: happens whenever
        this node starts *after* MAVROS is already up, not just on a fresh
        simultaneous boot) — a single service_is_ready() check right at the
        connection event is not reliable enough for something arming
        depends on.
        """
        if self.nav_dll_act_confirmed or not self.mavros_connected:
            return
        future = self._set_mavros_param("NAV_DLL_ACT", integer=0)
        if future is None:
            self.get_logger().warn("NAV_DLL_ACT set retrying — param service not ready yet")
            return

        def _on_result(f):
            ok = bool(f.result().results) and f.result().results[0].successful
            self.get_logger().info(f"NAV_DLL_ACT set -> 0: {ok}")
            if ok:
                self.nav_dll_act_confirmed = True
                if self._nav_dll_act_timer is not None:
                    self._nav_dll_act_timer.cancel()
                    self._nav_dll_act_timer = None

        future.add_done_callback(_on_result)

    def _set_mavros_param(self, name: str, *, integer: int = None, real: float = None):
        """Set an FCU parameter via MAVROS's ROS2-native parameter service
        (/mavros/param/set_parameters, rcl_interfaces/srv/SetParameters) —
        NOT mavros_msgs/srv/ParamSet, which this MAVROS version's
        /mavros/param/set actually serves as ParamSetV2 instead (a type
        mismatch a ParamSet client can never discover). Returns the pending
        future, or None if the service isn't discovered yet (caller decides
        whether/how to retry).
        """
        if not self.param_set_client.service_is_ready():
            return None
        value = (ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=integer)
                  if integer is not None else
                  ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=real))
        req = SetParameters.Request(parameters=[Parameter(name=name, value=value)])
        return self.param_set_client.call_async(req)

        future.add_done_callback(_on_result)

    def _home_cb(self, msg: HomePosition):
        self.home_lat = msg.geo.latitude
        self.home_lon = msg.geo.longitude

    def _gps_cb(self, msg: NavSatFix):
        # Validate before trusting: under Gazebo CPU load this stack has
        # produced corrupted telemetry (garbage odometry altitudes, and a
        # lat=155.52 -- physically impossible -- seen live). A corrupt or
        # no-fix NavSatFix relayed as-is flings the dashboard marker across
        # the world map; drop it here so the last good position stands.
        if msg.status.status < 0:  # NavSatStatus.STATUS_NO_FIX
            return
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        if abs(msg.latitude) > 90.0 or abs(msg.longitude) > 180.0:
            return
        if msg.latitude == 0.0 and msg.longitude == 0.0:  # no-fix sentinel
            return
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.gps_last_seen = time.monotonic()
        push_from_ros_thread({
            "type": "position",
            "lat": msg.latitude, "lon": msg.longitude,
            "heading": self.heading, "altitude": self.altitude,
        })

    def _odom_cb(self, msg: Odometry):
        self.altitude = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # MAVROS odom is ENU: atan2 yields yaw with 0 deg = East, CCW-positive.
        # The dashboard arrow (and any compass display) expects true compass
        # heading: 0 deg = North, clockwise-positive. Convert here so every
        # consumer (WebSocket push, REST status, travel log) gets compass.
        self.heading = (90.0 - math.degrees(math.atan2(siny, cosy))) % 360

        # Debounced trigger: reproduced live twice (2026-07-13, two
        # different worlds) a ONE-SAMPLE EKF altitude glitch after
        # re-arming post-landing -- e.g. 4.1m jumping to 18.0m within a
        # single odom tick, not a real climb. A genuine runaway still stays
        # over the ceiling on the next several samples (odom publishes at
        # ~30-50Hz here, so this costs at most a couple hundred ms of real
        # safety response time) -- a lone glitch does not.
        now = time.monotonic()
        # Measured against mission_manager's ground snapshot: SITL EKF z
        # accumulates several metres of drift per landing cycle, and an
        # absolute ceiling starts false-aborting once believed ground alt
        # approaches it (mission 3+ of a session, seen 2026-07-14).
        if (self.altitude - self.ground_z) >= ABORT_ALTITUDE_M:
            if self._alt_over_since is None:
                self._alt_over_since = now
            elif not self.alt_abort_triggered and (now - self._alt_over_since) >= ALT_ABORT_DEBOUNCE_S:
                self.alt_abort_triggered = True
                self.mission_state = "MISSION_ABORT"
                push_from_ros_thread({"type": "mission_state", "mission_state": "MISSION_ABORT"})
                self.get_logger().warn(
                    f"SAFETY: altitude {self.altitude - self.ground_z:.2f}m above ground ref "
                    f">= {ABORT_ALTITUDE_M}m limit for {ALT_ABORT_DEBOUNCE_S}s — aborting mission")
                abort_msg = String(); abort_msg.data = "ABORT"
                self.cmd_pub.publish(abort_msg)
        else:
            self._alt_over_since = None

    def _ground_z_cb(self, msg):
        self.ground_z = msg.data

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.degrees(math.copysign(math.pi / 2, sinp)) if abs(sinp) >= 1 \
            else math.degrees(math.asin(sinp))
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        push_from_ros_thread({
            "type": "imu",
            "pitch": pitch, "roll": roll, "yaw": yaw,
            "rate": {"x": msg.angular_velocity.x, "y": msg.angular_velocity.y, "z": msg.angular_velocity.z},
        })

    def _battery_cb(self, msg: BatteryState):
        self.battery_last_seen = time.monotonic()
        if msg.percentage is not None and msg.percentage >= 0:
            self.battery_pct = msg.percentage * 100.0

    def _make_heartbeat_cb(self, name):
        def cb(msg):
            self.node_last_seen[name] = time.monotonic()
        return cb

    def _safety_event_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if self.current_mission_id is not None:
            db.log_safety_event(self.current_mission_id, data.get("event_type", "UNKNOWN"), data.get("detail", ""))
        if data.get("event_type") == "MAVROS_LOST" or "NODE_HEARTBEAT" in data.get("event_type", ""):
            pass  # RTH outcome already recorded via mission_state transition -> ABORTED_RTH below

    # ── Camera callback ───────────────────────
    def _camera_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2
            cv2.line(frame, (cx-20, cy), (cx+20, cy), (0, 255, 255), 1)
            cv2.line(frame, (cx, cy-20), (cx, cy+20), (0, 255, 255), 1)
            cv2.putText(frame, f"ALT: {self.altitude:.1f}m", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            cv2.putText(frame, f"STATE: {self.mission_state}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            self.latest_frame = frame
        except Exception as e:
            self.get_logger().warn(f"Camera callback error: {e}", throttle_duration_sec=5.0)

    # ── Connectivity gate (Feature 9) ──────────────
    def _home_mismatch(self) -> bool:
        synced = db.get_home()
        if self.home_lat is None or synced is None or synced.get("lat") is None:
            return False
        return gps_distance_m(self.home_lat, self.home_lon, synced["lat"], synced["lon"]) > HOME_MISMATCH_THRESHOLD_M

    def _geofence_valid(self) -> bool:
        fence = db.get_config("geofence")
        verts = (fence or {}).get("vertices") or []
        if verts:
            # Operator-drawn fence: home and every uploaded waypoint must be inside.
            if self.home_lat is not None and \
                    not point_in_polygon(self.home_lat, self.home_lon, verts):
                return False
            return all(point_in_polygon(wp["lat"], wp["lon"], verts)
                       for wp in self.uploaded_waypoints)
        # No fence drawn — fall back to the original crude sanity radius.
        if self.home_lat is None or not self.uploaded_waypoints:
            return True
        for wp in self.uploaded_waypoints:
            if gps_distance_m(self.home_lat, self.home_lon, wp["lat"], wp["lon"]) > HOME_MISMATCH_THRESHOLD_M:
                return False
        return True

    def _stale_nodes(self):
        now = time.monotonic()
        return [name for name, seen in self.node_last_seen.items()
                if seen is None or (now - seen) > NODE_HEARTBEAT_STALE_S]

    @staticmethod
    def _is_stale(last_seen, threshold=NODE_HEARTBEAT_STALE_S):
        return last_seen is None or (time.monotonic() - last_seen) > threshold

    def all_clear(self):
        stale = self._stale_nodes()
        # mavros_connected/gps_lock/battery_ok used to be one-shot cached
        # booleans: once true, they stayed true forever, even after MAVROS
        # died and stopped publishing entirely — so the gate could report
        # ALL_CLEAR against a fully dead stack. Require a *recent* message
        # on each underlying topic, not just "ever received one".
        mavros_connected = self.mavros_connected and not self._is_stale(self.mavros_state_last_seen, MAVROS_STATE_STALE_S)
        gps_lock = (self.current_lat != 0.0 or self.current_lon != 0.0) and not self._is_stale(self.gps_last_seen)
        battery_ok = self.battery_pct > 10.0 and not self._is_stale(self.battery_last_seen, BATTERY_HEARTBEAT_STALE_S)
        checks = {
            "mavros_connected": mavros_connected,
            "nodes_alive": len(stale) == 0,
            "gps_lock": gps_lock,
            "home_set": self.home_lat is not None,
            "battery_ok": battery_ok,
            "home_position_match": not self._home_mismatch(),
            "geofence_valid": self._geofence_valid(),
        }
        ok = all(checks.values())
        reasons = [k for k, v in checks.items() if not v]
        if stale:
            reasons.append(f"stale_nodes:{stale}")
        return ok, reasons, checks

    def build_node_status_payload(self):
        ok, reasons, checks = self.all_clear()
        stale = set(self._stale_nodes())
        return {
            "type": "node_status",
            "nodes": {name: ("DISCONNECTED" if name in stale else "CONNECTED")
                      for name in MONITORED_NODES},
            "preflight": {
                "gps_lock": checks["gps_lock"],
                "satellites": None,
                "battery_pct": round(self.battery_pct, 1),
                "mavros_connected": checks["mavros_connected"],
                "home_set": checks["home_set"],
                "geofence_valid": checks["geofence_valid"],
                "home_position_match": checks["home_position_match"],
            },
            "all_clear": ok,
            "reasons": reasons,
        }

    def _push_node_status(self):
        if self.loop is None:
            return  # uvicorn/lifespan hasn't started yet — nothing could receive this anyway
        payload = self.build_node_status_payload()
        gate_key = (payload["all_clear"], tuple(sorted(payload["reasons"])))
        if gate_key == self._last_gate_key:
            return
        self._last_gate_key = gate_key
        push_from_ros_thread(payload)

    def _log_travel_point(self):
        if self.current_mission_id is not None:
            db.log_travel_point(self.current_mission_id, self.current_lat, self.current_lon,
                                 self.altitude, self.heading)

    # ── Geofence: push the fence into PX4 itself (QGC-style) ─────────────
    def _build_fence_request(self, vertices):
        req = WaypointPush.Request()
        req.start_index = 0
        for v in vertices:
            wp = MavWaypoint()
            wp.frame = 3  # GLOBAL_RELATIVE_ALT — the frame QGC uploads fence points in
            wp.command = NAV_FENCE_POLYGON_VERTEX_INCLUSION
            wp.is_current = False
            wp.autocontinue = True
            wp.param1 = float(len(vertices))  # vertex count of this polygon
            wp.x_lat, wp.y_long, wp.z_alt = float(v["lat"]), float(v["lon"]), 0.0
            req.waypoints.append(wp)
        return req

    def push_geofence_to_px4(self, vertices, action, timeout_s: float = 5.0) -> bool:
        """Blocking push, called from the FastAPI thread — rclpy spins on the
        main thread, so waiting on the future here doesn't deadlock."""
        if not self.geofence_push_client.service_is_ready():
            return False
        future = self.geofence_push_client.call_async(self._build_fence_request(vertices))
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_s):
            return False
        try:
            ok = bool(future.result().success)
        except Exception:
            ok = False
        if ok and vertices:
            self._set_mavros_param("GF_ACTION", integer=GF_ACTION_CODES[action])
            # Cap PX4's RTL climb at the project's own RTH altitude: PX4's
            # default RTL return altitude is far above ABORT_ALTITUDE_M
            # (10m), so a breach-triggered RTL would trip the backend's
            # altitude abort mid-return (observed live 2026-07-12).
            self._set_mavros_param("RTL_RETURN_ALT", real=float(RTH_ALTITUDE))
            self.geofence_push_confirmed = True
            self.get_logger().info(
                f"Geofence pushed to PX4: {len(vertices)} vertices, breach action '{action}'")
        return ok

    def clear_geofence_on_px4(self, timeout_s: float = 5.0) -> bool:
        """Wipe the FCU-side fence via /mavros/geofence/clear (the dedicated
        clear service — an empty WaypointPush is not a valid wipe)."""
        if not self.geofence_clear_client.service_is_ready():
            return False
        future = self.geofence_clear_client.call_async(WaypointClear.Request())
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_s):
            return False
        try:
            return bool(future.result().success)
        except Exception:
            return False

    def _repush_geofence(self):
        """Retry timer: the stored fence must survive a PX4/MAVROS restart,
        so keep re-pushing after each connect until PX4 confirms it. Runs
        async (no blocking wait) because this executes on the rclpy thread."""
        if self.geofence_push_confirmed or not self.mavros_connected:
            return
        fence = db.get_config("geofence")
        verts = (fence or {}).get("vertices") or []
        if not verts:
            self.geofence_push_confirmed = True  # nothing to push
            return
        if not self.geofence_push_client.service_is_ready():
            return
        future = self.geofence_push_client.call_async(self._build_fence_request(verts))

        def _on_result(f):
            try:
                ok = bool(f.result().success)
            except Exception:
                ok = False
            if ok:
                self._set_mavros_param(
                    "GF_ACTION",
                    integer=GF_ACTION_CODES.get(fence.get("action", "return"), 3))
                self._set_mavros_param("RTL_RETURN_ALT", real=float(RTH_ALTITUDE))
                self.geofence_push_confirmed = True
                self.get_logger().info(
                    f"Geofence re-pushed to PX4 after (re)connect ({len(verts)} vertices)")

        future.add_done_callback(_on_result)

    # ── Feature 5 (section 6): mission-level max speed via MAVROS ────────
    def set_max_speed(self, speed_ms: float):
        future = self._set_mavros_param("MPC_XY_VEL_MAX", real=float(speed_ms))
        if future is None:
            self.get_logger().warn("MAVROS param/set service not ready — speed not applied")
            return
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"MPC_XY_VEL_MAX set -> {bool(f.result().results) and f.result().results[0].successful}"))


def main():
    global ros_node
    rclpy.init()
    db.init_db()
    ros_node = BridgeNode()

    api_thread = threading.Thread(
        target=uvicorn.run,
        kwargs={
            "app":       app,
            "host":      "0.0.0.0",
            "port":      8000,
            "log_level": "warning"
        },
        daemon=True
    )
    api_thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
