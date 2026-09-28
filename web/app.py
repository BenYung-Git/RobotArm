"""Flask web app exposing the WebRobot as a JSON REST API + HTML frontend.

Endpoints
---------
GET  /                       -> HTML frontend (6 sliders + buttons)
GET  /api/state              -> current RobotState JSON
GET  /api/spec               -> RobotSpec JSON (joint names, limits, dof)
POST /api/connect            -> connect WebRobot
POST /api/disconnect         -> disconnect
POST /api/set_targets        -> set joint targets; body {"joints": [6 floats]}
POST /api/move_home          -> move to home pose
POST /api/load_pose          -> load JSON pose from data/poses/; body {"file": "pose_a.json"}
POST /api/emergency_stop     -> trip E-stop
POST /api/reset_estop        -> clear E-stop
GET  /api/health             -> {"ok": true} liveness probe

The frontend polls /api/state at ~30 Hz to animate motion smoothly.

DISCLAIMER
----------
This is a virtual demo only. The displayed kinematics are an
approximation, NOT an accurate UR5 digital twin, and NOT a digital
twin of ROKAE CR7, JAKA Zu7, or any real hardware.
"""
from __future__ import annotations

import logging
import sys
import threading
import time
from pathlib import Path

# Ensure project root is on sys.path so `robotarm.*` resolves whether this
# file is run directly (`python web/app.py`) or imported by a runner.
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from flask import Flask, jsonify, render_template, request  # noqa: E402

from robotarm.core.web_robot import WebRobot  # noqa: E402
from robotarm.core.virtual_controller_cabinet import VirtualControllerCabinet  # noqa: E402
from robotarm.core.tick_driver import TickDriver  # noqa: E402
from robotarm.exceptions import (  # noqa: E402
    CabinetStateError,
    JobAlreadyRunningError,
    JobValidationError,
    VirtualFaultError,
    VirtualInterlockError,
    VirtualSafetyStopError,
)
from robotarm.utils.pose_loader import PoseLoadError  # noqa: E402,F401

LOG = logging.getLogger("robotarm.web")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

# ---------------------------------------------------------------------------
# App + state
# ---------------------------------------------------------------------------

app = Flask(__name__, template_folder="templates", static_folder="static")

# ---------------------------------------------------------------------------
# i18n: EN <-> 中文
# ---------------------------------------------------------------------------
# Keys match `data-i18n="..."` attributes in templates/app.js.
# Fallback chain: requested lang -> English -> key string itself.
SUPPORTED_LANGS = ("en", "zh")
DEFAULT_LANG = "en"

TRANSLATIONS: dict[str, dict[str, str]] = {
    "en": {
        "title": "RobotArm — Virtual Demo",
        "subtitle": 'Local in-memory 6-DOF virtual arm (UR5 stand-in). <strong>NOT</strong> a digital twin of ROKAE CR7, JAKA Zu7, or any real hardware.',
        "btn_home": "Home",
        "btn_pose_a": "Pose A",
        "btn_pose_b": "Pose B",
        "btn_estop": "Virtual E-STOP",
        "btn_reset": "Reset E-Stop",
        "estop_banner": '⚠ E-STOP ACTIVE — sliders blocked. Click "Reset E-Stop" to resume.',
        "sliders_title": "Joint Control (6-DOF)",
        "loading": "Loading spec…",
        "canvas_title": "Robot Arm (Side View)",
        "status_title": "Live Status",
        "th_mode": "Mode",
        "th_id": "Robot ID",
        "th_conn": "Connected",
        "th_estop": "E-Stop",
        "th_joints": "Joints (deg)",
        "th_targets": "Targets (deg)",
        "th_tcp": "TCP x,y,z (m)",
        "caption": "TCP coordinates are an approximate reach estimate (NOT a true UR5 forward-kinematics solution) — see README §6 for the model disclaimer.",
        # dynamic strings (used by app.js via t())
        "estop_active": "E-STOP ACTIVE",
        "estop_clear": "E-Stop clear",
        "disconnected": "Disconnected",
        "connected": "Connected",
        "loading_failed": "Failed to load spec: ",
        # 6-DOF joint labels (index matches /api/spec joint_names order:
        # shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)
        "joint_0": "Base",
        "joint_1": "Shoulder",
        "joint_2": "Elbow",
        "joint_3": "Wrist 1",
        "joint_4": "Wrist 2",
        "joint_5": "Wrist 3",
        # Phase 1C — Cabinet / Safety / I/O / Job
        "id_mode": "Mode",
        "id_cabinet_type": "Cabinet Type",
        "id_real_ctrl": "Real Robot Control",
        "id_phys_cabinet": "Physical Controller",
        "id_cabinet_state": "Cabinet State",
        "cab_state_title": "Cabinet Status",
        "cab_motion": "Motion",
        "cab_safety": "Virtual Safety",
        "cab_fault": "Fault",
        "cab_updated": "Last Update",
        "job_title": "Demo Job",
        "job_id": "Job ID",
        "job_progress": "Progress",
        "job_step": "Current Step",
        "job_elapsed": "Elapsed",
        "btn_job_start": "Start Demo Job",
        "btn_job_no_active": "Job Complete",
        "safety_title": "Virtual Safety",
        "safety_note": "Simulation only. Not safety-rated. Never use to protect people or machinery.",
        "btn_reset_estop": "Reset E-Stop",
        "btn_prot_stop": "Virtual Protective Stop",
        "btn_reset_prot_stop": "Reset Protective Stop",
        "btn_inject_fault": "Inject Virtual Fault",
        "btn_reset_fault": "Reset Fault",
        "io_title": "Virtual I/O",
        "io_note": "Session-only simulation. Not connected to any physical I/O or controller.",
        "io_part_present": "Part Present",
        "io_fixture_clamped": "Fixture Clamped",
        "io_welder_ready": "Welder Ready",
        "io_safety_gate_closed": "Safety Gate Closed",
        "io_stack_green": "Stack Light (Green)",
        "io_stack_red": "Stack Light (Red)",
        "io_cycle_running": "Cycle Running",
        "motion_title": "Motion (Cabinet-mediated)",
        "footer_virtual_only": "Virtual-only simulation. Not connected to any real controller or hardware.",
        "fault_none": "—",
        # Phase 1C Status Light (task §3)
        "status_light_title": "Status Light",
        "status_light_color": "Color",
        "status_light_state_zh": "中文標籤",
        "status_light_state_en": "English Label",
        "status_light_reason_zh": "中文原因",
        "status_light_reason_en": "English Reason",
        "status_light_blink": "Blink",
        "status_light_yes": "Yes",
        "status_light_no": "No",
        "status_light_disclaimer_zh": "此為品牌中立的虛擬狀態燈，只反映本系統模擬狀態；並非 ROKAE、JAKA 或任何真實控制櫃的 LED 狀態。",
        "status_light_disclaimer_en": "Virtual-only simulation. Not safety-rated. Not connected to real I/O, PLC, controller, or hardware.",
        "status_light_brand_note": "Brand-neutral virtual status light — reflects only this system's simulated state; not an LED state of ROKAE, JAKA, or any real controller.",
    },
    "zh": {
        "title": "RobotArm — 虛擬示範",
        "subtitle": '本機記憶體內 6 自由度虛擬手臂 (UR5 替身)。<strong>並非</strong> ROKAE CR7、JAKA Zu7 或任何真實硬件的數位孿生。',
        "btn_home": "復位 (Home)",
        "btn_pose_a": "姿態 A",
        "btn_pose_b": "姿態 B",
        "btn_estop": "虛擬急停 (E-STOP)",
        "btn_reset": "解除急停",
        "estop_banner": '⚠ 急停已觸發 — 滑桿已鎖定。請按「解除急停」恢復操作。',
        "sliders_title": "關節控制 (6 自由度)",
        "loading": "載入規格中…",
        "canvas_title": "機械臂 (側視圖)",
        "status_title": "即時狀態",
        "th_mode": "模式",
        "th_id": "機械人 ID",
        "th_conn": "連線狀態",
        "th_estop": "急停",
        "th_joints": "關節角度 (度)",
        "th_targets": "目標角度 (度)",
        "th_tcp": "TCP x,y,z (米)",
        "caption": "TCP 座標為近似觸及估算 (並非真正的 UR5 正運動學解) — 詳見 README §6 免責聲明。",
        # dynamic strings
        "estop_active": "急停已觸發",
        "estop_clear": "急停已解除",
        "disconnected": "未連線",
        "connected": "已連線",
        "loading_failed": "載入規格失敗:",
        # 6-DOF 關節標籤
        "joint_0": "基座",
        "joint_1": "肩部",
        "joint_2": "肘部",
        "joint_3": "腕 1",
        "joint_4": "腕 2",
        "joint_5": "腕 3",
        # Phase 1C — Cabinet / Safety / I/O / Job
        "id_mode": "模式",
        "id_cabinet_type": "控制櫃類型",
        "id_real_ctrl": "真機控制",
        "id_phys_cabinet": "實體控制櫃",
        "id_cabinet_state": "控制櫃狀態",
        "cab_state_title": "控制櫃狀態",
        "cab_motion": "運動狀態",
        "cab_safety": "虛擬安全",
        "cab_fault": "錯誤",
        "cab_updated": "最後更新",
        "job_title": "示範工作",
        "job_id": "工作 ID",
        "job_progress": "進度",
        "job_step": "當前步驟",
        "job_elapsed": "耗時",
        "btn_job_start": "開始示範工作",
        "btn_job_no_active": "工作完成",
        "safety_title": "虛擬安全",
        "safety_note": "純模擬。並非安全等級功能。絕不可用於保護人員或機械設備。",
        "btn_reset_estop": "解除急停",
        "btn_prot_stop": "虛擬防護停止",
        "btn_reset_prot_stop": "解除防護停止",
        "btn_inject_fault": "注入虛擬錯誤",
        "btn_reset_fault": "解除錯誤",
        "io_title": "虛擬 I/O",
        "io_note": "僅限 session 模擬。並未連接任何實體 I/O 或控制櫃。",
        "io_part_present": "工件存在",
        "io_fixture_clamped": "治具夾緊",
        "io_welder_ready": "焊機就緒",
        "io_safety_gate_closed": "安全門關閉",
        "io_stack_green": "綠色堆疊燈",
        "io_stack_red": "紅色堆疊燈",
        "io_cycle_running": "週期運行中",
        "motion_title": "運動 (由控制櫃中介)",
        "footer_virtual_only": "純虛擬模擬。並未連接任何真實控制櫃或硬體。",
        "fault_none": "—",
        # Phase 1C Status Light (task §3)
        "status_light_title": "狀態燈",
        "status_light_color": "顏色",
        "status_light_state_zh": "中文標籤",
        "status_light_state_en": "English Label",
        "status_light_reason_zh": "中文原因",
        "status_light_reason_en": "English Reason",
        "status_light_blink": "閃爍",
        "status_light_yes": "是",
        "status_light_no": "否",
        "status_light_disclaimer_zh": "此為品牌中立的虛擬狀態燈，只反映本系統模擬狀態；並非 ROKAE、JAKA 或任何真實控制櫃的 LED 狀態。",
        "status_light_disclaimer_en": "Virtual-only simulation. Not safety-rated. Not connected to real I/O, PLC, controller, or hardware.",
        "status_light_brand_note": "品牌中立的虛擬狀態燈 — 只反映本系統模擬狀態；並非 ROKAE、JAKA 或任何真實控制櫃的 LED 狀態。",
    },
}


def _normalize_lang(raw: str | None) -> str:
    """Accept 'zh' / 'zh-tw' / 'zh-cn' / 'en' / '' → 'zh' or 'en'."""
    if not raw:
        return DEFAULT_LANG
    r = raw.strip().lower()
    if r.startswith("zh"):
        return "zh"
    if r.startswith("en"):
        return "en"
    return DEFAULT_LANG


ROBOT = WebRobot(robot_id="ur5_web_demo")
ROBOT.connect()

# Phase 1C: every motion, E-stop, reset, fault, and job runs through this
# cabinet. WebRobot is owned and mediated by it.
_DATA_ROOT = Path(__file__).resolve().parent.parent
CABINET = VirtualControllerCabinet(
    robot=ROBOT,
    cabinet_id="virtual_cabinet_01",
    poses_dir=_DATA_ROOT / "data" / "poses",
    io_seed_path=_DATA_ROOT / "data" / "virtual_io" / "default.json",
    job_spec_path=_DATA_ROOT / "data" / "jobs" / "demo_sequence.json",
    motion_timeout_s=10.0,
)

# Background tick driver: advances joint interpolation toward targets.
TICK_HZ = 60.0
_TICK_DRIVER: TickDriver | None = None


def _ensure_tick_thread() -> None:
    """Start the global TickDriver once. Idempotent."""
    global _TICK_DRIVER
    if _TICK_DRIVER is None or not (
        _TICK_DRIVER._thread is not None and _TICK_DRIVER._thread.is_alive()
    ):
        _TICK_DRIVER = TickDriver(ROBOT, hz=TICK_HZ)
        _TICK_DRIVER.start()


# ---------------------------------------------------------------------------
# HTML frontend
# ---------------------------------------------------------------------------

POSES_DIR = Path(__file__).resolve().parent.parent / "data" / "poses"


@app.get("/")
def index():
    _ensure_tick_thread()
    # Priority: ?lang=zh query > Accept-Language header > default (en)
    lang = _normalize_lang(request.args.get("lang"))
    if lang == DEFAULT_LANG and request.args.get("lang") is None:
        # No explicit ?lang= → try Accept-Language header
        lang = _normalize_lang(request.accept_languages.best)
    return render_template(
        "index.html",
        poses_dir=str(POSES_DIR),
        lang=lang,
        supported_langs=list(SUPPORTED_LANGS),
    )


@app.get("/api/translations")
@app.get("/api/translations/<lang>")
def translations(lang: str | None = None):
    """Return translation dict for the requested language.

    Always returns the fallback English dict too so the client can use
    it as a safety net for any missing key.
    """
    lang = _normalize_lang(lang or request.args.get("lang"))
    return jsonify(
        {
            "lang": lang,
            "supported": list(SUPPORTED_LANGS),
            "strings": TRANSLATIONS.get(lang, TRANSLATIONS[DEFAULT_LANG]),
            "fallback": TRANSLATIONS[DEFAULT_LANG],
        }
    )


# ---------------------------------------------------------------------------
# JSON API
# ---------------------------------------------------------------------------


def _state_dict() -> dict:
    s = ROBOT.get_state()
    return {
        "robot_id": s.robot_id,
        "mode": s.mode,
        "connected": s.connected,
        "e_stopped": s.e_stopped,
        "joints_rad": list(s.joints),
        "joints_deg": [round(j * 57.29577951308232, 3) for j in s.joints],
        "targets_rad": list(ROBOT.joint_targets),
        "targets_deg": [
            round(t * 57.29577951308232, 3) for t in ROBOT.joint_targets
        ],
        "tcp_position": [round(x, 4) for x in s.tcp_position],
        "tcp_orientation": [round(x, 4) for x in s.tcp_orientation],
    }


def _spec_dict() -> dict:
    spec = ROBOT.spec
    return {
        "name": spec.name,
        "brand": spec.brand,
        "dof": spec.dof,
        "joint_names": list(spec.controllable_joint_names),
        "lower_limits_rad": list(spec.joint_lower_limits_rad),
        "upper_limits_rad": list(spec.joint_upper_limits_rad),
        "lower_limits_deg": [
            round(v * 57.29577951308232, 2) for v in spec.joint_lower_limits_rad
        ],
        "upper_limits_deg": [
            round(v * 57.29577951308232, 2) for v in spec.joint_upper_limits_rad
        ],
        "ee_link_name": spec.ee_link_name,
    }


@app.get("/api/health")
def health():
    return jsonify({"ok": True, "ts": time.time()})


@app.get("/api/spec")
def spec():
    # Wrap in {ok, spec: ...} to match the POST endpoint response shape
    # so the frontend can use a single parser for both GET and POST.
    return jsonify({"ok": True, "spec": _spec_dict()})


@app.get("/api/state")
def state():
    return jsonify({"ok": True, "state": _state_dict()})


@app.post("/api/connect")
def connect():
    # WebRobot stays connected (cabinet owns it). Report current state.
    return jsonify({"ok": True, "state": _state_dict()})


@app.post("/api/disconnect")
def disconnect():
    # WebRobot is mediated by cabinet; we don't actually disconnect in
    # Phase 1C. Report current state instead.
    return jsonify({"ok": True, "state": _state_dict()})


@app.post("/api/set_targets")
def set_targets():
    body = request.get_json(force=True, silent=True) or {}
    joints = body.get("joints")
    if not isinstance(joints, list) or len(joints) != ROBOT.spec.dof:
        return (
            jsonify({"ok": False, "error": f"joints must be list of {ROBOT.spec.dof} floats"}),
            400,
        )
    try:
        CABINET.set_joint_targets(joints)
    except (VirtualSafetyStopError, JobAlreadyRunningError, CabinetStateError) as e:
        return jsonify({"ok": False, "error": str(e)}), 409
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, "state": _state_dict()})


@app.post("/api/move_home")
def move_home():
    try:
        CABINET.move_home_compat()
    except (VirtualSafetyStopError, JobAlreadyRunningError, CabinetStateError) as e:
        return jsonify({"ok": False, "error": str(e)}), 409
    except (FileNotFoundError, JobValidationError) as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, "state": _state_dict()})


@app.post("/api/load_pose")
def load_pose_endpoint():
    body = request.get_json(force=True, silent=True) or {}
    name = body.get("file", "").strip()
    if not name:
        return jsonify({"ok": False, "error": "missing 'file'"}), 400
    # Strict containment: no path traversal
    if "/" in name or "\\" in name or name.startswith("."):
        return jsonify({"ok": False, "error": "invalid filename"}), 400
    # strip the trailing .json — cabinet resolves via pose_name
    pose_name = name[:-5] if name.endswith(".json") else name
    try:
        CABINET.load_pose_compat(pose_name)
    except (VirtualSafetyStopError, JobAlreadyRunningError, CabinetStateError) as e:
        return jsonify({"ok": False, "error": str(e)}), 409
    except (FileNotFoundError, JobValidationError) as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, "state": _state_dict(), "loaded": name})


@app.post("/api/emergency_stop")
def e_stop():
    CABINET.emergency_stop_compat()
    return jsonify({"ok": True, "state": _state_dict()})


@app.post("/api/reset_estop")
def reset_estop():
    CABINET.reset_estop_compat()
    return jsonify({"ok": True, "state": _state_dict()})


@app.get("/api/poses")
def list_poses():
    files = sorted(p.name for p in POSES_DIR.glob("*.json"))
    return jsonify({"poses": files})


# ---------------------------------------------------------------------------
# Phase 1C — /api/v1/* cabinet-mediated routes
# ---------------------------------------------------------------------------
# All routes below forward to CABINET. UI / pad / browser MUST go through
# these endpoints; no direct robot control is exposed.
#
# Status payload is built by CABINET.get_full_status_payload() which reads
# live joints / targets / tcp from the WebRobot each call. Polling is the
# sole status push mechanism (HTTP GET); there is no WebSocket.
# ---------------------------------------------------------------------------


def _arm_snapshot() -> dict:
    """Snapshot the current arm joints / targets / TCP for the status payload.

    This is a thin wrapper around _state_dict() but returns just the fields
    the cabinet payload needs.
    """
    s = _state_dict()
    return {
        "joints_rad": s["joints_rad"],
        "targets_rad": s["targets_rad"],
        "tcp_position": s["tcp_position"],
    }


def _v1_status_payload() -> dict:
    arm = _arm_snapshot()
    return CABINET.get_full_status_payload(
        joint_positions_rad=tuple(arm["joints_rad"]),
        joint_targets_rad=tuple(arm["targets_rad"]),
        tcp_position=tuple(arm["tcp_position"]),
    )


# /api/v1/status — combined cabinet + arm status. Frontend polls this.
@app.get("/api/v1/status")
def v1_status():
    return jsonify({"ok": True, "status": _v1_status_payload()})


# /api/v1/cabinet/status — cabinet-only (no arm joints).
@app.get("/api/v1/cabinet/status")
def v1_cabinet_status():
    return jsonify({"ok": True, "status": CABINET.get_status().to_status_payload()})


# /api/v1/status-light — status light only (subset of status payload).
@app.get("/api/v1/status-light")
def v1_status_light():
    arm = _arm_snapshot()
    s = CABINET.get_status(
        joint_positions_rad=tuple(arm["joints_rad"]),
        joint_targets_rad=tuple(arm["targets_rad"]),
        tcp_position=tuple(arm["tcp_position"]),
    )
    return jsonify({"ok": True, "status_light": s.status_light.to_dict(),
                    "cabinet_state": s.cabinet_state.value})


# /api/v1/cabinet/estop — trigger virtual E-stop
@app.post("/api/v1/cabinet/estop")
def v1_cabinet_estop():
    try:
        new_state = CABINET.trigger_estop(source="api/v1")
    except CabinetStateError as e:
        return jsonify({"ok": False, "error": str(e)}), 409
    return jsonify({"ok": True, "cabinet_state": new_state.value,
                    "status": _v1_status_payload()})


# /api/v1/cabinet/reset-estop
@app.post("/api/v1/cabinet/reset-estop")
def v1_cabinet_reset_estop():
    try:
        new_state = CABINET.reset_estop()
    except CabinetStateError as e:
        return jsonify({"ok": False, "error": str(e)}), 409
    return jsonify({"ok": True, "cabinet_state": new_state.value,
                    "status": _v1_status_payload()})


# /api/v1/cabinet/protective-stop
@app.post("/api/v1/cabinet/protective-stop")
def v1_cabinet_protective_stop():
    try:
        new_state = CABINET.trigger_protective_stop(source="api/v1")
    except CabinetStateError as e:
        return jsonify({"ok": False, "error": str(e)}), 409
    return jsonify({"ok": True, "cabinet_state": new_state.value,
                    "status": _v1_status_payload()})


# /api/v1/cabinet/reset-protective-stop
@app.post("/api/v1/cabinet/reset-protective-stop")
def v1_cabinet_reset_protective_stop():
    try:
        new_state = CABINET.reset_protective_stop()
    except CabinetStateError as e:
        return jsonify({"ok": False, "error": str(e)}), 409
    return jsonify({"ok": True, "cabinet_state": new_state.value,
                    "status": _v1_status_payload()})


# /api/v1/cabinet/fault — inject a virtual fault. Body: {"code": "...", "message": "..."}
@app.post("/api/v1/cabinet/fault")
def v1_cabinet_fault():
    body = request.get_json(force=True, silent=True) or {}
    code = body.get("code") or "VIRTUAL_FAULT"
    message = body.get("message") or ""
    new_state = CABINET.trigger_fault(code, message)
    return jsonify({"ok": True, "cabinet_state": new_state.value,
                    "status": _v1_status_payload()})


# /api/v1/cabinet/reset-fault
@app.post("/api/v1/cabinet/reset-fault")
def v1_cabinet_reset_fault():
    try:
        new_state = CABINET.reset_fault()
    except CabinetStateError as e:
        return jsonify({"ok": False, "error": str(e)}), 409
    return jsonify({"ok": True, "cabinet_state": new_state.value,
                    "status": _v1_status_payload()})


# /api/v1/virtual-io — read full I/O snapshot
@app.get("/api/v1/virtual-io")
def v1_virtual_io_get():
    return jsonify({"ok": True, "io": CABINET.io.snapshot()})


# /api/v1/virtual-io/inputs/<signal> — toggle an input.
# Body: {"value": true|false}
@app.post("/api/v1/virtual-io/inputs/<signal>")
def v1_virtual_io_set(signal: str):
    body = request.get_json(force=True, silent=True) or {}
    if "value" not in body:
        return jsonify({"ok": False, "error": "missing 'value'"}), 400
    value = body["value"]
    # Reject anything that isn't a strict bool.
    if not isinstance(value, bool):
        return jsonify({"ok": False,
                        "error": f"value must be bool, got {type(value).__name__}"}), 400
    try:
        CABINET.set_input(signal, value)
    except JobValidationError as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    except ValueError as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, "io": CABINET.io.snapshot()})


# /api/v1/jobs/demo-sequence/start — start the demo job
@app.post("/api/v1/jobs/demo-sequence/start")
def v1_jobs_demo_sequence_start():
    try:
        jid = CABINET.start_job()
    except (CabinetStateError, JobAlreadyRunningError,
            VirtualSafetyStopError) as e:
        return jsonify({"ok": False, "error": str(e)}), 409
    except VirtualInterlockError as e:
        return jsonify({"ok": False, "error": str(e),
                        "missing_inputs": e.missing}), 409
    except (FileNotFoundError, JobValidationError) as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, "job_id": jid,
                    "status": _v1_status_payload()})


# /api/v1/jobs/current — read current job progress
@app.get("/api/v1/jobs/current")
def v1_jobs_current():
    progress = CABINET.current_job_progress()
    if progress is None:
        return jsonify({"ok": True, "job": None,
                        "status": _v1_status_payload()})
    return jsonify({"ok": True, "job": progress,
                    "status": _v1_status_payload()})


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5001)
    args = parser.parse_args()
    _ensure_tick_thread()
    LOG.info("Starting RobotArm web demo on %s:%d", args.host, args.port)
    # threaded=True so the tick thread + request handlers coexist
    app.run(host=args.host, port=args.port, debug=False, threaded=True)
