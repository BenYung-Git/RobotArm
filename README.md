# RobotArm — Virtual Demo (Phase 1C)

Local in-memory 6-DOF virtual robotic arm. **Virtual-only simulation. Not
connected to any real controller, PLC, I/O, fieldbus, serial device, or
vendor SDK.** This is a Flask + React-free HTML/JS demo driven entirely
by HTTP polling — there is no WebSocket, no Flask-SocketIO, no Socket.IO.

## What's in here

This is **Phase 1C** of a staged demo. The earlier phases are kept for
context but the *current* code base is Phase 1C.

| Phase | Status | What it does |
|-------|--------|--------------|
| **Phase 1A** | ✅ shipped | URDF asset + joint limits + PyBullet-driven 6-DOF motion (`demos/interactive_demo.py`) |
| **Phase 1B** | ✅ shipped | Flask web UI + 6 sliders + buttons + HTTP `/api/state` polling |
| **Phase 1C** | ✅ shipped (this build) | Adds the **Virtual Controller Cabinet** that mediates every motion, E-stop, fault, and job. Adds brand-neutral Virtual I/O, a backend-computed **Virtual Status Light**, and the Demo Sequence (Home → Pose A → Pose B → Home). |

## How to run it (local-only)

```bash
# From anywhere — no virtualenv required (uses the docRoute venv)
/home/ubuntu/docRoute/venv/bin/python /tmp/robotarm_demo/web/app.py \
    --host 127.0.0.1 --port 5001
```

Then open **http://127.0.0.1:5001/?lang=en** in a browser. Switch language
with `?lang=zh` for 中文.

The frontend polls `/api/v1/status` every **400 ms** — that's the only
status push mechanism. There is no WebSocket, no SSE, no long-poll.

## Phase 1C architecture

```
UI (HTML + JS, polls every 400 ms)
  ↓
HTTP /api/v1/* (Flask, cabinet-mediated)
  ↓
VirtualControllerCabinet (state machine + IO + JobRunner ownership)
  ├─ CabinetState   : OFFLINE | READY | RUNNING | ESTOP | PROTECTIVE_STOP | FAULT
  ├─ SafetyState    : normal | estop | protective_stop | fault
  ├─ MotionState    : idle | moving | stopped
  ├─ VirtualIO      : brand-neutral inputs (whitelist, session-only, no JSON write-back)
  │                   + read-only outputs (stack_light_*, cycle_running)
  └─ JobRunner      : event-driven; per-step timeout; (run_id, generation) ownership tokens
        ↓
VirtualRobot (WebRobot) — purely in-memory; never touches network/serial
```

**Rule:** every motion, E-stop, reset, fault, and job MUST traverse the
cabinet. The UI never talks to the arm directly. The cabinet mediates.

### Single source of truth

| What | Path |
|------|------|
| **Home pose** (all zeros) | `data/poses/home.json` |
| **Pose A** (forward reach) | `data/poses/pose_a.json` |
| **Pose B** (side reach)   | `data/poses/pose_b.json` |
| **Demo sequence** (Home → Pose A → Pose B → Home) | `data/jobs/demo_sequence.json` |
| **Virtual I/O seed** (default inputs + outputs) | `data/virtual_io/default.json` |

There is **no `pose_c.json`** — only three poses exist (intentional).

## API endpoints

All under `/api/v1/` (cabinet-mediated). There is also a small set of
legacy `/api/` endpoints (`/api/state`, `/api/spec`, `/api/connect`,
`/api/move_home`, `/api/load_pose`, `/api/emergency_stop`,
`/api/reset_estop`, `/api/poses`) — they all forward into the cabinet.

| Method | Path | Purpose |
|--------|------|---------|
| `GET`  | `/api/health` | Liveness probe |
| `GET`  | `/api/v1/status` | **Polled every 400 ms by the UI** — combined cabinet + arm snapshot including `status_light` |
| `GET`  | `/api/v1/status-light` | Status light only (subset of `/api/v1/status`) |
| `GET`  | `/api/v1/cabinet/status` | Cabinet-only (no arm joints) |
| `POST` | `/api/v1/cabinet/estop` | Trigger virtual E-stop |
| `POST` | `/api/v1/cabinet/reset-estop` | Clear E-stop |
| `POST` | `/api/v1/cabinet/protective-stop` | Trigger virtual protective stop |
| `POST` | `/api/v1/cabinet/reset-protective-stop` | Clear protective stop |
| `POST` | `/api/v1/cabinet/fault` | Inject a virtual fault (`{"code", "message"}`) |
| `POST` | `/api/v1/cabinet/reset-fault` | Clear fault |
| `GET`  | `/api/v1/virtual-io` | Full I/O snapshot (inputs + outputs) |
| `POST` | `/api/v1/virtual-io/inputs/<signal>` | Set one input (`{"value": true/false}`). Strict whitelist, strict bool, no JSON write-back. |
| `POST` | `/api/v1/jobs/demo-sequence/start` | Start the demo job (Home → Pose A → Pose B → Home) |
| `GET`  | `/api/v1/jobs/current` | Current job progress |

`/api/v1/virtual-io/outputs/*` is **deliberately not exposed as a write
endpoint** — outputs are cabinet-internal.

## Standard Virtual Status Light (backend-computed, UI-only-renders)

The status light is computed by the backend (`robotarm/core/status_light.py`).
The UI is a strict renderer: it reads `status_light.color`,
`status_light.is_blinking`, `status_light.label_zh`, `status_light.label_en`,
`status_light.reason_zh`, `status_light.reason_en` and displays them. It
never derives colour/label/blink from raw state.

| Cabinet state | Color | Blink | English label | Chinese label |
|---------------|-------|-------|---------------|----------------|
| `ESTOP` | red | **yes** | Virtual E-stop Active | 虛擬急停已觸發 |
| `PROTECTIVE_STOP` (with safety_gate open) | red | no | Virtual Fault / Motion Blocked — "Virtual safety gate is open; motion blocked…" | 虛擬故障／運動已封鎖 — "虛擬安全門未關閉…" |
| `PROTECTIVE_STOP` (other cause) | red | no | Virtual Fault / Motion Blocked | 虛擬故障／運動已封鎖 |
| `FAULT` | red | no | Virtual Fault / Motion Blocked (with the fault message) | 虛擬故障／運動已封鎖 |
| `OFFLINE` | gray | no | Offline / Disabled | 離線 / 停用 |
| `READY` with a required input False | yellow | no | Virtual Warning / Waiting (names the missing input) | 等待條件完成 |
| `READY` all inputs satisfied | green | no | Virtual System Ready | 虛擬系統就緒 |
| `RUNNING` all inputs satisfied | green | no | Virtual System Ready | 虛擬系統就緒 |

The UI panel always carries **three disclaimer lines** under the status
light, regardless of state:

1. Brand-neutral EN: "Brand-neutral virtual status light — reflects only
   this system's simulated state; not an LED state of ROKAE, JAKA, or any
   real controller."
2. Brand-neutral ZH: "此為品牌中立的虛擬狀態燈，只反映本系統模擬狀態；
   並非 ROKAE、JAKA 或任何真實控制櫃的 LED 狀態。"
3. Virtual-only: "Virtual-only simulation. Not safety-rated. Not connected
   to real I/O, PLC, controller, or hardware."

## Virtual I/O (brand-neutral, session-only)

Inputs (whitelist, mutable at runtime, in-memory only):

| Signal | English label |
|--------|----------------|
| `part_present` | Part Present |
| `fixture_clamped` | Fixture Clamped |
| `welder_ready` | Welder Ready |
| `safety_gate_closed` | Safety Gate Closed |

Outputs (cabinet-managed, read-only via API):

| Signal | English label |
|--------|----------------|
| `stack_light_green` | Stack Light (Green) |
| `stack_light_red` | Stack Light (Red) |
| `stack_light_yellow` | Stack Light (Yellow) |
| `cycle_running` | Cycle Running |

**Session-only rule:** `set_input()` mutates the in-memory dict and NEVER
writes back to `data/virtual_io/default.json`. Verified by pytest
(`tests/test_virtual_io.py::test_no_disk_write_back` and
`tests/test_phase1c_cabinet.py::test_cabinet_set_input_does_not_write_back_to_seed`).

## Demo Sequence (single source of truth)

`data/jobs/demo_sequence.json`:

```json
{
  "schema_version": 1,
  "job_id": "DEMO_SEQUENCE_001",
  "name": "Virtual Demo Sequence",
  "mode": "virtual_only",
  "required_inputs": ["part_present", "fixture_clamped",
                      "welder_ready", "safety_gate_closed"],
  "steps": [
    {"step_id": "step_01", "action": "move_pose", "pose": "home"},
    {"step_id": "step_02", "action": "move_pose", "pose": "pose_a"},
    {"step_id": "step_03", "action": "move_pose", "pose": "pose_b"},
    {"step_id": "step_04", "action": "move_pose", "pose": "home"}
  ]
}
```

After step_04 the cabinet transitions RUNNING → READY, the runner is
cleared, and `current_job_progress()` returns the **last completed
snapshot** (with `completed: true`, `total_steps: 4`) until the next job
overwrites it.

## Job-run ownership & race-condition guard

Each `JobRunner` instance has two identity tokens:

- `run_id` — Python `id(self)`, unique per runner instance.
- `generation` — bumped on every `start()`. Stamped onto every callback
  (`on_complete`, on_fault`, `on_safety_stop`) so the cabinet can verify
  ownership.

When the cabinet cancels a job (E-stop / protective-stop / fault), it
clears its identity. Any callback that arrives from the cancelled runner
later (e.g. a `VIRTUAL_MOTION_TIMEOUT` that fires after the cabinet has
been reset) is **dropped** — the cabinet state and `fault_code` remain
exactly as the user left them.

This is verified by:
- `tests/test_phase1c_cabinet.py::test_stale_callback_after_cancel_is_dropped`
- `tests/test_phase1c_cabinet.py::test_stale_callback_after_reset_estop_does_not_re_overwrite_fault`
- `tests/test_phase1c_cabinet.py::test_completed_progress_preserved_after_natural_completion`

## What this demo DOES NOT do

- ❌ No Cloudflare tunnel, no public exposure, no `trycloudflare.com`.
  The Flask server binds to `127.0.0.1` only. Verified:
  `ps aux | grep cloudflared` returns no rows; no `cloudflared` process
  exists in this session.
- ❌ No WebSocket, no Flask-SocketIO, no Socket.IO. Status push is HTTP
  polling at 400 ms (`POLL_MS` in `web/static/app.js`).
- ❌ No real controller connection. `controller_connection:
  "not_configured"`, `real_robot_control: "disabled"`, `mode:
  "virtual_only"`. Hardcoded.
- ❌ No PLC. No Modbus / OPC UA / EtherNet/IP / PROFINET / EtherCAT.
- ❌ No serial / RS-485 / RS-232. No `import serial`.
- ❌ No vendor SDK — no `rokae`, `jaka`, `urx`, `ur_rtde`, `pymodbus`,
  `opcua`, `asyncua`, `pycomm3`.
- ❌ No outbound network client. The only network code is the Flask
  listener; no `requests.get(...)`, `urllib`, `http.client`, `socket`.
  Verified by `tests/test_no_banned_imports_*` (static source scan).
- ❌ No Git commit / push / branch / remote access from this session.

## Running the tests

```bash
cd /tmp/robotarm_demo
/home/ubuntu/docRoute/venv/bin/pytest tests/ -v
```

Expected output:

```
tests/test_estop_blocks_movement.py ..........                          [  8%]
tests/test_home_motion.py ..                                            [ 10%]
tests/test_joint_limits.py .......                                      [ 17%]
tests/test_json_pose_validation.py ...........                           [ 27%]
tests/test_phase1c_cabinet.py .......................................... [ 66%]
...                                                                     [ 69%]
tests/test_safety_guard.py .........                                    [ 78%]
tests/test_urdf_asset.py ......                                         [ 83%]
tests/test_virtual_io.py ....................                           [100%]

===================== 111 passed in ~20s ======================
```

(Exact count varies; the Phase 1C group has 45 tests, the Phase 1 group
has 46, plus the 20 Virtual-IO tests = 111.)

## UI screenshots

| # | State | File | Cabinet | Light |
|---|-------|------|---------|-------|
| 1 | READY | `screenshots/01_ready_green.png` | `READY` | green, no blink |
| 2 | WAITING | `screenshots/02_waiting_yellow_safety_gate_open.png` | `READY` (gate open) | yellow, no blink |
| 3 | MOTION BLOCKED | `screenshots/03_motion_blocked_red_safety_gate_open.png` | `PROTECTIVE_STOP` | red, no blink |
| 4 | VIRTUAL FAULT | `screenshots/04_virtual_fault_red.png` | `FAULT` | red, no blink |
| 5 | VIRTUAL E-STOP | `screenshots/05_estop_blinking_red.png` | `ESTOP` | red, **blinking** |

Each screenshot shows the Status Light panel with: the bulb, the colour
cell, the blink cell, the Chinese label, the English label, the Chinese
reason, the English reason, and all three disclaimer paragraphs.

## Project structure

```
/tmp/robotarm_demo/
├── README.md                        ← you are here
├── requirements.txt
├── screenshots/                     ← 5 status-light screenshots (see above)
├── robotarm/
│   ├── __init__.py
│   ├── exceptions.py
│   ├── safety_guard.py              ← static + runtime guard
│   ├── core/
│   │   ├── cabinet_state.py         ← CabinetState + transition table
│   │   ├── virtual_io.py            ← session-only I/O bundle
│   │   ├── virtual_controller_cabinet.py  ← the mediator
│   │   ├── status_light.py          ← backend-computed status light
│   │   ├── job_runner.py            ← event-driven; per-step timeout; identity tokens
│   │   ├── tick_driver.py           ← 60 Hz background tick (extracted for tests)
│   │   ├── robot_spec.py            ← UR5_SPEC dataclass
│   │   ├── robot_state.py
│   │   ├── virtual_robot.py         ← PyBullet-backed 6-DOF arm
│   │   ├── web_robot.py             ← in-memory WebRobot (no PyBullet)
│   │   └── pose_loader.py (in utils/)
│   └── utils/pose_loader.py
├── demos/
│   └── interactive_demo.py          ← Phase 1A PyBullet GUI entry
├── web/
│   ├── app.py                       ← Flask + cabinet + tick driver
│   ├── templates/index.html         ← cabinet panel + status light panel
│   └── static/
│       ├── app.js                   ← 400 ms polling + cabinet UI binding
│       └── style.css                ← status light bulb + blink animation
├── data/
│   ├── poses/{home,pose_a,pose_b}.json
│   ├── jobs/demo_sequence.json
│   └── virtual_io/default.json
├── assets/ur5/...                   ← URDF + meshes + config (BSD-3)
└── tests/
    ├── conftest.py
    ├── test_virtual_io.py           ← 20 tests
    ├── test_phase1c_cabinet.py      ← 45 tests (incl. race-condition guard)
    ├── test_estop_blocks_movement.py
    ├── test_home_motion.py
    ├── test_joint_limits.py
    ├── test_json_pose_validation.py
    ├── test_safety_guard.py         ← static + runtime guard
    └── test_urdf_asset.py
```

## License & assets

- Demo code: project-internal.
- UR5 asset bundle (URDF + meshes + config) under `assets/ur5/`: BSD-3-Clause,
  © ROS-Industrial contributors. See `assets/ASSET_MANIFEST.md`.
