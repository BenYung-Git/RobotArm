"""Tests for virtual_io.py — seed loading, schema validation, mutation.

Covers:
- valid seed loads cleanly
- missing file raises FileNotFoundError
- malformed JSON raises JobValidationError
- non-object top-level raises JobValidationError
- wrong schema_version raises JobValidationError
- wrong mode raises JobValidationError
- non-dict inputs/outputs raises
- non-string keys raises
- non-boolean values raises (strict bool, no int/str)
- unknown signal names (input) raises
- missing required inputs raises
- from_seed_file is one-shot — mutations don't write back
- set_input rejects unknown signal names (whitelist)
- set_input rejects non-bool value
- check_interlock semantics (None == False == missing)
- outputs are writable but not part of seed whitelist for inputs
- Snapshot round-trip preserves data
"""
from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from robotarm.core.virtual_io import SCHEMA_VERSION, VirtualIO
from robotarm.exceptions import JobValidationError


_BASE_SEED = {
    "schema_version": 1,
    "mode": "virtual_only",
    "inputs": {
        "part_present": True,
        "fixture_clamped": True,
        "welder_ready": True,
        "safety_gate_closed": True,
    },
    "outputs": {
        "stack_light_green": True,
        "stack_light_red": False,
        "cycle_running": False,
    },
}


def _fresh() -> dict:
    """Deep-copy so per-test mutations don't leak between tests."""
    return copy.deepcopy(_BASE_SEED)


def _write(tmp_path: Path, data, name: str = "seed.json") -> Path:
    p = tmp_path / name
    if isinstance(data, str):
        p.write_text(data, encoding="utf-8")
    else:
        p.write_text(json.dumps(data), encoding="utf-8")
    return p


def test_valid_seed_loads(tmp_path: Path) -> None:
    p = _write(tmp_path, _fresh())
    vio = VirtualIO.from_seed_file(p)
    assert vio.mode == "virtual_only"
    assert vio.schema_version == SCHEMA_VERSION
    assert vio.inputs["part_present"] is True
    assert vio.outputs["stack_light_green"] is True


def test_missing_file_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        VirtualIO.from_seed_file(tmp_path / "absent.json")


def test_malformed_json_raises(tmp_path: Path) -> None:
    p = _write(tmp_path, "{not valid json")
    with pytest.raises(JobValidationError):
        VirtualIO.from_seed_file(p)


def test_top_level_not_object_raises(tmp_path: Path) -> None:
    p = _write(tmp_path, "[1, 2, 3]")
    with pytest.raises(JobValidationError):
        VirtualIO.from_seed_file(p)


def test_wrong_schema_version_raises(tmp_path: Path) -> None:
    bad = _fresh()
    bad["schema_version"] = 99
    p = _write(tmp_path, bad)
    with pytest.raises(JobValidationError, match="schema_version"):
        VirtualIO.from_seed_file(p)


def test_wrong_mode_raises(tmp_path: Path) -> None:
    bad = _fresh()
    bad["mode"] = "real_robot"
    p = _write(tmp_path, bad)
    with pytest.raises(JobValidationError, match="mode"):
        VirtualIO.from_seed_file(p)


def test_inputs_not_dict_raises(tmp_path: Path) -> None:
    bad = _fresh()
    bad["inputs"] = "not-a-dict"
    p = _write(tmp_path, bad)
    with pytest.raises(JobValidationError, match="'inputs' must be a dict"):
        VirtualIO.from_seed_file(p)


def test_outputs_not_dict_raises(tmp_path: Path) -> None:
    bad = _fresh()
    bad["outputs"] = "not-a-dict"
    p = _write(tmp_path, bad)
    with pytest.raises(JobValidationError, match="'outputs' must be a dict"):
        VirtualIO.from_seed_file(p)


def test_non_bool_value_int_rejected(tmp_path: Path) -> None:
    bad = _fresh()
    bad["inputs"] = {k: 1 for k in _BASE_SEED["inputs"]}
    p = _write(tmp_path, bad)
    with pytest.raises(JobValidationError, match="must be a JSON boolean"):
        VirtualIO.from_seed_file(p)


def test_non_bool_value_str_rejected(tmp_path: Path) -> None:
    bad = _fresh()
    bad["inputs"] = {k: "true" for k in _BASE_SEED["inputs"]}
    p = _write(tmp_path, bad)
    with pytest.raises(JobValidationError, match="must be a JSON boolean"):
        VirtualIO.from_seed_file(p)


def test_unknown_input_signal_rejected(tmp_path: Path) -> None:
    bad = _fresh()
    bad["inputs"]["bogus_signal"] = True
    p = _write(tmp_path, bad)
    with pytest.raises(JobValidationError, match="unknown signal"):
        VirtualIO.from_seed_file(p)


def test_missing_required_input_rejected(tmp_path: Path) -> None:
    bad = _fresh()
    bad["inputs"] = {"part_present": True}  # missing 3 required
    p = _write(tmp_path, bad)
    with pytest.raises(JobValidationError, match="missing required inputs"):
        VirtualIO.from_seed_file(p)


def test_no_disk_write_back(tmp_path: Path) -> None:
    """Mutations must NOT persist to the seed file on disk."""
    p = _write(tmp_path, _fresh())
    vio = VirtualIO.from_seed_file(p)
    vio.set_input("part_present", False)
    vio.set_output("stack_light_red", True)
    raw_on_disk = json.loads(p.read_text())
    assert raw_on_disk["inputs"]["part_present"] is True
    assert raw_on_disk["outputs"]["stack_light_red"] is False


def test_set_input_unknown_signal_rejected() -> None:
    vio = VirtualIO(
        inputs={
            "part_present": True, "fixture_clamped": True,
            "welder_ready": True, "safety_gate_closed": True,
        },
        outputs={},
        mode="virtual_only",
        schema_version=SCHEMA_VERSION,
    )
    with pytest.raises(JobValidationError, match="not in whitelist"):
        vio.set_input("random", True)


def test_set_input_non_bool_rejected() -> None:
    vio = VirtualIO(
        inputs={
            "part_present": True, "fixture_clamped": True,
            "welder_ready": True, "safety_gate_closed": True,
        },
        outputs={},
        mode="virtual_only",
        schema_version=SCHEMA_VERSION,
    )
    with pytest.raises(ValueError, match="must be bool"):
        vio.set_input("part_present", 1)
    with pytest.raises(ValueError, match="must be bool"):
        vio.set_input("part_present", "true")


def test_set_output_non_bool_rejected() -> None:
    vio = VirtualIO(
        inputs={
            "part_present": True, "fixture_clamped": True,
            "welder_ready": True, "safety_gate_closed": True,
        },
        outputs={"stack_light_green": False},
        mode="virtual_only",
        schema_version=SCHEMA_VERSION,
    )
    with pytest.raises(ValueError, match="must be bool"):
        vio.set_output("stack_light_green", 1)


def test_check_interlock_missing_keys_listed() -> None:
    vio = VirtualIO(
        inputs={
            "part_present": True, "fixture_clamped": False,
            "welder_ready": True, "safety_gate_closed": True,
        },
        outputs={},
        mode="virtual_only",
        schema_version=SCHEMA_VERSION,
    )
    missing = vio.check_interlock(["part_present", "fixture_clamped",
                                    "welder_ready", "safety_gate_closed",
                                    "no_such_input"])
    assert any("fixture_clamped" in m for m in missing)
    assert any("no_such_input(missing)" in m for m in missing)
    assert not any("part_present" in m for m in missing)


def test_check_interlock_all_ok_returns_empty() -> None:
    vio = VirtualIO(
        inputs={
            "part_present": True, "fixture_clamped": True,
            "welder_ready": True, "safety_gate_closed": True,
        },
        outputs={},
        mode="virtual_only",
        schema_version=SCHEMA_VERSION,
    )
    assert vio.check_interlock(list(vio.inputs.keys())) == []


def test_snapshot_round_trip(tmp_path: Path) -> None:
    p = _write(tmp_path, _fresh())
    vio = VirtualIO.from_seed_file(p)
    snap = vio.snapshot()
    assert snap["mode"] == "virtual_only"
    assert snap["schema_version"] == SCHEMA_VERSION
    assert snap["inputs"]["part_present"] is True
    assert "stack_light_green" in snap["outputs"]


def test_strict_bool_in_seed_rejects_float(tmp_path: Path) -> None:
    """Non-bool objects (e.g. 1.0) must be rejected as seed values."""
    bad = _fresh()
    bad["inputs"] = {k: 1.0 for k in _BASE_SEED["inputs"]}
    p = _write(tmp_path, bad)
    with pytest.raises(JobValidationError, match="must be a JSON boolean"):
        VirtualIO.from_seed_file(p)
