#!/usr/bin/env python3
"""Create and validate a single-bowl arm placement plan.

This helper is deliberately ROS-free.  It resolves semantic positions from
the checked-in YAML table, builds the fixed local command, and validates plan
identity/source hashes.  It never starts a process or dispatches robot actions.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import re
import shlex
import sys
import uuid


ROOT = Path(__file__).resolve().parents[2]
ARM_CONFIG = ROOT / "config" / "robots" / "arm.yaml"
ZONES_CONFIG = ROOT / "config" / "robots" / "arm_place_zones.yaml"
SKILL = ROOT / ".opencode" / "skills" / "gat-arm-deployment" / "SKILL.md"
LOCAL_SCRIPT = "/home/rob/table_clean_ws/run_pipeline_place_execute.sh"
PROTOCOL_VERSION = "arm-runner-v1"
MAX_TEXT = 512
UUID_RE = re.compile(r"^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[1-5][0-9a-fA-F]{3}-[89abAB][0-9a-fA-F]{3}-[0-9a-fA-F]{12}$")
HEX64_RE = re.compile(r"^[0-9a-fA-F]{64}$")


class ArmPlanError(ValueError):
    """Raised when a plan crosses the fixed arm boundary incorrectly."""


def _fail(reason: str) -> None:
    raise ArmPlanError(f"invalid_arm_plan:{reason}")


def _sha256(path: Path) -> str:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError as exc:
        raise ArmPlanError(f"source_unreadable:{path}") from exc


def _canonical(value) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False)


def plan_hash(value: dict) -> str:
    copied = json.loads(json.dumps(value, ensure_ascii=False, sort_keys=True))
    model = copied.get("model")
    if isinstance(model, dict):
        model = dict(model)
        model.pop("plan_hash", None)
        copied["model"] = model
    return hashlib.sha256(_canonical(copied).encode("utf-8")).hexdigest()


def _scalar(text: str):
    text = text.strip()
    if not text:
        return ""
    if text in ("true", "True"):
        return True
    if text in ("false", "False"):
        return False
    if text in ("null", "Null", "~"):
        return None
    try:
        return ast.literal_eval(text)
    except (SyntaxError, ValueError):
        return text.strip("\"'")


def _read_arm_yaml(path: Path) -> dict:
    """Read the small, fixed schema used by arm_place_zones.yaml.

    A full YAML dependency is intentionally avoided; this parser accepts only
    the scalar/list forms present in the checked-in configuration.
    """
    zones: dict[str, dict] = {}
    derived: dict[str, dict] = {}
    section = None
    current = None
    interpolation = {"formula": "P(t) = (1-t)A + tB", "allowed_range": [0.0, 1.0]}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())
        if indent == 0 and stripped.endswith(":"):
            section = stripped[:-1]
            current = None
            continue
        if section == "zones" and indent == 2 and stripped.endswith(":"):
            current = stripped[:-1]
            zones[current] = {}
            continue
        if section == "derived_midpoints" and indent == 2 and stripped.endswith(":"):
            current = stripped[:-1]
            derived[current] = {}
            continue
        if section == "interpolation_rule" and indent == 2 and ":" in stripped:
            key, value = stripped.split(":", 1)
            interpolation[key.strip()] = _scalar(value)
            continue
        if current is None or ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        target = zones if section == "zones" else derived
        target[current][key.strip()] = _scalar(value)
    if not zones or not derived:
        _fail("zones_config")
    return {"zones": zones, "derived_midpoints": derived, "interpolation_rule": interpolation}


def _read_robot_yaml(path: Path) -> dict:
    values = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        stripped = raw.split("#", 1)[0].strip()
        if ":" in stripped:
            key, value = stripped.split(":", 1)
            values[key] = str(_scalar(value))
    if values.get("transport") != "local" or values.get("workspace") != "/home/rob/table_clean_ws":
        _fail("robot_config")
    if values.get("path") != LOCAL_SCRIPT:
        _fail("local_program")
    return {"transport": "local", "workspace": values["workspace"]}


def _finite_number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        _fail(name)
    return float(value)


def _resolve_target(target: str, config: dict) -> tuple[str, str, dict, dict]:
    if type(target) is not str or not target.strip() or len(target) > MAX_TEXT:
        _fail("target")
    target = target.strip()
    matches: list[tuple[str, str]] = []
    for name, entry in config["zones"].items():
        if target == name or target in entry.get("aliases", []):
            matches.append(("zone", name))
    for name, entry in config["derived_midpoints"].items():
        if target == name or target in entry.get("aliases", []):
            matches.append(("derived", name))
    if len(matches) != 1:
        _fail("target_unknown_or_ambiguous")
    kind, resolved = matches[0]
    if kind == "zone":
        source = config["zones"][resolved]
        xyz = {axis: _finite_number(source.get(axis), f"zone_{axis}") for axis in "xyz"}
        resolution = {"requested_position": target, "resolved_position": resolved,
                      "resolution_method": "zone_lookup"}
    else:
        source = config["derived_midpoints"][resolved]
        endpoints = source.get("from")
        if not isinstance(endpoints, list) or len(endpoints) != 2 or any(e not in config["zones"] for e in endpoints):
            _fail("derived_endpoints")
        a, b = (config["zones"][name] for name in endpoints)
        xyz = {axis: (_finite_number(a[axis], f"zone_{axis}") +
                      _finite_number(b[axis], f"zone_{axis}")) / 2.0 for axis in "xyz"}
        resolution = {"requested_position": target, "resolved_position": resolved,
                      "resolution_method": "derived_midpoint_interpolation",
                      "interpolation": {"from": endpoints, "formula": "(A + B) / 2"}}
    resolution.update({"coordinate_frame": "base", "target_kind": "release_position_base_tool0",
                       "target_xyz": {axis: round(xyz[axis], 6) for axis in "xyz"},
                       "command_precision_decimals": 6})
    return target, resolved, resolution, xyz


def _local_command(xyz: dict, mode: str) -> tuple[list[str], str]:
    args = ["--x", f"{xyz['x']:.6f}", "--y", f"{xyz['y']:.6f}", "--z", f"{xyz['z']:.6f}"]
    script = LOCAL_SCRIPT if mode == "execute" else LOCAL_SCRIPT.replace("_execute.sh", "_test.sh")
    return args, shlex.join(["bash", script, *args])


def create_plan(target: str, *, provider: str, model_name: str, mode: str = "preview") -> dict:
    if mode not in ("preview", "execute"):
        _fail("mode")
    config = _read_arm_yaml(ZONES_CONFIG)
    robot = _read_robot_yaml(ARM_CONFIG)
    requested, resolved, resolution, xyz = _resolve_target(target, config)
    args, local_command = _local_command(resolution["target_xyz"], mode)
    script = LOCAL_SCRIPT if mode == "execute" else LOCAL_SCRIPT.replace("_execute.sh", "_test.sh")
    plan = {
        "schema_version": 1,
        "plan_id": str(uuid.uuid4()),
        "protocol_version": PROTOCOL_VERSION,
        "task_type": "arm_bowl_place",
        "raw_user_request": f"让机械臂把碗放到{requested}",
        "robot": {"robot_id": "arm-01", "kind": "robot_arm",
                  "transport": "local", "workspace": robot["workspace"]},
        "target": {"object": "bowl", "position": resolved},
        "source": {"arm_config": str(ARM_CONFIG.relative_to(ROOT)),
                   "place_zones": str(ZONES_CONFIG.relative_to(ROOT)),
                   "skill": str(SKILL.relative_to(ROOT)),
                   "arm_config_sha256": _sha256(ARM_CONFIG),
                   "place_zones_sha256": _sha256(ZONES_CONFIG),
                   "skill_sha256": _sha256(SKILL)},
        "source_of_truth": {"arm_config": str(ARM_CONFIG.relative_to(ROOT)),
                            "place_zones": str(ZONES_CONFIG.relative_to(ROOT)),
                            "skill": str(SKILL.relative_to(ROOT))},
        "placement_resolution": resolution,
        "object_selection_assumption": {"requested_object": "碗",
                                         "execution_assumption": "scene_has_only_one_bowl"},
        "safety": {"require_user_confirmation_before_live_run": True,
                    "do_not_assume_feedback": True, "do_not_guess_ros_interfaces": True},
        "local_execution": {"parameterized_program": script,
                            "program_arguments": args,
                            "program_command": local_command},
        "preflight": {"protocol_version": PROTOCOL_VERSION,
                       "arm_config_sha256": _sha256(ARM_CONFIG),
                       "place_zones_sha256": _sha256(ZONES_CONFIG)},
        "mode": mode,
        "limits": {"max_attempts": 1, "deadline_s": 900},
        "execution_status": "awaiting_confirmation",
        "model": {"provider": provider, "model": model_name},
    }
    plan["model"]["plan_hash"] = plan_hash(plan)
    return plan


def validate_plan(plan: dict, *, expected_mode: str | None = None) -> dict:
    if type(plan) is not dict or plan.get("schema_version") != 1:
        _fail("schema_version")
    if not isinstance(plan.get("plan_id"), str) or not UUID_RE.fullmatch(plan["plan_id"]):
        _fail("plan_id")
    if plan.get("protocol_version") != PROTOCOL_VERSION or plan.get("task_type") != "arm_bowl_place":
        _fail("protocol")
    if plan.get("mode") not in ("preview", "execute"):
        _fail("mode")
    if expected_mode and plan["mode"] != expected_mode:
        _fail("mode_mismatch")
    target = plan.get("placement_resolution")
    if type(target) is not dict or target.get("coordinate_frame") != "base":
        _fail("placement_resolution")
    xyz = target.get("target_xyz")
    if type(xyz) is not dict or set(xyz) != set("xyz"):
        _fail("target_xyz")
    for axis in "xyz":
        _finite_number(xyz[axis], f"target_{axis}")
    source = plan.get("source")
    if type(source) is not dict:
        _fail("source")
    for name, path in (("arm_config_sha256", ARM_CONFIG), ("place_zones_sha256", ZONES_CONFIG), ("skill_sha256", SKILL)):
        if source.get(name) != _sha256(path):
            _fail("source_changed:" + name)
    _, _, derived, _ = _resolve_target(target.get("requested_position"), _read_arm_yaml(ZONES_CONFIG))
    if target != derived:
        _fail("target_differs_from_coordinate_table")
    if plan.get("safety") != {"require_user_confirmation_before_live_run": True,
                              "do_not_assume_feedback": True, "do_not_guess_ros_interfaces": True}:
        _fail("safety")
    model = plan.get("model")
    if type(model) is not dict or not isinstance(model.get("provider"), str) or not isinstance(model.get("model"), str):
        _fail("model")
    if not isinstance(model.get("plan_hash"), str) or not HEX64_RE.fullmatch(model["plan_hash"]):
        _fail("model.plan_hash")
    if model["plan_hash"].lower() != plan_hash(plan):
        _fail("model.plan_hash_mismatch")
    local = plan.get("local_execution")
    expected_script = LOCAL_SCRIPT if plan["mode"] == "execute" else LOCAL_SCRIPT.replace("_execute.sh", "_test.sh")
    if type(local) is not dict or local.get("parameterized_program") != expected_script:
        _fail("local_program")
    robot = plan.get("robot")
    configured_robot = _read_robot_yaml(ARM_CONFIG)
    if (type(robot) is not dict or robot.get("transport") != "local"
            or robot.get("workspace") != configured_robot["workspace"] or "ssh" in robot):
        _fail("robot_identity")
    expected_args, expected_command = _local_command(xyz, plan["mode"])
    if local.get("program_arguments") != expected_args:
        _fail("local_arguments")
    if local.get("program_command") != expected_command or "ssh_command" in local:
        _fail("program_command")
    return json.loads(json.dumps(plan, ensure_ascii=False))


def load_plan(path: Path, *, expected_mode: str | None = None) -> dict:
    try:
        plan = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ArmPlanError("invalid_arm_plan:file") from exc
    return validate_plan(plan, expected_mode=expected_mode)


def validate_result(plan: dict, result: dict) -> dict:
    """Check that an arm report belongs to this plan and fixed command."""
    if type(result) is not dict:
        _fail("result_object")
    if "plan_id" in result and result["plan_id"] != plan["plan_id"]:
        _fail("result_plan_id_mismatch")
    target = result.get("placement_resolution", {}).get("target_xyz")
    if target is not None and target != plan["placement_resolution"]["target_xyz"]:
        _fail("result_target_mismatch")
    deployment = result.get("deployment")
    if type(deployment) is not dict:
        _fail("result_deployment")
    local = plan["local_execution"]
    if deployment.get("program_command") != local["program_command"]:
        _fail("result_program_command_mismatch")
    if deployment.get("transport") != "local" or "ssh_command" in deployment:
        _fail("result_transport")
    if type(deployment.get("exit_code")) is not int:
        _fail("result_exit_code")
    return json.loads(json.dumps(result, ensure_ascii=False))


def load_result(path: Path) -> dict:
    try:
        result = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ArmPlanError("invalid_arm_result:file") from exc
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Create/validate an arm plan; never runs robot actions.")
    parser.add_argument("--create", action="store_true")
    parser.add_argument("--target")
    parser.add_argument("--output")
    parser.add_argument("--provider", default="openai")
    parser.add_argument("--model", default="selected-model")
    parser.add_argument("--mode", choices=("preview", "execute"), default="preview")
    parser.add_argument("--plan")
    parser.add_argument("--validate", action="store_true")
    parser.add_argument("--hash", action="store_true", dest="show_hash")
    parser.add_argument("--expected-mode", choices=("preview", "execute"))
    parser.add_argument("--result")
    parser.add_argument("--baseline-result")
    return parser


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.create:
            if not args.target or not args.output:
                _fail("create_arguments")
            plan = create_plan(args.target, provider=args.provider, model_name=args.model, mode=args.mode)
            output = Path(args.output)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(json.dumps({"plan": str(output), "plan_id": plan["plan_id"],
                              "mode": plan["mode"], "plan_hash": plan["model"]["plan_hash"]},
                             ensure_ascii=False, sort_keys=True))
            return 0
        if not args.plan:
            _fail("plan_required")
        plan = load_plan(Path(args.plan), expected_mode=args.expected_mode)
        if args.baseline_result and not args.result:
            _fail("baseline_requires_result")
        result = validate_result(plan, load_result(Path(args.result))) if args.result else None
        baseline = validate_result(plan, load_result(Path(args.baseline_result))) if args.baseline_result else None
        if result is not None and baseline is not None:
            for field in ("program_command", "transport"):
                if result["deployment"].get(field) != baseline["deployment"].get(field):
                    _fail("result_command_changed:" + field)
        if args.show_hash:
            print(plan_hash(plan))
        elif args.validate:
            print(json.dumps({"valid": True, "plan_id": plan["plan_id"],
                              "mode": plan["mode"], "plan_hash": plan_hash(plan),
                              **({"result_valid": True} if result is not None else {})},
                             ensure_ascii=False, sort_keys=True))
        else:
            print(json.dumps(plan, ensure_ascii=False, indent=2))
        return 0
    except ArmPlanError as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
