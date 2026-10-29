#!/usr/bin/env python3
"""Validate and display a table-cleaning policy without ROS or network access.

This file is intentionally a small, deterministic boundary tool.  It never
executes a policy, starts a process, opens a socket, publishes ROS messages, or
accepts geometry/commands from an Agent.
"""
from __future__ import annotations

import argparse
import importlib.util
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from numbers import Real

_round_spec = importlib.util.spec_from_file_location('gat_table_round_protocol', Path(__file__).with_name('table_round_protocol.py'))
_round = importlib.util.module_from_spec(_round_spec)
_round_spec.loader.exec_module(_round)
validate_round_request = _round.validate_request
validate_round_decision = _round.validate_decision
validate_round_bundle = _round.validate_bundle

SCHEMA_VERSION = 1
TASK_TYPE = "clear_table"
RUNNER_PROTOCOL_VERSION = "table-runner-v1"
RUNNER_REVISION = "table-clean-runner-v2"
MAX_TEXT = 256
MAX_ITEMS = 256
UUID_RE = re.compile(r"^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[1-5][0-9a-fA-F]{3}-[89abAB][0-9a-fA-F]{3}-[0-9a-fA-F]{12}$")
HEX64_RE = re.compile(r"^[0-9a-fA-F]{64}$")


class PolicyError(ValueError):
    """Raised when an Agent policy crosses the table boundary incorrectly."""


def _fail(reason: str) -> None:
    raise PolicyError(f"invalid_table_policy:{reason}")


def _keys(value, required, optional=()):
    if type(value) is not dict:
        _fail("object")
    keys = set(value)
    if not set(required) <= keys or keys - set(required) - set(optional):
        _fail("fields")


def _text(value, name: str, *, empty: bool = False) -> str:
    if type(value) is not str or len(value) > MAX_TEXT:
        _fail(name)
    if not empty and (not value or value != value.strip() or any(char.isspace() for char in value)):
        _fail(name)
    return value


def _positive_int(value, name: str, *, minimum: int = 1, maximum: int = 1000000) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        _fail(name)
    return value


def _finite(value, name: str, *, positive: bool = False, maximum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        _fail(name)
    if positive and value <= 0:
        _fail(name)
    if maximum is not None and value > maximum:
        _fail(name)
    return float(value)


def _json_safe(value, depth: int = 0) -> None:
    if depth > 10:
        _fail("nesting")
    if value is None or type(value) is bool or type(value) is str:
        return
    if type(value) in (int, float):
        if isinstance(value, float) and not math.isfinite(value):
            _fail("nonfinite")
        return
    if type(value) is list:
        if len(value) > MAX_ITEMS:
            _fail("list_limit")
        for item in value:
            _json_safe(item, depth + 1)
        return
    if type(value) is dict:
        if len(value) > MAX_ITEMS:
            _fail("object_limit")
        for key, item in value.items():
            if type(key) is not str:
                _fail("json_key")
            _json_safe(item, depth + 1)
        return
    _fail("json_value")


def _copy(value):
    _json_safe(value)
    try:
        return json.loads(json.dumps(value, ensure_ascii=False, allow_nan=False,
                                     sort_keys=True, separators=(",", ":")))
    except (TypeError, ValueError, OverflowError) as exc:
        raise PolicyError("invalid_table_policy:json") from exc


def validate_policy(value, *, expected_mode=None, require_deployment_metadata=False):
    """Validate high-level semantic intent; no geometry or commands are accepted."""
    _keys(value, {"schema_version", "plan_id", "task_type", "selector", "priority", "limits", "mode"},
          {"runner_protocol_version", "workspace_revision", "model", "reason"})
    if type(value["schema_version"]) is not int or value["schema_version"] != SCHEMA_VERSION:
        _fail("schema_version")
    if type(value["plan_id"]) is not str or not UUID_RE.fullmatch(value["plan_id"]):
        _fail("plan_id")
    if value["task_type"] != TASK_TYPE:
        _fail("task_type")
    selector = value["selector"]
    _keys(selector, {"kind"}, {"attributes"})
    if selector["kind"] != "all_bowls":
        _fail("selector_unsupported")
    if "attributes" in selector and (type(selector["attributes"]) is not dict or selector["attributes"]):
        _fail("selector_attributes_unsupported")
    priority = value["priority"]
    _keys(priority, {"order", "regions"})
    if priority["order"] not in ("largest_first", "distance_first"):
        _fail("priority_order")
    if type(priority["regions"]) is not list or priority["regions"]:
        _fail("priority_regions_unsupported")
    limits = value["limits"]
    _keys(limits, {"max_observation_rounds", "max_replans", "deadline_s"},
          {"max_reobservations", "max_completed_objects"})
    _positive_int(limits["max_observation_rounds"], "max_observation_rounds", maximum=10000)
    _positive_int(limits["max_replans"], "max_replans", maximum=10000)
    _finite(limits["deadline_s"], "deadline_s", positive=True, maximum=86400)
    if "max_reobservations" in limits:
        _positive_int(limits["max_reobservations"], "max_reobservations", minimum=0, maximum=10000)
    if "max_completed_objects" in limits:
        _positive_int(limits["max_completed_objects"], "max_completed_objects", maximum=10000)
    if value["mode"] not in ("preview", "execute"):
        _fail("mode")
    if expected_mode is not None and value["mode"] != expected_mode:
        _fail("mode_mismatch")
    if "runner_protocol_version" in value:
        if value["runner_protocol_version"] != RUNNER_PROTOCOL_VERSION:
            _fail("runner_protocol_version")
    elif require_deployment_metadata:
        _fail("runner_protocol_version_missing")
    if "workspace_revision" in value:
        _text(value["workspace_revision"], "workspace_revision")
        if "/" in value["workspace_revision"] or "\\" in value["workspace_revision"]:
            _fail("workspace_revision_path")
    elif require_deployment_metadata:
        _fail("workspace_revision_missing")
    if "model" in value:
        model = value["model"]
        _keys(model, {"provider", "model", "plan_hash"})
        _text(model["provider"], "model.provider")
        _text(model["model"], "model.model")
        if type(model["plan_hash"]) is not str or not HEX64_RE.fullmatch(model["plan_hash"]):
            _fail("model.plan_hash")
        # The hash excludes this self-referential field.  Checking it here
        # prevents a stale policy from being relabeled as a fresh plan.
        if model["plan_hash"].lower() != policy_hash(value):
            _fail("model.plan_hash_mismatch")
    elif require_deployment_metadata:
        _fail("model_missing")
    if "reason" in value and (type(value["reason"]) is not str or len(value["reason"]) > MAX_TEXT):
        _fail("reason")
    return _copy(value)


def canonical_json(value) -> str:
    return json.dumps(_copy(value), ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False)


def policy_hash(value) -> str:
    """Hash the policy while omitting the self-referential model.plan_hash field."""
    copied = _copy(value)
    model = copied.get("model")
    if isinstance(model, dict) and "plan_hash" in model:
        model = dict(model)
        model.pop("plan_hash")
        copied["model"] = model
    return hashlib.sha256(canonical_json(copied).encode("utf-8")).hexdigest()


def _reject_duplicate_keys(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            _fail("duplicate_key")
        value[key] = item
    return value


def load_policy(path: str | Path, *, expected_mode=None, require_deployment_metadata=False):
    policy_path = Path(path)
    if not policy_path.is_file():
        raise PolicyError("invalid_table_policy:file")
    try:
        with policy_path.open("r", encoding="utf-8") as stream:
            value = json.load(stream, object_pairs_hook=_reject_duplicate_keys)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise PolicyError("invalid_table_policy:json_file") from exc
    return validate_policy(value, expected_mode=expected_mode,
                           require_deployment_metadata=require_deployment_metadata)


def validate_runner_result(policy, value, baseline=None):
    """Validate the local result envelope returned by table_clean_ws.

    This checks identity and preflight metadata only.  The runner remains the
    authority for motion status and observation details; this helper prevents a
    result from another workspace or policy being attached to the plan.
    """
    if type(value) is not dict:
        _fail("result_object")
    if value.get("plan_id") != policy["plan_id"]:
        _fail("result_plan_id_mismatch")
    preflight = value.get("preflight")
    if type(preflight) is not dict:
        _fail("result_preflight")
    required = {"runner_protocol_version", "runner_revision", "workspace_revision",
                "config_digest", "model_digest"}
    if not required <= set(preflight):
        _fail("result_preflight_fields")
    if preflight["runner_protocol_version"] != RUNNER_PROTOCOL_VERSION:
        _fail("result_protocol_mismatch")
    if preflight["runner_revision"] != RUNNER_REVISION:
        _fail("result_runner_revision_mismatch")
    if preflight["workspace_revision"] != policy.get("workspace_revision"):
        _fail("result_workspace_revision_mismatch")
    for name in ("config_digest", "model_digest"):
        if type(preflight[name]) is not str or not HEX64_RE.fullmatch(preflight[name]):
            _fail("result_" + name)
    if preflight.get("policy_hash") != policy_hash(policy):
        _fail("result_policy_hash_mismatch")
    if baseline is not None:
        baseline_preflight = baseline.get("preflight") if type(baseline) is dict else None
        if type(baseline_preflight) is not dict:
            _fail("baseline_preflight")
        for name in ("runner_protocol_version", "runner_revision", "workspace_revision",
                     "config_digest", "model_digest"):
            if preflight.get(name) != baseline_preflight.get(name):
                _fail("result_preflight_changed:" + name)
    return True


def validate_preview_baseline(policy, baseline_policy, baseline):
    """Bind a new execute policy to an actual, successful online preview."""
    validate_policy(policy, expected_mode="execute", require_deployment_metadata=True)
    validate_policy(baseline_policy, expected_mode="preview", require_deployment_metadata=True)
    if policy["plan_id"] == baseline_policy["plan_id"]:
        _fail("baseline_requires_new_plan_id")
    for key in ("task_type", "selector", "priority", "limits",
                "runner_protocol_version", "workspace_revision"):
        if policy[key] != baseline_policy[key]:
            _fail("baseline_policy_changed:" + key)
    validate_runner_result(baseline_policy, baseline)
    if (baseline.get("passed") is not True or baseline.get("mode") != "plan_only"
            or baseline.get("outcome") != "planned"
            or baseline.get("selection") != "clear_table"
            or baseline.get("hardware_motion_requested") is not False
            or baseline.get("simulated", False)):
        _fail("baseline_requires_successful_online_preview")
    return baseline["preflight"]


def load_result(path: str | Path):
    result_path = Path(path)
    if not result_path.is_file():
        raise PolicyError("invalid_table_policy:result_file")
    try:
        with result_path.open("r", encoding="utf-8") as stream:
            return json.load(stream, object_pairs_hook=_reject_duplicate_keys)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise PolicyError("invalid_table_policy:result_json") from exc


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate/display a table task policy; never runs robot actions.")
    parser.add_argument("--task-policy", required=True, help="JSON policy file")
    parser.add_argument("--validate", action="store_true", help="validate the policy")
    parser.add_argument("--hash", action="store_true", dest="show_hash", help="print canonical policy hash")
    parser.add_argument("--print", action="store_true", dest="show_policy", help="print canonical policy JSON")
    parser.add_argument("--expected-mode", choices=("preview", "execute"))
    parser.add_argument("--require-deployment-metadata", action="store_true")
    parser.add_argument("--result", help="optional runner result.json to validate against the policy")
    parser.add_argument("--baseline-result", help="optional earlier result whose preflight must match --result")
    parser.add_argument("--baseline-policy", help="preview policy belonging to --baseline-result, for a new execute plan")
    return parser


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        policy = load_policy(args.task_policy, expected_mode=args.expected_mode,
                             require_deployment_metadata=args.require_deployment_metadata)
        if args.baseline_policy and not args.baseline_result:
            _fail("baseline_policy_requires_result")
        if args.baseline_result and not args.result and not args.baseline_policy:
            _fail("baseline_requires_result")
        baseline = load_result(args.baseline_result) if args.baseline_result else None
        if baseline is not None:
            if args.baseline_policy:
                baseline_policy = load_policy(args.baseline_policy, require_deployment_metadata=True)
                validate_preview_baseline(policy, baseline_policy, baseline)
            else:
                validate_runner_result(policy, baseline)
        result = load_result(args.result) if args.result else None
        if result is not None:
            validate_runner_result(policy, result, baseline=baseline)
    except PolicyError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    # Default output is a compact validation summary; all output is derived from
    # the validated file and does not execute, mutate, or upload anything.
    if args.show_policy:
        print(canonical_json(policy))
    if args.show_hash:
        print(policy_hash(policy))
    if not args.show_policy and not args.show_hash:
        summary = {"valid": True, "plan_id": policy["plan_id"],
                   "mode": policy["mode"], "policy_hash": policy_hash(policy)}
        if args.result:
            summary["result_valid"] = True
        print(json.dumps(summary,
                         ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
