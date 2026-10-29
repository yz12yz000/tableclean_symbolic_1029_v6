"""Deterministic preflight metadata shared by the table runner and Agent.

The values identify the runner contract and the exact effective configuration
and model files selected for a run.  This module deliberately has no ROS or
third-party imports so preview and offline validation can use the same code.
"""

from dataclasses import asdict, is_dataclass
import hashlib
import json
from pathlib import Path

from table_agent_protocol import (RUNNER_PROTOCOL_VERSION, RUNNER_REVISION,
                                  WORKSPACE_REVISION, policy_hash)


def _plain(value):
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return value


def _json_bytes(value):
    return json.dumps(_plain(value), ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode("utf-8")


def _json_digest(value):
    return hashlib.sha256(_json_bytes(value)).hexdigest()


def _model_path(root, value):
    if not value:
        return None
    path = Path(str(value)).expanduser()
    if path.is_absolute():
        return path
    return Path(root) / "src" / "yolo_vision" / path


def _file_digest(root, paths):
    """Hash model files plus stable missing-path markers.

    A missing model is still represented in the digest, so a preflight result
    cannot be mistaken for one made with a different model.  The actual model
    loader remains responsible for rejecting a missing model before execution.
    """
    entries = []
    for raw in paths:
        path = Path(raw) if raw is not None else None
        if path is None:
            entries.append(("<none>", "<none>"))
            continue
        try:
            label = path.resolve().relative_to(Path(root).resolve()).as_posix()
        except ValueError:
            label = str(path.resolve())
        if path.is_file():
            entries.append((label, hashlib.sha256(path.read_bytes()).hexdigest()))
        elif path.is_dir():
            found = False
            for child in sorted(item for item in path.rglob("*") if item.is_file()):
                found = True
                try:
                    child_label = child.resolve().relative_to(Path(root).resolve()).as_posix()
                except ValueError:
                    child_label = str(child.resolve())
                entries.append((child_label, hashlib.sha256(child.read_bytes()).hexdigest()))
            if not found:
                entries.append((label, "<empty>"))
        else:
            entries.append((label, "<missing>"))
    return _json_digest(entries)


def build_preflight(*, root, vision=None, rim=None, grasp=None, tray=None,
                    policy=None):
    """Return stable metadata for a particular effective runner configuration."""
    root = Path(root).resolve()
    config = dict(vision=_plain(vision), rim=_plain(rim), grasp=_plain(grasp),
                  tray=_plain(tray))
    # Preview and execution must identify the same motion parameters.  The
    # plan-only switch is checked independently by the runner, not a change
    # to the calibration or trajectory configuration.
    if isinstance(config['grasp'], dict):
        config['grasp'] = dict(config['grasp'])
        config['grasp'].pop('plan_only', None)
    model_paths = []
    if vision is not None:
        model_paths.append(_model_path(root, getattr(vision, "model_path", None)))
    if isinstance(tray, dict) and getattr(vision, 'scene_classes', None) is None:
        yolo = tray.get("yolo") or {}
        if isinstance(yolo, dict):
            model_paths.append(_model_path(root, yolo.get("model_path")))
    metadata = {
        "runner_protocol_version": RUNNER_PROTOCOL_VERSION,
        "runner_revision": RUNNER_REVISION,
        "workspace_revision": WORKSPACE_REVISION,
        "config_digest": _json_digest(config),
        "model_digest": _file_digest(root, model_paths),
    }
    if isinstance(policy, dict):
        metadata["policy_hash"] = policy_hash(policy)
    return metadata
