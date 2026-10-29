"""Hardware-free checks for the nested gat-table policy boundary."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "generated" / "table"))
import table_task  # noqa: E402


class TableTaskTests(unittest.TestCase):
    def setUp(self):
        self.policy_path = ROOT / "generated" / "table" / "task_policy.json"
        self.policy = table_task.load_policy(self.policy_path, require_deployment_metadata=True)

    def test_sample_policy_is_stable_and_preview_only(self):
        self.assertEqual(self.policy["mode"], "preview")
        self.assertEqual(self.policy["model"]["plan_hash"], table_task.policy_hash(self.policy))
        self.assertEqual(self.policy["selector"]["kind"], "all_bowls")

    def test_geometry_and_color_fields_are_rejected(self):
        for field in ("x", "pose", "trajectory", "command"):
            value = dict(self.policy, **{field: 1})
            with self.assertRaisesRegex(table_task.PolicyError, "fields"):
                table_task.validate_policy(value)
        value = dict(self.policy, selector={"kind": "green"})
        with self.assertRaisesRegex(table_task.PolicyError, "selector_unsupported"):
            table_task.validate_policy(value)

    def test_stale_hash_and_duplicate_json_are_rejected(self):
        value = json.loads(self.policy_path.read_text(encoding="utf-8"))
        value["model"]["plan_hash"] = "f" * 64
        with self.assertRaisesRegex(table_task.PolicyError, "plan_hash_mismatch"):
            table_task.validate_policy(value)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.json"
            path.write_text('{"schema_version":1,"schema_version":1}', encoding="utf-8")
            with self.assertRaisesRegex(table_task.PolicyError, "duplicate_key"):
                table_task.load_policy(path)

    def test_arm_and_table_artifacts_are_separate(self):
        self.assertTrue((ROOT / "generated" / "arm" / "task_plan.json").is_file())
        self.assertFalse((ROOT / "generated" / "arm" / "task_policy.json").exists())
        self.assertTrue((ROOT / "reports" / "table_last_run.json").is_file())

    def test_result_preflight_is_bound_to_policy_and_baseline(self):
        preflight = {
            "runner_protocol_version": "table-runner-v1",
            "runner_revision": "table-clean-runner-v2",
            "workspace_revision": self.policy["workspace_revision"],
            "config_digest": "a" * 64,
            "model_digest": "b" * 64,
            "policy_hash": table_task.policy_hash(self.policy),
        }
        baseline = {"plan_id": self.policy["plan_id"], "preflight": preflight}
        result = {"plan_id": self.policy["plan_id"], "preflight": dict(preflight)}
        self.assertTrue(table_task.validate_runner_result(self.policy, result, baseline=baseline))
        changed = {"plan_id": self.policy["plan_id"],
                   "preflight": dict(preflight, config_digest="c" * 64)}
        with self.assertRaisesRegex(table_task.PolicyError, "result_preflight_changed:config_digest"):
            table_task.validate_runner_result(self.policy, changed, baseline=baseline)


if __name__ == "__main__":
    unittest.main()
