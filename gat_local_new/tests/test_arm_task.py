"""Hardware-free checks for the single-bowl arm plan boundary."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "generated" / "arm"))
import arm_task  # noqa: E402


class ArmTaskTests(unittest.TestCase):
    def test_create_right_side_uses_coordinate_table_and_fixed_runner(self):
        plan = arm_task.create_plan("右侧", provider="openai", model_name="test-model")
        self.assertEqual(plan["placement_resolution"]["target_xyz"],
                         {"x": -0.785412, "y": 0.121427, "z": 0.176408})
        self.assertEqual(plan["local_execution"]["program_arguments"],
                         ["--x", "-0.785412", "--y", "0.121427", "--z", "0.176408"])
        self.assertEqual(plan["model"]["plan_hash"], arm_task.plan_hash(plan))
        self.assertTrue(arm_task.validate_plan(plan)["safety"]["require_user_confirmation_before_live_run"])

    def test_alias_and_unknown_target(self):
        plan = arm_task.create_plan("桌子右侧", provider="openai", model_name="test-model")
        self.assertEqual(plan["target"]["position"], "右侧")
        with self.assertRaisesRegex(arm_task.ArmPlanError, "target_unknown_or_ambiguous"):
            arm_task.create_plan("右侧; rm -rf /", provider="openai", model_name="test-model")

    def test_execute_plan_and_current_workspace(self):
        plan = arm_task.create_plan("右侧", provider="openai", model_name="test-model", mode="execute")
        self.assertEqual(arm_task.validate_plan(plan, expected_mode="execute")["mode"], "execute")
        self.assertEqual(plan['local_execution']['parameterized_program'],
                         '/home/rob/table_clean_ws/run_pipeline_place_execute.sh')

    def test_coordinates_cannot_be_relabelled_with_new_hash(self):
        plan = arm_task.create_plan("右侧", provider="openai", model_name="test-model")
        plan["placement_resolution"]["target_xyz"]["z"] += .05
        plan["model"]["plan_hash"] = arm_task.plan_hash(plan)
        with self.assertRaisesRegex(arm_task.ArmPlanError, "target_differs_from_coordinate_table"):
            arm_task.validate_plan(plan)

    def test_stale_coordinate_source_is_rejected(self):
        plan = arm_task.create_plan("右侧", provider="openai", model_name="test-model")
        plan["source"]["place_zones_sha256"] = "0" * 64
        with self.assertRaisesRegex(arm_task.ArmPlanError, "source_changed:place_zones_sha256"):
            arm_task.validate_plan(plan)

    def test_cli_round_trip_and_tamper_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task_plan.json"
            plan = arm_task.create_plan("右侧", provider="openai", model_name="test-model")
            path.write_text(json.dumps(plan, ensure_ascii=False), encoding="utf-8")
            loaded = arm_task.load_plan(path)
            self.assertEqual(loaded["plan_id"], plan["plan_id"])
            plan["local_execution"]["program_arguments"][1] = "0.0"
            plan["model"]["plan_hash"] = arm_task.plan_hash(plan)
            path.write_text(json.dumps(plan, ensure_ascii=False), encoding="utf-8")
            with self.assertRaisesRegex(arm_task.ArmPlanError, "local_arguments"):
                arm_task.load_plan(path)

    def test_tampered_local_command_is_rejected(self):
        plan = arm_task.create_plan("右侧", provider="openai", model_name="test-model")
        plan["local_execution"]["program_command"] += " --unexpected"
        plan["model"]["plan_hash"] = arm_task.plan_hash(plan)
        with self.assertRaisesRegex(arm_task.ArmPlanError, "program_command"):
            arm_task.validate_plan(plan)

    def test_result_is_bound_to_plan_and_command(self):
        plan = arm_task.create_plan("右侧", provider="openai", model_name="test-model")
        result = {"plan_id": plan["plan_id"],
                  "placement_resolution": plan["placement_resolution"],
                  "deployment": {"program_command": plan["local_execution"]["program_command"],
                                 "transport": "local",
                                 "exit_code": 0}}
        self.assertTrue(arm_task.validate_result(plan, result))
        result["plan_id"] = "00000000-0000-4000-8000-000000000000"
        with self.assertRaisesRegex(arm_task.ArmPlanError, "result_plan_id_mismatch"):
            arm_task.validate_result(plan, result)


if __name__ == "__main__":
    unittest.main()
