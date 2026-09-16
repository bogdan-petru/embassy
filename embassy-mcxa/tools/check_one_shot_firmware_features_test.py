"""Fail-closed controls for the firmware graph gate; no Cargo invocation."""

import argparse
from contextlib import redirect_stderr
import io
import subprocess
import unittest
from unittest.mock import patch

import check_one_shot_firmware_features as gate


class GraphPolicyTests(unittest.TestCase):
    def test_empty_default_feature_and_unrelated_packages(self):
        rows = gate.check_graph("app v0.1.0 (/workspace/app)|default\nsafe-mmio v0.3.1|\n")
        self.assertEqual(rows, ["safe-mmio v0.3.1|"])
        self.assertEqual(gate.check_graph("safe-mmio v0.3.1|default"), ["safe-mmio v0.3.1|default"])

    def test_allowed_duplicate_rows_and_crlf(self):
        self.assertEqual(len(gate.check_graph("safe-mmio v0.3.1|\r\nsafe-mmio v0.3.1| (*)\r\n")), 2)

    def test_missing_package(self):
        for graph in ("", "app v0.1.0|default", "safe-mmio-helper v0.3.1|"):
            with self.subTest(graph=graph), self.assertRaisesRegex(gate.PolicyError, "missing"):
                gate.check_graph(graph)

    def test_unexpected_version_even_alongside_expected_version(self):
        for graph in ("safe-mmio v0.4.0|", "safe-mmio v0.3.1|\nsafe-mmio v0.4.0| (*)"):
            with self.subTest(graph=graph), self.assertRaisesRegex(gate.PolicyError, "Unexpected"):
                gate.check_graph(graph)

    def test_same_version_source_override_needs_review(self):
        with self.assertRaisesRegex(gate.PolicyError, "Unexpected"):
            gate.check_graph("safe-mmio v0.3.1 (/workspace/replacement)|")

    def test_forbidden_feature_in_every_position_and_dedup_row(self):
        for features in ("custom-mmio", "custom-mmio,default", "default,custom-mmio", "custom-mmio (*)"):
            with self.subTest(features=features), self.assertRaisesRegex(gate.PolicyError, "custom-mmio enabled"):
                gate.check_graph("safe-mmio v0.3.1|\nsafe-mmio v0.3.1|" + features)

    def test_malformed_package_row_is_not_silently_skipped(self):
        for malformed in ("safe-mmio", "safe-mmio|", "safe-mmio v0.3.1", "safe-mmio v0.3.1|default|custom-mmio"):
            with self.subTest(row=malformed), self.assertRaises(gate.PolicyError):
                gate.check_graph("safe-mmio v0.3.1|\n" + malformed)

    def test_malformed_feature_and_dedup_syntax_fail_closed(self):
        for features in ("default custom-mmio", "default,", "default (*) custom-mmio", "(*)", "custom-mmio(**)"):
            with self.subTest(features=features), self.assertRaisesRegex(gate.PolicyError, "Malformed"):
                gate.check_graph("safe-mmio v0.3.1|" + features)


class CargoInvocationTests(unittest.TestCase):
    def setUp(self):
        self.args = argparse.Namespace(
            manifest_path="examples/mcxa5xx/Cargo.toml", features="i2c-one-shot",
            target="thumbv8m.main-none-eabihf", no_default_features=False,
        )

    def test_command_matches_firmware_features_and_excludes_dev(self):
        command = gate.cargo_command(self.args)
        self.assertEqual(command[:2], ["cargo", "tree"])
        for option, value in (
            ("--manifest-path", self.args.manifest_path), ("--features", "i2c-one-shot"),
            ("--target", self.args.target), ("-e", "normal,build"),
            ("--prefix", "none"), ("--format", "{p}|{f}"), ("--color", "never"),
        ):
            self.assertEqual(command[command.index(option) + 1], value)
        self.assertIn("--locked", command)
        self.assertNotIn("--no-default-features", command)

    def test_explicit_hal_configuration_preserves_no_defaults(self):
        self.args.manifest_path = "embassy-mcxa/Cargo.toml"
        self.args.features = "mcxa2xx,i2c-one-shot,defmt"
        self.args.no_default_features = True
        command = gate.cargo_command(self.args)
        self.assertIn("--no-default-features", command)
        self.assertEqual(command[command.index("--features") + 1], self.args.features)

    def test_cargo_failure_cannot_pass_with_partial_valid_stdout(self):
        result = subprocess.CompletedProcess([], 101, "safe-mmio v0.3.1|", "resolver failed\n")
        with patch.object(gate.subprocess, "run", return_value=result), redirect_stderr(io.StringIO()):
            with self.assertRaisesRegex(gate.PolicyError, r"cargo tree failed \(exit 101\)"):
                gate.read_graph(["cargo", "tree"])

    def test_cargo_runs_from_repo_root_without_shell(self):
        command = gate.cargo_command(self.args)
        result = subprocess.CompletedProcess(command, 0, "safe-mmio v0.3.1|", "")
        with patch.object(gate.subprocess, "run", return_value=result) as run:
            self.assertEqual(gate.read_graph(command), "safe-mmio v0.3.1|")
            run.assert_called_once_with(command, cwd=gate.REPO_ROOT, text=True, capture_output=True, check=False)


if __name__ == "__main__":
    unittest.main()
