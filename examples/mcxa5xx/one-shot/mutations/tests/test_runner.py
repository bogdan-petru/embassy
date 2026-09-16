"""Host-only runner checks; generated fixture commits never touch Embassy."""

from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import runner


def compiler_case():
    return {
        "id": "fixture-compiler", "kind": "compiler", "source": "src/input.rs", "cwd": ".",
        "anchor": "let value = good;", "replacement": "let value = bad;",
        "guarantee": "Fixture rejection must identify the changed token.",
        "command": [sys.executable, "driver.py", "--message-format=json"],
        "expected": {"code": "E0616", "message_contains": "field `bad` is private", "span": {"text": "bad"}},
        "evidence": ["Generated host fixture; no hardware evidence."],
    }


def result(stdout="", stderr="", returncode=1, timed_out=False):
    return {"stdout": stdout, "stderr": stderr, "returncode": returncode, "timed_out": timed_out}


def compiler_output(span, code="E0616", file_name="src/input.rs", primary=True):
    return json.dumps({"reason": "compiler-message", "message": {
        "level": "error", "code": {"code": code}, "message": "field `bad` is private",
        "spans": [{"file_name": file_name, "is_primary": primary, **span}],
    }})


class MatchingTests(unittest.TestCase):
    def setUp(self):
        self.case = compiler_case()
        self.tree = Path(tempfile.gettempdir()).resolve() / "mutation-matching-fixture"
        self.changed, self.span = runner.mutate("let value = good;\n", self.case)

    def check(self, actual, case=None, span=None):
        runner.check_mutant(case or self.case, actual, span or self.span, self.tree, self.tree)

    def test_exact_diagnostic_and_primary_span_are_accepted(self):
        self.check(result(compiler_output(self.span)))

    def test_named_unsafe_lint_still_requires_exact_diagnostic(self):
        case = deepcopy(self.case)
        case["expected"]["code"] = "unsafe_code"
        self.check(result(compiler_output(self.span, code="unsafe_code")), case=case)
        for code in ("unused_unsafe", "dead_code", "E0133"):
            with self.subTest(code=code), self.assertRaises(runner.Rejected):
                self.check(result(compiler_output(self.span, code=code)), case=case)

    def test_unrelated_error_path_and_span_are_rejected(self):
        wrong = [
            compiler_output(self.span, code="E0308"),
            compiler_output(self.span, file_name="src/other.rs"),
            compiler_output({"byte_start": 0, "byte_end": 3}),
            compiler_output(self.span, primary=False),
            compiler_output(self.span) + "\n" + compiler_output(self.span, code="E0308"),
            compiler_output(self.span) + "\n" + compiler_output(self.span),
        ]
        for output in wrong:
            with self.subTest(output=output), self.assertRaises(runner.Rejected):
                self.check(result(output))

    def test_link_failure_and_generic_failure_do_not_pass(self):
        for output in ("", "undefined reference to __safe_mmio_read", "error: could not compile"):
            with self.subTest(output=output), self.assertRaises(runner.Rejected):
                self.check(result(output))

    def test_surviving_and_timeout_mutations_do_not_pass(self):
        for actual in (result(returncode=0), result(compiler_output(self.span), timed_out=True)):
            with self.assertRaises(runner.Rejected):
                self.check(actual)

    def test_baseline_must_succeed(self):
        with self.assertRaises(runner.Rejected):
            runner.check_baseline(self.case, result())
        with self.assertRaises(runner.Rejected):
            runner.check_baseline(self.case, result(compiler_output(self.span), returncode=0))

    def test_unique_nonempty_anchor_and_real_change_are_required(self):
        for source in ("no match", "let value = good; let value = good;"):
            with self.assertRaises(runner.Rejected):
                runner.mutate(source, self.case)
        case = deepcopy(self.case)
        case["replacement"] = case["anchor"]
        with self.assertRaises(runner.Rejected):
            runner.mutate(case["anchor"], case)

    def test_crlf_and_utf8_offsets_are_exact(self):
        case = deepcopy(self.case)
        case.update(anchor="good;\nnext;", replacement="bad;\nnext;")
        changed, span = runner.mutate("// café\r\ngood;\r\nnext;\r\n", case)
        self.assertEqual(changed, "// café\r\nbad;\r\nnext;\r\n")
        self.assertEqual(changed.encode()[span["byte_start"]:span["byte_end"]], b"bad")

    def test_repeated_span_requires_explicit_occurrence(self):
        case = deepcopy(self.case)
        case["replacement"] = "bad; bad;"
        with self.assertRaises(runner.Rejected):
            runner.mutate(case["anchor"], case)
        case["expected"]["span"]["occurrence"] = 2
        _, span = runner.mutate(case["anchor"], case)
        self.assertEqual(span, {"byte_start": 5, "byte_end": 8})

    def test_behavior_requires_exact_test_and_assertion_without_compile_error(self):
        case = deepcopy(self.case)
        case.update(kind="behavior", expected={"test": "effects::ordered_cleanup", "assertion": "QUIESCE_BEFORE_RESET"})
        good = "running 1 test\ntest effects::ordered_cleanup ... FAILED\n"
        self.check(result(good, "panic: QUIESCE_BEFORE_RESET"), case=case)
        bad = [
            result("running 0 tests\n", "QUIESCE_BEFORE_RESET"),
            result(good.replace("ordered_cleanup", "another_test"), "QUIESCE_BEFORE_RESET"),
            result(good, "unrelated panic"),
            result(good + compiler_output(self.span), "QUIESCE_BEFORE_RESET"),
            result(good.replace("running 1 test", "running 2 tests"), "QUIESCE_BEFORE_RESET"),
            result(good + "running 1 test\ntest other::test ... FAILED\n", "QUIESCE_BEFORE_RESET"),
        ]
        for actual in bad:
            with self.subTest(actual=actual), self.assertRaises(runner.Rejected):
                self.check(actual, case=case)
        with self.assertRaises(runner.Rejected):
            runner.check_baseline(case, result("running 0 tests", returncode=0))
        runner.check_baseline(case, result(good.replace("FAILED", "ok"), returncode=0))

    def test_path_escape_and_unowned_cleanup_are_rejected(self):
        for path in ("../outside", "/outside", "C:/outside", "src\\outside"):
            with self.subTest(path=path), self.assertRaises(runner.Rejected):
                runner.relative_path(path)
        with self.assertRaises(runner.Rejected):
            runner.remove_owned_worktree(self.tree, self.tree, self.tree)


FIXTURE_DRIVER = '''import json
from pathlib import Path
import sys
text = Path("src/input.rs").read_bytes()
if b"bad" in text:
    start = text.index(b"bad")
    print(json.dumps({"reason": "compiler-message", "message": {
        "level": "error", "code": {"code": "E0616"},
        "message": "field `bad` is private", "spans": [{
            "file_name": "src/input.rs", "is_primary": True,
            "byte_start": start, "byte_end": start + 3}]}}))
    sys.exit(1)
print(json.dumps({"reason": "build-finished", "success": True}))
'''


@unittest.skipUnless(shutil.which("git"), "Git is required for isolated worktree self-tests")
class WorktreeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="mutation-self-test-")
        self.root = Path(self.temporary.name)
        self.repo = self.root / "fixture-repo"
        self.repo.mkdir()
        (self.repo / "src").mkdir()
        (self.repo / "src/input.rs").write_text("let value = good;\n", encoding="utf-8")
        (self.repo / "driver.py").write_text(FIXTURE_DRIVER, encoding="utf-8")
        runner.git(self.repo, "init", "--quiet")
        runner.git(self.repo, "config", "user.name", "Mutation Fixture")
        runner.git(self.repo, "config", "user.email", "fixture@example.invalid")
        runner.git(self.repo, "config", "core.autocrlf", "false")
        runner.git(self.repo, "add", "src/input.rs", "driver.py")
        runner.git(self.repo, "commit", "--quiet", "-m", "Generated fixture baseline")
        self.commit = runner.git(self.repo, "rev-parse", "HEAD")
        self.manifest = {"version": 1, "relevant_paths": ["src", "driver.py"], "timeout_seconds": 30, "cases": [compiler_case()]}
        self.manifest_path = self.root / "manifest.json"
        self.manifest_path.write_text(json.dumps(self.manifest), encoding="utf-8")

    def tearDown(self):
        self.temporary.cleanup()

    def args(self):
        return SimpleNamespace(repo=str(self.repo), baseline=self.commit, manifest=str(self.manifest_path),
                               case=None, output=str(self.root / "results"), keep_worktrees=False)

    def test_real_worktree_baseline_mutation_logs_and_cleanup(self):
        with redirect_stdout(io.StringIO()):
            code, output = runner.run(self.args())
        self.assertEqual(code, 0)
        report = json.loads((output / "summary.json").read_text())
        self.assertEqual(report["baseline"], self.commit)
        self.assertEqual(report["cases"][0]["status"], "PASS")
        self.assertIn("rustc", report["cases"][0]["toolchain"])
        self.assertIn("cargo", report["cases"][0]["toolchain"])
        self.assertLessEqual(set(report["build_environment"]), set(runner.RECORDED_ENVIRONMENT))
        self.assertFalse(report["cases"][0]["worktree_retained"])
        self.assertFalse(Path(report["worktree_root"]).exists())
        self.assertTrue((output / "fixture-compiler/mutation.diff").is_file())
        self.assertTrue((output / "fixture-compiler/baseline.stdout.log").is_file())
        self.assertTrue((output / "fixture-compiler/mutant.stderr.log").is_file())
        self.assertIn("PASS", (output / "ledger.md").read_text())
        self.assertEqual((self.repo / "src/input.rs").read_text(), "let value = good;\n")
        self.assertEqual(runner.git(self.repo, "status", "--porcelain"), "")
        self.assertEqual(runner.git(self.repo, "worktree", "list", "--porcelain").count("worktree "), 1)

    def test_dirty_staged_and_untracked_inputs_are_rejected(self):
        source = self.repo / "src/input.rs"
        source.write_text("changed\n")
        with self.assertRaisesRegex(runner.Rejected, "uncommitted"):
            runner.preflight(self.repo, self.commit, self.manifest)
        runner.git(self.repo, "add", "src/input.rs")
        with self.assertRaisesRegex(runner.Rejected, "uncommitted"):
            runner.preflight(self.repo, self.commit, self.manifest)
        # Restore only this newly generated fixture through a normal edit/add.
        source.write_text("let value = good;\n")
        runner.git(self.repo, "add", "src/input.rs")
        (self.repo / "src/untracked.rs").write_text("untracked input")
        with self.assertRaisesRegex(runner.Rejected, "uncommitted"):
            runner.preflight(self.repo, self.commit, self.manifest)

    def test_unrelated_dirty_file_is_preserved_and_ignored(self):
        (self.repo / "unrelated.txt").write_text("user note")
        self.assertEqual(runner.preflight(self.repo, self.commit, self.manifest), self.commit)

    def test_manifest_allows_only_reviewed_named_compiler_lints(self):
        self.manifest["cases"][0]["expected"]["code"] = "unsafe_code"
        self.manifest_path.write_text(json.dumps(self.manifest))
        manifest, _ = runner.load_manifest(self.manifest_path)
        self.assertEqual(manifest["cases"][0]["expected"]["code"], "unsafe_code")
        for code in ("dead_code", "unused_unsafe", "unsafe_*", "", "E123"):
            with self.subTest(code=code), self.assertRaises(runner.Rejected):
                self.manifest["cases"][0]["expected"]["code"] = code
                self.manifest_path.write_text(json.dumps(self.manifest))
                runner.load_manifest(self.manifest_path)

    def test_nonexistent_baseline_and_existing_output_are_rejected(self):
        with self.assertRaises(runner.Rejected):
            runner.preflight(self.repo, "missing-commit", self.manifest)
        args = self.args()
        Path(args.output).mkdir()
        with self.assertRaises(FileExistsError):
            runner.run(args)

    def test_duplicate_anchor_fails_before_running_any_command(self):
        self.manifest["cases"][0]["anchor"] = "missing source anchor"
        self.manifest_path.write_text(json.dumps(self.manifest))
        with redirect_stdout(io.StringIO()):
            code, output = runner.run(self.args())
        self.assertEqual(code, 1)
        report = json.loads((output / "summary.json").read_text())
        self.assertEqual(report["cases"][0]["status"], "FAIL")
        self.assertNotIn("baseline_result", report["cases"][0])
        self.assertFalse(report["cases"][0]["worktree_retained"])

    def test_mutant_dependency_lock_change_cannot_count_as_rejection(self):
        script = FIXTURE_DRIVER.replace('if b"bad" in text:',
            'Path("Cargo.lock").write_bytes(b"mutant" if b"bad" in text else b"positive")\nif b"bad" in text:')
        (self.repo / "driver.py").write_text(script, encoding="utf-8")
        runner.git(self.repo, "add", "driver.py")
        runner.git(self.repo, "commit", "--quiet", "-m", "Generated dependency-changing fixture")
        self.commit = runner.git(self.repo, "rev-parse", "HEAD")
        with redirect_stdout(io.StringIO()):
            code, output = runner.run(self.args())
        report = json.loads((output / "summary.json").read_text())
        self.assertEqual(code, 1)
        self.assertIn("lockfiles changed", report["cases"][0]["reason"])
        self.assertEqual((output / "fixture-compiler/baseline-lockfiles/Cargo.lock").read_bytes(), b"positive")

    @unittest.skipUnless(shutil.which("cargo"), "Cargo is required for real Rust diagnostic/harness checks")
    def test_real_rust_diagnostic_and_behavior_outputs(self):
        (self.repo / "Cargo.toml").write_text(
            '[package]\nname = "mutation-runner-fixture"\nversion = "0.0.0"\nedition = "2021"\n',
            encoding="utf-8",
        )
        (self.repo / "src/lib.rs").write_text(
            '#[cfg(test)]\nmod tests {\n    #[test]\n    fn smoke() {\n'
            '        let value: u8 = 1;\n        assert!(value == 1, "FIXTURE_INVARIANT");\n    }\n}\n',
            encoding="utf-8",
        )
        runner.git(self.repo, "add", "Cargo.toml", "src/lib.rs")
        runner.git(self.repo, "commit", "--quiet", "-m", "Generated Rust fixture")
        self.commit = runner.git(self.repo, "rev-parse", "HEAD")
        compiler = compiler_case()
        compiler.update(source="src/lib.rs", anchor="let value: u8 = 1;", replacement='let value: u8 = "bad";',
                        command=["cargo", "test", "--lib", "--message-format=json"],
                        expected={"code": "E0308", "message_contains": "mismatched types", "span": {"text": '"bad"'}})
        behavior = deepcopy(compiler)
        behavior.update(id="fixture-behavior", kind="behavior", anchor='assert!(value == 1, "FIXTURE_INVARIANT");',
                        replacement='assert!(value == 2, "FIXTURE_INVARIANT");',
                        command=["cargo", "test", "--lib", "--message-format=json", "tests::smoke", "--", "--exact", "--nocapture"],
                        expected={"test": "tests::smoke", "assertion": "FIXTURE_INVARIANT"})
        self.manifest.update(relevant_paths=["src", "driver.py", "Cargo.toml"], cases=[compiler, behavior], timeout_seconds=90)
        self.manifest_path.write_text(json.dumps(self.manifest))
        with redirect_stdout(io.StringIO()):
            code, output = runner.run(self.args())
        report = json.loads((output / "summary.json").read_text())
        details = "\n".join(path.read_text(errors="replace") for path in output.glob("*/*.log"))
        self.assertEqual(code, 0, f"{report}\n{details}")
        self.assertEqual([case["status"] for case in report["cases"]], ["PASS", "PASS"])
        for case in report["cases"]:
            self.assertIn("Cargo.lock", case["baseline_lockfiles"])
            self.assertEqual(case["baseline_lockfiles"], case["mutant_lockfiles"])
            self.assertTrue((output / case["id"] / "baseline-lockfiles/Cargo.lock").is_file())

    @unittest.skipUnless(shutil.which("cargo"), "Cargo is required for the real unsafe-code lint control")
    def test_forbidden_unsafe_exact_span_and_cap_lints_survival_control(self):
        (self.repo / "Cargo.toml").write_text(
            '[package]\nname = "mutation-unsafe-fixture"\nversion = "0.0.0"\nedition = "2021"\n',
            encoding="utf-8",
        )
        (self.repo / "src/lib.rs").write_text("pub mod protocol;\n", encoding="utf-8")
        (self.repo / "src/protocol.rs").write_text(
            '#![forbid(unsafe_code)]\npub fn value() -> u8 {\n    let byte = 7;\n    byte\n}\n',
            encoding="utf-8",
        )
        runner.git(self.repo, "add", "Cargo.toml", "src/lib.rs", "src/protocol.rs")
        runner.git(self.repo, "commit", "--quiet", "-m", "Generated forbidden-unsafe fixture")
        self.commit = runner.git(self.repo, "rev-parse", "HEAD")
        case = compiler_case()
        unsafe_block = "unsafe { core::ptr::read_volatile(&byte) }"
        case.update(id="fixture-forbidden-unsafe", source="src/protocol.rs", anchor="    byte\n",
                    replacement=f"    {unsafe_block}\n",
                    command=["cargo", "check", "--manifest-path", "Cargo.toml", "--lib", "--message-format=json"],
                    expected={"code": "unsafe_code", "message_contains": "usage of an `unsafe` block", "span": {"text": unsafe_block}})
        self.manifest.update(relevant_paths=["src", "driver.py", "Cargo.toml"], cases=[case], timeout_seconds=90)
        self.manifest_path.write_text(json.dumps(self.manifest))
        with mock.patch.dict(os.environ, {"RUSTFLAGS": "-Dwarnings", "CARGO_ENCODED_RUSTFLAGS": "-Dwarnings"}), redirect_stdout(io.StringIO()):
            code, output = runner.run(self.args())
        report = json.loads((output / "summary.json").read_text())
        details = "\n".join(path.read_text(errors="replace") for path in output.glob("*/*.log"))
        self.assertEqual(code, 0, f"{report}\n{details}")
        self.assertEqual(report["cases"][0]["status"], "PASS")

        # Keep the source, command, and matcher identical. Capping lints makes
        # the injected unsafe block compile; that MUST be a survived mutation,
        # never a passing compiler rejection or an unrelated-error fallback.
        args = self.args()
        args.output = str(self.root / "cap-lints-control")
        with mock.patch.dict(os.environ, {"RUSTFLAGS": "--cap-lints=allow", "CARGO_ENCODED_RUSTFLAGS": "--cap-lints=allow"}), redirect_stdout(io.StringIO()):
            code, output = runner.run(args)
        report = json.loads((output / "summary.json").read_text())
        self.assertEqual(code, 1, report)
        self.assertEqual(report["cases"][0]["status"], "FAIL")
        self.assertEqual(report["cases"][0]["mutant_result"]["returncode"], 0)
        self.assertEqual(report["cases"][0]["reason"], "Mutation survived: command succeeded")


if __name__ == "__main__":
    unittest.main()
