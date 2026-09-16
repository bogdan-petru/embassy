#!/usr/bin/env python3
"""Run reviewed one-shot mutations only in disposable committed worktrees."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import signal
import subprocess
import sys
import tempfile
import time


class Rejected(RuntimeError):
    """A precondition or expected invariant was not established."""


RECORDED_ENVIRONMENT = ("RUSTFLAGS", "RUSTDOCFLAGS", "CARGO_ENCODED_RUSTFLAGS", "CARGO_BUILD_TARGET")
NAMED_COMPILER_LINTS = frozenset({"unsafe_code"})


def relative_path(value: str) -> str:
    if not isinstance(value, str) or not value or "\\" in value:
        raise Rejected(f"Expected a relative forward-slash path: {value!r}")
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or ":" in value:
        raise Rejected(f"Path escapes the repository: {value!r}")
    return str(path)


def inside(root: Path, relative: str) -> Path:
    candidate = (root / relative_path(relative)).resolve()
    if not candidate.is_relative_to(root.resolve()):
        raise Rejected(f"Resolved path escapes {root}: {candidate}")
    return candidate


def git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args], capture_output=True, text=True,
        encoding="utf-8", errors="replace", check=False,
    )
    if result.returncode:
        raise Rejected(f"git {' '.join(args)} failed: {result.stderr.strip()}")
    return result.stdout.strip()


def load_manifest(path: Path) -> tuple[dict, str]:
    raw = path.read_bytes()
    manifest = json.loads(raw)
    if manifest.get("version") != 1:
        raise Rejected("Only mutation manifest version 1 is supported")
    paths = manifest.get("relevant_paths", [])
    if not paths:
        raise Rejected("relevant_paths must enumerate the build inputs")
    for path in paths:
        relative_path(path)
    seen = set()
    for case in manifest.get("cases", []):
        case_id = case["id"]
        if not re.fullmatch(r"[a-z][a-z0-9-]*", case_id) or case_id in seen:
            raise Rejected(f"Invalid or duplicate mutation id: {case_id}")
        seen.add(case_id)
        source = relative_path(case["source"])
        if not any(source == p or source.startswith(p.rstrip("/") + "/") for p in paths):
            raise Rejected(f"{case_id}: source is outside relevant_paths")
        relative_path(case["cwd"])
        if not case.get("anchor") or case["anchor"] == case.get("replacement"):
            raise Rejected(f"{case_id}: empty anchor or no-op mutation")
        if not isinstance(case.get("replacement"), str):
            raise Rejected(f"{case_id}: replacement must be text")
        command = case.get("command")
        if not isinstance(command, list) or not command or not all(isinstance(a, str) and a for a in command):
            raise Rejected(f"{case_id}: command must be an argument list")
        if "--message-format=json" not in command:
            raise Rejected(f"{case_id}: command must emit Cargo JSON diagnostics")
        expected = case["expected"]
        if case["kind"] == "compiler":
            if not re.fullmatch(r"E[0-9]{4}", expected["code"]) and expected["code"] not in NAMED_COMPILER_LINTS:
                raise Rejected(f"{case_id}: an exact E#### code or reviewed named lint is required")
            if not expected.get("message_contains") or not expected["span"].get("text"):
                raise Rejected(f"{case_id}: diagnostic message and changed span are required")
        elif case["kind"] == "behavior":
            harness = command[command.index("--") + 1:] if "--" in command else []
            if expected["test"] not in command or "--exact" not in harness or "--nocapture" not in harness:
                raise Rejected(f"{case_id}: select the named test with -- --exact --nocapture")
            if not expected.get("assertion"):
                raise Rejected(f"{case_id}: a unique invariant assertion marker is required")
        else:
            raise Rejected(f"{case_id}: unsupported mutation kind")
        if not case.get("guarantee") or not isinstance(case.get("evidence"), list):
            raise Rejected(f"{case_id}: guarantee and evidence list are required")
    return manifest, hashlib.sha256(raw).hexdigest()


def preflight(repo: Path, baseline: str, manifest: dict) -> str:
    commit = git(repo, "rev-parse", "--verify", baseline + "^{commit}")
    dirty = git(repo, "status", "--porcelain=v1", "--untracked-files=all", "--", *manifest["relevant_paths"])
    if dirty:
        raise Rejected("Relevant inputs have uncommitted changes; commit them explicitly before running.\n" + dirty)
    return commit


def mutate(source: str, case: dict) -> tuple[str, dict]:
    # Git checkouts may use CRLF. Match the manifest's reviewed LF text using
    # the file's newline spelling without rewriting unrelated bytes.
    newline = "\r\n" if "\r\n" in source else "\n"
    anchor = case["anchor"].replace("\r\n", "\n").replace("\n", newline)
    replacement = case["replacement"].replace("\r\n", "\n").replace("\n", newline)
    count = source.count(anchor)
    if count != 1:
        raise Rejected(f"{case['id']}: anchor matched {count} times, expected exactly once")
    if anchor == replacement:
        raise Rejected(f"{case['id']}: mutation changes no bytes")
    offset = source.index(anchor)
    changed = source[:offset] + replacement + source[offset + len(anchor):]
    span = {"byte_start": len(source[:offset].encode()), "byte_end": len((source[:offset] + replacement).encode())}
    if case["kind"] == "compiler":
        target = case["expected"]["span"]
        text = target["text"].replace("\r\n", "\n").replace("\n", newline)
        positions = [match.start() for match in re.finditer(re.escape(text), replacement)]
        occurrence = target.get("occurrence")
        if occurrence is None and len(positions) != 1:
            raise Rejected(f"{case['id']}: span text must be unique or declare occurrence")
        occurrence = 1 if occurrence is None else occurrence
        if not isinstance(occurrence, int) or not 1 <= occurrence <= len(positions):
            raise Rejected(f"{case['id']}: span occurrence is out of range")
        start = offset + positions[occurrence - 1]
        span = {"byte_start": len(changed[:start].encode()), "byte_end": len(changed[:start + len(text)].encode())}
    return changed, span


def diagnostics(stdout: str) -> list[dict]:
    errors = []
    for line in stdout.splitlines():
        try:
            item = json.loads(line)
        except (ValueError, TypeError):
            continue  # libtest's output shares stdout with Cargo JSON.
        if isinstance(item, dict) and item.get("reason") == "compiler-message":
            message = item.get("message", {})
            if message.get("level") == "error":
                if not message.get("spans") and re.fullmatch(r"aborting due to \d+ previous errors?", message.get("message", "")):
                    continue
                errors.append(message)
    return errors


def named_test(output: str, name: str, outcome: str) -> bool:
    # One exact test prevents another failing test from supplying the marker.
    counts = re.findall(r"^running (\d+) tests?\s*$", output, re.MULTILINE)
    outcomes = re.findall(r"^test (\S+) \.\.\. (ok|FAILED|ignored)\s*$", output, re.MULTILINE)
    return counts == ["1"] and outcomes == [(name, outcome)]


def check_baseline(case: dict, result: dict) -> None:
    if result["returncode"] != 0 or result["timed_out"] or diagnostics(result["stdout"]):
        raise Rejected("Positive baseline did not succeed with the same command")
    if case["kind"] == "behavior" and not named_test(result["stdout"], case["expected"]["test"], "ok"):
        raise Rejected("Positive baseline did not execute the exact named test successfully")


def check_mutant(case: dict, result: dict, span: dict, tree: Path, cwd: Path) -> None:
    if result["timed_out"]:
        raise Rejected("Mutation timed out; a timeout is not an invariant failure")
    if result["returncode"] == 0:
        raise Rejected("Mutation survived: command succeeded")
    errors = diagnostics(result["stdout"])
    expected = case["expected"]
    if case["kind"] == "behavior":
        if errors:
            raise Rejected("Behavioral mutation failed to compile")
        if not named_test(result["stdout"], expected["test"], "FAILED"):
            raise Rejected("Behavioral mutation did not fail the exact named test")
        if expected["assertion"] not in result["stdout"] + result["stderr"]:
            raise Rejected("Behavioral failure did not report the invariant assertion")
        return
    wanted_file = inside(tree, case["source"])
    matched = 0
    for error in errors:
        error_code = (error.get("code") or {}).get("code")
        matching_span = False
        for actual in error.get("spans", []):
            file_name = actual.get("file_name", "")
            actual_file = (cwd / file_name).resolve()
            if (actual.get("is_primary") and actual_file == wanted_file
                    and actual.get("byte_start") == span["byte_start"]
                    and actual.get("byte_end") == span["byte_end"]):
                matching_span = True
        if error_code != expected["code"] or expected["message_contains"] not in error.get("message", "") or not matching_span:
            raise Rejected("Compiler emitted an unexpected diagnostic, path, or primary span")
        matched += 1
    if matched != 1:
        raise Rejected(f"Expected exactly one matching compiler diagnostic, observed {matched}")


def execute(command: list[str], cwd: Path, env: dict, timeout: int, prefix: Path) -> dict:
    started = time.monotonic()
    kwargs = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {"start_new_session": True}
    process = subprocess.Popen(command, cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, **kwargs)
    timed_out = False
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        timed_out = True
        if os.name == "nt":
            subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], capture_output=True, check=False)
        else:
            os.killpg(process.pid, signal.SIGKILL)
        try:
            stdout, stderr = process.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            raise Rejected("Command tree did not terminate; its worktree must be retained")
    prefix.with_suffix(".stdout.log").write_bytes(stdout)
    prefix.with_suffix(".stderr.log").write_bytes(stderr)
    return {"returncode": process.returncode, "timed_out": timed_out,
            "elapsed_seconds": round(time.monotonic() - started, 3),
            "stdout": stdout.decode("utf-8", "replace"), "stderr": stderr.decode("utf-8", "replace")}


def remove_owned_worktree(repo: Path, root: Path, tree: Path) -> None:
    resolved = tree.resolve()
    if resolved.parent != root.resolve() or resolved == repo.resolve() or not (resolved / ".git").is_file():
        raise Rejected(f"Refusing to remove an unrecognized worktree: {resolved}")
    registered = [Path(line[9:]).resolve() for line in git(repo, "worktree", "list", "--porcelain").splitlines() if line.startswith("worktree ")]
    if resolved not in registered:
        raise Rejected(f"Worktree is not registered: {resolved}")
    git(repo, "worktree", "remove", "--force", str(resolved))


def toolchain_versions(cwd: Path, env: dict) -> dict:
    versions = {}
    for command in (["rustc", "-vV"], ["cargo", "--version"]):
        try:
            result = subprocess.run(command, cwd=cwd, env=env, capture_output=True, text=True,
                                    encoding="utf-8", errors="replace", check=False, timeout=30)
            versions[command[0]] = {"command": command, "returncode": result.returncode,
                                    "stdout": result.stdout, "stderr": result.stderr}
        except (OSError, subprocess.TimeoutExpired) as error:
            versions[command[0]] = {"command": command, "unavailable": str(error)}
    return versions


def lockfiles(tree: Path, save_to: Path | None = None) -> dict[str, str]:
    hashes = {}
    # Cargo output is outside this source worktree. Include ignored lockfiles
    # generated by the positive command as well as any tracked lockfiles.
    for lock in sorted(tree.rglob("Cargo.lock")):
        relative = lock.relative_to(tree).as_posix()
        data = inside(tree, relative).read_bytes()
        hashes[relative] = hashlib.sha256(data).hexdigest()
        if save_to is not None:
            snapshot = inside(save_to, relative)
            snapshot.parent.mkdir(parents=True, exist_ok=True)
            snapshot.write_bytes(data)
    return hashes


def persist_report(report: dict, output: Path) -> None:
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    rows = ["# Recorded mutation results", "", f"Baseline: `{report['baseline']}`  ",
            f"Manifest SHA-256: `{report['manifest_sha256']}`", "",
            "A PASS records the observed expected rejection, not general hardware correctness.", "",
            "| Case | Guarantee | Kind | Result | Logs |", "| --- | --- | --- | --- | --- |"]
    for case in report["cases"]:
        clean = lambda value: str(value).replace("|", "\\|").replace("\n", " ")
        rows.append(f"| {case['id']} | {clean(case['guarantee'])} | {case['kind']} | {case['status']} | [{case['id']}]({case['id']}/) |")
    rows.extend(["", "Declared evidence references (not newly validated by this run):", ""])
    for case in report["cases"]:
        for evidence in case["evidence"]:
            rows.append(f"- {case['id']}: {evidence}")
    (output / "ledger.md").write_text("\n".join(rows) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> tuple[int, Path]:
    repo = Path(git(Path(args.repo).resolve(), "rev-parse", "--show-toplevel")).resolve()
    manifest_path = Path(args.manifest).resolve()
    manifest, digest = load_manifest(manifest_path)
    commit = preflight(repo, args.baseline, manifest)
    selected = set(args.case or [])
    cases = [case for case in manifest["cases"] if not selected or case["id"] in selected]
    if not cases or selected - {case["id"] for case in cases}:
        raise Rejected("Select at least one known manifest case")
    output = Path(args.output).resolve() if args.output else Path(tempfile.mkdtemp(prefix="one-shot-mutation-results-"))
    if args.output:
        output.mkdir(parents=True, exist_ok=False)
    root = Path(tempfile.mkdtemp(prefix="one-shot-mutation-worktrees-"))
    env = os.environ.copy()
    env.update(CARGO_TERM_COLOR="never", CARGO_INCREMENTAL="0")
    report = {"baseline": commit, "requested_baseline": args.baseline, "manifest": str(manifest_path),
              "manifest_sha256": digest, "worktree_root": str(root), "cases": []}
    report["build_environment"] = {key: env[key] for key in RECORDED_ENVIRONMENT if key in env}
    persist_report(report, output)
    (output / "manifest.json").write_bytes(manifest_path.read_bytes())
    try:
        for case in cases:
            case_output = output / case["id"]
            case_output.mkdir()
            # Each execution gets separate output. Besides avoiding stale
            # mutable build state, this avoids overwriting an executable that
            # Windows may still hold briefly after its test process exits.
            tree = root / case["id"]
            record = {key: case[key] for key in ("id", "kind", "guarantee", "evidence", "command", "cwd", "source", "expected")}
            record.update(status="FAIL", worktree=str(tree))
            report["cases"].append(record)
            added = False
            retain = args.keep_worktrees
            command_in_flight = False
            try:
                # Recheck before every case: another editor may have changed an
                # input while an earlier worktree was running.
                preflight(repo, commit, manifest)
                git(repo, "worktree", "add", "--detach", str(tree), commit)
                added = True
                source = inside(tree, case["source"])
                cwd = inside(tree, case["cwd"])
                record["toolchain"] = toolchain_versions(cwd, env)
                original = source.read_bytes().decode("utf-8")
                changed, span = mutate(original, case)
                record["mutated_span"] = span
                timeout = int(case.get("timeout_seconds", manifest.get("timeout_seconds", 300)))
                if timeout <= 0:
                    raise Rejected("timeout_seconds must be positive")
                command_in_flight = True
                env["CARGO_TARGET_DIR"] = str(case_output / "baseline-target")
                positive = execute(case["command"], cwd, env, timeout, case_output / "baseline")
                command_in_flight = False
                retain |= positive["timed_out"]
                record["baseline_result"] = {k: v for k, v in positive.items() if k not in ("stdout", "stderr")}
                record["baseline_result"]["cargo_target_dir"] = env["CARGO_TARGET_DIR"]
                check_baseline(case, positive)
                record["baseline_lockfiles"] = lockfiles(tree, case_output / "baseline-lockfiles")
                source.write_bytes(changed.encode("utf-8"))
                (case_output / "mutation.diff").write_text(git(tree, "diff", "--", case["source"]) + "\n", encoding="utf-8")
                command_in_flight = True
                env["CARGO_TARGET_DIR"] = str(case_output / "mutant-target")
                negative = execute(case["command"], cwd, env, timeout, case_output / "mutant")
                command_in_flight = False
                retain |= negative["timed_out"]
                record["mutant_result"] = {k: v for k, v in negative.items() if k not in ("stdout", "stderr")}
                record["mutant_result"]["cargo_target_dir"] = env["CARGO_TARGET_DIR"]
                record["mutant_lockfiles"] = lockfiles(tree)
                if record["mutant_lockfiles"] != record["baseline_lockfiles"]:
                    raise Rejected("Dependency lockfiles changed after the positive baseline")
                check_mutant(case, negative, span, tree, cwd)
                record.update(status="PASS", reason="Observed the specified invariant rejection")
            except (Rejected, OSError, ValueError) as error:
                record["reason"] = str(error)
                retain |= command_in_flight
            finally:
                if added and not retain and not command_in_flight:
                    try:
                        remove_owned_worktree(repo, root, tree)
                    except Rejected as error:
                        record.update(status="FAIL", cleanup_error=str(error))
                record["worktree_retained"] = tree.exists()
                persist_report(report, output)
            print(f"{case['id']}: {record['status']} - {record.get('reason', '')}", flush=True)
    finally:
        # Only remove an empty directory allocated by this invocation. Crashes
        # and uncertain process termination leave worktrees available to inspect.
        try:
            root.rmdir()
        except OSError:
            pass
    return (0 if all(case["status"] == "PASS" for case in report["cases"]) else 1), output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, help="Explicit committed revision; never created by the runner")
    parser.add_argument("--repo", default=".")
    parser.add_argument("--manifest", default=str(Path(__file__).with_name("manifest.json")))
    parser.add_argument("--case", action="append", help="Case id; repeat to select several")
    parser.add_argument("--output", help="New results directory; must not already exist")
    parser.add_argument("--keep-worktrees", action="store_true")
    try:
        code, output = run(parser.parse_args())
    except (Rejected, OSError, ValueError, KeyError) as error:
        print(f"Mutation run rejected: {error}", file=sys.stderr)
        return 2
    print(f"Results: {output}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
