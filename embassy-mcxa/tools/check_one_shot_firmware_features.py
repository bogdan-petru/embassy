#!/usr/bin/env python3
"""Reject host MMIO instrumentation in a resolved one-shot firmware graph.

Run from any directory; defaults match the MCXA5xx one-shot example build.
For another firmware build, pass its manifest, target, features and default-
feature choice explicitly. This checks the graph, not firmware linking.
"""

import argparse
from pathlib import Path
import re
import shlex
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
EXPECTED_PACKAGE = "safe-mmio v0.3.1"
PACKAGE_START = re.compile(r"^safe-mmio(?:\s|\||$)")
FEATURE_LIST = re.compile(r"(?:[A-Za-z0-9_+-]+(?:,[A-Za-z0-9_+-]+)*)?")


class PolicyError(ValueError):
    """The graph does not establish the reviewed firmware feature policy."""


def check_graph(graph: str) -> list[str]:
    """Check every safe-mmio row, including duplicates and extra versions."""
    rows = [line.strip() for line in graph.splitlines() if PACKAGE_START.match(line.strip())]
    if not rows:
        raise PolicyError("safe-mmio missing from firmware graph")
    for row in rows:
        normalized = row.removesuffix(" (*)")
        package, separator, features = normalized.partition("|")
        # A same-version path/git replacement also needs explicit review.
        if not separator or package != EXPECTED_PACKAGE:
            raise PolicyError(f"Unexpected safe-mmio package/row: {row}")
        if not FEATURE_LIST.fullmatch(features):
            raise PolicyError(f"Malformed safe-mmio features: {row}")
        if "custom-mmio" in features.split(","):
            raise PolicyError("custom-mmio enabled in firmware graph")
    return rows


def cargo_command(args: argparse.Namespace) -> list[str]:
    command = [
        "cargo", "tree", "--manifest-path", args.manifest_path,
        "--features", args.features, "--target", args.target, "--locked",
        # `-e features` alone implicitly includes dev edges. Avoid them even
        # when a host-test/dev dependency enables custom-mmio elsewhere.
        "-e", "normal,build", "--prefix", "none", "--format", "{p}|{f}",
        "--charset", "ascii", "--color", "never",
    ]
    if args.no_default_features:
        command.append("--no-default-features")
    return command


def read_graph(command: list[str]) -> str:
    result = subprocess.run(command, cwd=REPO_ROOT, text=True, capture_output=True, check=False)
    if result.stderr:
        print(result.stderr, file=sys.stderr, end="")
    if result.returncode:
        raise PolicyError(f"cargo tree failed (exit {result.returncode})")
    return result.stdout


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest-path", default="examples/mcxa5xx/Cargo.toml")
    parser.add_argument("--features", default="i2c-one-shot")
    parser.add_argument("--target", default="thumbv8m.main-none-eabihf")
    parser.add_argument("--no-default-features", action="store_true")
    command = cargo_command(parser.parse_args())
    print("Firmware feature gate: " + shlex.join(command), flush=True)
    try:
        rows = check_graph(read_graph(command))
    except (OSError, PolicyError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print(f"PASS: no custom-mmio in firmware ({len(rows)} checked package rows)")
    for row in rows:
        print("  " + row)
    return 0


if __name__ == "__main__":
    sys.exit(main())
