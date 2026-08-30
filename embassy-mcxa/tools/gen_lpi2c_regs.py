#!/usr/bin/env python3
"""Generate the mechanical sections of src/i2c/lpi2c_regs.rs from the PAC.

The Tock-style register map duplicates exactly one thing from the PAC:
register OFFSETS. Everything else (field meaning, access typing policy)
is either single-sourced in the PAC or owned here. This script removes
the transcription risk from that one duplication: it reads the offsets
from the PAC's own generated accessors (nxp-pac
`meta_peripherals/mcxa/LPI2C.rs`) and emits the `register_structs!`
block, the `offset_of!` assertion block, and the runtime
`check_layout` list between the GENERATED markers in lpi2c_regs.rs.

The MANIFEST below is the policy input and stays hand-written: which
registers the hot paths map, which Tock access type each gets (the
driver's contract, deliberately at least as strict as the SVD's), and
the doc line. Offsets are never written by hand.

Usage:
    python tools/gen_lpi2c_regs.py [--pac PATH] [--check]

--check verifies the committed file matches the generated output (use
after a PAC bump) without writing. Default mode rewrites the generated
sections in place. Runtime `check_layout()` and the `offset_of!`
asserts remain active as independent guards.

The equivalent upstream fix would be a chiptool output mode emitting
access-typed cells directly; until then this keeps regeneration
one command away.
"""

import argparse
import io
import json
import os
import re
import subprocess
import sys

# (pac_accessor, tock_access, [doc lines])
MANIFEST = [
    ("param", "ReadOnly", ["Parameter Register (FIFO sizes). PAC type: `Param`."]),
    ("mcr", "ReadWrite", ["Controller Control Register. PAC type: `Mcr`."]),
    ("msr", "ReadWrite", ["Controller Status Register (W1C flags). PAC type: `Msr`."]),
    ("mier", "ReadWrite", ["Controller Interrupt Enable Register. PAC type: `Mier`."]),
    ("mder", "ReadWrite", ["Controller DMA Enable Register. PAC type: `Mder`."]),
    ("mcfgr1", "ReadWrite", ["Controller Configuration 1. PAC type: `Mcfgr1`."]),
    ("mfsr", "ReadOnly", ["Controller FIFO Status Register. PAC type: `Mfsr`."]),
    (
        "mtdr",
        "WriteOnly",
        [
            "Controller Transmit Data Register (command + data).",
            "Write-only: reading it back is meaningless. PAC type: `Mtdr`.",
        ],
    ),
    (
        "mrdr",
        "ReadOnly",
        [
            "Controller Receive Data Register. Read-only, and a read",
            "*pops* the RX FIFO. PAC type: `Mrdr`.",
        ],
    ),
    ("scr", "ReadWrite", ["Target Control Register. PAC type: `Scr`."]),
    ("ssr", "ReadWrite", ["Target Status Register (W1C flags). PAC type: `Ssr`."]),
    ("sier", "ReadWrite", ["Target Interrupt Enable Register. PAC type: `Sier`."]),
    ("stdr", "WriteOnly", ["Target Transmit Data Register. Write-only. PAC type: `Stdr`."]),
    (
        "srdr",
        "ReadOnly",
        [
            "Target Receive Data Register. Read-only, and a read *pops*",
            "the RX FIFO. PAC type: `Srdr`.",
        ],
    ),
]

ACCESSOR_RE = re.compile(
    r"pub const fn (\w+)\(self\) -> crate::pac::common::Reg<\w+, crate::pac::common::(R|W|RW)>\s*\{"
    r"\s*unsafe \{ crate::pac::common::Reg::from_ptr\(self\.ptr\.wrapping_add\(0x([0-9a-fA-F]+)usize\)",
    re.MULTILINE,
)

# The Tock access type must be at least as strict as the PAC's SVD-derived
# access: never allow a direction the PAC forbids.
COMPATIBLE = {
    "R": {"ReadOnly"},
    "W": {"WriteOnly"},
    "RW": {"ReadOnly", "WriteOnly", "ReadWrite"},
}

STRUCT_BEGIN = "// BEGIN GENERATED (gen_lpi2c_regs.py): register_structs"
STRUCT_END = "// END GENERATED: register_structs"
ASSERT_BEGIN = "// BEGIN GENERATED (gen_lpi2c_regs.py): offset assertions"
ASSERT_END = "// END GENERATED: offset assertions"
CHECK_BEGIN = "// BEGIN GENERATED (gen_lpi2c_regs.py): layout checks"
CHECK_END = "// END GENERATED: layout checks"


def find_pac() -> str:
    """Resolve the SAME nxp-pac checkout the build uses, via
    `cargo metadata --locked` (honors Cargo.lock and CARGO_HOME, and
    REFUSES to update the lockfile — a `--check` must never mutate the
    tree). The package is resolved as embassy-mcxa's DIRECT dependency
    through the resolve graph, so it stays unambiguous even if several
    `nxp-pac` versions ever enter the dependency graph. `--pac` remains
    the manual override."""
    manifest = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Cargo.toml"))
    try:
        out = subprocess.check_output(
            ["cargo", "metadata", "--format-version", "1", "--locked", "--manifest-path", manifest],
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as e:
        sys.exit(
            f"cargo metadata --locked failed ({e}); on a fresh checkout run a "
            "cargo check first so Cargo.lock exists, or pass --pac explicitly"
        )
    meta = json.loads(out)

    root_id = (meta.get("resolve") or {}).get("root")
    if root_id is None:
        sys.exit("cargo metadata has no resolve root (workspace layout changed?); pass --pac")
    nodes = {n["id"]: n for n in meta["resolve"]["nodes"]}
    packages = {p["id"]: p for p in meta["packages"]}

    dep_ids = [
        d["pkg"]
        for d in nodes[root_id].get("deps", [])
        if packages.get(d["pkg"], {}).get("name") == "nxp-pac"
    ]
    if len(dep_ids) != 1:
        sys.exit(f"expected exactly one direct nxp-pac dependency, found {len(dep_ids)}; pass --pac")

    root = os.path.dirname(packages[dep_ids[0]]["manifest_path"])
    path = os.path.join(root, "src", "meta_peripherals", "mcxa", "LPI2C.rs")
    if not os.path.exists(path):
        sys.exit(f"resolved nxp-pac at {root} but LPI2C.rs is missing (layout changed?)")
    return path


def parse_offsets(pac_path: str) -> dict:
    src = io.open(pac_path, encoding="utf-8").read()
    out = {}
    for name, access, off in ACCESSOR_RE.findall(src):
        out[name] = (int(off, 16), access)
    if not out:
        sys.exit(f"no accessors parsed from {pac_path} (PAC syntax changed?)")
    return out


def gen_struct(offsets: dict) -> str:
    lines = [
        STRUCT_BEGIN,
        "register_structs! {",
        "    /// LPI2C register block (hot-path subset; gaps are configuration",
        "    /// registers still accessed through the PAC).",
        "    pub LpI2cRegisters {",
    ]
    pos = 0
    rsv = 0
    for name, tock_access, docs in MANIFEST:
        if name not in offsets:
            sys.exit(f"register {name!r} not found in the PAC")
        off, pac_access = offsets[name]
        if tock_access not in COMPATIBLE[pac_access]:
            sys.exit(f"{name}: Tock access {tock_access} is looser than the PAC's {pac_access}")
        if off < pos:
            sys.exit(f"{name}: offset 0x{off:03x} overlaps previous register (0x{pos:03x})")
        if off > pos:
            lines.append(f"        (0x{pos:03x} => _reserved{rsv}),")
            rsv += 1
        for d in docs:
            lines.append(f"        /// {d}")
        lines.append(f"        (0x{off:03x} => pub {name}: {tock_access}<u32>),")
        pos = off + 4
    lines.append(f"        (0x{pos:03x} => @END),")
    lines.append("    }")
    lines.append("}")
    lines.append(STRUCT_END)
    return "\n".join(lines)


def gen_asserts(offsets: dict) -> str:
    lines = [
        ASSERT_BEGIN,
        "// Offsets generated from the PAC's own accessors",
        "// (nxp-pac/src/meta_peripherals/mcxa/LPI2C.rs).",
        "const _: () = {",
        "    use core::mem::offset_of;",
    ]
    for name, _, _ in MANIFEST:
        off, _ = offsets[name]
        lines.append(f"    assert!(offset_of!(LpI2cRegisters, {name}) == 0x{off:03x});")
    lines.append("};")
    lines.append(ASSERT_END)
    return "\n".join(lines)


def gen_checks(offsets: dict) -> str:
    lines = [CHECK_BEGIN.strip()]
    for name, _, _ in MANIFEST:
        lines.append(f"    check!({name}, {name});")
    lines.append("    " + CHECK_END)
    return "\n".join(lines)


def splice(text: str, begin: str, end: str, new: str) -> str:
    i = text.index(begin)
    j = text.index(end) + len(end)
    return text[:i] + new + text[j:]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pac", default=None)
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    pac_path = args.pac or find_pac()
    offsets = parse_offsets(pac_path)

    here = os.path.dirname(os.path.abspath(__file__))
    target = os.path.join(here, "..", "src", "i2c", "lpi2c_regs.rs")
    text = io.open(target, encoding="utf-8", newline="").read()

    # Preserve the file's own line-ending convention, so --check never
    # reports false staleness on a CRLF checkout and regeneration never
    # produces mixed endings.
    eol = "\r\n" if "\r\n" in text else "\n"

    for marker in (STRUCT_BEGIN, STRUCT_END, ASSERT_BEGIN, ASSERT_END, CHECK_BEGIN, CHECK_END):
        if marker not in text:
            sys.exit(f"marker missing in lpi2c_regs.rs: {marker!r}")

    updated = splice(text, STRUCT_BEGIN, STRUCT_END, gen_struct(offsets).replace("\n", eol))
    updated = splice(updated, ASSERT_BEGIN, ASSERT_END, gen_asserts(offsets).replace("\n", eol))
    updated = splice(updated, CHECK_BEGIN, CHECK_END, gen_checks(offsets).replace("\n", eol))

    if args.check:
        if updated != text:
            sys.exit("lpi2c_regs.rs is out of date with the PAC — rerun tools/gen_lpi2c_regs.py")
        print(f"lpi2c_regs.rs matches the PAC ({len(MANIFEST)} registers)")
    else:
        io.open(target, "w", encoding="utf-8", newline="").write(updated)
        print(f"lpi2c_regs.rs regenerated from {pac_path} ({len(MANIFEST)} registers)")


if __name__ == "__main__":
    main()
