#!/usr/bin/env python3
"""Headless scenario runner with golden-diff assertion.

For each `<name>.scenario.yaml` under `testbench/scenarios/`, feeds its
`steps:` into the native binary as one JSON command per line and
compares stdout to `<name>.expected.jsonl`. Any diff fails the run.

Regenerate goldens with `--update-goldens` (reviewers should see the
diff in the resulting PR).

Usage:
    uv run --with pyyaml python testbench/run_scenarios.py
    uv run --with pyyaml python testbench/run_scenarios.py --update-goldens
    uv run --with pyyaml python testbench/run_scenarios.py --binary path/to/smartevse-sim
"""
from __future__ import annotations

import argparse
import difflib
import json
import subprocess
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
DEFAULT_BIN = HERE / "native" / "build" / "smartevse-sim"
SCENARIOS_DIR = HERE / "scenarios"


def run_scenario(binary: Path, scenario_path: Path) -> str:
    steps = yaml.safe_load(scenario_path.read_text())["steps"]
    stdin = "".join(json.dumps(step) + "\n" for step in steps)
    proc = subprocess.run(
        [str(binary)],
        input=stdin,
        capture_output=True,
        text=True,
        timeout=10,
        check=True,
    )
    return proc.stdout


def compare(name: str, actual: str, expected: str) -> bool:
    if actual == expected:
        print(f"PASS  {name}")
        return True
    diff = difflib.unified_diff(
        expected.splitlines(keepends=True),
        actual.splitlines(keepends=True),
        fromfile=f"{name}.expected",
        tofile=f"{name}.actual",
    )
    print(f"FAIL  {name}")
    sys.stdout.writelines(diff)
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", type=Path, default=DEFAULT_BIN)
    ap.add_argument("--update-goldens", action="store_true")
    ap.add_argument("scenario", nargs="*", help="optional subset of scenario names")
    args = ap.parse_args()

    if not args.binary.exists():
        print(f"binary not found: {args.binary}", file=sys.stderr)
        print("build it first: (cd testbench/native && make)", file=sys.stderr)
        return 2

    scenarios = sorted(SCENARIOS_DIR.glob("*.scenario.yaml"))
    if args.scenario:
        wanted = set(args.scenario)
        scenarios = [p for p in scenarios if p.stem.replace(".scenario", "") in wanted or p.name in wanted]
        if not scenarios:
            print(f"no scenarios matched: {args.scenario}", file=sys.stderr)
            return 2

    failed = 0
    for scenario in scenarios:
        name = scenario.name.removesuffix(".scenario.yaml")
        golden = scenario.with_suffix(".yaml").with_name(f"{name}.expected.jsonl")
        actual = run_scenario(args.binary, scenario)

        if args.update_goldens:
            golden.write_text(actual)
            print(f"WROTE {name}  ({len(actual.splitlines())} events)")
            continue

        if not golden.exists():
            print(f"MISS  {name}  (no golden at {golden.name}; use --update-goldens)")
            failed += 1
            continue

        if not compare(name, actual, golden.read_text()):
            failed += 1

    if args.update_goldens:
        return 0
    print()
    print(f"{len(scenarios) - failed}/{len(scenarios)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
