"""Command-line entrypoint for simulation and validation."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

from . import __version__
from .config import load_config
from .eskf import InertialEskf
from .pipeline import generate_all_events
from .transport import read_events, write_multi_replay
from .truth import generate_andromeda_truth
from .validation import calculate_metrics, load_validation, write_error_plot, write_report, write_states


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _source_revision() -> str | None:
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _versions() -> dict[str, str]:
    versions = {"digital_twin": __version__, "python": sys.version.split()[0]}
    for package in ("numpy", "matplotlib", "rocketpy"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = "not-installed"
    return versions


def run_simulation(config_path: Path, seed: int | None, output: Path) -> dict[str, Any]:
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to overwrite non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config = load_config(config_path)
    actual_seed = config.simulation.seed if seed is None else seed

    truth, truth_summary = generate_andromeda_truth(config)
    events = generate_all_events(truth, config, actual_seed)
    event_path = output / "events.ndjson"
    replay_stats = write_multi_replay(events, event_path, output)
    round_trip_events = list(read_events(event_path))
    estimates = InertialEskf(config).run(round_trip_events)
    states_path = output / "states.csv"
    write_states(states_path, estimates)
    replay_paths = {name: output / name for name in replay_stats}
    metrics = calculate_metrics(truth, round_trip_events, estimates, config, replay_paths)

    validation_path = output / "validation.json"
    validation_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_report(output / "validation.md", metrics, truth_summary)
    write_error_plot(output / "errors.png", truth, estimates, config.simulation.clock_hz)

    manifest = {
        "schema_version": 2,
        "seed": actual_seed,
        "source_revision": _source_revision(),
        "configuration": {
            "path": str(config.source_path),
            "sha256": _sha256(config.source_path),
        },
        "versions": _versions(),
        "truth_summary": truth_summary,
        "calibration": {
            "status": "uncalibrated-generic-defaults",
            "deferred": [
                "sensor mounting and lever arms",
                "ADXL375 scale, bias, and misalignment",
                "BMP581 pressure-port and thermal behavior",
                "receiver-specific GNSS errors, latency, and wire codec",
            ],
        },
        "artifacts": {
            path.name: {"bytes": path.stat().st_size, "sha256": _sha256(path)}
            for path in (event_path, states_path, validation_path, *replay_paths.values())
        },
        "passed": metrics["passed"],
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Cornell Rocketry multi-sensor digital twin")
    subparsers = parser.add_subparsers(dest="command", required=True)
    run_parser = subparsers.add_parser("run", help="generate truth, replay, ESKF states, and validation")
    run_parser.add_argument("--config", type=Path, required=True)
    run_parser.add_argument("--seed", type=int)
    run_parser.add_argument("--output", type=Path, required=True)
    validate_parser = subparsers.add_parser("validate", help="check an existing run's validation gates")
    validate_parser.add_argument("--run", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    try:
        if arguments.command == "run":
            manifest = run_simulation(arguments.config, arguments.seed, arguments.output)
            print(json.dumps({"output": str(arguments.output.resolve()), "passed": manifest["passed"]}))
            return 0 if manifest["passed"] else 1
        validation = load_validation(arguments.run / "validation.json")
        for name, passed in validation["gates"].items():
            print(f"{'PASS' if passed else 'FAIL'} {name}")
        return 0 if validation["passed"] else 1
    except (FileExistsError, FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
