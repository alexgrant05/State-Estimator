# Setup and CLI

## Supported environment

- Python 3.10 through 3.13
- RocketPy 1.12.1
- Exact reference dependencies in `requirements-lock.txt`
- Installable package metadata in `pyproject.toml`

Linux is the reference development and CI environment.

## Installation

From `sim/`:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt
.venv/bin/python -m pip install --no-deps -e .
```

The lock file installs the complete environment. `--no-deps` on the editable
install prevents the broader version ranges in `pyproject.toml` from replacing
locked packages.

Windows uses the same commands with `.venv\Scripts\python.exe` in place of
`.venv/bin/python`.

## Run a simulation

```bash
python -m digital_twin run \
  --config config/andromeda.toml \
  --seed 42 \
  --output outputs/andromeda-seed-42
```

Arguments:

| Argument | Required | Meaning |
| --- | --- | --- |
| `--config PATH` | Yes | TOML configuration to load |
| `--seed INTEGER` | No | Overrides `simulation.seed` |
| `--output PATH` | Yes | New or empty run directory |

The command generates truth, events, binary replay, ESKF states, metrics, a
plot, and a manifest. It rereads `events.ndjson` before estimation so the normal
run exercises event serialization and decoding.

## Validate an existing run

```bash
python -m digital_twin validate --run outputs/andromeda-seed-42
```

This reads `validation.json`, prints each recorded gate, and returns failure if
any gate failed. It does not regenerate data or recompute hashes.

## Exit codes

| Code | Meaning |
| --- | --- |
| `0` | Command completed and all validation gates passed |
| `1` | Simulation completed or validation loaded, but at least one gate failed |
| `2` | Input, configuration, filesystem, or runtime error |

The run command will not overwrite a non-empty output directory. Use a new run
name for every configuration or seed.

## Tests

```bash
python -m pytest -q
```

Target a file or test during development:

```bash
python -m pytest tests/test_transport.py -q
python -m pytest tests/test_multisensor.py::test_high_g_handoff_uses_adxl -q
```
