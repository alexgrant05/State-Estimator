from dataclasses import replace
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from digital_twin.eskf import InertialEskf
from digital_twin.frames import rocketpy_initial_quaternion
from digital_twin.pipeline import generate_all_events
from digital_twin.transport import REPLAY_FILENAMES, event_to_json, transaction_bytes_for_event
from digital_twin.truth import analytic_truth


def test_compact_active_stack_golden_replay(twin_config):
    expected = json.loads((Path(__file__).parent / "fixtures" / "golden_active_replay_manifest.json").read_text())
    config = replace(twin_config, simulation=replace(twin_config.simulation, pad_duration_s=expected["pad_duration_s"]))
    quaternion = rocketpy_initial_quaternion(config.launch.rail_inclination_deg, config.launch.rail_heading_deg)
    truth = analytic_truth(expected["duration_s"], initial_quaternion=quaternion, elevation_msl_m=config.launch.elevation_msl_m)
    events = generate_all_events(truth, config, expected["seed"])
    ordered = sorted(events, key=lambda event: (event.arrival_ticks, int(event.sensor_id), event.sequence_number))
    logical = "".join(json.dumps(event_to_json(event), sort_keys=True, separators=(",", ":")) + "\n" for event in ordered).encode()
    assert len(events) == expected["event_count"]
    assert hashlib.sha256(logical).hexdigest() == expected["hashes"]["events.ndjson"]
    for sensor, name in REPLAY_FILENAMES.items():
        selected = [event for event in ordered if event.sensor_id == sensor]
        replay = b"".join(transaction_bytes_for_event(event) for event in selected)
        assert len(selected) == expected["counts"][name]
        assert hashlib.sha256(replay).hexdigest() == expected["hashes"][name]
    states = InertialEskf(config).run(ordered)
    final = states[-1]
    assert len(states) == expected["state_count"]
    assert final.state_ticks == expected["final_state_ticks"]
    expected_state = expected["final_state"]
    assert final.position_enu_m == pytest.approx(expected_state["position_enu_m"], abs=1e-10)
    assert final.velocity_enu_mps == pytest.approx(expected_state["velocity_enu_mps"], abs=1e-10)
    assert final.q_body_to_nav == pytest.approx(expected_state["q_body_to_nav"], abs=1e-12)
    assert final.accel_bias_body_mps2 == pytest.approx(expected_state["accel_bias_body_mps2"], abs=1e-10)
    assert final.gyro_bias_body_rps == pytest.approx(expected_state["gyro_bias_body_rps"], abs=1e-12)
    assert np.diag(final.covariance) == pytest.approx(expected_state["covariance_diagonal"], abs=1e-10)
