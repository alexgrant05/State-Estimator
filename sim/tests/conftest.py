from pathlib import Path

import pytest

from digital_twin.config import load_config


@pytest.fixture(scope="session")
def config_path() -> Path:
    return Path(__file__).parents[1] / "config" / "andromeda.toml"


@pytest.fixture(scope="session")
def twin_config(config_path):
    return load_config(config_path)

