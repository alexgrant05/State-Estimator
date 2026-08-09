"""Multi-sensor event generation and shared auxiliary-SPI scheduling."""

from __future__ import annotations

from dataclasses import replace

from .adis16470 import Adis16470Model
from .adxl375 import Adxl375Model
from .bmp581 import Bmp581Model
from .config import TwinConfig
from .gnss import GenericGnssModel
from .types import MeasurementEvent, SensorId, TruthSample


def schedule_aux_spi(events: list[MeasurementEvent], config: TwinConfig) -> list[MeasurementEvent]:
    auxiliary = [event for event in events if event.sensor_id in (SensorId.ADXL375, SensorId.BMP581)]
    others = [event for event in events if event.sensor_id not in (SensorId.ADXL375, SensorId.BMP581)]
    auxiliary.sort(key=lambda item: (item.measurement_ticks, 0 if item.sensor_id == SensorId.ADXL375 else 1, item.sequence_number))
    bus_free = 0
    scheduled: list[MeasurementEvent] = []
    for event in auxiliary:
        # Data-register burst plus a separate status-register transaction.
        bits = 72
        spi_clock = config.adxl375.spi_clock_hz if event.sensor_id == SensorId.ADXL375 else config.bmp581.spi_clock_hz
        transfer_ticks = int(round(bits / spi_clock * config.simulation.clock_hz))
        start = max(event.measurement_ticks, bus_free)
        arrival = start + transfer_ticks
        bus_free = arrival
        scheduled.append(replace(event, arrival_ticks=arrival))
    return sorted(others + scheduled, key=lambda item: (item.arrival_ticks, int(item.sensor_id), item.sequence_number))


def generate_all_events(truth: list[TruthSample], config: TwinConfig, seed: int) -> list[MeasurementEvent]:
    events = Adis16470Model(config.adis16470, config.simulation, seed).generate(truth)
    events += Adxl375Model(config.adxl375, config.simulation, seed).generate(truth)
    events += Bmp581Model(config.bmp581, config.simulation, seed).generate(truth)
    events += GenericGnssModel(config.gnss, config.launch, config.simulation, seed).generate(truth)
    return schedule_aux_spi(events, config)
