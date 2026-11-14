"""Legacy ADIS16470 burst decoder retained only for old replay inspection."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable

ADIS_BURST_COMMAND = 0x6800
ADIS_TRANSACTION_BITS = 176


def adis_checksum(words: Iterable[int]) -> int:
    values = tuple(words)
    if len(values) != 9:
        raise ValueError("ADIS checksum requires nine data words")
    return sum(((word >> 8) & 0xFF) + (word & 0xFF) for word in values) & 0xFFFF


@dataclass(frozen=True, slots=True)
class AdisBurst:
    diag_stat: int
    gyro_words: tuple[int, int, int]
    accel_words: tuple[int, int, int]
    temperature_word: int
    data_counter: int
    checksum: int

    @property
    def words(self) -> tuple[int, ...]:
        return (self.diag_stat, *self.gyro_words, *self.accel_words, self.temperature_word, self.data_counter, self.checksum)

    @classmethod
    def create(cls, diag_stat: int, gyro_words: tuple[int, int, int], accel_words: tuple[int, int, int], temperature_word: int, data_counter: int) -> "AdisBurst":
        data = (diag_stat, *gyro_words, *accel_words, temperature_word, data_counter)
        return cls(diag_stat, gyro_words, accel_words, temperature_word, data_counter, adis_checksum(data))

    def valid_checksum(self) -> bool:
        return self.checksum == adis_checksum(self.words[:-1])

    def payload_bytes(self) -> bytes:
        return struct.pack(">10H", *(word & 0xFFFF for word in self.words))

    def transaction_bytes(self) -> bytes:
        return struct.pack(">11H", ADIS_BURST_COMMAND, *(word & 0xFFFF for word in self.words))

    @classmethod
    def from_payload_bytes(cls, payload: bytes) -> "AdisBurst":
        if len(payload) != 20:
            raise ValueError("ADIS response payload must be 20 bytes")
        values = struct.unpack(">10H", payload)
        return cls(values[0], tuple(values[1:4]), tuple(values[4:7]), values[7], values[8], values[9])

    @classmethod
    def from_transaction_bytes(cls, transaction: bytes) -> "AdisBurst":
        if len(transaction) != 22:
            raise ValueError("ADIS transaction must be 22 bytes / 176 bits")
        command, *values = struct.unpack(">11H", transaction)
        if command != ADIS_BURST_COMMAND:
            raise ValueError("invalid ADIS burst command")
        return cls.from_payload_bytes(struct.pack(">10H", *values))
