"""uORB-style typed publish/subscribe message bus.

PX4 modules communicate exclusively through uORB topics rather than direct
function calls -- a rate-control module doesn't call the mixer, it publishes
an ActuatorControls message and the mixer (running at its own rate)
subscribes to it. This is the single biggest thing that makes a sim feel like
PX4 rather than a monolithic script, so it is reproduced here even though a
direct-call architecture would be simpler for a program this size.

Latest-value semantics (PX4's default for most topics): each topic holds only
its most recent message plus the sim time it was published and a sequence
number, not a queue. Thread-safe (one lock guards the whole bus) since the
GCS server thread reads topics the physics thread is concurrently publishing.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Generic, TypeVar

T = TypeVar("T")


@dataclass
class Sample(Generic[T]):
    timestamp: float
    value: T
    seq: int


class MessageBus:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._topics: dict[str, Sample] = {}

    def publish(self, topic: str, value: Any, timestamp: float) -> None:
        with self._lock:
            prev = self._topics.get(topic)
            seq = (prev.seq + 1) if prev is not None else 0
            self._topics[topic] = Sample(timestamp=timestamp, value=value, seq=seq)

    def get(self, topic: str) -> Sample | None:
        with self._lock:
            return self._topics.get(topic)

    def get_value(self, topic: str, default: Any = None) -> Any:
        s = self.get(topic)
        return s.value if s is not None else default

    def snapshot(self) -> dict[str, Sample]:
        """All topics at once, for the logger/dashboard -- one lock
        acquisition instead of one per topic."""
        with self._lock:
            return dict(self._topics)


class Subscriber(Generic[T]):
    """Remembers the last sequence number it has seen, so a module can ask
    'is there a NEW message since I last checked' (PX4's updated() idiom)."""

    def __init__(self, bus: MessageBus, topic: str) -> None:
        self._bus = bus
        self._topic = topic
        self._last_seq = -1

    def updated(self) -> bool:
        s = self._bus.get(self._topic)
        return s is not None and s.seq != self._last_seq

    def get(self) -> Sample | None:
        s = self._bus.get(self._topic)
        if s is not None:
            self._last_seq = s.seq
        return s
