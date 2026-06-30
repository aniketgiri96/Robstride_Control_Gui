"""Low-rate board power read emits PowerInfo with power = VBUS * Iq.

No hardware and no Qt event loop: a fake bus captures the registers that
``_read_power`` reads and a captured-signal stub records what the worker emits.
"""

from __future__ import annotations

from robstride_gui import worker as wk
from robstride_gui.protocol import ParameterType


class FakeBus:
    """Stub bus returning canned register values for read_param."""

    def __init__(self, values: dict[int, float]):
        self._values = values  # param index -> value
        self.reads: list[int] = []

    def read_param(self, device_id: int, param):
        self.reads.append(param.index)
        return self._values.get(param.index)


def _worker_with_bus(bus: FakeBus) -> wk.ControlWorker:
    worker = wk.ControlWorker()
    worker._bus = bus
    return worker


def test_read_power_emits_vbus_current_and_product():
    bus = FakeBus({ParameterType.VBUS.index: 24.0,
                   ParameterType.IQ_FILTERED.index: 1.5})
    worker = _worker_with_bus(bus)
    captured: list[tuple[int, wk.PowerInfo]] = []
    worker.powerUpdated.connect(lambda did, info: captured.append((did, info)))

    worker._read_power(4)

    assert len(captured) == 1
    device_id, info = captured[0]
    assert device_id == 4
    assert info.vbus == 24.0
    assert info.iq == 1.5
    assert info.power == 24.0 * 1.5  # 36.0 W


def test_read_power_emits_nothing_when_register_unavailable():
    # iqf missing -> no reliable power estimate -> stay silent rather than guess.
    bus = FakeBus({ParameterType.VBUS.index: 24.0})
    worker = _worker_with_bus(bus)
    captured: list = []
    worker.powerUpdated.connect(lambda did, info: captured.append(info))

    worker._read_power(4)

    assert captured == []
