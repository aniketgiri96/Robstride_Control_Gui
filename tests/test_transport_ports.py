"""Tests for serial-port auto-detection (no hardware needed).

These mock ``serial.tools.list_ports.comports`` so the logic that decides
"is a usable port available, and which one" is covered deterministically.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from robstride_gui import transport as t


def _port(device, description="n/a", hwid="n/a"):
    """A stand-in for pyserial's ListPortInfo (only the fields we read)."""
    return SimpleNamespace(device=device, description=description, hwid=hwid)


@pytest.fixture
def fake_comports(monkeypatch):
    """Install a fake comports() returning the given list of ports."""
    def install(ports):
        monkeypatch.setattr(
            "serial.tools.list_ports.comports", lambda: list(ports))
    return install


# --- usb filtering --------------------------------------------------------------


def test_legacy_ttys_ports_are_filtered_out_by_default(fake_comports):
    fake_comports([_port(f"/dev/ttyS{i}") for i in range(4)])
    assert t.list_serial_port_details() == []
    assert t.auto_detect_serial_port() is None


def test_usb_serial_port_is_detected(fake_comports):
    fake_comports([
        _port("/dev/ttyS0"),
        _port("/dev/ttyUSB0", "USB Serial", "USB VID:PID=1A86:7523"),
    ])
    details = t.list_serial_port_details()
    assert [d.device for d in details] == ["/dev/ttyUSB0"]
    assert details[0].is_usb is True
    assert t.auto_detect_serial_port() == "/dev/ttyUSB0"


def test_usb_only_false_includes_legacy_ports(fake_comports):
    fake_comports([_port("/dev/ttyS0"), _port("/dev/ttyS1")])
    assert len(t.list_serial_ports(usb_only=False)) == 2
    assert t.list_serial_ports() == []  # usb_only defaults to True


# --- ranking --------------------------------------------------------------------


def test_can_like_port_ranks_first(fake_comports):
    fake_comports([
        _port("/dev/ttyUSB1", "USB Serial", "USB VID:PID=0403:6001"),  # plain FTDI
        _port("/dev/ttyACM0", "USB-CAN Adapter", "USB VID:PID=16D0:0AAA"),
    ])
    auto = t.auto_detect_serial_port()
    assert auto == "/dev/ttyACM0"  # description mentions CAN -> ranked first


def test_label_combines_device_and_description(fake_comports):
    fake_comports([_port("/dev/ttyUSB0", "CH340 USB-Serial", "USB VID:PID=1A86:7523")])
    info = t.list_serial_port_details()[0]
    assert info.label() == "/dev/ttyUSB0 - CH340 USB-Serial"


def test_label_falls_back_to_device_when_no_description(fake_comports):
    fake_comports([_port("/dev/ttyUSB0", "n/a", "USB VID:PID=1A86:7523")])
    info = t.list_serial_port_details()[0]
    assert info.label() == "/dev/ttyUSB0"


# --- exclusive open -------------------------------------------------------------


def test_serial_open_requests_exclusive_lock(monkeypatch):
    """open() must pass exclusive=True so a second process cannot share the port.

    A non-exclusive open lets a stale GUI instance keep reading the same adapter
    and silently steal the motor's reply frames, making Detect report "0".
    """
    captured = {}

    class FakeSerial:
        def __init__(self, port, baud, **kwargs):
            captured["port"] = port
            captured["baud"] = baud
            captured["kwargs"] = kwargs
            self.is_open = True

        def reset_input_buffer(self):
            pass

    monkeypatch.setattr("serial.Serial", FakeSerial)
    monkeypatch.setattr(t.time, "sleep", lambda *_: None)  # don't wait 0.2s

    transport = t.SerialATTransport("/dev/ttyUSB0")
    transport.open()

    assert captured["port"] == "/dev/ttyUSB0"
    assert captured["kwargs"].get("exclusive") is True
    assert transport.is_open is True


def test_serial_open_busy_port_raises_transport_error(monkeypatch):
    """A port already held exclusively surfaces as a TransportError, not a silent share."""
    def busy(*_args, **_kwargs):
        raise OSError("[Errno 16] Device or resource busy: '/dev/ttyUSB0'")

    monkeypatch.setattr("serial.Serial", busy)
    monkeypatch.setattr(t.time, "sleep", lambda *_: None)

    transport = t.SerialATTransport("/dev/ttyUSB0")
    with pytest.raises(t.TransportError, match="Cannot open serial port"):
        transport.open()


# --- no pyserial ----------------------------------------------------------------


def test_missing_pyserial_returns_empty(monkeypatch):
    import builtins

    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "serial.tools" or name.startswith("serial.tools"):
            raise ImportError("no pyserial")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    assert t.list_serial_port_details() == []
    assert t.auto_detect_serial_port() is None
