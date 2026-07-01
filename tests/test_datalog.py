"""TelemetryLogger records graph-feedback samples to a tab-separated .txt file.

Logging is opt-in: nothing is written until ``start()`` is called. No Qt and no
hardware - drive the logger directly against a temp path and read the file back.
"""

from __future__ import annotations

from robstride_gui.datalog import COLUMNS, TelemetryLogger
from robstride_gui.protocol import RAD_S_TO_RPM


def test_nothing_written_until_start(tmp_path):
    path = tmp_path / "telemetry.txt"
    logger = TelemetryLogger()

    # Not recording: samples are dropped and no file appears.
    logger.log_status(1, 0.0, 0.0, 0.0, 25.0)
    assert not logger.is_recording
    assert not path.exists()


def test_writes_header_and_sample_row(tmp_path):
    path = tmp_path / "telemetry.txt"
    logger = TelemetryLogger()
    logger.start(path)

    logger.log_status(1, position=0.5, velocity=1.25, torque=0.04,
                      temperature=31.5)
    logger.stop()

    lines = path.read_text().splitlines()
    assert lines[0].split("\t") == list(COLUMNS)
    cols = lines[1].split("\t")
    assert cols[1] == "1"               # device_id
    assert cols[2] == "0.500000"        # position_rad
    assert cols[3] == f"{1.25 * RAD_S_TO_RPM:.6f}"  # velocity_rpm (1.25 rad/s)
    assert cols[4] == "0.040000"        # torque_nm
    assert cols[5] == "31.50"           # temperature_c
    assert cols[6] == ""                # vbus blank until a power read arrives


def test_stop_then_log_is_dropped(tmp_path):
    path = tmp_path / "telemetry.txt"
    logger = TelemetryLogger()
    logger.start(path)
    logger.log_status(1, 0.1, 0.0, 0.0, 25.0)
    logger.stop()

    logger.log_status(1, 9.9, 0.0, 0.0, 25.0)  # after stop: ignored

    lines = path.read_text().splitlines()
    assert len(lines) == 2  # header + the single in-recording sample


def test_power_is_carried_into_later_rows(tmp_path):
    path = tmp_path / "telemetry.txt"
    logger = TelemetryLogger()
    logger.start(path)

    logger.update_power(2, vbus=24.0, iq=1.5, power=36.0)
    logger.log_status(2, position=0.0, velocity=0.0, torque=0.0,
                      temperature=30.0)
    logger.stop()

    cols = path.read_text().splitlines()[1].split("\t")
    assert cols[1] == "2"
    assert cols[6] == "24.000"          # vbus_v
    assert cols[7] == "1.500"           # iq_a
    assert cols[8] == "36.000"          # power_w


def test_faults_recorded_in_last_column(tmp_path):
    path = tmp_path / "telemetry.txt"
    logger = TelemetryLogger()
    logger.start(path)

    logger.log_status(3, 0.0, 0.0, 0.0, 90.0, faults="overtemp")
    logger.stop()

    cols = path.read_text().splitlines()[1].rstrip("\n").split("\t")
    assert cols[-1] == "overtemp"
