"""RobStride custom control GUI.

A robust, dependency-light replacement for the official RobStride MotorStudio
GUI. Speaks the RobStride CAN 2.0 protocol over either:

* a USB-CAN adapter in serial "AT" mode (``/dev/ttyUSB*`` @ 921600), or
* a SocketCAN kernel interface (``can0`` @ 1 Mbps).

The protocol layer is transport-agnostic: every command is built as a
``(comm_type, extra_data, device_id, data)`` tuple and serialized by the active
transport.
"""

__version__ = "0.1.0"
