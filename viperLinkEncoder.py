from enum import Enum
import struct

MAX_KEY_LEN = 16
MAX_FIELD_LEN = 32
CONFIG_KEY_LIST = [
    "YAW_RR", "ROLL_RR", "PITCH_RR",
    "ROLL_PID_KP", "ROLL_PID_KI", "ROLL_PID_KD",
    "PITCH_PID_KP", "PITCH_PID_KI", "PITCH_PID_KD",
]


class viperLinkCommand(Enum):
    Arm = 0
    Disarm = 1
    Control = 2
    Waypoint = 3
    Config = 4
    Reserved = 0xffff


class viperLinkMode(Enum):
    Stabilize = 0
    Race = 1
    GpsHold = 2
    FollowWaypoint = 3
    RetToBase = 4
    Land = 5
    Reserved = 0xff


class viperLinkEncoder:
    def __init__(self):
        pass

    def encodeHeader(self, cmd, payload):
        return struct.pack("!HH", cmd.value, len(payload)) + payload

    def encodeArm(self):
        return self.encodeHeader(viperLinkCommand.Arm, b'')

    def encodeDisarm(self):
        return self.encodeHeader(viperLinkCommand.Disarm, b'')

    def encodeControl(self, pitch, yaw, roll, throttle, mode=viperLinkMode.Stabilize, altHold=0):
        return self.encodeHeader(
            viperLinkCommand.Control,
            struct.pack("!hhhHBB", pitch, yaw, roll, throttle, mode.value, altHold) + b'\0' * 8
        )

    def encodeConfig(self, fieldId: str, value: str):
        if len(fieldId) > MAX_KEY_LEN - 1:
            raise ValueError()
        if len(value) > MAX_FIELD_LEN - 1:
            raise ValueError()
        data = fieldId.encode('ascii').ljust(MAX_KEY_LEN, b'\0')
        data += struct.pack("!H", len(value))
        data += value.encode('ascii').ljust(MAX_FIELD_LEN, b'\0')
        return self.encodeHeader(
            viperLinkCommand.Config,
            data
        )

    def encodeWaypoint(self, guid, action, gps_data):
        # gps data = tuple[2]
        if action:
            action = 1  # remove
        else:
            action = 0  # add
        return self.encodeHeader(
            viperLinkCommand.Waypoint,
            struct.pack("!IIddff", guid, action, gps_data[0], gps_data[1], 0.0, 0.0)
        )
