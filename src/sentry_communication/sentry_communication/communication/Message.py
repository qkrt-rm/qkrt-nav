"""
This file is part of HuskyBot CV.
Copyright (C) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import struct
from abc import ABC, abstractmethod

from .CRC import calculateCRC8, calculateCRC16


class DJIMessage(ABC):
    """
    Abstract base class for all messages following the DJI serial protocol.
    Defines the common interface and basic functionality for messages.

    Structure of a Serial Message:
    +-----------------+------------------------------------------------------------+
    | Byte Number     | Byte Description                                           |
    +=================+============================================================+
    | Frame Header                                                                 |
    +-----------------+------------------------------------------------------------+
    | 0               | Frame Head Byte (0xA5)                                     |
    +-----------------+------------------------------------------------------------+
    | 1               | Frame Data Length, LSB                                     |
    +-----------------+------------------------------------------------------------+
    | 2               | Frame Data Length, MSB                                     |
    +-----------------+------------------------------------------------------------+
    | 3               | Frame Sequence Number                                      |
    +-----------------+------------------------------------------------------------+
    | 4               | CRC8 of the frame, (bytes 0 - 3)                           |
    +-----------------+------------------------------------------------------------+
    | 5               | Message Type, LSB                                          |
    +-----------------+------------------------------------------------------------+
    | 6               | Message Type, MSB                                          |
    +-----------------+------------------------------------------------------------+
    | Body - Data Length bytes                                                     |
    +-----------------+------------------------------------------------------------+
    | Message CRC                                                                  |
    +-----------------+------------------------------------------------------------+
    | 7 + Data Length | CRC16 of header and frame, LSB (bytes 0 - 6 + Data Length) |
    +-----------------+------------------------------------------------------------+
    | 8 + Data Length | CRC16 of header and frame, MSB                             |
    +-----------------+------------------------------------------------------------+
    """

    HEAD_BYTE = 0xA5

    @abstractmethod
    def getID(self) -> int:
        """
        Returns the unique identifier for the message type.
        Used by the receiver to distinguish between various message types.
        """
        pass

    def getPayload(self) -> bytes:
        """
        Provides the main data content (payload) of the message.
        To be implemented by subclasses based on specific requirements.
        """
        pass

    def createMessage(self, sequence_num: int = 0) -> bytes:
        """
        Constructs the complete binary representation of the message for transmission.
        This includes a header, payload, and checksum to ensure data integrity.

        Args:
            sequence_num (int): Optional sequence number for tracking message order.

        Returns:
            bytes: Full binary message ready to be sent over a serial connection.
        """

        payload = self.getPayload()
        payload_len = len(payload)

        # Build out the message header
        msg = struct.pack("B", self.HEAD_BYTE)
        msg += struct.pack("<H", payload_len)
        msg += struct.pack("B", sequence_num)
        msg += struct.pack("B", calculateCRC8(msg))

        # Append the payload
        msg += struct.pack("<H", self.getID())
        msg += payload
        msg += struct.pack("<H", calculateCRC16(msg))

        return msg


class RobotPositionMessage(DJIMessage):
    """
    Represents a message that communicates the position of a robot in 3D space.
    The position is encoded as three floats (x, y, z) in little-endian format.
    """

    def __init__(self, position: list[float]):
        """
        Initializes the RobotPositionMessage with the given position.

        Args:
            position (list[float]): An array of 3 floats representing [x, y, z].
        """
        self.position = position

    def getID(self) -> int:
        """
        Returns the unique message ID for RobotPositionMessage.
        ID: 0x01
        """
        return 0x01

    def getPayload(self) -> bytes:
        """
        Constructs the payload containing the robot's position.

        Returns:
            bytes: Encoded position data as 3 floats (x, y, z) in little-endian format.
        """
        # Unpacks the list into the 3 float slots for struct.pack
        return struct.pack("<fff", *self.position)


class NavMessage(DJIMessage):
    """
    Represents a message that communicates the velocity command computed from AutoNav.
    The command is encoded as three floats (Vx, Vy, w) in little-endian format.
    """

    def __init__(self, cmd: list[float]):
        """
        Initializes the NavMessage with the given commanded velocity.

        Args:
            cmd (list[float]): An array of 3 floats representing [Vx, Vy, w].
        """
        self.cmd = cmd

    def getID(self) -> int:
        """
        Returns the unique message ID for NavMessage.
        ID: 0x02
        """
        return 0x02

    def getPayload(self) -> bytes:
        """
        Constructs the payload containing the velocity command.

        Returns:
            bytes: Encoded velocity command data as 3 floats (x, y, z) in little-endian format.
        """
        # Unpacks the list into the 3 float slots for struct.pack
        return struct.pack("<fff", *self.cmd)