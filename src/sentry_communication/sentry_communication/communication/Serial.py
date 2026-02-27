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

import serial


class Serial:
    """
    Simple wrapper around the pyserial library to simplify serial communication.
    Provides basic functionality for configuring and writing to a serial port.
    """

    def __init__(self, port_name: str, baudrate: int):
        """
        Initializes the serial connection with the specified port and baud rate.
        Ensures the port is open and ready for communication.

        Args:
            port_name (str): The name of the serial port (e.g., 'COM3', '/dev/ttyUSB0', '/dev/ttyTHS0').
            baudrate (int): The communication speed in bits per second.
        """
        self.port = serial.Serial(
            port=port_name,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.port.reset_input_buffer()

        if not self.port.isOpen():
            self.port.open()

    def write(self, data: bytes):
        """
        Sends binary data over the serial port.

        Args:
            data (bytes): The binary data to be transmitted.
        """
        self.port.write(data)

    def read(self, size: int):
        """
        Reads binary data over the serial port.

        Args:
            size (int): Number of bytes to read.
                size – 
        Returns:
            Bytes read from the port.
        """
        return self.port.read(size)