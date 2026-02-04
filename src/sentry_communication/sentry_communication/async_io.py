import asyncio
import struct
from sentry_communication.communication import RobotPositionMessage, Serial
from sentry_communication.communication.Receive import parse_frame


def crc8(data):
    """Calculate CRC8 for frame header (bytes 0-3)"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc = crc << 1
        crc &= 0xFF
    return crc

def crc16(data):
    """Calculate CRC16 for entire message (bytes 0 to 6+data_length)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc = crc >> 1
    return crc

class MessageValidationResult:
    def __init__(self, valid, message, errors=None):
        self.valid = valid
        self.message = message
        self.errors = errors or []
    
    def __str__(self):
        if self.valid:
            return "✓ VALID MESSAGE"
        else:
            return f"✗ INVALID MESSAGE: {', '.join(self.errors)}"

class SerialMessageParser:
    def __init__(self, start_byte=0xA5):
        self.buffer = bytearray()
        self.start_byte = start_byte
        self.max_buffer_size = 4096
        
    def feed_data(self, new_data):
        """Call this whenever serial data arrives"""
        self.buffer.extend(new_data)
        
        if len(self.buffer) > self.max_buffer_size:
            self.buffer = self.buffer[-self.max_buffer_size:]
        
        return self.extract_messages()
    
    def extract_messages(self):
        """Extract complete messages from buffer"""
        messages = []
        
        while True:
            # Find start byte (0xA5)
            try:
                start_idx = self.buffer.index(self.start_byte)
            except ValueError:
                self.buffer.clear()
                break
            
            # Remove junk before start byte
            if start_idx > 0:
                print(f"⚠ Discarded {start_idx} bytes of junk data")
                self.buffer = self.buffer[start_idx:]
            
            # Need at least 7 bytes for header
            if len(self.buffer) < 7:
                break  # Incomplete header, wait for more data
            
            # Parse data length (bytes 1-2, little endian)
            data_length = struct.unpack('<H', self.buffer[1:3])[0]
            
            # Sanity check on data length
            if data_length > 1024:  # Adjust based on your max message size
                print(f"⚠ Suspicious data length: {data_length}, discarding frame")
                self.buffer = self.buffer[1:]  # Remove false start byte
                continue
            
            # Total message length: 7 byte header + data_length + 2 byte CRC16
            total_length = 7 + data_length + 2
            
            # Check if complete message arrived
            if len(self.buffer) < total_length:
                print(f"⏳ Waiting for complete message: have {len(self.buffer)}/{total_length} bytes")
                break  # Wait for more data
            
            # Extract complete message
            message = bytes(self.buffer[:total_length])
            
            # Validate the message
            validation = self.validate_message(message)
            messages.append(validation)
            
            # Remove processed message from buffer
            self.buffer = self.buffer[total_length:]
        
        return messages
    
    def validate_message(self, msg):
        """Validate message structure and checksums"""
        errors = []
        
        # Check minimum length
        if len(msg) < 9:  # 7 header + 0 data + 2 CRC16
            errors.append(f"Message too short: {len(msg)} bytes")
            return MessageValidationResult(False, msg, errors)
        
        # Check start byte
        if msg[0] != self.start_byte:
            errors.append(f"Invalid start byte: 0x{msg[0]:02X} (expected 0x{self.start_byte:02X})")
        
        # Parse header
        data_length = struct.unpack('<H', msg[1:3])[0]
        expected_length = 7 + data_length + 2
        
        # Check total length matches
        if len(msg) != expected_length:
            errors.append(f"Length mismatch: got {len(msg)}, expected {expected_length}")
            return MessageValidationResult(False, msg, errors)
        
        # Validate CRC8 (header bytes 0-3)
        header_data = msg[0:4]
        received_crc8 = msg[4]
        calculated_crc8 = crc8(header_data)
        
        if received_crc8 != calculated_crc8:
            errors.append(f"CRC8 mismatch: received 0x{received_crc8:02X}, calculated 0x{calculated_crc8:02X}")
        
        # Validate CRC16 (entire message except last 2 bytes)
        message_data = msg[:-2]
        received_crc16 = struct.unpack('<H', msg[-2:])[0]
        calculated_crc16 = crc16(message_data)
        
        if received_crc16 != calculated_crc16:
            errors.append(f"CRC16 mismatch: received 0x{received_crc16:04X}, calculated 0x{calculated_crc16:04X}")
        
        # Return validation result
        valid = len(errors) == 0
        return MessageValidationResult(valid, msg, errors)


class AsyncSerialReader:
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.parser = SerialMessageParser(start_byte=0xA5)
        
        # Statistics
        self.stats = {
            'total_messages': 0,
            'valid_messages': 0,
            'invalid_messages': 0,
            'bytes_received': 0
        }
    
    async def read_loop(self):
        """Async read loop"""
        while True:
            # Check if data available
            if self.serial_port.port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.port.in_waiting)
                self.stats['bytes_received'] += len(data)
                
                messages = self.parser.feed_data(data)
                
                for validation in messages:
                    self.stats['total_messages'] += 1
                    
                    if validation.valid:
                        self.stats['valid_messages'] += 1
                        self.print_message(validation)
                    else:
                        self.stats['invalid_messages'] += 1
                        self.print_invalid_message(validation)
            
            # Non-blocking sleep
            await asyncio.sleep(0.001)
    
    def print_message(self, validation):
        """Print validated message"""
        msg = validation.message
        
        print(f"\n{'='*60}")
        print(f"✓ VALID MESSAGE ({len(msg)} bytes)")
        print(f"{'='*60}")
        print(f"Raw: {msg.hex()}")
        
        # Parse header
        header = msg[0]
        data_length = struct.unpack('<H', msg[1:3])[0]
        seq_num = msg[3]
        crc8_val = msg[4]
        msg_type = struct.unpack('<H', msg[5:7])[0]
        
        print(f"\nHeader:")
        print(f"  Start Byte:   0x{header:02X}")
        print(f"  Data Length:  {data_length}")
        print(f"  Sequence #:   {seq_num}")
        print(f"  CRC8:         0x{crc8_val:02X} ✓")
        print(f"  Message Type: 0x{msg_type:04X}")
        
        # Payload
        if data_length > 0:
            payload = msg[7:7+data_length]
            print(f"\nPayload ({data_length} bytes):")
            print(f"  Hex: {payload.hex()}")
            
            # Try to display as ASCII if printable
            if all(32 <= b < 127 for b in payload):
                print(f"  ASCII: {payload.decode('ascii')}")
        
        # CRC16
        crc16_val = struct.unpack('<H', msg[-2:])[0]
        print(f"\nCRC16: 0x{crc16_val:04X} ✓")
        
        print(f"{'='*60}\n")
    
    def print_invalid_message(self, validation):
        """Print invalid message with errors"""
        msg = validation.message
        
        print(f"\n{'!'*60}")
        print(f"✗ INVALID MESSAGE ({len(msg)} bytes)")
        print(f"{'!'*60}")
        print(f"Raw: {msg.hex()}")
        print(f"\nErrors:")
        for error in validation.errors:
            print(f"  ✗ {error}")
        
        # Try to parse what we can
        if len(msg) >= 7:
            print(f"\nPartial Parse:")
            print(f"  Start Byte:   0x{msg[0]:02X}")
            data_length = struct.unpack('<H', msg[1:3])[0]
            print(f"  Data Length:  {data_length}")
            print(f"  Sequence #:   {msg[3]}")
            print(f"  CRC8:         0x{msg[4]:02X}")
            msg_type = struct.unpack('<H', msg[5:7])[0]
            print(f"  Message Type: 0x{msg_type:04X}")
        
        print(f"{'!'*60}\n")
    
    def print_stats(self):
        """Print reception statistics"""
        print(f"\n{'='*60}")
        print("STATISTICS")
        print(f"{'='*60}")
        print(f"Total Messages:   {self.stats['total_messages']}")
        print(f"Valid Messages:   {self.stats['valid_messages']}")
        print(f"Invalid Messages: {self.stats['invalid_messages']}")
        print(f"Bytes Received:   {self.stats['bytes_received']}")
        
        if self.stats['total_messages'] > 0:
            success_rate = (self.stats['valid_messages'] / self.stats['total_messages']) * 100
            print(f"Success Rate:     {success_rate:.1f}%")
        
        print(f"{'='*60}\n")


async def print_stats_periodically(reader):
    """Async task to print stats every 10 seconds"""
    while True:
        await asyncio.sleep(10)
        reader.print_stats()


async def main():
    from serial_wrapper import Serial
    
    serial = Serial("/dev/ttyTHS1", 115200)
    reader = AsyncSerialReader(serial)
    
    print("Starting async serial reader with validation... Press Ctrl+C to stop")
    
    # Create tasks
    read_task = asyncio.create_task(reader.read_loop())
    stats_task = asyncio.create_task(print_stats_periodically(reader))
    
    try:
        # Run both tasks concurrently
        await asyncio.gather(read_task, stats_task)
    except KeyboardInterrupt:
        print("\nStopping...")
        reader.print_stats()  # Final stats


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass