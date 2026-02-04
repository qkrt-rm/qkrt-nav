#threaded blocking read
import struct
import threading
import time
from collections import Counter

from sentry_communication.communication import RobotPositionMessage, Serial
from sentry_communication.communication.Receive import parse_frame
from sentry_communication.communication.CRC import calculateCRC8, calculateCRC16
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

# MCB uses UART1 at 115200 baud


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
    def __init__(self, logger, start_byte=0xA5):
        self.buffer = bytearray()
        self.start_byte = start_byte
        self.max_buffer_size = 4096
        self.logger = logger
        self.debug_mode = True  # Enable debug output
        self.chunk_count = 0
        
    def feed_data(self, new_data):
        """Call this whenever serial data arrives"""
        self.chunk_count += 1
        
        # DEBUG: Show first few chunks
        if self.chunk_count <= 3:
            self.logger.info(f"\n🔍 Chunk {self.chunk_count}: Received {len(new_data)} bytes")
            self.logger.info(f"   Hex (first 50): {new_data[:50].hex()}")
            self.logger.info(f"   Dec (first 20): {[f'{b:3d}' for b in new_data[:20]]}")
            
            if self.start_byte in new_data:
                idx = new_data.index(self.start_byte)
                self.logger.info(f"   ✓ Found start byte 0x{self.start_byte:02X} at position {idx}")
                self.logger.info(f"   Next 20 bytes: {new_data[idx:idx+20].hex()}")
            else:
                self.logger.warn(f"   ✗ No start byte 0x{self.start_byte:02X} in this chunk")
        
        self.buffer.extend(new_data)
        
        if len(self.buffer) > self.max_buffer_size:
            self.buffer = self.buffer[-self.max_buffer_size:]
        
        return self.extract_messages()
    
    def extract_messages(self):
        """Extract complete messages from buffer"""
        messages = []
        
        if self.debug_mode and self.chunk_count <= 5:
            self.logger.info(f"\n📦 Parser state: buffer has {len(self.buffer)} bytes")
        
        while True:
            # Find start byte (0xA5)
            try:
                start_idx = self.buffer.index(self.start_byte)
                if self.debug_mode and self.chunk_count <= 5:
                    self.logger.info(f"   ✓ Found start byte 0x{self.start_byte:02X} at index {start_idx}")
            except ValueError:
                if self.debug_mode and self.chunk_count <= 5:
                    self.logger.warn(f"   ✗ No start byte 0x{self.start_byte:02X} found in buffer")
                    self.logger.warn(f"   Buffer contents (first 50 bytes): {bytes(self.buffer[:50]).hex()}")
                self.buffer.clear()
                break
            
            # Remove junk before start byte
            if start_idx > 0:
                self.logger.warn(f"   ⚠ Discarded {start_idx} bytes of junk data")
                self.buffer = self.buffer[start_idx:]
            
            # Need at least 7 bytes for header
            if len(self.buffer) < 7:
                if self.debug_mode and self.chunk_count <= 5:
                    self.logger.info(f"   ⏳ Need 7 bytes for header, only have {len(self.buffer)}")
                break  # Incomplete header, wait for more data
            
            # Parse data length (bytes 1-2, little endian)
            data_length = struct.unpack('<H', self.buffer[1:3])[0]
            
            if self.debug_mode and self.chunk_count <= 5:
                self.logger.info(f"   📏 Data length field says: {data_length} bytes")
            
            # Sanity check on data length
            if data_length > 1024:  # Adjust based on your max message size
                self.logger.warn(f"   ⚠ Suspicious data length: {data_length}, discarding frame")
                self.logger.warn(f"   Header bytes: {bytes(self.buffer[:7]).hex()}")
                self.buffer = self.buffer[1:]  # Remove false start byte
                continue
            
            # Total message length: 7 byte header + data_length + 2 byte CRC16
            total_length = 7 + data_length + 2
            
            if self.debug_mode and self.chunk_count <= 5:
                self.logger.info(f"   📐 Total message should be: {total_length} bytes (7 header + {data_length} data + 2 CRC)")
            
            # Check if complete message arrived
            if len(self.buffer) < total_length:
                if self.debug_mode and self.chunk_count <= 5:
                    self.logger.info(f"   ⏳ Waiting for complete message: have {len(self.buffer)}/{total_length} bytes")
                break  # Wait for more data
            
            # Extract complete message
            message = bytes(self.buffer[:total_length])
            
            if self.debug_mode and self.chunk_count <= 5:
                self.logger.info(f"   ✓ Extracted complete message: {message.hex()}")
            
            # Validate the message
            validation = self.validate_message(message)
            messages.append(validation)
            
            # Remove processed message from buffer
            self.buffer = self.buffer[total_length:]
            
            if self.debug_mode and self.chunk_count <= 5:
                self.logger.info(f"   🗑️  Removed message from buffer, {len(self.buffer)} bytes remaining")
        
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
        calculated_crc8 = calculateCRC8(header_data)
        
        if received_crc8 != calculated_crc8:
            errors.append(f"CRC8 mismatch: received 0x{received_crc8:02X}, calculated 0x{calculated_crc8:02X}")
        
        # Validate CRC16 (entire message except last 2 bytes)
        message_data = msg[:-2]
        received_crc16 = struct.unpack('<H', msg[-2:])[0]
        calculated_crc16 = calculateCRC16(message_data)
        
        if received_crc16 != calculated_crc16:
            errors.append(f"CRC16 mismatch: received 0x{received_crc16:04X}, calculated 0x{calculated_crc16:04X}")
        
        # Return validation result
        valid = len(errors) == 0
        return MessageValidationResult(valid, msg, errors)


class SerialReader:
    def __init__(self, serial_port, logger):
        self.serial_port = serial_port
        self.parser = SerialMessageParser(logger, start_byte=0xA5)
        self.running = False
        self.thread = None
        self.logger = logger
        
        # Statistics
        self.stats = {
            'total_messages': 0,
            'valid_messages': 0,
            'invalid_messages': 0,
            'bytes_received': 0
        }
        self.all_bytes = bytearray()  # Keep all bytes for analysis
        self.read_count = 0
        
    def start(self):
        """Start reading in dedicated thread"""
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
    
    def _read_loop(self):
        """Continuous blocking read - never misses data"""
        while self.running:
            # Check if data is available
            if self.serial_port.port.in_waiting > 0:
                # Read all available data
                data = self.serial_port.read(self.serial_port.port.in_waiting)
                self.stats['bytes_received'] += len(data)
                self.all_bytes.extend(data)
                self.read_count += 1
                
                # DEBUG: Show byte frequency every 10 reads
                if self.read_count % 10 == 0:
                    self.analyze_byte_frequency()
                
                messages = self.parser.feed_data(data)
                
                for validation in messages:
                    self.stats['total_messages'] += 1
                    
                    if validation.valid:
                        self.stats['valid_messages'] += 1
                        self.print_message(validation)
                    else:
                        self.stats['invalid_messages'] += 1
                        self.print_invalid_message(validation)
            else:
                time.sleep(0.001)  # Small sleep to prevent CPU spinning
    
    def analyze_byte_frequency(self):
        """Analyze most common bytes in received data"""
        if len(self.all_bytes) > 0:
            byte_counts = Counter(self.all_bytes)
            self.logger.info('\n📊 Most common bytes in all received data:')
            for byte_val, count in byte_counts.most_common(10):
                percentage = (count / len(self.all_bytes)) * 100
                self.logger.info(
                    f'   0x{byte_val:02X} ({byte_val:3d}): '
                    f'{count:5d} times ({percentage:.1f}%)'
                )
    
    def print_message(self, validation):
        """Print validated message"""
        msg = validation.message
        
        self.logger.info("="*60)
        self.logger.info(f"✓ VALID MESSAGE ({len(msg)} bytes)")
        self.logger.info("="*60)
        self.logger.info(f"Raw: {msg.hex()}")
        
        # Parse header
        header = msg[0]
        data_length = struct.unpack('<H', msg[1:3])[0]
        seq_num = msg[3]
        crc8_val = msg[4]
        msg_type = struct.unpack('<H', msg[5:7])[0]
        
        self.logger.info("Header:")
        self.logger.info(f"  Start Byte:   0x{header:02X}")
        self.logger.info(f"  Data Length:  {data_length}")
        self.logger.info(f"  Sequence #:   {seq_num}")
        self.logger.info(f"  CRC8:         0x{crc8_val:02X} ✓")
        self.logger.info(f"  Message Type: 0x{msg_type:04X}")
        
        # Payload
        if data_length > 0:
            payload = msg[7:7+data_length]
            self.logger.info(f"Payload ({data_length} bytes):")
            self.logger.info(f"  Hex: {payload.hex()}")
            
            # Try to display as ASCII if printable
            if all(32 <= b < 127 for b in payload):
                self.logger.info(f"  ASCII: {payload.decode('ascii')}")
        
        # CRC16
        crc16_val = struct.unpack('<H', msg[-2:])[0]
        self.logger.info(f"CRC16: 0x{crc16_val:04X} ✓")
        self.logger.info("="*60)
    
    def print_invalid_message(self, validation):
        """Print invalid message with errors"""
        msg = validation.message
        
        self.logger.error("!"*60)
        self.logger.error(f"✗ INVALID MESSAGE ({len(msg)} bytes)")
        self.logger.error("!"*60)
        self.logger.error(f"Raw: {msg.hex()}")
        self.logger.error("Errors:")
        for error in validation.errors:
            self.logger.error(f"  ✗ {error}")
        
        # Try to parse what we can
        if len(msg) >= 7:
            self.logger.error("Partial Parse:")
            self.logger.error(f"  Start Byte:   0x{msg[0]:02X}")
            data_length = struct.unpack('<H', msg[1:3])[0]
            self.logger.error(f"  Data Length:  {data_length}")
            self.logger.error(f"  Sequence #:   {msg[3]}")
            self.logger.error(f"  CRC8:         0x{msg[4]:02X}")
            msg_type = struct.unpack('<H', msg[5:7])[0]
            self.logger.error(f"  Message Type: 0x{msg_type:04X}")
        
        self.logger.error("!"*60)
    
    def print_stats(self):
        """Print reception statistics"""
        self.logger.info("="*60)
        self.logger.info("STATISTICS")
        self.logger.info("="*60)
        self.logger.info(f"Total Messages:   {self.stats['total_messages']}")
        self.logger.info(f"Valid Messages:   {self.stats['valid_messages']}")
        self.logger.info(f"Invalid Messages: {self.stats['invalid_messages']}")
        self.logger.info(f"Bytes Received:   {self.stats['bytes_received']}")
        self.logger.info(f"Read Operations:  {self.read_count}")
        
        if self.stats['bytes_received'] > 0:
            self.logger.info(f"Avg Bytes/Read:   {self.stats['bytes_received'] / max(1, self.read_count):.1f}")
        
        if self.stats['total_messages'] > 0:
            success_rate = (self.stats['valid_messages'] / self.stats['total_messages']) * 100
            self.logger.info(f"Success Rate:     {success_rate:.1f}%")
        
        self.logger.info("="*60)
        
        # Show buffer sample if we have data
        if len(self.all_bytes) > 0:
            self.logger.info(f"\n📦 All data sample (first 100 bytes):")
            self.logger.info(f"{bytes(self.all_bytes[:100]).hex()}")
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()


class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyTHS1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('stats_interval', 10.0)
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        stats_interval = self.get_parameter('stats_interval').value
        
        # Initialize serial port
        self.get_logger().info(f"Initializing serial port {serial_port} at {baud_rate} baud...")
        self.serial = Serial(serial_port, baud_rate)
        
        # Wait a moment for port to stabilize
        time.sleep(0.5)
        
        # Initialize serial reader
        self.reader = SerialReader(self.serial, self.get_logger())
        
        # Create timer for stats
        self.stats_timer = self.create_timer(stats_interval, self.print_stats_callback)
        
        # Start reading
        self.get_logger().info("Starting serial reader with validation and debugging...")
        self.get_logger().info("Watch for:")
        self.get_logger().info("  - Raw data chunks (first 3)")
        self.get_logger().info("  - Start byte 0xA5 detection")
        self.get_logger().info("  - Message parsing steps")
        self.get_logger().info("  - Validation results")
        self.reader.start()
    
    def print_stats_callback(self):
        """Timer callback to print statistics"""
        self.reader.print_stats()
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.get_logger().info("Stopping serial reader...")
        self.reader.stop()
        self.reader.print_stats()  # Final stats
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = SerialReaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()