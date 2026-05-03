import threading
import queue
import struct
from sentry_communication.communication import RobotPositionMessage, Serial
from sentry_communication.communication.Receive import parse_frame

class SerialMessageParser:
    # Same as Option 1
    def __init__(self, start_byte=0xA5):
        self.buffer = bytearray()
        self.start_byte = start_byte
        self.max_buffer_size = 4096
        
    def feed_data(self, new_data):
        self.buffer.extend(new_data)
        if len(self.buffer) > self.max_buffer_size:
            self.buffer = self.buffer[-self.max_buffer_size:]
        return self.extract_messages()
    
    def extract_messages(self):
        messages = []
        while True:
            try:
                start_idx = self.buffer.index(self.start_byte)
            except ValueError:
                self.buffer.clear()
                break
            
            if start_idx > 0:
                self.buffer = self.buffer[start_idx:]
            
            if len(self.buffer) < 7:
                break
            
            data_length = struct.unpack('<H', self.buffer[1:3])[0]
            total_length = 7 + data_length + 2
            
            if len(self.buffer) < total_length:
                break
            
            message = bytes(self.buffer[:total_length])
            messages.append(message)
            self.buffer = self.buffer[total_length:]
        
        return messages


class EventDrivenSerialReader:
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.parser = SerialMessageParser(start_byte=0xA5)
        self.message_queue = queue.Queue()
        self.running = False
        self.read_thread = None
        self.process_thread = None
        
    def start(self):
        """Start reader and processor threads"""
        self.running = True
        
        # Thread 1: Read serial data
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        
        # Thread 2: Process messages
        self.process_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.process_thread.start()
    
    def _read_loop(self):
        """Continuously read and parse serial data"""
        while self.running:
            if self.serial_port.port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.port.in_waiting)
                messages = self.parser.feed_data(data)
                
                for msg in messages:
                    self.message_queue.put(msg)
            else:
                time.sleep(0.001)
    
    def _process_loop(self):
        """Process messages from queue"""
        while self.running:
            try:
                msg = self.message_queue.get(timeout=0.1)
                self.print_message(msg)
            except queue.Empty:
                continue
    
    def print_message(self, msg):
        """Print parsed message"""
        print(f"\n{'='*60}")
        print(f"Raw message ({len(msg)} bytes): {msg.hex()}")
        
        if len(msg) >= 7:
            header = msg[0]
            data_length = struct.unpack('<H', msg[1:3])[0]
            seq_num = msg[3]
            crc8 = msg[4]
            msg_type = struct.unpack('<H', msg[5:7])[0]
            
            print(f"Header: 0x{header:02X}")
            print(f"Data Length: {data_length}")
            print(f"Sequence Number: {seq_num}")
            print(f"CRC8: 0x{crc8:02X}")
            print(f"Message Type: 0x{msg_type:04X}")
            
            if len(msg) >= 7 + data_length:
                payload = msg[7:7+data_length]
                print(f"Payload: {payload.hex()}")
                
            if len(msg) >= 7 + data_length + 2:
                crc16 = struct.unpack('<H', msg[7+data_length:7+data_length+2])[0]
                print(f"CRC16: 0x{crc16:04X}")
        
        print(f"{'='*60}")
    
    def stop(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join()
        if self.process_thread:
            self.process_thread.join()


if __name__ == "__main__":
    from serial_wrapper import Serial
    
    serial = Serial("/dev/ttyTHS1", 115200)
    reader = EventDrivenSerialReader(serial)
    
    print("Starting event-driven serial reader... Press Ctrl+C to stop")
    reader.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
        reader.stop()