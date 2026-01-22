import serial                                                                 
import struct                                                                 
                                                                            
# DJISerial frame format                                                      
FRAME_HEADER = 0xA5                                                           
SENSOR_DATA_MSG_TYPE = 0x0002                                                 
                                                                            
# SensorData struct: 4 floats + 9 floats + 1 uint32 = 56 bytes                
SENSOR_DATA_SIZE = 56                                                         
SENSOR_DATA_FORMAT = '<4f 3f 3f 3f I'  # little-endian                        
                                                                            
def crc8(data):                                                               
    """CRC8 calculation (same as Taproot)"""                                  
    crc = 0xFF                                                                
    for byte in data:                                                         
        crc ^= byte                                                           
        for _ in range(8):                                                    
            if crc & 0x80:                                                    
                crc = (crc << 1) ^ 0x31                                       
            else:                                                             
                crc <<= 1                                                     
            crc &= 0xFF                                                       
    return crc                                                                
                                                                            
def parse_sensor_data(data):                                                  
    values = struct.unpack(SENSOR_DATA_FORMAT, data)                          
    return {                                                                  
        'wheel_velocities': values[0:4],  # LF, LB, RB, RF in rad/s           
        'gyro': {'x': values[4], 'y': values[5], 'z': values[6]},  # rad/s    
        'accel': {'x': values[7], 'y': values[8], 'z': values[9]},  # m/s^2   
        'orientation': {'yaw': values[10], 'pitch': values[11], 'roll':       
values[12]},                                                                  
        'timestamp_ms': values[13]                                            
    }                                                                         
                                                                            
def main():                                                                   
    ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.1)                  
    buffer = bytearray()                                                      
                                                                            
    while True:                                                               
        buffer.extend(ser.read(256))                                          
                                                                            
        # Find frame header                                                   
        while len(buffer) > 0 and buffer[0] != FRAME_HEADER:                  
            buffer.pop(0)                                                     
                                                                            
        # Need at least header (5 bytes) + msg_type (2) + data + crc16 (2)    
        if len(buffer) < 9:                                                   
            continue                                                          
                                                                            
        data_length = buffer[1] | (buffer[2] << 8)                            
        frame_size = 5 + 2 + data_length + 2  # header + msg_type + data + crc16                                                                         
                                                                            
        if len(buffer) < frame_size:                                          
            continue                                                          
                                                                            
        msg_type = buffer[5] | (buffer[6] << 8)                               
                                                                            
        if msg_type == SENSOR_DATA_MSG_TYPE and data_length == SENSOR_DATA_SIZE:                                                             
            sensor_data = parse_sensor_data(bytes(buffer[7:7+SENSOR_DATA_SIZE]))                        
            print(f"Wheels: {sensor_data['wheel_velocities']}")               
            print(f"Gyro: {sensor_data['gyro']}")                             
            print(f"Accel: {sensor_data['accel']}")                           
                                                                            
        buffer = buffer[frame_size:]                                          
                                                                            
if __name__ == '__main__':                                                    
    main()  
