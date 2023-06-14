import serial
import time

if __name__ == '__main__':
    ser = serial.Serial('COM5', 115200, timeout=1)
    print("Opened port: "+ ser.name)
    ser.flush()
    
    while True: 
                input_string = '%s\n' % input(">>:") 
                ser.write(input_string.encode('UTF8'))
                
                time.sleep(.5)
                
                receive_string=ser.readline().decode('UTF8').rstrip()
                
                print(receive_string)