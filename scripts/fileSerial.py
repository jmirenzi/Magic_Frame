import serial
import time

if __name__ == '__main__':
    ser = serial.Serial('COM5', 115200, timeout=1)
    print("Opened port: "+ ser.name)
    ser.flush()
    file = open('/test_gcode/test.gcode','r')
    Commands = file.readlines()
    num_commands = len(Commands)

    for command in Commands:
        ser.write(command.encode('UTF8'))


    while True: 
                input_string = '%s\n' % input(">>:") 
                ser.write(input_string.encode('UTF8'))
                time.sleep(.1)
                ser.read(1)