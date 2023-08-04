import serial
import time


if __name__ == '__main__':
    ser = serial.Serial('COM5', 115200, timeout=1)
    print("Opened port: "+ ser.name)
    ser.flush()
    file = open('C:/Users/jmire/Documents/PlatformIO/Projects/Magic_Frame/scripts/test_gcode/test.gcode','r')
    Commands = file.readlines()
    num_commands = len(Commands)
    counter=0

    for command in Commands:
        counter+=1
        # print(command)
        print("{:3.2f}%".format(100*counter/num_commands),end="\r")
        ser.write(command.encode('UTF8'))
        while True:
            if ser.read().decode() == ".":
                break
