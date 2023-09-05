import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)

data = []
for i in range(50):
    b = ser.readline()
    string_n = b.decode()
    string = string_n.rstrip()
    flt = string
    print(flt)
    data.append(flt)
    time.sleep(0.1)
ser.close()

for line in data:
    print(line)
