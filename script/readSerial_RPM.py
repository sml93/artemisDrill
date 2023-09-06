import csv
import time
import serial
import threading as th


# ser = serial.Serial('/dev/ttyACM0', 9600)
ser = serial.Serial('/dev/ttyACM1', 9600)
time.sleep(2)


keep_going = True


data = []


def key_capture_thread():
    global keep_going
    input()
    keep_going = False


def run_sensor():
    ''' Press enter to break out of the loop '''
    th.Thread(target=key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()
    while keep_going:
        b = ser.readline()
        string_n = b.decode()
        string = string_n.strip("RPM:")
        print(int(string))
        data.append(int(string))
    print('out of loop')
    with open('RPM_reading.csv', 'w') as f:
        writer = csv.writer(f)
        for line in data:
            # print(line)
            writer.writerow(line)
        print('writing done')

run_sensor()



# while True:
#     print('run')
#     if keyboard.is_pressed("q"):
#         print("q pressed, ending loop")
#         break

# print('out')



# for i in range(100):
#     b = ser.readline()
#     string_n = b.decode()
#     string = string_n.rstrip()
#     flt = string
#     print(flt)
#     data.append(flt)
#     time.sleep(0.1)
# ser.close()


# for line in data:
    # print(line)
