import csv
import time
import serial
import threading as th


ser = serial.Serial('/dev/ttyACM0', 9600)
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
        string = string_n.strip("Current: ")
        print(float(string))
        data.append(string)
    print('out of loop')
    with open('curr_reading.csv', 'w') as f:
        writer = csv.writer(f)
        for line in data:
            # print(line)
            writer.writerow([line])
        print('writing done')


run_sensor()



# for i in range(50):
#     b = ser.readline()
#     string_n = b.decode()
#     string = string_n.rstrip()
#     flt = string
#     print(flt)
#     data.append(flt)
#     time.sleep(0.1)
# ser.close()


# with open('current_reading.csv', 'w') as f:
#     writer = csv.writer(f)
#     for line in data:
#         print(line)
#         writer.writerow(line)


