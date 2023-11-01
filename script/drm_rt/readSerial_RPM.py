import csv
import time
import serial
import threading as th


ser = serial.Serial('/dev/ttyACM0', 9600)
# ser = serial.Serial('/dev/ttyACM1', 9600)
time.sleep(2)


keep_going = True


data = []
time_list = []


def key_capture_thread():
    global keep_going
    input()
    keep_going = False


def run_sensor():
    ''' Press enter to break out of the loop '''
    time_start = time.time() 
    th.Thread(target=key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()
    while keep_going:
        b = ser.readline()
        string_n = b.decode()
        string = string_n.strip("RPM:")
        time_count = time.time() - time_start
        print(int(time_count), float(string))
        data.append(string)
        time_list.append(time_count)
    print('out of loop')
    with open('RPM_22sept_airena.csv', 'w') as f:
        writer = csv.writer(f)
        for line in data:
            # print(line)
            writer.writerow([line])
        print('writing data done')

    with open('time_22sept_airena.csv', 'w') as k:
        writer = csv.writer(k)
        for i in range(len(time_list)):
            writer.writerow([time_list[i]])
        print('writing timestamp done')

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
