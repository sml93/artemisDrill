import csv
import numpy as np

fileRPM = "RPM_record.txt"

# fileCurr = "curr_reading.txt"

fileTime = "time_record.txt"

rpm_list = []
# data_list2 = []
time_list = []

data = open(fileRPM, "r")
# data2 = open(fileCurr, "r")
data3 = open(fileTime, "r")

def converter():
  for line in data:
    # print(line)
    line_strip = line.strip("''\n,")
    rpm_list.append(line_strip)

  # for line2 in data2:
  #   # print(line)
  #   line_strip2 = line2.strip("''\n,")
  #   data_list2.append(line_strip2)

  for line3 in data3:
    line_strip3 = line3.strip("''\n,")
    time_list.append(line_strip3)


  with open('RPM_recordConvert.csv', 'w') as f:
    writer = csv.writer(f)
    for i in range(len(rpm_list)):
      # print(data_list[i])
      writer.writerow([rpm_list[i]])
    print('Writing done...')


  with open('time_recordConvert.csv', 'w') as f:
    writer = csv.writer(f)
    for i in range(len(time_list)):
      writer.writerow([time_list[i]])
    print("Writing done...")

  # with open('Current_converted.csv', 'w') as f:
  #   writer2 = csv.writer(f)
  #   for i in range(len(data_list2)):
  #     # print(data_list2[i])
  #     writer2.writerow([data_list2[i]])
  #   print('Writing done...')

  return rpm_list, time_list


if __name__ == "__main__":
  converter()
  