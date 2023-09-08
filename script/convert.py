import csv
import numpy as np

fileRPM = "RPM_reading.txt"

fileCurr = "curr_reading.txt"

data_list = []
data_list2 = []

data = open(fileRPM, "r")
data2 = open(fileCurr, "r")

def converter():
  for line in data:
    # print(line)
    line_strip = line.strip("''\n,")
    data_list.append(line_strip)

  for line2 in data2:
    # print(line)
    line_strip2 = line2.strip("''\n,")
    data_list2.append(line_strip2)


  with open('RPM_converted.csv', 'w') as f:
    writer = csv.writer(f)
    for i in range(len(data_list)):
      # print(data_list[i])
      writer.writerow([data_list[i]])
    print('Writing done...')

  with open('Current_converted.csv', 'w') as f:
    writer2 = csv.writer(f)
    for i in range(len(data_list2)):
      # print(data_list2[i])
      writer2.writerow([data_list2[i]])
    print('Writing done...')

  return data_list, data_list2



  