import openpyxl
from itertools import islice

df = openpyxl.load_workbook("RPM_consolidated.xlsx")

df1 = df.active

rpm_1000 = []
ext_1000 = []
new_1000 = []
rpm_2000 = []
ext_2000 = []
new_2000 = []
rpm_3000 = []
ext_3000 = []
new_3000 = []

''' Get thrust values '''   
def downsample_to_proportion(rows, proportion=1):
    return list(islice(rows, 0, len(rows), int(1/proportion)))

def getRPM():
  for row in range(2, df1.max_row):
    for col in df1.iter_cols(3,3):
      rpm_1000.append(col[row].value)
    for col in df1.iter_cols(7,7):
      rpm_2000.append(col[row].value)
    for col in df1.iter_cols(11,11):
      rpm_3000.append(col[row].value)
  ext_1000 = downsample_to_proportion(range(0,49), 1.0)
  for i in range(len(ext_1000)):
    new_1000.append(float(rpm_1000[ext_1000[i]]))
  ext_2000 = downsample_to_proportion(range(0,49), 1.0)
  for i in range(len(ext_2000)):
    new_2000.append(float(rpm_2000[ext_2000[i]]))
  ext_3000 = downsample_to_proportion(range(0,49), 1.0)
  for i in range(len(ext_3000)):
    new_3000.append(float(rpm_3000[ext_3000[i]]))
  # print(ext_3000)

  # ext_1000 = downsample_to_proportion(range(0,43), 1)
  # for i in range(len(ext_1000)):
  #   ext_1000.append(rpm_1000[ext_1000[i]])
  # print("1000: ", rpm_1000)
  # print("2000: ", rpm_2000)
  # print("3000: ", rpm_3000)
  # print("ext_1000:", ext_1000)
  # print(len(ext_1000))

  return new_1000, new_2000, new_3000

getRPM()