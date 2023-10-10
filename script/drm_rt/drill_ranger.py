#! /usr/env/bin python3
import sys
sys.path.insert(0, '../..')
import time
import rospy
import numpy as np

from serial import Serial as device
from std_msgs.msg import String, Float32

TIMEOUT = 0.1
sensor_frequency = 10

serial_port_name = "/dev/ttyUSB0"
serial_port_baudrate = 115200
port = device(serial_port_name, serial_port_baudrate, timeout=TIMEOUT)

# Initialising serial port handshake
port.write('wwww\r\n'.encode())	## encodes the str to byte
port.readline()

# Getting product information
port.write('?\r\n'.encode())	## encodes the str to bytes
productInfo = port.readline()
print('Product information: ' + productInfo.decode('utf-8'))	## decodes the productInfo from bytes to str


def talker():
  pub = rospy.Publisher('ranger', Float32, queue_size=10)
  rospy.init_node('range_sensor', anonymous=True)
  r = rospy.Rate(sensor_frequency)
  msg = Float32()

  while not rospy.is_shutdown():
    try: 
      port.write('LD\r\n'.encode())	## encodes the str to bytes
      distanceStr = port.readline()
      # Convert the distance string response into a number
      distanceCM = float(distanceStr)
      msg.data = round(distanceCM,3)

      # Do what you want with the distance information here
      rospy.loginfo('ranger: {0:.2f} m'.format(distanceCM))
      pub.publish(msg)

      # Wait for 50ms before the next reading is taken
      # time.sleep(0.05)
      time.sleep(1)
    except rospy.ROSException as e:
      print("Interrupted")
      pass
    
    r.sleep()

  rospy.spin()

if __name__ == "__main__":
  try: talker()
  except rospy.ROSInterruptException:
    pass
