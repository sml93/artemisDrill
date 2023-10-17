#! /usr/bin/env python3
import time
import rospy
import numpy as np
from std_msgs.msg import Header, Float32, Int16
from geometry_msgs.msg import PoseStamped


def powspace(start, stop, power, num):
    start = np.power(start, 1/float(power))
    stop = np.power(stop, 1/float(power))
    return np.power( np.linspace(start, stop, num=num), power)

throttleList = []
throttleList = powspace(0.01, 0.6, 2, 60)
throttleList = throttleList.tolist()
rpmList = np.random.normal(2000, 200, 100)
rangeList = []
rangeList = powspace(0.01, 1.0, 2, 80)
rangeList = rangeList.tolist()

to80_list = []
to80 = powspace(0.6,0.7,2,21)
to80_list = to80.tolist()

for i in range(1,21):
    throttleList.append(to80_list[i])

to100 = powspace(0.7, 0.01, 2, 21)
to100_list = to100.tolist()

for i in range(1,21):
    throttleList.append(to100_list[i])

# print(np.shape(throttleList))
# print(throttleList)    

to100_rangeList = []
to100_range = powspace(1.0, 0.04, 2, 21)
to100_rangelist = to100_range.tolist()
for i in range(1,21):
    rangeList.append(to100_rangelist[i])

# print(np.shape(rangeList))
# print(rangeList) 

currentList = []
currentList = powspace(0.01, 40.37, 2, 60)
currentList = currentList.tolist()
currto100_list = []
currto100 = powspace(40.37, 0, 2, 41)
currto100_list = currto100.tolist()
for i in range(1, 41):
    currentList.append(currto100_list[i])


def sender():
    global throttleList, rpmList, rangeList
    throttle_pub = rospy.Publisher('/throttle', Float32, queue_size=10)
    rpm_pub = rospy.Publisher('/rpm', Int16, queue_size=10)
    range_pub = rospy.Publisher('/ranger', Float32, queue_size=10)
    current_pub = rospy.Publisher('/current', Float32, queue_size=10)
    rospy.init_node('poc_sender', anonymous=True)
    throttle_msg = Float32()
    rpm_msg = Int16()
    range_msg = Float32()
    current_msg = Float32()
    # rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for i in range(0,100):
            throttle_msg.data = throttleList[i]
            rpm_msg.data = int(rpmList[i])
            range_msg.data = rangeList[i]
            current_msg.data = currentList[i]
            throttle_pub.publish(throttle_msg)
            rpm_pub.publish(rpm_msg)
            range_pub.publish(range_msg)
            current_pub.publish(current_msg)
            print('throttle: ', throttle_msg.data)
            print('rpm: ', rpm_msg.data)
            print('range: ', range_msg.data)
            print('current: ', current_msg.data)
            time.sleep(1)
        # rospy.loginfo(pub_msg)
        # rospy.loginfo(alp_msg)
        # rate.sleep()

def main():
    sender()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
