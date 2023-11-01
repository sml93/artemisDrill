#! /usr/bin/env python3
import time
import rospy
import numpy as np
from statistics import median
from collections import deque
from std_msgs.msg import Header, Float32, Int16, Empty
from geometry_msgs.msg import PoseStamped


ranger = 1.0
rpm = 1 
throttle = 0
current = 0.0

throttle_pub = rospy.Publisher('/throttle_pub', Float32, queue_size=10)
rpm_pub = rospy.Publisher('/rpm_pub', Int16, queue_size=10)
range_pub = rospy.Publisher('/ranger_pub', Float32, queue_size=10)
current_pub = rospy.Publisher('/current_pub', Float32, queue_size=10)

throttle_msg = Float32()
rpm_msg = Int16()
range_msg = Float32()
current_msg = Float32()


def rolling_median(l, n):
    d = deque(l[0:n], n)
    yield median(d)
    for num in l[n:]:
        d.append(num)
        yield median(d)


def standoff_callback(msg):
    global ranger
    ranger = msg.data


def rpm_callback(msg):
    global rpm
    if msg.data == 0:
        rpm = 1
    else:
        rpm = msg.data


def throttle_callback(msg):
    global throttle
    throttle = msg.data


def currrent_callback(msg):
    global current
    curr_list = []
    # current = msg.data
    for i in range(0,15):
        curr_list.append(19.41 * msg.data)
    current = sum(curr_list)/len(curr_list)
    # current = rolling_median(curr_list, 3)
    # current = float(current)


def sender():
    global ranger, rpm, throttle, current
    throttle_pub.publish(throttle)
    rpm_pub.publish(rpm)
    range_pub.publish(ranger)
    current_pub.publish(current)


def listener():
    rospy.Subscriber('/ranger', Float32, standoff_callback)
    rospy.Subscriber('/rpm', Int16, rpm_callback)
    rospy.Subscriber('/throttle', Float32, throttle_callback)
    rospy.Subscriber('/curr', Float32, currrent_callback)
    rospy.init_node('topic_sender', anonymous=True)
    while not rospy.is_shutdown():
        sender()
        time.sleep(0.5)


# def sender():
#     global ranger, rpm, throttle

#     rospy.init_node('poc_sender', anonymous=True)

#     throttle_msg = Float32()
#     rpm_msg = Int16()
#     range_msg = Float32()
#     # rate = rospy.Rate(10)

#     while not rospy.is_shutdown():
#         for i in range(0,100):
#             throttle_msg.data = throttleList[i]
#             rpm_msg.data = int(rpmList[i])
#             range_msg.data = rangeList[i]
#             throttle_pub.publish(throttle_msg)
#             rpm_pub.publish(rpm_msg)
#             range_pub.publish(range_msg)
#             print(throttle_msg)
#             print(rpm_msg)
#             print(range_msg)
#             time.sleep(1)
#         # rospy.loginfo(pub_msg)
#         # rospy.loginfo(alp_msg)
#         # rate.sleep()

def main():
    # sender()
    listener()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
