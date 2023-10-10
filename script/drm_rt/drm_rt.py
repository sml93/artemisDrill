#! /usr/env/bin python3
import os
import csv
import rospy
import numpy as np
import datetime as dt
import threading as th
import xlsxwriter

## Helper functions
import motor
import linear_cf

from ceilingEffect import thrustCE
from matplotlib import pyplot as plt
from matplotlib import animation as animation
from std_msgs.msg import String, Float32, Int16

tnrfont = {'fontname':'Times New Roman'}
# fig = plt.figure()
# ax = fig.add_subplot(1,1,1)
xs = []
ys = []
log = []
max_wr = 0
keep_going = True 


def key_capture_thread():
  global keep_going
  input()
  keep_going = False


class drmRT():
  def __init__(self, selection):
    self.init_params(selection)
    self.init_nodes()


  def init_params(self, selection):
    global max_wr, keep_going
    ''' topic related '''
    self.cdist = 0.05
    self.rpm = 2000
    self.current = 0
    self.UAVthrust = 0
    self.initialRpm = 0

    ''' system params '''
    self.selection = selection
    self.force_ce = 0
    self.K_spG = 0.1
    self.K_spL = 0.2
    self.mg = 750/1000 * 9.81
    self.F_total = 0
    self.sp_L = (0.08-self.cdist)*self.K_spL
    self.sp_G = (0.08-self.cdist)*self.K_spG
    self.zeta = 0
    self.epsilon = 0.95
    self.sigma = 0.0025
    self.lamb = 0.0001
    self.miu = 1
    self.zeta = np.tan(np.deg2rad(40))
    self.ncutter = 2
    self.motorThrust = motor.getThrust()
    self.motorThrottle = motor.getThrottle()
    self.depthCut = 0
    self.matThickness = 10    # mm
    self.weightBit = 0
    self.widthbit = 0.003
    self.norm_wr = 0
    self.max_wr = max_wr
    self.saveList = []
    self.timeList = []


  def init_nodes(self):
    rospy.init_node('drmRT')
    self.rate = rospy.Rate(100)
    self.init_pubsub()
    self.run_node()
    self.rate.sleep()


  def init_pubsub(self):
    # rospy.Subscriber('/ranger', String, self.ranger_callback)
    rospy.Subscriber('/ranger', Float32, self.ranger_callback)
    rospy.Subscriber('/rpm', Int16, self.rpm_callback)
    # rospy.Subscriber('/current', Float32, self.current_callback)
    rospy.Subscriber('/throttle', Float32, self.thrust_callback)
    self.wr_pub = rospy.Publisher('/wr', Float32, queue_size=1)
    self.wr_msg = Float32()


  def ranger_callback(self, msg):
    self.cdist = msg.data
    rospy.loginfo('ranger: {0:.2f} m'.format(self.cdist))


  def rpm_callback(self, msg):
    self.rpm = msg.data
    rospy.loginfo('rpm: {0:.2f} m'.format(self.rpm))


  def current_callback(self, msg):
    self.current = msg.data
    rospy.loginfo('current: {0:.2f} m'.format(self.current))


  def thrust_callback(self, msg):
    self.UAVthrust = msg.data
    rospy.loginfo('throttle: {0:.2f} m'.format(self.UAVthrust))


  def ce_thrust(self):
    ''' to calculate ceiling effect '''
    self.force_ce = thrustCE(self.cdist).getThrust()
    # print("F_ce: ", self.force_ce)


  def getMotorThrust(self):
    ''' get motor throttle in realtime, convert to thrust '''
    self.fit_m, self.fit_c = linear_cf.solveLinear(self.motorThrust, self.motorThrottle)
    self.thrustEst = linear_cf.linear_roots(self.UAVthrust, self.fit_m, self.fit_c)


  def getTotalThrust(self):
    ''' sum of z-forces on UAV '''
    self.getMotorThrust()       # call prior function
    self.ce_thrust()
    if self.selection == 1:     # ceiling
      self.F_ztotal = self.thrustEst + self.force_ce - self.sp_L
    else:                       # beam
      self.F_ztotal = self.thrustEst - (self.sp_L + self.sp_G)


  def depthofCut(self):
    ''' get depth of cut '''
    self.getTotalThrust()       # call prior function
    if (2.0*9.81) <= self.F_ztotal < (3.0*9.81):
      self.cutTime = 1.266
    elif (3.0*9.81) <= self.F_ztotal < (4.0*9.81):
      self.cutTime = 0.833
    else: self.cutTime = 0.48
    self.depthCut = (self.matThickness/self.cutTime)/self.rpm


  def drm(self):
    ''' for drm '''
    self.depthofCut()           # call prior function
    self.weightBit = (self.zeta*self.epsilon*self.depthCut) + (self.ncutter*self.sigma*self.lamb)
    wr = self.weightBit/self.widthbit
    if wr > self.max_wr:
      self.max_wr = wr
    self.norm_wr = round((wr / self.max_wr), 3)
    self.wr_msg.data = self.norm_wr
    self.wr_pub.publish(self.wr_msg)
    self.dataSave()
    print('wr:', wr)
    print('max_wr: ', self.max_wr)
    print('self.norm_wr', self.norm_wr)
    # self.plotter()
    # self.imgPlotter()


  def dataSave(self):
    self.saveList.append(self.norm_wr)
    self.timeList.append(dt.datetime.now().strftime('%H:%M:%S'))


  def imgPlotter(self):
    plt.figure()
    plt.plot(self.timeList, self.saveList)
    plt.ylabel('Normalized resistance')
    plt.xlabel('Time (HH:MM:SS)')
    plt.title('rt-DRM')
    plt.savefig('drm_rt/figure.svg', dpi=600)
    plt.show()
    # workbook = xlsxwriter.Workbook('data.xlsx')
    # worksheet = workbook.add_worksheet('Sheet1')
    # head = ['norm_wr', 'time']
    # bold = workbook.add_format({'bold':True})

    # worksheet.write_row(0,0,head,bold)
    # for i in range(len(self.saveList)):
    #   worksheet.write(i, 0, self.saveList[i])
    #   worksheet.write(i, 1, self.timeList[i])
    # workbook.close()
    with open('drm_rt/data.csv', 'w') as k:
      writer = csv.writer(k)
      for i in range(len(self.saveList)):
        writer.writerow([self.saveList[i], self.timeList[i]])    


  def plotter(self):
    """ for plotting """
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    ani = animation.FuncAnimation(fig, self.animate, fargs=(xs, ys), interval=1000)
    plt.show()



  def animate(self, i, xs, ys):
    xs.append(dt.datetime.now().strftime('%H:%M:%S'))
    ys.append(self.norm_wr)

    xs = xs[-20:]
    ys = ys[-20:]

    ax.clear()
    ax.plot(xs, ys)
    plt.pause(0.05)

    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Normalized drm over time')
    plt.ylabel('Drill resistance (norm)')


  def run_node(self):
    th.Thread(target=key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()
    while keep_going: 
      self.drm()
      try:
        self.rate.sleep()
      except rospy.ROSInterruptException:
        pass
    self.imgPlotter()


if __name__ == "__main__":
  val = float(input("Select 1: ceiling or 2: beam? \n"))
  try:
    drmRT(val)
    # rospy.spin()
  except rospy.ROSInterruptException:
    pass