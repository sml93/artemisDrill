## For ICRA24 Resistograph Work (without ros features)
## Import basic libraries
import rospy
import numpy as np
import xlsxwriter

## Helper functions
import motor
import linear_cf
# import convert
import logExtract
import rpmExtract

from ceilingEffect import thrustCE
from matplotlib import pyplot as plt


class resistograph():
    def __init__(self, throttle, rpm, cdist, matThickness, rpmChoice):
        ''' init params '''
        self.angVel = 10
        self.K_srG = 0.1
        self.K_srL = 0.2
        self.r_prop = 5/39.37
        self.z_ceiling = 0.15       # distance from UAV to ceiling
        self.inputVel = 5
        self.force_ce = 0
        self.thrustEst = 0
        self.mg = 750/1000 * 9.81
        self.cdist = cdist
        self.spring_forceL = self.cdist*self.K_srL
        self.spring_forceG = self.cdist*self.K_srG
        self.ceiling_feed = 0
        self.ceiling_feed_list = []
        self.beam_feed = 0
        self.matThickness = matThickness     # mm
        self.feedRate = 0
        self.rpmChoice = rpmChoice

        self.cutTime = 3.
        self.throttle = throttle
        self.rpm = rpm
        self.rpm_list = []
        self.rpm1000 = []
        self.rpm2000 = []
        self.rpm3000 = []
        self.curr_motor_list = []
        self.thrust_list = []
        self.time_list = []
        self.norm = []
        # self.cdist_list = []
        self.timeDrill = []

        self.Fnc =[]
        self.Ftc = []
        self.weightBit = []
        self.widthbit = 0.003
        self.FR = 6/(2.5/60)
        # self.depthCut = self.FR/self.rpm
        self.depthCut = []
        self.zeta = np.tan(np.deg2rad(40))
        # self.epsilon = self.Ftc/(self.widthbit*self.depthCut)
        self.epsilon = 0.95
        self.sigma = 0.0025
        self.lamb = 0.0001
        self.miu = 1    # F_tf/F_nf
        self.ncutter = 2

        self.motorThrust = motor.getThrust()
        self.motorThrottle = motor.getThrottle()
        self.fit_m = 0
        self.fit_c = 0


    def getThrustEst(self):
        ''' function for estimated thrust force - motor spec sheet'''
        self.fit_m, self.fit_c = linear_cf.solveLinear(self.motorThrust, self.motorThrottle)
        self.thrustEst = linear_cf.linear_roots(self.throttle, self.fit_m, self.fit_c)
        print(self.thrustEst)

    
    def ceilingEffect(self):
        ''' function to get ceiling effect force '''
        self.force_ce = thrustCE(self.cdist).getThrust()
        print(self.force_ce)


    def thrustForce(self):
        ''' function to get thrust force '''
        pass


    def depthofCut(self):
        ''' Penetration Rate = thickness / min '''
        for i in range(len(self.ceiling_feed_list)):
            self.feedRate = self.ceiling_feed_list[i]
            # if self.feedRate < (3.0*9.81):
            #     self.cutTime = 3.
            if (3.0*9.81) <= self.feedRate < (4.0*9.81):
                self.cutTime = 1.266          #5
            elif (4.0*9.81) <= self.feedRate < (5.0*9.81):
                self.cutTime = 0.833          #4
            else: self.cutTime = 0.48      #3
            if self.rpmChoice == 1000:
                self.depthCut.append((self.matThickness / self.cutTime)/float(self.rpm1000[i]))
            elif self.rpmChoice == 2000:  
                self.depthCut.append((self.matThickness / self.cutTime)/float(self.rpm2000[i]))
            else:
                # print(self.rpm3000[i])
                self.depthCut.append((self.matThickness/self.cutTime)/float(self.rpm3000[i]))
        # print(self.depthCut)


    def F_nc(self):
        for i in range(len(self.depthCut)):
            self.epsilon = self.Ftc/(self.widthbit*self.depthCut)
            self.Fnc.append((self.zeta*self.epsilon*self.depthCut[i])/self.ncutter + self.ncutter(self.sigma*self.widthbit*self.lamb))


    def F_tc(self):
        for i in range(len(self.depthCut)):
            self.Ftc.append(self.epsilon*(self.widthbit*self.depthCut[i]) + (self.miu*self.sigma*self.widthbit*self.lamb))
    

    def weightOnBit(self):
        wr_list = []
        for i in range(len(self.depthCut)):
            self.weightBit.append((self.zeta*self.epsilon*self.depthCut[i]) + self.ncutter*self.sigma*self.lamb)
            Wr = self.weightBit[i]/self.widthbit
            # print(Wr)
            wr_list.append(Wr)
        self.norm = [(float(i)/max(wr_list)) for i in wr_list]
        print(self.norm)
        # plt.plot(range(len(self.norm)), self.norm, label='data@5000RPM')
        plt.plot(self.timeDrill, self.norm, label='data@3000RPM')
        plt.title('Beam Drilling @ 3000rpm')
        plt.xlabel('Duration of drill [secs]')
        plt.ylabel('Normalised drilling resistance')
        # plt.xlim([30,120])
        plt.legend()
        plt.show()
    

    def beamFeed(self):
        ''' function to calculate beam drilling '''
        thrustEst_N = (4*self.thrustEst/1000.0) * 9.81
        self.beam_feed = thrustEst_N - (self.mg + self.spring_forceG + self.spring_forceL)
        return self.beam_feed

    
    def ceilingFeed(self):
        ''' function for ceiling drilling '''
        # thrustEst_N = (4*self.thrustEst/1000.0) * 9.81
        # self.ceiling_feed = (4*self.force_ce + thrustEst_N) - (self.mg + self.spring_forceL)
        for i in range(len(self.thrust_list)):
            Est_thrust = (4*self.thrust_list[i]*1746.84)/1000 * 9.81
            self.ceiling_feed = (4*self.force_ce + Est_thrust) - (self.mg + self.spring_forceL)
            # print (self.ceiling_feed)
            self.ceiling_feed_list.append(self.ceiling_feed)
        # print(self.ceiling_feed_list)
        plt.plot(range(len(self.ceiling_feed_list)), self.ceiling_feed_list, label='data')
        plt.title('Ceiling Feedrate')
        plt.legend()
        plt.show()


    def drillForce(self):
        ''' function to calculate drill resistance - check iPad '''
        resist = (self.zeta*self.epsilon*self.FR)/(2*self.rpm) + (2*self.sigma*self.lamb)
        print(resist)
        return resist

    
    def powerTransmission(self):
        ''' function to calculate power transmission '''
        r_shaft = (np.power(x_shaft,2) + np.power(y_shaft,2))/(4*y_shaft)
        return r_shaft

    
    def plotter(self):
        ''' function for plotting resistograph '''
        plt.figure()
        plt.plot(range(len(self.rpm_list)), self.rpm_list)
        plt.show()
        plt.figure()
        # plt.plot(range(len(self.curr_motor_list)), self.curr_motor_list)
        # plt.show()
        plt.plot(range(len(self.thrust_list)), self.thrust_list)
        plt.show()


    def getLists(self):
        # rpm_list, time_list = convert.converter()
        # for i in range(len(rpm_list)):
        #     self.rpm_list.append(rpm_list[i])
        #     # print("RPM: ", self.rpm_list[i])
        # # print(len(self.rpm_list))

        # # for i in range(len(curr_motor_list)):
        # #     self.curr_motor_list.append(curr_motor_list[i])
        # #     # print("Current: ", self.curr_motor_list[i])

        # for i in range(len(time_list)):
        #     self.time_list.append(time_list[i])
        #     # print("Current: ", self.curr_motor_list[i])

        
        thrust_list, truncated_list = logExtract.getThrust()
        for i in range(len(truncated_list)):
            self.thrust_list.append(truncated_list[i])
            # print('Thrust: ', self.thrust_list[i])
        # print(len(self.thrust_list))

        # self.rpm1000, self.rpm2000, self.rpm3000 = rpmExtract.getRPM()
        self.rpm3000 = rpmExtract.getRPM()
        self.timeDrill = rpmExtract.getTime()
        self.timeDrill.pop(0)
        # self.cdist_list = cdistExtract.getCdist()




    def saveData(self):
        workbook = xlsxwriter.Workbook("normNew_beam5.xlsx")
        worksheet = workbook.add_worksheet()

        i = 1
        for line in range(len(self.norm)):
            worksheet.write(i, 1, self.norm[line])
            worksheet.write(i, 0, self.timeDrill[line])
            i += 1
        workbook.close()



def main():
    run = resistograph(100, 500, 0.1, 10, 3000)
    run.getThrustEst()
    run.getLists()
    run.ceilingEffect()
    run.ceilingFeed()
    run.depthofCut()
    run.weightOnBit()
    run.saveData()
    run.plotter()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass