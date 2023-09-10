## For ICRA24 Resistograph Work (without ros features)
## Import basic libraries
import rospy
import numpy as np

## Helper functions
import motor
import linear_cf
import convert
import logExtract

from ceilingEffect import thrustCE
from matplotlib import pyplot as plt


class resistograph():
    def __init__(self, throttle, rpm, curr_motor, curr_drill, cdist):
        ''' init params '''
        self.angVel = 10
        self.K_srG = 1
        self.K_srL = 1
        self.r_prop = 5/39.37
        self.z_ceiling = 0.15       # distance from UAV to ceiling
        self.inputVel = 5
        self.force_ce = 0
        self.thrustEst = 0
        self.mg = 750/1000 * 9.81
        self.spring_forceL = cdist/self.K_srL
        self.spring_forceG = cdist/self.K_srG
        self.ceiling_feed = 0
        self.beam_feed = 0

        self.throttle = throttle
        self.rpm = rpm
        self.rpm_list = []
        self.curr_motor_list = []
        self.thrust_list = []
        self.curr_motor = curr_motor
        self.curr_drill = curr_drill
        self.cdist = cdist

        self.Fnc =0
        self.Ftc = 0
        self.widthbit = 0.003
        self.FR = 1
        self.depthCut = self.FR/self.rpm
        self.zeta = np.tan(np.deg2rad(40))
        self.epsilon = self.Ftc/(self.widthbit*self.depthCut)
        self.sigma = 0.0025
        self.lamb = 0.0001

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
    

    def beamFeed(self):
        ''' function to calculate beam drilling '''
        thrustEst_N = (4*self.thrustEst/1000.0) * 9.81
        self.beam_feed = thrustEst_N - (self.mg + self.spring_forceG + self.spring_forceL)
        return self.beam_feed

    
    def ceilingFeed(self):
        ''' function for ceiling drilling '''
        thrustEst_N = (4*self.thrustEst/1000.0) * 9.81
        self.ceiling_feed = (4*self.force_ce + thrustEst_N) - (self.mg + self.spring_forceL)
        print (self.ceiling_feed)
        for i in range(len(self.thrust_list)):
            Est_thrust = (4*self.thrust_list[i]*1746.84)/1000 * 9.81
            self.ceiling_feed = (4*self.force_ce + Est_thrust) - (self.mg + self.spring_forceL)
            print (self.ceiling_feed)
        


    def F_nc(self):
        pass

    def F_tc(self):
        return self.epsilon*(self.widthbit*self.depthCut)


    def drillForce(self):
        ''' function to calculate drill resistance - check iPad '''
        resist = (self.zeta*self.epsilon*self.FR)/(2*self.rpm) + (2*self.sigma*self.lamb)
        print(resist)
        return resist

    
    def powerTransmission(self):
        ''' function to calculate power transmission '''
        pass

    
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
        rpm_list, curr_motor_list = convert.converter()
        for i in range(len(rpm_list)):
            self.rpm_list.append(rpm_list[i])
            # print("RPM: ", self.rpm_list[i])
        # print(len(self.rpm_list))

        for i in range(len(curr_motor_list)):
            self.curr_motor_list.append(curr_motor_list[i])
            # print("Current: ", self.curr_motor_list[i])
        
        thrust_list, truncated_list = logExtract.getThrust()
        for i in range(len(truncated_list)):
            self.thrust_list.append(truncated_list[i])
            # print('Thrust: ', self.thrust_list[i])
        # print(len(self.thrust_list))




def main():
    run = resistograph(100, 500, 0, 0, 0.1)
    run.getThrustEst()
    run.getLists()
    run.ceilingEffect()
    run.ceilingFeed()
    run.plotter()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass