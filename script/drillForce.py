## For ICRA24 Resistograph Work (without ros features)
## Import basic libraries
import rospy
import numpy as np

## Helper functions
import motor
import linear_cf

from ceilingEffect import thrustCE


class resistograph():
    def __init__(self, throttle, rpm, curr_motor, curr_drill, cdist):
        ''' init params '''
        self.angVel = 10
        self.k_sr = 1
        self.r_prop = 5/39.37
        self.z_ceiling = 0.15       # distance from UAV to ceiling
        self.inputVel = 5
        self.force_ce = 0
        self.thrustEst = 0

        self.throttle = throttle
        self.rpm = rpm
        self.curr_motor = curr_motor
        self.curr_drill = curr_drill
        self.cdist = cdist

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
    

    def beam_drill(self):
        ''' function to for beam drilling '''
        pass

    
    def ceiling_drill(self):
        ''' function for ceiling drilling '''
        thrustEst_N = (4*self.thrustEst/1000.0) * 9.81
        ceiling_force = self.force_ce + thrustEst_N
        print (ceiling_force)

    
    def drillForce(self):
        ''' function to calculate drill force '''
        pass

    
    def powerTransmission(self):
        ''' function to calculate power transmission '''
        pass

    
    def plotter(self):
        ''' function for plotting resistograph '''
        pass


def main():
    run = resistograph(40, 0, 0, 0, 0.1)
    run.getThrustEst()
    run.ceilingEffect()
    run.ceiling_drill()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass