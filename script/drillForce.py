# For Resistograph

import rospy
import numpy as np

from ceilingEffect import thrustCE
from motorThrust import thrustMotor


class resistograph:
    def __init__(self):
        ''' init params '''
        self.angVel = 10
        self.k_sr = 1
        self.r_prop = 5/39.37
        self.z_ceiling = 0.15       # distance from UAV to ceiling
        self.inputVel = 5

        self.force_ce = 0
        self.force_thrust = 0
        pass

    def ceilingEffect(self):
        ''' to get ceiling effect force '''
        self.force_ce = thrustCE
        pass

    def thrustForce(self):
        ''' to get thrust force '''
        self.force_thrust = thrustMotor
        pass

    def beam_drill(self):
        ''' function for beam drilling '''
        pass

    def ceiling_drill(self):
        ''' function for ceiling drilling '''
        pass

    def drillForce(self):
        ''' function to calculate drill force '''
        pass

    def powerTransmission(self):
        ''' function to calculate power transmission '''
        pass

    def plotter(self):
        ''' helper function for plotting resistograph '''
        pass



if __name__ == "__main":
    try:
        resistograph()
    except rospy.ROSInterruptException:
        pass