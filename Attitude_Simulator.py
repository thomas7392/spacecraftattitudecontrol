from math import dist
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
from utils import *

#=========================================
# Prepare the simulations
#=========================================

# Initial states
roll_0 = 5 # theta 1
pitch_0 = 5 # theta 2
yaw_0 = 5 # theta 3

roll_deriv_0 = 0 # omega 1
pitch_deriv_0 = 0 # omega 2
yaw_deriv_0 = 0 # omega 3

# Desired angles (reference angles)
roll_reff = 0
pitch_reff = 0
yaw_reff = 0

# Creater initial states
initial_state = np.deg2rad(np.array([roll_0, 
                          pitch_0,
                          yaw_0, 
                          roll_deriv_0, 
                          pitch_deriv_0,
                          yaw_deriv_0]))

# Data on spacecraft
J = np.array([2700, 2300, 3000])
disturbance_torque=np.array([0.001, 0.001, 0.001]) 
#disturbance_torque=np.array([0, 0, 0]) 

# Data on orbit
mu = 3.986004418e14
a = 6371e3 + 700e3
period = 2 * np.pi * np.sqrt(a**3/mu)
n = 2 * np.pi / period

#=========================================
# Set Controller properties
#=========================================


# https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
# Ziegler-Nichols method 
Ku1 = 15
Kt1 = 87

Ku2 =  15
Kt2  = 81

Ku3 = 15
Kt3 = 90


#=========================================
# Simulate controller 
#=========================================
state_history = simulate_attitude(initial_state, J
                disturbance_torque = disturbance_torque,
                termination_time = period, 
                dt_control = 10,
                p = [0.8*Ku1, 0.8 * Ku2, 0.8 * Ku3],
                i = [0, 0, 0],
                d = [1.5 * 0.1 * Kt1 * Ku1, 1.5 * 0.1 * Kt2 * Ku2, 1.5 *  0.1 * Kt3 * Ku3])


#=========================================
# Store result
#=========================================


