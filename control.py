'''
This file wil contain the PID controller
'''

import numpy as np
from dynamics import EoM

Ku1 = 15
Kt1 = 85 #87

Ku2 =  15
Kt2  = 79 #81

Ku3 = 15
Kt3 = 89 #90

factor = 2.2
#factor = 0


def PID(angle,
        integrated,
        derivative,
        p,
        i,
        d):
    '''
    PID controller
    '''
    moment = p * angle + i * integrated + d * derivative
    return moment


def simulate_control(estimated_state, reference_angles, J, n):
    '''
    Find the optimal control torque with a pid controller
    '''

    # Prepare the PID controller settings
    p = [0.8*Ku1, 0.8 * Ku2, 0.8 * Ku3]
    i = [0, 0, 0]
    d = [factor * 0.1 * Kt1 * Ku1, factor * 0.1 * Kt2 * Ku2, factor *  0.1 * Kt3 * Ku3]

    # Extract proportional error
    roll_error = reference_angles[0] - estimated_state[0]
    pitch_error = reference_angles[1] - estimated_state[1]
    yaw_error = reference_angles[2] - estimated_state[2]

    # Extract derivative of error
    roll_vel = -1 * EoM(0, estimated_state, [0, 0, 0], [0, 0, 0], J, n)[0]
    pitch_vel = -1 * EoM(0, estimated_state, [0, 0, 0], [0, 0, 0], J, n)[1]
    yaw_vel = -1 * EoM(0, estimated_state, [0, 0, 0], [0, 0, 0], J, n)[2]

    roll_int = 0
    pitch_int = 0
    yaw_int = 0

    # Find the control torque
    roll_moment = PID(roll_error, roll_int, roll_vel, p[0], i[0], d[0])
    pitch_moment = PID(pitch_error, pitch_int, pitch_vel, p[1], i[1], d[1])
    yaw_moment = PID(yaw_error, yaw_int, yaw_vel, p[2], i[2], d[2])

    control_torque = np.array([roll_moment, pitch_moment, yaw_moment])

    return control_torque