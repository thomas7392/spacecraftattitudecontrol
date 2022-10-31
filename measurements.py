'''
Functionailty to simulate the measurements from a gyro and
a star tracker
'''

import numpy as np

def simulate_measurement_from_state(state,
                        attitude_noise = False,
                        gyro_bias= None,
                        bias = None,
                        time = True):
    '''
    Convert the real dynamics to simulated measurements
        time (Bool): True if state[0] is time value, False otherwise
    '''

    # Copy states of the

    state_m = np.copy(state)

    # Add white-normal-noise to euler angle observations from the star trackers
    if attitude_noise:
        state_m[time:3+time] += np.random.normal(loc=0, scale=np.deg2rad(0.1),
                                                        size = (len(state_m[:3])))

    # Add a bias to the gyro observations of the rotational rate in the spacecraft body frame
    if gyro_bias:
        state_m[3+time:] += bias
        #states_m[3:] += np.repeat(bias, len(states_m), axis = 0).reshape(-1, len(states_m)).T

    return state_m

