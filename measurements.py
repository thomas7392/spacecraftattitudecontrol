'''
Functionailty to simulate the measurements from a gyro and
a star tracker
'''

import numpy as np

def simulate_measurements(states,
                        attitude_noise = False,
                        gyro_bias= None,
                        bias = None):
    '''
    Convert the real dynamics to simulated measurements
    '''

    # Copy states of the
    states_m = np.copy(states)

    # Add white-normal-noise to euler angle observations from the star trackers
    if attitude_noise:
        states_m[:,:3] += np.random.normal(loc=0, scale=np.deg2rad(0.1),
                                                        size = (len(states), 3))

    # Add a bias to the gyro observations of the rotational rate in the spacecraft body frame
    if gyro_bias:
        states_m[:,3:] += np.repeat(bias, len(states), axis = 0).reshape(-1, len(states)).T

    return states_m

