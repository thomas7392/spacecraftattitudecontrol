'''
A kalman filter for attitude state estimation
'''

import numpy as np

def propagate_state(old_state):
    '''
    Propagate the state. The prediction step.
    '''
    next_state = old_state

    return next_state


def makeF():
    '''
    Create the linearized state propagation matrix
    '''
    F = np.zeros((6, 6))
    return F

def makeH():
    '''
    Create the linearized observation matrix
    '''

    H = np.zeros((1, 1))
    return H

def kalman_filter():
    estimated_state = np.zeros(6)
    return estimated_state

