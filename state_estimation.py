'''
A kalman filter for attitude state estimation
'''

import numpy as np
from dynamics import EoM

def propagate_state(old_attitude, old_bias, observations, observations_old,
                    control_torque, J, n, dt):
    '''
    Propagate the state. The prediction step.
    '''

    predicted_state = np.zeros(6)

    # predicted_state[:3] = old_attitude[:3] +\
    #      kinematics_with_bias(np.concatenate([old_attitude[:3], observations_old[3:]]), \
    #                         old_bias, n) * dt

    predicted_state[:3] = old_attitude[:3] +  EoM(0, np.concatenate((old_attitude[:3], observations_old[3:] - old_bias)),
                            control_torque, np.array([0, 0, 0]), J, n)[:3] * dt

    predicted_state[3:] = old_bias

    return predicted_state

def makePHI(attitude, bias, n, dt):
    '''
    Create the linearized state transition matrix
    '''
    F = np.zeros((6, 6))

    theta = attitude[:3]
    omega = attitude[3:]

    # d_theta1/dx
    F[0, 0] = (omega[1] - bias[1]) * np.tan(theta[1]) * np.cos(theta[0]) -\
            (omega[2] - bias[2]) * np.tan(theta[1]) * np.sin(theta[0])
    F[0, 1] = ((omega[1] - bias[1]) * np.sin(theta[0]) +\
                (omega[2] - bias[2]) * np.cos(theta[0])) / np.cos(theta[1])**2 +\
                    n * np.sin(theta[2]) * np.tan(theta[1]) / np.cos(theta[1])
    F[0, 2] = n * np.cos(theta[2]) / np.cos(theta[1])

    # d_theta2/dx
    F[1, 0] = - (omega[1] - bias[1]) * np.sin(theta[0]) - (omega[2]-bias[2]) * np.cos(theta[0])
    F[1, 1] = 0
    F[1, 2] = - n * np.sin(theta[2])

    # d_theta3/dx
    F[2, 0] = ((omega[1]-bias[1]) * np.cos(theta[0]) -\
                (omega[2] - bias[2]) * np.sin(theta[0])) / np.cos(theta[1])
    F[2, 1] = ((omega[1] - bias[1]) * np.sin(theta[0]) +\
                (omega[2] - bias[2]) * np.cos(theta[0])) * np.tan(theta[1]) / np.cos(theta[1]) +\
                    n * np.sin(theta[2]) / np.cos(theta[1])**2
    F[2, 2] = n * np.tan(theta[1]) * np.cos(theta[2])


    F[0, 3] = -1
    F[0, 4] = - np.sin(theta[0]) * np.tan(theta[1])
    F[0, 5] = - np.cos(theta[0]) * np.tan(theta[1])

    F[1, 4] = - np.cos(theta[0])
    F[1, 5] = + np.sin(theta[0])

    F[2, 4] = - np.sin(theta[0]) / np.cos(theta[1])
    F[2, 5] = - np.cos(theta[0]) / np.cos(theta[1])

    PHI = np.eye(6) + dt * F @ np.eye(6)
    return PHI

def makeH():
    '''
    Create the linearized observation matrix (in this particular case not linearized)
    '''

    # H = np.diag([1, 1, 1, -1, -1, -1])

    H = np.array([[1, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0]])
    return H

def makeR():
    '''
    Create the measurement noise matrix
    '''

    R = np.diag([np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1)])

    return R

def makeQ(r1, r2):
    '''
    Create the process noise matrix
    '''

    diag = np.concatenate((r1*np.ones(3), r2*np.ones(3)))
    Q = np.diag(diag)
    return Q

def kalman_filter(old_attitude,
                old_bias,
                observations,
                observations_old,
                P_old,
                dt_control,
                control_torque,
                J,
                n,
                r1 = 1e-5,
                r2 = 1e-5):

    # Propagate the state
    predict_state = propagate_state(old_attitude, old_bias, observations, observations_old,
                                                control_torque, J, n, dt_control)

    # Create the linearized state transition matrix and process noise matrix
    PHI = makePHI(np.concatenate((old_attitude[:3], observations_old[3:])), old_bias, n, dt_control)
    Q = makeQ(r1, r2)

    # Propagate the covariance
    P_predict = PHI @ P_old @ PHI.T + Q

    # Calculate the linearized observations matrix
    H = makeH()
    R = makeR()

    # Calculate the difference between observations and predictions
    dy = observations[:3] - predict_state[:3]

    # Calculate kalman gain matrix
    K = P_predict @ H.T @ np.linalg.pinv(H @ P_predict @ H.T + R)
    # K = np.linalg.solve((H @ P_predict @ H.T) + R, P_predict @ H.T)

    # Update state estimation
    update_state = predict_state + K @ dy
    P_update = (np.eye(6) - K @ H) @ P_predict

    # Create estimations
    estimated_attitude = np.zeros(6)
    estimated_bias = update_state[3:]
    estimated_attitude[:3] = update_state[:3]
    estimated_attitude[3:] = observations[3:] - estimated_bias

    return estimated_attitude, estimated_bias, P_update

