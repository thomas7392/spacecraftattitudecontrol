'''
This file wil contain real world dynamics of the attitude of the S/C
'''

# Imports
import numpy as np
import scipy.integrate as integrate


def EoM(t,
        y,
        control_torque,
        disturbance_torque,
        J,
        n):
    '''
    Dynamics and Kinematics of the euler angles describing the rotation between the body fixed frame (b)
    and the local vertical/horizontal reference frame (a)
    '''

    deriv = np.zeros(6)

    M = np.array([[np.cos(y[1]), np.sin(y[0])*np.sin(y[1]),  np.cos(y[0])*np.sin(y[1])],
                  [0,            np.cos(y[0])*np.cos(y[1]),  - np.sin(y[0]) * np.cos(y[1])],
                  [0,            np.sin(y[0]),               np.cos(y[0])]])

    # Kinetmatic differential equation
    deriv[:3] = (1 / np.cos(y[1])) * M @ y[3:] + (n/np.cos(y[1])) *\
         np.array([np.sin(y[2]), np.cos(y[1])*np.cos(y[2]), np.sin(y[1])*np.sin(y[2])])

    # Dynamic equations of motion
    deriv[3] = J[0]**(-1.) * ((J[1] - J[2]) * y[4]*y[5] - 3 * n**2 * (J[1] - J[2]) * np.sin(y[0]) *\
         np.cos(y[1]) * np.cos(y[0]) * np.cos(y[1]) + control_torque[0] + disturbance_torque[0])
    deriv[4] = J[1]**(-1.) * ((J[2] - J[0]) * y[5]*y[3] + 3 * n**2 * (J[2] - J[0]) * np.cos(y[0]) *\
         np.cos(y[1]) * np.sin(y[1]) + control_torque[1] + disturbance_torque[1])
    deriv[5] = J[2]**(-1.) * ((J[0] - J[1]) * y[3]*y[4] + 3 * n**2 * (J[0] - J[1]) * np.sin(y[1]) *\
         np.sin(y[0]) * np.cos(y[1]) + control_torque[2] + disturbance_torque[2])

    return deriv


def simulate_dynamics(t0,
                    tf,
                    initial_state,
                    control_torque,
                    disturbance_torque,
                    J,
                    n):
     '''
     Propagete the attitude state (euler angles LVLH frame to body frame in a 3-2-1 sequence,
     rotational rate in the body frame) from t0 to tf given a certain control_torque, a disturbance
     torquea for a spacecraft with diagonal intertia matrix J in a circular orbit with mean orbital
     motion n.
     '''

     result = integrate.solve_ivp(EoM, (t0, tf),
                              initial_state,
                              rtol = 1e-10,
                              atol = 1e-10,
                              args = [control_torque, disturbance_torque, J, n])

     # Extract time and states except for initial state
     states = result.y.T[1:]
     times = result.t[1:]

     states_with_time = np.concatenate((times.reshape(-1, 1), states), axis = 1)

     return states_with_time

