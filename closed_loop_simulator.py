'''
This file contains the closed loop simulator of an attitude
control simulations
'''

# General Imports
import numpy as np
import scipy.integrate as integrate

# Attitude GNC imports
from dynamics import simulate_dynamics
from measurements import simulate_measurements
from control import simulate_control
from state_estimation import kalman_filter

def simulate_attitude(initial_state,
            J,
            reference_angles,
            n,
            disturbance_torque = np.array([0, 0, 0]),
            termination_time = 300,
            dt_control = 2,
            p = [0.8*Ku1, 0.8 * Ku2, 0.8 * Ku3],
            i = [0, 0, 0],
            d = [factor * 0.1 * Kt1 * Ku1, factor * 0.1 * Kt2 * Ku2, factor *  0.1 * Kt3 * Ku3],
            gyro_bias = False,
            attitude_noise = False,
            state_estimation = False,
            control = False,
            bias = np.deg2rad(np.array([0.1, -0.1, 0.15]))):

    '''
    The closed loop simulator for an earth observing controlled satellite
    in a circular orbit
    '''

    # Prepare simulations
    time = 0
    control_torque = np.array([0, 0, 0])

    # Prepare real dynamics data storage
    state_history = np.array([np.concatenate((np.array([time]), initial_state))])
    control_torque_history = np.array([np.concatenate((np.array([time]), control_torque))])

    # Prepare measured dynamics data storage
    state_history_m = np.copy(state_history)
    if attitude_noise:
        state_history_m[0,1:4] += np.random.normal(loc=0, scale=np.deg2rad(0.1), size = 3)
    if gyro_bias:
        state_history_m[0,4:] += bias

    # Prepare estimated dynamics data storage
    state_history_e = np.copy(state_history_m)

    # Closed control loop
    while time < termination_time:

        #==========================================
        # Simulate the real dynamics
        #==========================================

        states, times = simulate_dynamics(time,
                                        time + dt_control,
                                        state_history[-1, 1:],
                                        control_torque,
                                        disturbance_torque,
                                        J,
                                        n)
        # Store result
        states_with_time = np.concatenate((times.reshape(-1, 1), states), axis = 1)
        state_history = np.append(state_history, states_with_time, axis = 0)

        #==================================
        # Simulate the measurements
        #==================================

        states_m = simulate_measurements(states,
                            attitude_noise = attitude_noise,
                            gyro_bias = gyro_bias,
                            bias = bias)

        states_m_with_time = np.concatenate((times.reshape(-1, 1), states_m), axis = 1)
        state_history_m = np.append(state_history_m, states_m_with_time, axis = 0)

        #==================================
        # State estimation (EKF)
        # TODO
        #==================================

        if state_estimation:
            estimated_state = kalman_filter(estimated_state[-1], states_m)
        else:
            estimated_state = states_m

        estimated_state_with_time = np.array([np.concatenate((np.array([time]), estimated_state))])
        state_history_e = np.append(state_history_e, estimated_state_with_time, axis = 0)

        #==================================
        # Control (PD)
        #==================================

        if control:
            control_torque = simulate_control(states_m[-1], reference_angles, J, n)
        else:
            control_torque = np.zeros(3)

        # Store control torque
        control_torque_with_time = v
        control_torque_history = np.append(control_torque_history, control_torque_with_time, axis = 0)

        #========================================
        # Prepating to move to next control node
        #========================================

        # Update time and control torque and print progress
        time = time + dt_control
        print("Progress = %.2f %%" % (time*100/termination_time), end="\r")

    return state_history, state_history_m, control_torque_history