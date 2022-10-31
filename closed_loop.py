'''
This file contains the closed loop simulator of an attitude
control simulation
'''

# General Imports
import numpy as np
import scipy.integrate as integrate

# Attitude GNC imports
from dynamics import simulate_dynamics
from measurements import simulate_measurement_from_state
from control import simulate_control
from state_estimation import kalman_filter

def simulate_attitude(initial_state,
            J,
            reference_angles,
            n,
            disturbance_torque = np.array([0, 0, 0]),
            termination_time = 300,
            dt_control = 2,
            gyro_bias = False,
            attitude_noise = False,
            state_estimation = False,
            control = False,
            bias = np.deg2rad(np.array([0.1, -0.1, 0.15])),
            r2 = 1e-9):

    '''
    The closed loop simulator for an earth observing controlled satellite
    in a circular orbit.
    '''

    # Prepare simulations
    time = 0
    control_torque = np.array([0, 0, 0])

    # Prepare real dynamics data storage
    state_history = np.array([np.concatenate((np.array([time]), initial_state))])
    control_torque_history = np.array([np.concatenate((np.array([time]), control_torque))])

    # Prepare measured dynamics data storage
    initial_measurement = simulate_measurement_from_state(state_history[-1],
                                            attitude_noise= attitude_noise,
                                            gyro_bias= gyro_bias,
                                            bias= bias)
    state_history_m = np.array([initial_measurement])

    # Prepare estimated dynamics data storage
    state_history_e = np.copy(state_history_m)
    bias_history_e = np.array([[time, 0, 0, 0]])

    # Prepare initial covariance
    P0 = np.diag([np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1), 1, 1, 1])
    covariance_history_e = np.array([P0])

    # Closed control loop
    while time < termination_time:

        #==========================================
        # Simulate the real dynamics
        #==========================================

        states_with_time = simulate_dynamics(time,
                                        time + dt_control,
                                        state_history[-1, 1:],
                                        control_torque,
                                        disturbance_torque,
                                        J,
                                        n)
        # Store result
        state_history = np.append(state_history, states_with_time, axis = 0)

        #==================================
        # Simulate the measurements
        #==================================

        # Only simulate final state as meausurement because control time = sample time
        states_m_with_time = simulate_measurement_from_state(states_with_time[-1],
                            attitude_noise = attitude_noise,
                            gyro_bias = gyro_bias,
                            bias = bias)

        state_history_m = np.vstack((state_history_m, states_m_with_time))

        #==================================
        # State estimation (EKF)
        #==================================

        if state_estimation:
            estimated_state, estimated_bias, estimated_covariance = kalman_filter(state_history_e[-1,1:],
                                                            bias_history_e[-1,1:],
                                                            state_history_m[-1,1:],
                                                            state_history_m[-2,1:],
                                                            covariance_history_e[-1],
                                                            dt_control,
                                                            control_torque,
                                                            J,
                                                            n,
                                                            r2 = r2)

        else:
            estimated_state = np.copy(states_m_with_time)[1:]
            estimated_bias = np.array([0, 0, 0])
            estimated_covariance = np.zeros((6, 6))

        estimated_state_with_time = np.array([np.concatenate((np.array([state_history[-1,0]]), estimated_state))])
        estimated_bias_with_time = np.array([np.concatenate((np.array([state_history[-1,0]]), estimated_bias))])

        state_history_e = np.append(state_history_e, estimated_state_with_time, axis = 0)
        bias_history_e = np.append(bias_history_e, estimated_bias_with_time, axis = 0)

        covariance_history_e = np.append(covariance_history_e, np.array([estimated_covariance]), axis = 0)

        #==================================
        # Control (PD)
        #==================================

        if control:
            control_torque = simulate_control(estimated_state, reference_angles, J, n)
        else:
            control_torque = np.zeros(3)

        # Store control torque
        control_torque_with_time = np.array([np.concatenate((np.array([time]), control_torque))])
        control_torque_history = np.append(control_torque_history, control_torque_with_time, axis = 0)

        #========================================
        # Prepating to move to next control node
        #========================================

        # Update time and control torque and print progress
        time = time + dt_control
        print("Progress = %.2f %%" % (time*100/termination_time), end="\r")

    return state_history, state_history_m, control_torque_history,\
            state_history_e, bias_history_e, covariance_history_e