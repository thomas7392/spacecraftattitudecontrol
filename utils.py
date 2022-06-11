'''
This file wil contain well established utils for the attitude assignment
'''

# Imports 
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate



# Dynamics and Kinematics of the euler angles describing the rotation between the body fixed frame (b) 
# and the local vertical/horizontal reference frame (a)
def EoM(t, y, control_torque, disturbance_torque, J):
    
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

# Controller
def PID(angle, integrated, derivative, p, i, d):
    moment = p * angle + i * integrated + d * derivative
    return moment


# Closed loop simulator 
def simulate_attitude(initial_state, J
            disturbance_torque = np.array([0, 0, 0]),
            termination_time = 300, 
            dt_control = 2, 
            p = [0, 0, 0],
            i = [0., 0., 0],
            d = [0., 0, 0], 
            gyro_bias = False, 
            attitude_noise = False, 
            state_estimation = False):
    
    # Prepare simulations
    time = 0
    control_torque = np.array([0, 0, 0])
    state_history = np.array([np.concatenate((np.array([time]), initial_state))])
    state_history_m = np.array([])
    
    # Control loop
    while time < termination_time:

        #==========================================
        # Simulate the real dynamics
        #==========================================
        result = integrate.solve_ivp(EoM, (time, time + dt_control), 
                                     state_history[-1, 1:], 
                                     rtol = 1e-10, 
                                     atol = 1e-10,
                                     args = [control_torque])

        # Extract time and states except for initial state
        states = result.y.T[1:]
        times = result.t[1:]

        # Store history
        states_with_time = np.concatenate((times.reshape(-1, 1), states), axis = 1) 
        state_history = np.append(state_history, states_with_time, axis = 0) 
        
        #==================================
        # Simulate the measurements 
        # (with noise)
        # TODO
        #==================================
        
        # if attitude_noise:
        #     attitude_measurements = states[:,1:4
        # else:
        #     attitude_measurements = state_history[
        
        # if gyro_noise: 
            
        #     #TODO
        #     pass
        # else:
            
        #     pass
        

        #==================================
        # State estimation (EKF)
        # TODO
        #==================================
        
        if state_estimation: 
            #TODO
            pass

        #==================================
        # Control (PD)
        #==================================
        
        # Extract proportional error
        roll_error = 1 * (roll_reff - states[-1, 0] )
        pitch_error = 1 * (pitch_reff - states[-1, 1])
        yaw_error = 1 * ( yaw_reff - states[-1, 2])

        # Extract derivative of error
        roll_vel = -1 * EoM(0, state_history[-1,1:], [0, 0, 0])[0]
        pitch_vel = -1 * EoM(0, state_history[-1,1:], [0, 0, 0])[1]
        yaw_vel = -1 * EoM(0, state_history[-1,1:], [0, 0, 0])[2]

        # Extract integration of error
        roll_int = -1*integrate.trapezoid(result.y.T[:,0], x=result.t)
        pitch_int = -1*integrate.trapezoid(result.y.T[:,1], x=result.t)
        yaw_int = -1*integrate.trapezoid(result.y.T[:,2], x=result.t)

        # roll_int = -1 *  integrate.trapezoid(state_history[:,1], x=state_history[:,0])
        # pitch_int = -1 * integrate.trapezoid(state_history[:,2], x=state_history[:,0])
        # yaw_int = -1 * integrate.trapezoid(state_history[:,3], x=state_history[:,0])

        # Find the control torque 
        roll_moment = PID(roll_error, roll_int, roll_vel, p[0], i[0], d[0])
        pitch_moment = PID(pitch_error, pitch_int, pitch_vel, p[1], i[1], d[1])
        yaw_moment = PID(yaw_error, yaw_int, yaw_vel, p[2], i[2], d[2])

        
        #========================================
        # Prepating to move to next control node
        #========================================
        
        # Update time and control torque and print progress
        control_torque = np.array([roll_moment, pitch_moment, yaw_moment])
        time = time + dt_control
        print("Progress = ", time*100/termination_time, "%", end="\r")
        
    return state_history