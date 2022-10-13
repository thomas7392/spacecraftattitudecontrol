import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
from closed_loop_simulator import simulate_attitude
import os

current_dir = os.path.dirname(os.path.abspath("__file__"))

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
reference_angles = np.array([0, 0, 0])

# Creater initial states
initial_state = np.deg2rad(np.array([roll_0,
                          pitch_0,
                          yaw_0,
                          roll_deriv_0,
                          pitch_deriv_0,
                          yaw_deriv_0]))

# Data on spacecraft
J = np.array([2700, 2300, 3000])
disturbance_torque = np.array([0.001, 0.001, 0.001])
#disturbance_torque=np.array([0, 0, 0])

# Data on orbit
mu = 3.986004418e14
a = 6371e3 + 700e3
period = 2 * np.pi * np.sqrt(a**3/mu)
n = 2 * np.pi / period

#=========================================
# Set Controller properties
#=========================================


#=========================================
# Simulate controller
#=========================================

# # Q3 performance
# state_history, state_history_m, control_torque = simulate_attitude(initial_state, J, reference_angles, n,
#                 disturbance_torque = disturbance_torque,
#                 termination_time = period,
#                 dt_control = 1,
#                 gyro_bias = False,
#                 attitude_noise = False,
#                 state_estimation = False,
#                 control = True)

# sub_directory = "/output/Q3/"
# output_path = current_dir + sub_directory

# #=========================================
# # Store result
# #=========================================
# if os.path.exists(output_path):
#         pass
# else:
#     os.makedirs(output_path)

# np.savetxt(output_path + 'state_history.dat', state_history)
# np.savetxt(output_path + 'control_torque.dat', control_torque)
# np.savetxt(output_path + 'measurements.dat', state_history_m)


# # Q4 simulating measurment noise and control.
# state_history, state_history_m, control_torque_history,\
#     state_history_e, bias_history_e, covariance_history_e = simulate_attitude(initial_state,
#                 J,
#                 reference_angles,
#                 n,
#                 disturbance_torque = disturbance_torque,
#                 termination_time = period,
#                 dt_control = 2,
#                 gyro_bias = True,
#                 attitude_noise = True,
#                 state_estimation = False,
#                 control = True)

# sub_directory = "/output/Q4/zerobias/"
# output_path = current_dir + sub_directory

# #=========================================
# # Store result
# #=========================================
# if os.path.exists(output_path):
#         pass
# else:
#     os.makedirs(output_path)

# np.savetxt(output_path + 'state_history.dat', state_history)
# np.savetxt(output_path + 'control_torque.dat', control_torque)
# np.savetxt(output_path + 'measurements.dat', state_history_m)



# Q8 simulating measurment noise, control and state estimation (EKF)
state_history, state_history_m, control_torque_history,\
    state_history_e, bias_history_e, covariance_history_e = simulate_attitude(
                initial_state,
                J,
                reference_angles,
                n,
                disturbance_torque = disturbance_torque,
                termination_time = period,
                dt_control = 2,
                gyro_bias = True,
                attitude_noise = True,
                state_estimation = True,
                control = True)

sub_directory = "/output/Q8/attempt2/"
output_path = current_dir + sub_directory

#=========================================
# Store result
#=========================================
if os.path.exists(output_path):
        pass
else:
    os.makedirs(output_path)

np.savetxt(output_path + 'state_history.dat', state_history)
np.savetxt(output_path + 'control_torque.dat', control_torque_history)
np.savetxt(output_path + 'measurements.dat', state_history_m)
np.savetxt(output_path + 'state_history_e.dat', state_history_e)
np.savetxt(output_path + 'bias_history_e.dat', bias_history_e)
np.save(output_path + 'covariance_history_e', covariance_history_e)



