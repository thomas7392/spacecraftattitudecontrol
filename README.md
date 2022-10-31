# spacecraftattitudecontrol
The code for the master course Spacecraft Attitude Dynamics and Control at the TU Delft in the year 2022. This repo consists of two parts. There are the .py files that make up the attitude controller and there is the analysis folder that contains some `.ipynb` notebooks that analyse some specific cases. 

### The simulator
The entry point for simulating an attitude controller is the `Attitude_simulator.py` file where a bunch of settings can be inserted and where the `simulate_attitude` function is called from. This file will also store the results.
The backbone of this code is the `simulate_attitude` function inside the `control_loop.py` file. This function contains the control loop and calls the other helper `.py` 
files for the propagation (`dynamics.py`), measurement simulation (`measurements.py`), state estimations (`state_estimation.py`) and controller (`control.py`) steps consecutively. 

### The examples 
The analysis folders contains the analysis of three cases: A controller with perfect measurements (`Q3.ipynb`), A controller with practical measurements with angular velocity bias and
euler angle white-normal noise (`Q4.ipynb`) and finally a controller with practical measurements but with a state estimator (`Q8.ipynb`). It also holds the process used to optimise the PD controller in `PD_tuning.ipynb`) 
Currently the `Attitude_simulator.py` is configured such that if is run, it will create the folder òuput` with precisely the files that can be read by the notebooks in the `analysis` folder. They are the required cases for the TU Delft course
