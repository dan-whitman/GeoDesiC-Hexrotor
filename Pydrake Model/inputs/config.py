##### PYDRAKE IMPORTS
from pydrake.math import RollPitchYaw

##### OTHER IMPORTS
import numpy as np
import argparse
from pathlib import Path

##### USER INPUTS
simulation_duration = 5.0 # (s)
simulation_step = 0.01 # (s)
simulation_rate = 1.0 # ()

rotor_initial_position = [0.0, 0.0, 0.5] # (m)
rotor_initial_rpy = RollPitchYaw(0.0, 0.0, 0.0) # (rad)
rotor_desired_position = [0.1, 0.0, 1.0] # (m)
rotor_desired_rpy = RollPitchYaw(0.0, 0.0, np.pi/4) # (rad)

prop_thrust_ratio = 1.0 # ()
prop_moment_ratio = 0.1 # ()

static_friction = 0.7 # () world ground surface friction
dynamic_friction = 0.5 # () world ground dynamic friction

triad_length = 0.15 # (m)
triad_radius = 0.005 # (m)

##### SCRIPT ARGUMENTS
argparser = argparse.ArgumentParser(description='HexRotor Simulation Model') # creating parser

argparser.add_argument('--frames', type=int, required=False, default=0, help='(1) for model frame visibility, (!=1) for no frames')
argparser.add_argument('--quats', type=int, required=False, default=0, help='(1) for quaternion base coordinates, (!=1) for rpy.')
argparser.add_argument('--discrete', type=int, required=False, default=0, help='(1) for discrete dynamics, (!=1) for continuous.')

args = None # empty parser arguments

##### SCRIPT DIRECTORIES
INP_DIR = Path('inputs') # inputs directory
MOD_DIR = Path('models') # model directory
UTI_DIR = Path('utils') # utils directory

##### OTHER VALUES
g_mag = None # gravity

q_num = None # number of state coordinates
v_num = None # number of state velocities
u_num = None # number of control inputs

meshcat = None # meshcat visualization

v_zero = None # initial hexrotor velocities
u_zero = None # initial propeller thrusts