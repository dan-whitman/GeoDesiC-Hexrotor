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

rotor_initial_position = np.array([1.0, 1.0, 0.5]) # (m)
rotor_initial_rpy = RollPitchYaw(0.0, 0.0, np.pi/4) # (rad)
# rotor_initial_rpy = RollPitchYaw(np.pi/2, -np.pi/6, 0.0) # (rad)
rotor_initial_angles = np.array([0.0, 0.0, 0.0]) # (rad)

rotor_equil_position = np.array([0.0, 0.0, 1.0]) # (m)
# rotor_equil_rpy = RollPitchYaw(np.pi/4, np.pi/4, 0.0) # (rad)
rotor_equil_rpy = RollPitchYaw(np.pi/2, -np.pi/6, 0.0) # (rad)
# rotor_equil_rpy = RollPitchYaw(0.0, 0.0, 0.0) # (rad)
rotor_equil_corrected = True # (bool)

prop_thrust_ratio = 1.0 # ()
prop_moment_ratio = 0.1 # ()

static_friction = 0.7 # () world ground surface friction
dynamic_friction = 0.5 # () world ground dynamic friction

triad_length = 0.15 # (m)
triad_radius = 0.005 # (m)

##### HEXROTOR GEOMETRY CONFIG (are input into xacro function)
base_radius = 0.15 / 2.0 # (m)
arm_radius = 0.005 # (m)
arm_length = 0.07 # (m)
split_zsize = 0.05 # (m)

xacro_args = { # dictionary of xacro args (must manually change every time arg is altered)
    'base_radius_arg': f'{base_radius}',
    'arm_radius_arg': f'{arm_radius}',
    'arm_length_arg': f'{arm_length}',
    'split_zsize_arg': f'{split_zsize}',
}

##### CONTROL GAINS
lqr_pos_gains = 100.0 * np.ones(3) # position control gains for lqr
lqr_rpy_gains = 30.0 * np.ones(3) # rpy control gains for lqr
lqr_arm_gains = 1.0 * np.ones(3) # arm angle gains for lqr
u_thrust_gains = 1.0 # propeller thrust gains for lqr

##### SCRIPT ARGUMENTS
argparser = argparse.ArgumentParser(description='HexRotor Simulation Model') # creating parser

argparser.add_argument('-f', '--frames', type=int, required=False, default=0, help='(1) for model frame visibility, (!=1) for no frames')
argparser.add_argument('-q', '--quats', type=int, required=False, default=0, help='(1) for quaternion base coordinates, (!=1) for rpy')
argparser.add_argument('-d', '--discrete', type=int, required=False, default=0, help='(1) for discrete dynamics, (!=1) for continuous')
argparser.add_argument('-s', '--sim', type=int, required=False, default=1, help='(1) for running the simulation, (!=1) for skipping it')
argparser.add_argument('-c', '--control', type=str, required=False, default='lqr', help=f'\'lqr\' for lqr control')

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

q_initial = None # initial hexrotor states
v_initial = None # initial hexrotor velocities
u_initial = None # initial propeller thrusts
q_equil = None # equilibrium hexrotor states
v_equil = None # equilibrium hexrotor velocities
u_equil = None # equilibrium propeller thrusts

rotor_mass = None # hexrotor mass (kg)
rotor_com = None # hexrotor com (m)