##### PYDRAKE IMPORTS
from pydrake.systems.framework import Context
from pydrake.multibody.plant import MultibodyPlant

##### OTHER IMPORTS
import numpy as np

##### SELF_DEFINED IMPORTS
import inputs.config as config

##### EQUILIBRIUM SOLUTION FUNCTION (gives tangential and normal forces for desired orientation)
def EquilInvSoln(R_WR: np.array) -> np.array:
    print(f'\n=====SOLVING FOR GENERAL ARM FORCES=====')

    # defining constants (for easier typsetting)
    g = config.rotor_mass * config.g_mag # gravity force
    z = config.rotor_com[2] # z offset to COM for propeller force origin
    r = config.base_radius + config.arm_length + config.split_zsize / 2 # r length to propeller force origin
    R31 = R_WR[2,0] # R_3^1
    R32 = R_WR[2,1] # R_3^2
    R33 = R_WR[2,2] # R_3^3

    # defining each solution
    u1T = -g*(4*R32 - 5*np.sqrt(2)*R31) / 10 # tangential for arm 1
    u2T = 3*g*R32 / 5 # tangential for arm 2
    u3T = -g*(4*R32 + 5*np.sqrt(2)*R31) / 10 # tangential for arm 3

    u1N = g*(2*r*R33 + 2*z*R31 + 3*np.sqrt(2)*z*R32) / (6*r) # normal for arm 1
    u2N = g*R33/3 - (2*g*z*R31) / (3*r) # normal for arm 2
    u3N = g*(2*r*R33 + 2*z*R31 - 3*np.sqrt(2)*z*R32) / (6*r) # normal for arm 3

    # return solution
    return np.array([u1T, u2T, u3T, u1N, u2N, u3N])

##### EQUILIBRIUM PROPELLER FORCE FUNCTION (inputs solved generalized forces and returns individual propeller forces and arm angles)
def EquilPropArmSoln(u_gen: np.array) -> tuple[np.array, np.array]:
    print(f'\n=====FINDING PROPELLER THRUSTS AND ARM ANGLES=====')

    # finding propeller thrusts
    u1_total = np.sqrt(u_gen[0]**2 + u_gen[3]**2) # total force for arm 1
    u2_total = np.sqrt(u_gen[1]**2 + u_gen[4]**2) # total force for arm 2
    u3_total = np.sqrt(u_gen[2]**2 + u_gen[5]**2) # total force for arm 3

    u_equil = np.array([u1_total/2, u1_total/2, u2_total/2, u2_total/2, u3_total/2, u3_total/2]) # individual thrusts array

    # finding arm angles
    theta_1 = np.atan2(u_gen[0], u_gen[3]) # angle for arm 1
    theta_2 = np.atan2(u_gen[1], u_gen[4]) # angle for arm 2
    theta_3 = np.atan2(u_gen[2], u_gen[5]) # angle for arm 3

    theta_equil = np.array([theta_1, theta_2, theta_3]) # angle array

    # returning solutions
    return u_equil, theta_equil

##### EQUILIBRIUM STATE SOLVER (inputs plant and context and solves for state and control to force equilibrium)
def EquilSolve(plant: MultibodyPlant, plant_context: Context) -> tuple[np.array, np.array, np.array]:
    print(f'\n=====SOLVING FOR EQUILIBRIUM STATE=====')

    # evaluating orientation of hexrotor
    rotor_body = plant.GetBodyByName(name='rotor_base') # hexrotor base
    X_WR = plant.EvalBodyPoseInWorld(context=plant_context, body=rotor_body) # transform from world to rotor
    R_WR = X_WR.rotation().matrix() # get hexrotor rotation matrix

    q_equil = plant.GetPositions(context=plant_context) # get hexrotor position from plant
    # v_equil = plant.GetVelocities(context=context) # get hexrotor velocities from plant

    # solve for equilibrium forcing state and control (IMPLEMENT NUMERICAL SOLVER AS WELL?)
    u_gen = EquilInvSoln(R_WR=R_WR) # arm force solutions
    u_equil, theta_equil = EquilPropArmSoln(u_gen=u_gen) # finding propeller thrusts and arm angles

    # setting equilibrium context states
    q_equil[-3:] = theta_equil # solved hexrotor arm angles
    v_equil = np.zeros(config.v_num) # solved velocities (CAN CHANGE TO ALLOW TARGET VELOCITIES LATER)
    u_equil = u_equil # solved propeller thrusts

    # returning solved states
    return q_equil, v_equil, u_equil



    

