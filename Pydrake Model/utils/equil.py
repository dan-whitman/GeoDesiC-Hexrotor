##### PYDRAKE IMPORTS
from pydrake.systems.framework import Context
from pydrake.multibody.plant import MultibodyPlant

##### OTHER IMPORTS
import numpy as np

##### SELF_DEFINED IMPORTS
import inputs.config as config

##### EQUILIBRIUM SOLUTION FUNCTION (gives tangential and normal forces for desired orientation)
def EquilInvSoln(R_WR: np.array, q_arms: np.array, corrected: bool) -> np.array:
    print(f'\n=====SOLVING FOR GENERAL ARM FORCES=====')

    # defining constants (for easier typsetting)
    g = config.rotor_mass * config.g_mag # gravity force
    z = config.arm_radius - config.rotor_com[2] # z offset to COM for propeller force origin
    d = config.arm_radius # rotational correction terms as arm angles change
    r = config.base_radius + config.arm_length + config.split_zsize / 2 # r length to propeller force origin

    theta1 = q_arms[0] # arm angle 1
    theta2 = q_arms[1] # arm angle 2
    theta3 = q_arms[2] # arm angle 3

    z1 = z - d*np.cos(theta1) # corrected z for arm 1
    z2 = z - d*np.cos(theta2) # corrected z for arm 2
    z3 = z - d*np.cos(theta3) # corrected z for arm 3

    d1 = d*np.sin(theta1) # corrected d for arm 1
    d2 = d*np.sin(theta2) # corrected d for arm 2
    d3 = d*np.sin(theta3) # corrected d for arm 3

    R31 = R_WR[2,0] # R_3^1
    R32 = R_WR[2,1] # R_3^2
    R33 = R_WR[2,2] # R_3^3

    # defining each solution
    u1T = -g*(4.0*R32 - 5.0*np.sqrt(2.0)*R31) / 10.0 # tangential for arm 1
    u2T = 3.0*g*R32 / 5.0 # tangential for arm 2
    u3T = -g*(4.0*R32 + 5.0*np.sqrt(2.0)*R31) / 10.0 # tangential for arm 3

    u1N = g*(2.0*r*R33 + 2.0*z*R31 + 3.0*np.sqrt(2.0)*z*R32) / (6.0*r) # normal for arm 1
    u2N = g*R33/3.0 - (2.0*g*z*R31) / (3.0*r) # normal for arm 2
    u3N = g*(2.0*r*R33 + 2.0*z*R31 - 3.0*np.sqrt(2.0)*z*R32) / (6.0*r) # normal for arm 3

    # defining corrected solution
    A = np.array([ # constructed static matrix
        [1.0/np.sqrt(2.0), 0.0, -1.0/np.sqrt(2.0), 0.0, 0.0, 0.0],
        [-0.5, 1.0, -0.5, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        [z1/2.0, -z2, z3/2.0, r/np.sqrt(2.0)-d1/2.0, d2, -r/np.sqrt(2.0)-d3/2.0],
        [z1/np.sqrt(2.0), 0.0, -z3/np.sqrt(2.0), -r/2.0+d1/np.sqrt(2.0), r, -r/2.0-d3/np.sqrt(2.0)],
        [-3.0*r/4.0+d1/np.sqrt(2.0), -r, -3.0*r/4.0-d3/np.sqrt(2.0), 0.0, 0.0, 0.0]
    ])

    b = np.array([R31*g, R32*g, R33*g, 0.0, 0.0, 0.0]) # desired force vector

    u_gen = np.linalg.inv(A) @ b # solving for trusts

    # return solution
    if not corrected: return np.array([u1T, u2T, u3T, u1N, u2N, u3N])
    else: return u_gen

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
    u_gen = EquilInvSoln(R_WR=R_WR, q_arms=q_equil[-3:], corrected=config.rotor_equil_corrected) # arm force solutions
    u_equil, theta_equil = EquilPropArmSoln(u_gen=u_gen) # finding propeller thrusts and arm angles

    # setting equilibrium context states
    q_equil[-3:] = theta_equil # solved hexrotor arm angles
    v_equil = np.zeros(config.v_num) # solved velocities (CAN CHANGE TO ALLOW TARGET VELOCITIES LATER)
    u_equil = u_equil # solved propeller thrusts

    # returning solved states
    return q_equil, v_equil, u_equil



    

