##### PYDRAKE IMPORTS
from pydrake.systems.framework import Context
from pydrake.multibody.plant import MultibodyPlant

##### OTHER IMPORTS
import numpy as np

##### SELF_DEFINED IMPORTS
import inputs.config as config

##### EQUILIBRIUM STATE SOLVER (inputs plant and context and solves for state and control to force equilibrium)
def EquilSolve(plant: MultibodyPlant, plant_context: Context) -> tuple[np.array, np.array, np.array]:
    print(f'\n=====SOLVING FOR EQUILIBRIUM STATE=====')

    # getting desired plant pose and velocities
    q_equil = plant.GetPositions(context=plant_context) # get hexrotor pose from plant
    # v_equil = plant.GetVelocities(context=context) # get hexrotor velocities from plant

    # solve for equilibrium forcing state and control (IMPLEMENT ACTUAL SOLVER LATER)
    q_equil[-3:] = np.array([0.0, 0.0, 0.0]) # solving for hexrotor arm angles

    v_equil = np.zeros(config.v_num) # solved velocities (CAN CHANGE TO ALLOW TARGET VELOCITIES LATER)

    rotor_instance = plant.GetModelInstanceByName(name='hex_rotor') # hexrotor instance
    rotor_mass = plant.CalcTotalMass(context=plant_context, model_instances=[rotor_instance]) # finding hexrotor mass
    prop_thrust = 1.00 * config.g_mag * rotor_mass / config.u_num # splitting thurst over all propellers (assumes all propellers facing up)
    u_equil = prop_thrust * np.ones(config.u_num) # solving for control inputs

    # returning solved states
    return q_equil, v_equil, u_equil



    

