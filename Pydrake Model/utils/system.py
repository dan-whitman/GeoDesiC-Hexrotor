##### PYDRAKE IMPORTS
from pydrake.systems.framework import DiagramBuilder, Diagram, Context
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant, CoulombFriction, BaseBodyJointType, PropellerInfo, Propeller
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform
from pydrake.geometry import SceneGraph, ProximityProperties, AddContactMaterial, HalfSpace

##### OTHER IMPORTS
import numpy as np

##### SELF_DEFINED IMPORTS
import inputs.config as config
from utils.xacro import XacroToURDF
from utils.visual import VisualPlantSceneFrames

##### HEXROTOR PLANT FUNCTION (creates hexrotor plant/scene graph objects)
def HexRotorPlantScene(builder: DiagramBuilder, time_step: float) -> tuple[MultibodyPlant, SceneGraph]:
    print(f'\n=====CREATING HEXROTOR PLANT=====')

    # creating the plant and scene graph
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder=builder, time_step=time_step)

    parser = Parser(plant=plant) # initialize parser
    # inspector = scene_graph.model_inspector() # initialize inspector

    # extracting constants
    config.g_mag = np.linalg.norm(plant.gravity_field().gravity_vector())

    # adding floor to world
    X_WG = RigidTransform.Identity() # identity transform

    proximity_properties = ProximityProperties()
    surface_friction = CoulombFriction(static_friction=config.static_friction, dynamic_friction=config.dynamic_friction) # floor friction
    AddContactMaterial(friction=surface_friction, properties=proximity_properties)

    plant.RegisterCollisionGeometry(
        body=plant.world_body(), X_BG=X_WG, shape=HalfSpace(), name='ground_collision', properties=proximity_properties
    )

    # adding models to plant
    HexRotor = XacroToURDF(filepath=str(config.MOD_DIR / 'HexRotor.urdf.xacro'))

    parser.AddModelsFromString(file_contents=HexRotor, file_type='urdf')

    # configuring quaternions
    if config.args.quats != 1: plant.SetBaseBodyJointType(joint_type=BaseBodyJointType.kRpyFloatingJoint) # if not using quaternions set to rpy

    # adding visualization frames
    if config.args.frames: # if adding visual frames
        plant, scene_graph = VisualPlantSceneFrames(plant=plant, scene_graph=scene_graph)

    plant.Finalize() # finalize the plant

    return plant, scene_graph # return completed plant and scene graph

##### HEXROTOR PROPELLER FUNCTION (returns propeller info for builder)
def HexRotorPropellers(plant: MultibodyPlant) -> Propeller:
    print(f'\n=====CREATING HEXROTOR PROPELLERS=====')

    # finding propeller bodies and frames
    rotor_prop_names = ['rotor_prop_1r', 'rotor_prop_1l', 'rotor_prop_2r', 'rotor_prop_2l','rotor_prop_3r', 'rotor_prop_3l'] # link names
    rotor_prop_bodies = [plant.GetBodyByName(name=rotor_prop_name) for rotor_prop_name in rotor_prop_names] # model bodies

    # adding the propellers
    rotor_prop_info = []
    for i, rotor_prop_body in enumerate(rotor_prop_bodies):
        thrust_ratio = config.prop_thrust_ratio # thrust ratio for propeller
        moment_ratio = config.prop_moment_ratio * (-1)**i # moment ratio for propeller (positive and negative to signify direction of spin)

        X_RP = RigidTransform.Identity() # identity transform

        # add to propeller list
        rotor_prop_info.append(PropellerInfo(rotor_prop_body.index(), X_BP=X_RP, thrust_ratio=thrust_ratio, moment_ratio=moment_ratio)) # propeller for split

    # return completed propeller list
    return Propeller(rotor_prop_info)

##### HEXROTOR DIAGRAM FUNCTION (creates open-loop hexrotor diagram)
def HexRotorDiagram() -> Diagram:
    print(f'\n=====CREATING HEXROTOR DIAGRAM=====')

    # initiating the builder
    open_builder = DiagramBuilder()

    # setting model time step
    if config.args.discrete == 1: time_step = config.simulation_step # use simulation step
    else: time_step = 0.0 # use continuous time

    # creating plant and scene graph
    plant, scene_graph = HexRotorPlantScene(builder=open_builder, time_step=time_step)

    # creating the propellers
    propellers = open_builder.AddNamedSystem(name='propellers', system=HexRotorPropellers(plant=plant))

    # extracting constants
    config.q_num = plant.num_positions() # number of state coordinates
    config.v_num = plant.num_velocities() # number of state velocities
    config.u_num = propellers.get_command_input_port().size() # number of control inputs

    # wiring the diagram
    open_builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port())
    open_builder.Connect(propellers.get_spatial_forces_output_port(), plant.get_applied_spatial_force_input_port())

    # exposing ports
    open_builder.ExportInput(input=propellers.get_command_input_port(), name='propeller_thrusts') # propeller inputs
    open_builder.ExportOutput(output=plant.get_state_output_port(), name='rotor_state') # hexrotor output
    open_builder.ExportOutput(output=scene_graph.get_query_output_port(), name='geometry_query') # geometry output

    # complete diagram
    return open_builder.Build()

##### HEXROTOR INITIAL CONTEXT FUNCTION (creates initial context from diagram)
def HexRotorContext(open_diagram: Diagram) -> Context:
    print(f'\n=====CREATING HEXROTOR CONTEXT=====')

    # creating contexts
    open_context = open_diagram.CreateDefaultContext() # creating numerical context
    plant = open_diagram.GetSubsystemByName(name='plant') # obtaining plant
    plant_context = plant.GetMyMutableContextFromRoot(root_context=open_context) # getting plant context

    # placing hexrotor
    rotor_instance = plant.GetModelInstanceByName(name='hex_rotor') # hexrotor instance

    rotor_body = plant.GetBodyByName(name='rotor_base')
    X_WR = RigidTransform(rpy=config.rotor_initial_rpy, p=config.rotor_initial_position) # target transform
    plant.SetFreeBodyPose(context=plant_context, body=rotor_body, X_JpJc=X_WR) # placing hexrotor

    # defining initial conditions
    config.v_zero = np.zeros(config.v_num) # setting velocities

    rotor_mass = plant.CalcTotalMass(context=plant_context, model_instances=[rotor_instance])
    prop_thrust = 1.00 * config.g_mag * rotor_mass / config.u_num # splitting thurst over all propellers
    config.u_zero = prop_thrust * np.ones(config.u_num) # setting control

    config.u_zero[0] = 0.999 * config.u_zero[0]
    config.u_zero[2] = 0.999 * config.u_zero[2]
    config.u_zero[4] = 0.999 * config.u_zero[4]

    # assigning initial conditions
    plant.SetVelocities(context=plant_context, v=config.v_zero) # hexrotor velocities

    open_diagram_input_port = open_diagram.GetInputPort(port_name='propeller_thrusts')
    open_diagram_input_port.FixValue(context=open_context, value=config.u_zero) # hexrotor thrusts

    # return context
    return open_context