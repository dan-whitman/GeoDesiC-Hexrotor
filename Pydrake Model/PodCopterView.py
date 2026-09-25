##### PYDRAKE IMPORTS
# from pydrake.all import 

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import Linearize
from pydrake.systems.analysis import Simulator
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, Propeller, PropellerInfo, BaseBodyJointType
from pydrake.multibody.tree import FixedOffsetFrame
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.visualization import ModelVisualizer, AddFrameTriadIllustration
from pydrake.geometry import StartMeshcat, Role, Box

##### OTHER IMPORTS
import argparse
import numpy as np
import scipy.linalg

##### SELF-DEFINED IMPORTS
from XacroUtils import XacroToURDF

##### PARSING ARGUMENTS
argparser = argparse.ArgumentParser(description="PodCopter Controllability Model") # creating parser

argparser.add_argument("--vis", type=int, required=False, default=1, help="(1) for meshcat visualization, (!=1) else.")
argparser.add_argument("--quat", type=int, required=False, default=1, help="(1) for quaternion base coordinates, (!=1) for rpy.")

args = argparser.parse_args() # getting the arguments

##### DRAKE MODEL
builder = DiagramBuilder() # initiating the builder

##### MATHEMATICAL MODEL
plant, scene_graph = AddMultibodyPlantSceneGraph(builder=builder, time_step=0.0) # creating the plant and scene graph

parser = Parser(plant) # initialize parser
inspector = scene_graph.model_inspector() # initialize inspector

# loading all models to parser
PodCopter = XacroToURDF('PodCopter.urdf.xacro')
parser.AddModelsFromString(PodCopter, 'urdf')
# print(PodCopter)

''' ##### CREATING PROPELLERS (NO PROP URDF)
# finding split bodies and dimensions
copter_split_names = ['copter_split_1', 'copter_split_2', 'copter_split_3'] # link names
copter_split_bodies = [plant.GetBodyByName(copter_split_name) for copter_split_name in copter_split_names] # model bodies
copter_split_frame_ids = [plant.GetBodyFrameIdOrThrow(copter_split_body.index()) for copter_split_body in copter_split_bodies] # body frame ids
copter_split_geo_ids = [inspector.GetGeometries(copter_split_frame_id, Role.kProximity) for copter_split_frame_id in copter_split_frame_ids] # body geometries
copter_split_shapes = [inspector.GetShape(copter_split_geo_id[0]) for copter_split_geo_id in copter_split_geo_ids] # body shapes
copter_split_dims = [copter_split_shape.size() for copter_split_shape in copter_split_shapes if isinstance(copter_split_shape, Box)] # body dimensions

# adding the propellers
copter_prop_info = []
for copter_split_body, copter_split_dim in zip(copter_split_bodies, copter_split_dims):
    thrust_ratio = 1.0 # thrust ratio for prop
    moment_ratio = 0.0 # moment ratio for prop (pos and negative to signify direction of prop spin)

    X_BP_right = RigidTransform(RollPitchYaw(np.pi/2, 0, 0), [copter_split_dim[0]/2, -copter_split_dim[1]/2, copter_split_dim[2]/2])
    X_BP_left =  RigidTransform(RollPitchYaw(np.pi/2, 0, 0), [-copter_split_dim[0]/2, -copter_split_dim[1]/2, copter_split_dim[2]/2])
    
    copter_prop_info.append(PropellerInfo(copter_split_body.index(), X_BP=X_BP_right, thrust_ratio=thrust_ratio, moment_ratio=moment_ratio)) # right propeller for split
    copter_prop_info.append(PropellerInfo(copter_split_body.index(), X_BP=X_BP_left, thrust_ratio=thrust_ratio, moment_ratio=moment_ratio)) # left propeller for split
'''

# '''##### CREATING PROPELLERS (PROP URDF)
# finding prop bodies and frames
copter_prop_names = ['copter_prop_1r', 'copter_prop_1l', 'copter_prop_2r', 'copter_prop_2l','copter_prop_3r', 'copter_prop_3l'] # link names
copter_prop_bodies = [plant.GetBodyByName(copter_prop_name) for copter_prop_name in copter_prop_names] # model bodies

# adding the propellers
copter_prop_info = []
for i, copter_prop_body in enumerate(copter_prop_bodies):
    thrust_ratio = 1.0 # thrust ratio for prop
    moment_ratio = 0.0 * (-1)**i # moment ratio for prop (pos and negative to signify direction of prop spin)

    X_BP = RigidTransform(RollPitchYaw(0, 0, 0), [0, 0, 0]) # identity transform

    # can add some check to determine correct spin directions for each prop
    copter_prop_info.append(PropellerInfo(copter_prop_body.index(), X_BP=X_BP, thrust_ratio=thrust_ratio, moment_ratio=moment_ratio)) # propeller for split
# '''

if args.quat != 1: plant.SetBaseBodyJointType(BaseBodyJointType.kRpyFloatingJoint) # if not using quaternions
plant.Finalize() # finalize the plant

##### VISUAL MODEL
if args.vis == 1: # if visualizing
    meshcat = StartMeshcat() # initialize meshcat
    # print(f'Open this URL in your browser: {meshcat.web_url()}')

    # creating visualizer
    visualizer = ModelVisualizer(
        visualize_frames=True, browser_new=True, meshcat=meshcat, triad_length=0.15, triad_radius=0.005
    )

    # loading all models to visualizer
    visparser = visualizer.parser() # finding parser
    visplant = visparser.plant() # finding plant
    visscene_graph = visparser.scene_graph() # finding scene graph

    visparser.AddModelsFromString(PodCopter, 'urdf')

    ''' ##### CREATING PROPELLERS (VISUAL, NO PROP URDF)
    copter_vissplits = [visplant.GetBodyByName(copter_split_name) for copter_split_name in copter_split_names] # model bodies

    # finding the propeller frames
    copter_visprop_info = []
    for copter_vissplit, copter_split_dim in zip(copter_vissplits, copter_split_dims):
        X_BP_right = RigidTransform(RollPitchYaw(np.pi/2, 0, 0), [copter_split_dim[0]/2, -copter_split_dim[1]/2, copter_split_dim[2]/2])
        X_BP_left =  RigidTransform(RollPitchYaw(np.pi/2, 0, 0), [-copter_split_dim[0]/2, -copter_split_dim[1]/2, copter_split_dim[2]/2])
        
        copter_visprop_info.append(PropellerInfo(copter_vissplit.index(), X_BP=X_BP_right)) # right propeller for split
        copter_visprop_info.append(PropellerInfo(copter_vissplit.index(), X_BP=X_BP_left)) # left propeller for split

    # adding to visualization
    for i, copter_visprop in enumerate(copter_visprop_info):
        copter_visprop_body = visplant.get_body(copter_visprop.body_index) # find parent body

        # creating frame
        copter_visprop_frame = visplant.AddFrame(
            FixedOffsetFrame(name=f'prop_frame_{i}', P=copter_visprop_body.body_frame(), X_PF=copter_visprop.X_BP)
        )

        # adding triad (NOT NEEDED)
        # AddFrameTriadIllustration( # adding to visualizer
        #     scene_graph=visscene_graph, plant=visplant, frame=copter_visprop_frame, length=0.15, radius=0.005
        # )
    '''

    # running the visualizer
    visualizer.Finalize()
    visualizer.Run(loop_once=False) # (False) to continuously run and visualize

##### BUILDING MODELS
# connecting props to control
propellers = builder.AddSystem(Propeller(copter_prop_info)) # creating the propellers

builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port())
builder.Connect(propellers.get_spatial_forces_output_port(), plant.get_applied_spatial_force_input_port())

builder.ExportInput(propellers.get_command_input_port(), 'propeller_thrusts')
builder.ExportOutput(plant.get_state_output_port(), 'copter_state')

diagram = builder.Build() # building final diagram

##### FINDING DYNAMICS ABOUT HOVER
context = diagram.CreateDefaultContext() # creating numerical context
plant_context = plant.GetMyMutableContextFromRoot(context)

q_num = plant.num_positions() # number of q coords
v_num = plant.num_velocities() # number of velocities
u_num = propellers.get_command_input_port().size() # number of control inputs

# printing coordinate names for reference
q_names = plant.GetPositionNames()
v_names = plant.GetVelocityNames()
for i, name in enumerate(q_names): print(f'q[{i}] -> {name}')
for i, name in enumerate(v_names): print(f'v[{i}] -> {name}')

# defining equilibrium position and control
q_equil = np.zeros(q_num) # zero vector
if args.quat == 1:
    q_equil[0] = 1 # qw quaternion element
    q_equil[6] = 1 # z base element (in air)
else: q_equil[5] = 1 # z base element (in air)

v_equil = np.zeros(v_num) # zero vector

plant.SetPositions(plant_context, q_equil) # setting position
plant.SetVelocities(plant_context, v_equil) # setting velocity

prop_thrust = 9.81 * plant.CalcTotalMass(plant_context) / u_num
u_equil = prop_thrust * np.ones(u_num) # setting control

diagram_input_port = diagram.get_input_port(0)
diagram_input_port.FixValue(context, u_equil)

# finding the linearized system (xd = Ax + Bu)
linear_system = Linearize(
    system=diagram, context=context, input_port_index=diagram_input_port.get_index()
    # , equilibrium_check_tolerance=np.inf # if need to adjust equilibrium condition
)

# finding matrices A and B
A = linear_system.A()
B = linear_system.B()

print(f'\nLinearized Matrices:')
print(f'\nA = {A}')
print(f'\nB = {B}')

# determining controllability
C = np.array([])
for i in range(q_num):
    Ctmp = np.linalg.matrix_power(A, i)
    if i == 0: C = Ctmp@B
    else: C = np.append(C, Ctmp@B, axis=1)

rankA = np.linalg.matrix_rank(A)
rankB = np.linalg.matrix_rank(B)
rankC = np.linalg.matrix_rank(C)

print(f'\nShape of A = {np.shape(A)}, Shape of B = {np.shape(B)}')
print(f'Rank of A = {rankA}, Rank of B = {rankB}, Rank of C = {rankC}')

if rankC != q_num + v_num: # if not full rank
    uncont_basis = scipy.linalg.null_space(C.T)
    print(f'\nUncontrollable Basis = \n{uncont_basis.T}')

##### RUNNING SIMULATION
simulator = Simulator(diagram) # starting simulation (for visualization)
simulator.Initialize()