##### PYDRAKE IMPORTS
from pydrake.systems.framework import DiagramBuilder, Diagram
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.primitives import AffineSystem

##### OTHER IMPORTS
import numpy as np

##### SELF_DEFINED IMPORTS
import inputs.config as config

##### LQR FUNCTION (returns calculated LQR matrix K given weight matrices)
def ControlLQRK(affine_system: AffineSystem, Q: np.array, R: np.array, N: np.array) -> np.array:
    print(f'\n=====FINDING LQR GAIN=====')

    # extracting linear system matrices
    A = affine_system.A()
    B = affine_system.B()
    f0 = affine_system.f0() # (CAN IMPLEMENT LATER IF NEEDED)

    # finding controller gain
    (K, _) = LinearQuadraticRegulator(A=A, B=B, Q=Q, R=R, N=N)

    # return gain matrix
    return K

##### LQR DIAGRAM FUNCTION (creates LQR diagram block for given linearized system)
def ControlLQRDiagram(affine_system: AffineSystem) -> Diagram:
    print(f'\n=====CREATING LQR DIAGRAM=====')

    # initiating the builder
    lqr_builder = DiagramBuilder()

    # create LQR gain matrices (CAN MAKE MORE ADJUSTABLE LATER)
    Q = np.ones(config.q_num + config.v_num) # Q matrix gain vector
    if config.args.quats == 1: # if using quaternions
        Q[1:4] = config.lqr_rpy_gains # orientation gains
        Q[4:7] = config.lqr_pos_gains # position gains
    else: # if using rpy
        Q[0:3] = config.lqr_rpy_gains # orientation gains
        Q[3:6] = config.lqr_pos_gains # position gains
    Q[-3:] = config.lqr_arm_gains # arm gains

    R = np.ones(config.u_num) # R matrix gain vector

    N = np.zeros((config.q_num + config.v_num, config.u_num)) # N matrix gain vector

    Q = np.diag(Q) # diagonalize Q vector
    R = np.diag(R) # diagonalize R vector

    # find LQR K gain
    K = ControlLQRK(affine_system=affine_system, Q=Q, R=R, N=N)

    # constructing LQR diagram
    A = np.zeros((0, 0)) # controller A
    B = np.zeros((0, config.u_num)) # controller B
    C = np.zeros((config.u_num, 0)) # controller C
    D = -K # controller D

    x0 = np.hstack((config.q_equil, config.v_equil)) # equilibrium state
    u0 = config.u_equil + K @ x0 # constant offset term

    lqr = lqr_builder.AddNamedSystem(
        name='lqr', system=AffineSystem(A=A, B=B, C=C, D=D, y0=u0)
    )

    # exposing ports
    lqr_builder.ExportInput(input=lqr.get_input_port(), name='rotor_state')
    lqr_builder.ExportOutput(output=lqr.get_output_port(), name='propeller_thrusts')

    # complete diagram
    return lqr_builder.Build()    

##### CONTROL DIAGRAM FUNCTION (creates closed-loop diagram from open-loop and control diagrams)
def ControlDiagram(open_diagram: Diagram, control_diagram: Diagram) -> Diagram:
    print(f'\n=====CREATING CONTROL DIAGRAM=====')

    # initiating the builder
    closed_builder = DiagramBuilder()

    # adding subsystems
    closed_builder.AddNamedSystem(name='open_diagram', system=open_diagram) # open-loop system
    closed_builder.AddNamedSystem(name='control_diagram', system=control_diagram) # control diagram

    # wiring the systems together
    closed_builder.Connect(open_diagram.GetOutputPort(port_name='rotor_state'), control_diagram.GetInputPort(port_name='rotor_state'))
    closed_builder.Connect(control_diagram.GetOutputPort(port_name='propeller_thrusts'), open_diagram.GetInputPort(port_name='propeller_thrusts'))

    # exposing ports
    closed_builder.ExportOutput(name='geometry_query', output=open_diagram.GetOutputPort(port_name='geometry_query'))

    # complete diagram
    return closed_builder.Build()  