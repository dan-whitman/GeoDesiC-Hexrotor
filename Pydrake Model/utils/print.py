##### PYDRAKE IMPORTS
from pydrake.systems.framework import Diagram
from pydrake.systems.primitives import AffineSystem

##### OTHER IMPORTS
import numpy as np
import scipy.linalg as la

##### SELF_DEFINED IMPORTS
import inputs.config as config

##### HEXROTOR INFORMATION PRINT FUNCTION (prints information about hexrotor)
def PrintHexRotor() -> None:
    print(f'\n=====HEXROTOR INFORMATION=====')

    # print hexrotor body COM
    print(f'x_com = {config.rotor_com[0]}, y_com = {config.rotor_com[1]}, z_com = {config.rotor_com[2]}')

##### STATE VARIABLE PRINT FUNCTION (prints state coordinates and velocities)
def PrintGenCoords(diagram: Diagram) -> None:
    print(f'\n=====GENERALIZED COORDINATES=====')

    # obtain plant from diagram
    plant = diagram.GetSubsystemByName(name='plant')

    # find and print generalized coordinates/velocities
    q_names = plant.GetPositionNames()
    v_names = plant.GetVelocityNames()
    for i, name in enumerate(q_names): print(f'q[{i}] -> {name}') # print each coordinate
    print('')
    for i, name in enumerate(v_names): print(f'v[{i}] -> {name}') # print each velocity

##### LINEAR SYSTEM PRINT FUNCTION (prints linear system quantities)
def PrintLinearSys(affine_system: AffineSystem) -> None:
    print(f'\n=====LINEARIZED SYSTEM CHARACTERISTICS=====')

    # finding linear system terms
    A = affine_system.A()
    B = affine_system.B()
    f0 = affine_system.f0()

    print(f'Linearized Factors:')
    print(f'A =', A)
    print(f'\nB =', B)
    print(f'\nf0 =', f0)

    # determining controllability
    C = np.array([])
    for i in range(config.q_num): # constructing C matrix
        Ctmp = np.linalg.matrix_power(A, i) 
        if i == 0: C = Ctmp @ B
        else: C = np.append(C, Ctmp @ B, axis=1)

    # finding matrix ranks
    rankA = np.linalg.matrix_rank(A)
    rankB = np.linalg.matrix_rank(B)
    rankC = np.linalg.matrix_rank(C)

    print(f'\nShape of A = {np.shape(A)}, Shape of B = {np.shape(B)}')
    print(f'Rank of A = {rankA}, Rank of B = {rankB}, Rank of C = {rankC}')

    if rankC != config.q_num + config.v_num: # if C is not fully controllable
        uncont_basis = la.null_space(C.T) # find uncontrollable basis
        print(f'\nUncontrollable Basis =\n', uncont_basis.T)

    # finding SVD decomposition for stability
    _, CS, _ = la.svd(C)
    print(f'\nSingular Values of C =\n', CS)