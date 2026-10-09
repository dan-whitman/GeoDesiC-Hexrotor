##### PYDRAKE IMPORTS
from pydrake.systems.framework import Diagram
from pydrake.systems.primitives import LinearSystem

##### OTHER IMPORTS
import numpy as np
import scipy.linalg as la

##### SELF_DEFINED IMPORTS
import inputs.config as config

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
def PrintLinearSys(linear_system: LinearSystem) -> None:
    print(f'\n=====LINEARIZED SYSTEM CHARACTERISTICS=====')

    # finding A and B matrices
    A = linear_system.A()
    B = linear_system.B()

    print(f'\nLinearized Matrices:')
    print(f'A = {A}')
    print(f'\nB = {B}')

    # finding system parameters
    # state_num = np.shape(A)[0] # number of rows/columns
    # q_num = int(np.floor(state_num)) # if odd, using quaternion representation

    # determining controllability
    C = np.array([])
    for i in range(q_num):
        Ctmp = np.linalg.matrix_power(A, i)
        if i == 0: C = Ctmp @ B
        else: C = np.append(C, Ctmp @ B, axis=1)

    rankA = np.linalg.matrix_rank(A)
    rankB = np.linalg.matrix_rank(B)
    rankC = np.linalg.matrix_rank(C)

    print(f'\nShape of A = {np.shape(A)}, Shape of B = {np.shape(B)}')
    print(f'Rank of A = {rankA}, Rank of B = {rankB}, Rank of C = {rankC}')

    # if not fully controllable
    if rankC != state_num: # if not full rank
        uncont_basis = la.null_space(C.T)
        print(f'\nUncontrollable Basis =\n', uncont_basis.T)

    # finding SVD decomposition
    _, CS, _ = la.svd(C)
    print(f'\nSingular Values of C =\n', CS)