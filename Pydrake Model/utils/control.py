##### PYDRAKE IMPORTS
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.primitives import LinearSystem, AffineSystem

##### OTHER IMPORTS
import numpy as np

##### LQR FUNCTION (returns calculated LQR matrix K given weight matrices)
def LQRK(linear_system: LinearSystem, Q: np.array, R: np.array, N: np.array):
    # extracting linear system matrices
    A = linear_system.A()
    B = linear_system.B()

    # finding controller gain
    (K, _) = LinearQuadraticRegulator(A=A, B=B, Q=Q, R=R, N=N)

##### LQR WIRE FUNCTION (inputs open loop system and adds LQR controller to it)
def LQRSys(open_diagram, K):
    # finding system size
    (state_num, u_num) = np.shape(K)

    A = np.zeros((0, 0)) # controller A
    B = np.zeros((0, u_num)) # controller B
    C = np.zeros((u_num, 0)) # controller C
    D = -K # controller D
    # y0 = 