##### PYDRAKE IMPORTS
from pydrake.systems.primitives import LinearSystem, FirstOrderTaylorApproximation

##### OTHER IMPORTS

##### LINEARIZE FUNCTION (takes in diagram and context to find linearized system)
def LinearSys(system, context) -> LinearSystem:
    # finding the linearized affine system (xd = Ax + Bu + f0)
    affine_system =  FirstOrderTaylorApproximation(system=system, context=context)

    # converting to regular linear system
    A = affine_system.A()
    B = affine_system.B()
    return LinearSystem(A=A, B=B)