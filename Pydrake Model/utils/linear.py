##### PYDRAKE IMPORTS
from pydrake.systems.framework import Diagram, Context
from pydrake.systems.primitives import AffineSystem, FirstOrderTaylorApproximation

##### OTHER IMPORTS

##### SELF_DEFINED IMPORTS

##### LINEARIZE FUNCTION (takes in diagram and context to find linearized system)
def LinearSys(system: Diagram, context: Context) -> AffineSystem:
    print(f'\n=====LINEARIZING SYSTEM=====')

    # finding the linearized affine system (xd = Ax + Bu + f0)
    return FirstOrderTaylorApproximation(system=system, context=context)