##### CONFIG
# must run the following in terminal to establish pydrake environment variables OR add to ~./bashrc:
'''
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
'''

##### PYDRAKE IMPORTS
# from pydrake.all import 

##### OTHER IMPORTS

##### SELF-DEFINED IMPORTS
import inputs.config as config
from utils.system import HexRotorDiagram, HexRotorInitialContext, HexRotorEquilContext
from utils.linear import LinearSys
from utils.control import ControlLQRDiagram, ControlDiagram
from utils.simulate import SimDiagram, SimContext, SimRun
from utils.print import PrintGenCoords, PrintLinearSys, PrintHexRotor
from utils.visual import VisualHold

##### SCRIPT INITIALIZATION FUNCTION (initializes commands and config args)
def ScriptInit() -> None:
    print(f'\n=====INITIALIZING SCRIPT=====')

    # making directories
    config.INP_DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist
    config.MOD_DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist
    config.UTI_DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist

    # parsing arguments
    config.args = config.argparser.parse_args()

##### MODEL SCRIPT (runs the model)
if __name__ == '__main__':
    ScriptInit() # initialize the script with arguments

    # creating open-loop system
    open_diagram = HexRotorDiagram() # create open-loop diagram
    open_initial_context = HexRotorInitialContext(open_diagram=open_diagram) # define open-loop initial context
    open_equil_context = HexRotorEquilContext(open_diagram=open_diagram) # define open-loop equilibrium context

    # linearizing system
    open_affine_system = LinearSys(system=open_diagram, context=open_equil_context)

    # creating closed-loop system (ADD MORE CONTROL OPTIONS LATER)
    if config.args.control == 'lqr': control_diagram = ControlLQRDiagram(affine_system=open_affine_system) # create lqr controller
    closed_diagram = ControlDiagram(open_diagram=open_diagram, control_diagram=control_diagram) # create closed-loop diagram

    # creating simulation system
    sim_diagram = SimDiagram(closed_diagram=closed_diagram) # create final simulaton diagram
    sim_context = SimContext(sim_diagram=sim_diagram, initial_context=open_initial_context) # define final simulation context
    SimRun(sim_diagram=sim_diagram, sim_context=sim_context) # running the simulation

    # printing system information
    PrintGenCoords(diagram=open_diagram)
    PrintLinearSys(affine_system=open_affine_system)
    PrintHexRotor()

    VisualHold() # holding visualization open (CHANGE LATER TO MAKE VISUALIZATION OPTIONAL)

    # (ADD EXPORTING DATA LATER)

    print(f'=====CLOSING SCRIPT=====')