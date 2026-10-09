##### CONFIG
# must run the following in terminal to establish pydrake environment variables:
'''
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
'''
# will work out an automatica way to establish this later

##### PYDRAKE IMPORTS
# from pydrake.all import 

##### OTHER IMPORTS

##### SELF-DEFINED IMPORTS
import inputs.config as config
from utils.system import HexRotorDiagram, HexRotorContext
from utils.simulate import SimDiagram, SimContext, SimRun
from utils.print import PrintGenCoords
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
    open_initial_context = HexRotorContext(open_diagram=open_diagram) # define open-loop initial context

    # creating closed-loop system

    # creating simulation system
    sim_diagram = SimDiagram(closed_diagram=open_diagram) # create final simulaton diagram
    sim_context = SimContext(sim_diagram=sim_diagram, initial_context=open_initial_context) # define final simulation context
    SimRun(sim_diagram=sim_diagram, sim_context=sim_context) # running the simulation

    # printing system information
    PrintGenCoords(diagram=open_diagram)

    VisualHold() # holding visualization open (CHANGE LATER TO MAKE OPTIONAL)

    # add saving data?

    print(f'\n=====CLOSING SCRIPT=====')

    

# linear_system = LinearSys(system=open_diagram, context=context)

# # determining LQR weights
# Q = np.eye(q_num + v_num) # LQR weight matrix
# R = np.eye(u_num) # LQR weight matrix
# N = np.zeros((q_num + v_num, u_num)) # LQR weight matrix

# K = LQRK(linear_system=linear_system, Q=Q, R=R, N=N)