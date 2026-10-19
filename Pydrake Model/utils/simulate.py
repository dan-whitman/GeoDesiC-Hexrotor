##### PYDRAKE IMPORTS
from pydrake.systems.framework import DiagramBuilder, Diagram, Context
from pydrake.systems.analysis import Simulator

##### OTHER IMPORTS

##### SELF_DEFINED IMPORTS
import inputs.config as config
from utils.visual import VisualInit, VisualStart, VisualStop

##### SIMULATION DIAGRAM FUNCTION (inputs closed diagram and adds visualizer for simulation)
def SimDiagram(closed_diagram: Diagram) -> Diagram:
    print(f'\n=====CREATING SIMULATION DIAGRAM=====')

    # initiating the builder
    sim_builder = DiagramBuilder()

    # adding the closed-loop diagram
    sim_builder.AddNamedSystem(name='closed_diagram', system=closed_diagram)


    # adding visualizaer (CHANGE LATER TO MAKE VISUALIZATION OPTIONAL)
    sim_query_object_port = closed_diagram.GetOutputPort(port_name='geometry_query')
    sim_builder, _ = VisualInit(builder=sim_builder, query_object_port=sim_query_object_port)

    # complete diagram
    return sim_builder.Build()

##### SIMULATOR CONTEXT FUNCTION (creates simulator context from diagram)
def SimContext(sim_diagram: Diagram, initial_context: Context) -> Context:
    print(f'\n=====CREATING SIMULATION CONTEXT=====')

    # creating contexts
    sim_context = sim_diagram.CreateDefaultContext() # creating numerical context
    closed_diagram = sim_diagram.GetSubsystemByName(name='closed_diagram') # obtaining closed-loop system

    sim_closed_context = closed_diagram.GetMyMutableContextFromRoot(root_context=sim_context)
    sim_closed_context.SetStateAndParametersFrom(source=initial_context)

    # return context
    return sim_context

##### SIMULATOR RUN FUNCTION (runs the simulator)
def SimRun(sim_diagram: Diagram, sim_context: Context) -> None:
    print(f'\n=====STARTING SIMULATION=====')

    # creating simulator    
    simulator = Simulator(system=sim_diagram, context=sim_context)
    simulator.Initialize()
    simulator.set_target_realtime_rate(realtime_rate=config.simulation_rate)

    # starting visualization (CHANGE LATER TO MAKE OPTIONAL)
    VisualStart()

    # running simulation
    simulator.AdvanceTo(boundary_time=config.simulation_duration) # advance simulation

    # end visualization (CHANGE LATER TO MAKE OPTIONAL)
    VisualStop()