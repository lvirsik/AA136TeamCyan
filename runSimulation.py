import numpy as np
from Simulator.simulation import Simulation
from Simulator.simulationConstants import *
from Graphing.GUI import *
from PathPlanner.plan import PlannedTrajectory


# Setup Simulation
planned_trajectory = PlannedTrajectory(TARGET_ALTITUDE, FINAL_TIME, TIMESTEP).trajectory
sim = Simulation(FINAL_TIME, TIMESTEP, INITIAL_STATE, planned_trajectory)

# Run Simulation
trajectory = sim.propogate()
sim.display_end_info()

# Graphs
create_gui(sim, planned_trajectory, trajectory, TIMESTEP, FINAL_TIME)
