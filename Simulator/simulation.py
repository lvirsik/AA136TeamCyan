import numpy as np
import scipy.integrate
import Vehicle.satellite
from scipy.spatial.transform import Rotation
from Simulator.dynamics import *
from Simulator.simulationConstants import GRAVITY as g
from Simulator.simulationConstants import RHO as rho
from GNC.controller import *

class Simulation:
    """ Class Representing the Simulation and associated data"""
    def __init__(self, timefinal, simulation_timestep, starting_state, planned_trajectory):
        # Create Engine Object inside Rocket
        self.state_previous = starting_state
        self.state = starting_state
        self.statedot_previous = np.zeros((1,len(self.state)))
        self.satellite = Vehicle.satellite.Satellite(simulation_timestep)
        self.ideal_trajectory = planned_trajectory
        self.error_history = np.empty((0,12))
        
        # Simulation Variables
        self.ts = simulation_timestep
        self.tf = timefinal
        self.previous_time = 0
        self.current_step = 0
        

    def propogate(self):
        """ Simple propogator

        Inputs:
            state_0 = initial state
            tf = end time of simulation
            ts = timestep

        Returns:
            solution = position data at each timestamp
            
        """
        # Pull Time Information
        tf = self.tf
        ts = self.ts

        state = self.state

        # Create t vector from 0 to tf with timestep ts
        t_span = np.linspace(0, int(tf/ts)*ts, num=int(tf/ts)+1)
        t = (0,self.tf)

        solution = scipy.integrate.solve_ivp(self.wrapper_state_to_stateDot, t, state, args=(self.satellite, self.ideal_trajectory, t_span), t_eval=t_span, max_step=ts/5)
        state_history = solution['y'].T
        return state_history
    
    def wrapper_state_to_stateDot(self, t, state, satellite, ideal_trajectory, t_vec):

        # Check if we are on an actual simulation timestep or if this is ode solving shenanigans
        if (t == 0) or (t >= t_vec[self.current_step] and self.previous_time < t_vec[self.current_step]):
                        
            # Calculate Errors
            state_error = state - ideal_trajectory[self.current_step]
            self.error_history = np.vstack([self.error_history, state_error])
            
            # Control
            A = compute_A(state, satellite, self.ts)
            B = compute_B(state, satellite, self.ts)
            K = compute_K_flight(len(state), A, B)
            U = control_satellite(K, state_error)
            
            satellite.update_sat_state(U)
        
            if not t == t_vec[-1]:
                self.current_step += 1
            self.previous_time = t
                    
        if t == 0:
            self.statedot_previous = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        
        statedot = full_dynamics(state, satellite, self.ts, t)
        return statedot