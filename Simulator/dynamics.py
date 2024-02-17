import numpy as np
from scipy.spatial.transform import Rotation
from Simulator.simulationConstants import GRAVITY as g
from Simulator.simulationConstants import RHO as rho
from Simulator.simulationConstants import *

def dynamics_for_state_space_control(state, satellite, dt, u):
    # Pull Params
    m = satellite.mass
    v = state[3:6]
    w = state[9:12]

    # Build Statedot
    statedot = np.zeros(len(state))
    statedot[0:3] = v
    statedot[6:9] = w
    
    # Rocket rotations
    pitch = state[6] # Angle from rocket from pointing up towards positive x axis
    yaw = state[7] # Angle from rocket from pointing up towards positive y axis
    roll = state[8] # Roll, ccw when looking down on rocket
    R = Rotation.from_euler('xyz', [yaw, -pitch, -roll]).as_matrix()
    R_inv = np.linalg.inv(R)

    # Calculate Accelerations in rocket frame
    a_rf = np.array([u[0], u[1], u[2]])

    # Convert Accelerations from rocket frame to global frame
    a_global = np.dot(R_inv, a_rf)

    # Calculate Alphas
    torque = [u[3], u[4], u[5]]
    I_dot = (satellite.I - satellite.I_prev) / dt
    alphas = np.dot(np.linalg.inv(satellite.I), torque - np.cross(w, np.dot(satellite.I, w)) - np.dot(I_dot, w))

    statedot[3:6] = a_global.tolist()
    statedot[9:12] = alphas.tolist()

    return statedot

def full_dynamics(state, satellite, dt, t):
    # Pull Params
    commands = satellite.acceleration_commands
    T = 0 #No Thrust for now
    m = satellite.mass
    v = state[3:6]
    w = state[9:12]

    # Build Statedot
    statedot = np.zeros(len(state))
    statedot[0:3] = v
    statedot[6:9] = w
    
    # Rocket rotations
    pitch = state[6] # Angle from rocket from pointing up towards positive x axis
    yaw = state[7] # Angle from rocket from pointing up towards positive y axis
    roll = state[8] # Roll, ccw when looking down on rocket
    R = Rotation.from_euler('xyz', [yaw, -pitch, -roll]).as_matrix()
    R_inv = np.linalg.inv(R)

    # Calculate Accelerations in rocket frame
    a_rf = np.array([commands[0], commands[1], commands[2]])

    # Convert Accelerations from rocket frame to global frame
    a_global = np.dot(R_inv, a_rf)

    # Calculate Alphas
    torque = np.array([commands[3], commands[4], commands[5]])
    I_dot = (satellite.I - satellite.I_prev) / dt
    alphas = np.dot(np.linalg.inv(satellite.I), torque - np.cross(w, np.dot(satellite.I, w)) - np.dot(I_dot, w))

    statedot[3:6] = a_global.tolist()
    statedot[9:12] = alphas.tolist()

    return statedot