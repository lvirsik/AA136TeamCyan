import numpy as np
from scipy.spatial.transform import Rotation
from Simulator.simulationConstants import GRAVITY as g
from Simulator.simulationConstants import RHO as rho
from Simulator.simulationConstants import *

def full_dynamics(state, satellite, dt, t):
    # Pull Params
    throttle = satellite.engine.throttle
    T = satellite.engine.get_thrust(t, throttle)
    m = satellite.mass
    lever_arm = satellite.com
    engine_length = satellite.engine.length
    posX = satellite.engine.posx
    posY = satellite.engine.posy
    v = state[3:6]
    w = state[9:12]

    # Convert Actuator Positions to Cyclindrical Coords
    gimbal_R = np.sqrt((posX ** 2) + (posY ** 2))
    gimbal_theta = np.arctan2(posY, posX)
    gimbal_psi = np.arctan2(gimbal_R, engine_length)

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
    aX_rf = (T * np.sin(gimbal_psi) * -np.cos(gimbal_theta) / m) + (-1 * g * R[0][2])
    aY_rf = (T * np.sin(gimbal_psi) * -np.sin(gimbal_theta) / m) + (-1 * g * R[1][2])
    aZ_rf = (T * np.cos(gimbal_psi) / m) + (-1 * g * R[2][2])
    a_rf = np.array([aX_rf, aY_rf, aZ_rf])

    # Convert Accelerations from rocket frame to global frame
    a_global = np.dot(R_inv, a_rf)

    # Calculate Alphas
    torque = np.array([(T * np.sin(gimbal_psi) * -np.cos(gimbal_theta) * lever_arm),
                        (T * np.sin(gimbal_psi) * -np.sin(gimbal_theta) * lever_arm),
                        0])
    I_dot = (satellite.I - satellite.I_prev) / dt
    alphas = np.dot(np.linalg.inv(satellite.I), torque - np.cross(w, np.dot(satellite.I, w)) - np.dot(I_dot, w))

    statedot[3:6] = a_global.tolist()
    statedot[9:12] = alphas.tolist()

    return statedot