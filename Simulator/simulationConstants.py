import numpy as np
import random

# Simulation Variables
# State is all relative to global frame except Z rotation which is rocket frame
# Pitch is defined as angle from upward z axis towards pos x axis, yaw is angle from upward z towards pos y, and roll is ccw looking down on rocket
# Rotation order yaw, pitch, roll
INITIAL_STATE = np.array([0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, -2]) # Start State [X, Y, Z, VX, VY, VZ, THX, THY, THZ, OMX, OMY, OMZ]
FINAL_TIME = 20
TIMESTEP = 0.1
TARGET_ALTITUDE = 50 #meters

# Physical Constants
GRAVITY = 9.81 # m/s^2
RHO = 1.225 #kg/m^3

RAD2DEG = 57.2958

# Disturbance for pos and throttle
RANDOMIZED_ERROR_POS = 0.0001 # meter
CONSTANT_ERROR_POS = 0.001 # meter
RANDOMIZED_ERROR_THROTTLE = 0.01
CONSTANT_ERROR_THROTTLE = 0.01

# Noise for state
STATE_SIGMA = 0.001
STATE_MU = 0
ROLL_SIGMA = 0.1
ROLL_MU = 0

