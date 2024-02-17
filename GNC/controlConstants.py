# Constraints on Actuators

# Position Constraints
MAX_POS = 0.1 # meters from centerline of rocket
MAX_THROTTLE = 1 # meters from centerline of rocket
MIN_THROTTLE = 0.2 # percent

# Velocity Constraints
MAX_THROTTLE_RATE = 0.5 # throttles per second, ie 5 --> 500 percent per second
MAX_POS_RATE = 0.01# m per second - actuator speed in direction with a bit of saftey margin - should be 0.0092

# Linearization
STEP_SIZE = 0.001

# LQR Controller Penalties for regular flight
Q_X = 1
Q_Y = 1
Q_Z = 1
Q_VX = 1
Q_VY = 1
Q_VZ = 1
Q_PIT = 1
Q_YAW = 1
Q_ROL = 1
Q_VPIT = 1
Q_VYAW = 1
Q_VROL = 1
R_P = 1
R_Y = 1
R_R = 1
R_X = 1
R_Y = 1
R_Z = 1

## Kalman Filter
SIGMA_PROCESS_NOISE = 0.01