import numpy as np
import control
from copy import deepcopy, copy
from Simulator.dynamics import dynamics_for_state_space_control
from Simulator.simulationConstants import GRAVITY as g
from GNC.controlConstants import *

def control_satellite(K, state_error):
    """ Function that is called to get control inputs at each time step
    
    Inputs:
    - Controller K matrix
    - State error in global frame
    - linearized control input 
    
    Output:
    - Control Input Vector U
    """
    U = np.dot(-K, state_error) # U is the desired accelerations
    return U

def compute_K_flight(len_state, A, B):
    """Compute the K matrix for the flight phase of the mission using lqr
    
    Inputs:
    - length of state
    - A matrix
    - B matrix 
    """
            
    # Q and R
    Q = np.identity(len_state) #Minus 2 to remove roll stuff plus 3 to get integral control
    R = np.identity(6)

    Q[0][0] = 1 / (Q_X ** 2)
    Q[1][1] = 1 / (Q_Y ** 2)
    Q[2][2] = 1 / (Q_Z ** 2)
    Q[3][3] = 1 / (Q_VX ** 2)
    Q[4][4] = 1 / (Q_VY ** 2)
    Q[5][5] = 1 / (Q_VZ ** 2)
    Q[6][6] = 1 / (Q_PIT ** 2)
    Q[7][7] = 1 / (Q_YAW ** 2)
    Q[8][8] = 1 / (Q_ROL ** 2)
    Q[9][9] = 1 / (Q_VPIT ** 2)
    Q[10][10] = 1 / (Q_VYAW ** 2)
    Q[11][11] = 1 / (Q_VROL ** 2)
    
    R[0][0] = 1 / (R_X ** 2)
    R[1][1] = 1 / (R_Y ** 2)
    R[2][2] = 1 / (R_Z ** 2)
    R[3][3] = 1 / (R_P ** 2)
    R[4][4] = 1 / (R_Y ** 2)
    R[5][5] = 1 / (R_R ** 2)

    K,S,E = control.lqr(A, B, Q, R) # The K that this spits out has the additional 3 integral terms tacked onto the end, so must add errorx, y, z onto end of 
    
    return K

def compute_A(state, rocket, dt):
    """ Compute Jacobian for the A matrix (State dot wrt State)
    
    Inputs:
    - state (1x12)
    - Control input U (1x3)
    - rocket object
    - timestep length
    """
    h = STEP_SIZE
    jacobian = np.zeros((len(state), len(state)))
    for i in range(len(state)):
        state_plus = deepcopy(state).astype(float)
        state_minus = deepcopy(state).astype(float)
        state_plus[i] = state_plus[i] + h
        state_minus[i] = state_minus[i] - h
        statedot_plus = dynamics_for_state_space_control(state_plus, rocket, dt, [0, 0, 0, 0, 0, 0])
        statedot_minus = dynamics_for_state_space_control(state_minus, rocket, dt, [0, 0, 0, 0, 0, 0])
        jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
    return jacobian.T

def compute_B(state, rocket, dt):
    """ Compute Jacobian for the B matrix (State dot wrt control input)
    
    Inputs:
    - state (1x12)
    - Control input U (1x3)
    - rocket object
    - timestep length
    """
    h = STEP_SIZE
    jacobian = np.zeros((6, len(state)))
    for i in range(6):
        u_plus = [0, 0, 0, 0, 0, 0, 0]
        u_minus = [0, 0, 0, 0, 0, 0, 0]
        u_plus[i] = h
        u_minus[i] = -h
        statedot_plus = dynamics_for_state_space_control(state, rocket, dt, u_plus)
        statedot_minus = dynamics_for_state_space_control(state, rocket, dt, u_minus)
        jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
    return jacobian.T

