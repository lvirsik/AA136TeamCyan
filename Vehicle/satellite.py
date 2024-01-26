from Vehicle.satelliteConstants import *

class Satellite:
    def __init__(self, simulation_timestep):
        self.ts = simulation_timestep
        self.mass = MASS
        self.I = MOI
        self.I_prev = MOI
        self.engine_length = ENGINE_LENGTH
        self.com = COM
        self.engine_posx = 0
        self.engine_posy = 0