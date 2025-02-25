import numpy as np
from simulator import PendulumSimulator
from controller import PendulumLQR

# Define the parameters for the pendulum system
config = {
    'm': 1.0,       # Mass of the pend
    'l': 1.0,       # Length of the pendulum
    'g': 9.81,      # Acceleration due to gravity
    'scale': 250,   # Scale for rendering
    'damping': 0.1, # Damping factor
    'hz': 100,
    'decimation': 5,
    'torque_max': 10
}

# Define the LQR controller parameters
Q = np.diag([30, 10])
R = np.array([[1]])
setpoint = np.array([-np.pi/2, 0])
controller = PendulumLQR(config['m'], config['l'], setpoint[0], Q, R, g=config['g'], dt =  config['decimation'] / config['hz'])
simulator = PendulumSimulator(controller, **config)
simulator.simulate()

