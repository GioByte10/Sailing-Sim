import numpy as np

class MotorCommand:

    def __init__(self):

        # torques to be sent to motors
        self.wh_torque = 0.0 # wheel torque input
        self.wi_torque = 0.0 # winch torque input

    def as_vector(self):
        return np.array([
            self.wh_torque,
            self.wi_torque
        ])