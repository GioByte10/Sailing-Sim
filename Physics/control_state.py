import numpy as np

class ControlState:
    """
    Stores derived control variables used by the simulator.

    Inputs:
        wheel angle (from haptic_state)
        winch angle (from haptic_state)

    Derived:
        rudder_angle
        sail_angle
    """

    def __init__(self):
        self.rudder_angle = 0.0
        self.sail_angle = 0.0

    def update(self, haptic_state, params):

        wheel_angle = haptic_state.wh[0]
        winch_angle = haptic_state.wi[0]

        # Rudder angle
        delta = wheel_angle / params.steering_ratio
        delta = np.clip(delta,
                        -params.rudder_angle_limit,
                        params.rudder_angle_limit)

        # Sail angle
        sail_angle = winch_angle / params.winch_ratio
        sail_angle = np.clip(sail_angle, 
                            -params.sailAngleMax,
                            params.sailAngleMax)

        self.rudder_angle = delta
        self.sail_angle = sail_angle


    def as_vector(self):
        return np.array([self.rudder_angle, self.sail_angle])