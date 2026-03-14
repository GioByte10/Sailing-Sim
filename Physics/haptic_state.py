import numpy as np

class HapticState:
    # Haptic Interface: Wheel and Winch
    # wh = steering wheel kinematics [angular position, velocity, acceleration]
    # wi = main sail winch kinematics [angular positon, velocity, acceleration]

    def __init__(self, wh=None, wi=None):
        if wh is None:
             wh = np.zeros(3)
        
        if wi is None:
            wi = np.zeros(3)

        self.wh = np.array(wh, dtype=float)
        self.wi = np.array(wi, dtype = float)

    def copy(self):
        # returns deep copy of haptic state
        return HapticState(self.wh.copy(), self.wi.copy())
    
    def as_vector(self):
        # returns full 6 elment vector state
        return np.concatenate([self.wh, self. wi])
    
    @staticmethod
    def from_vector(x):
        # construct haptic state from vector

        wh = x[0:3]
        wi = x[3:6]

        return HapticState(wh, wi)
