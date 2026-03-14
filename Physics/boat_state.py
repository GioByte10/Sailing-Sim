import numpy as np

class BoatState:

    # nu = [x y z phi theta psi ] is boat orientation and position in earth frame
    # v = [u v w p q r] is linear and angular velocities of boat in boat frame
    # alpha = sail angle relative to boat frame

    def __init__(self, nu=None, v=None, alpha=None):
        
        if nu is None:
            nu = np.zeros(6)

        if v is None:
            v = np.zeros(6) 
        
        if alpha is None:
            alpha = 0

        self.nu = np.array(nu, dtype=float)
        self.v = np.array(v, dtype = float)
        self.alpha = alpha

    def copy(self):
        # returns deep copy of state
        return BoatState(self.nu.copy(), self.v.copy())
    
    def as_vector(self):
        # returns full 12 state vector
        return np.concatenate((self.nu, self.v))
    
    @staticmethod
    def from_vector(x):
        # decomposes 13 element vector into state
        nu = x[0:6]
        v = x[6:11]
        alpha = x[13]

        return BoatState(nu, v)