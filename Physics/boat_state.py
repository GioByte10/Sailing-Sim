import numpy as np

class BoatState:

    # params are parameters in params.py
    # nu = [x y z phi theta psi ] position/orientation in earth frame
    # v = [u v w p q r] linear/angular velocity in body frame

    def __init__(self, params, nu=None, v=None):

        self.p = params
        
        if nu is None:
            nu = np.zeros(6)
            
        if v is None:
            v = np.zeros(6)
            v[0] = 4 # 2 knots or 4 m/s

        self.nu = np.array(nu, dtype=float)
        self.v = np.array(v, dtype=float)

    def copy(self):
        return BoatState(self.p, self.nu.copy(), self.v.copy())

    def as_vector(self):
        return np.concatenate((self.nu, self.v))

    @staticmethod
    def from_vector(params, x):

        nu = x[0:6]
        v = x[6:12]

        return BoatState(params, nu, v)