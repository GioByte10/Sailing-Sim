import numpy as np
from boat_state import BoatState

def euler_integrate(boat_state, tau, params):
    """
    Integrates boat state forward in time using Euler Fist Order

    Inputs:
        boat_state
        tau: force and moment vector [X,Y,Z,K,M,N]
        params
    Output:
        new_state : new boat_state
    """

    nu_k=boat_state.nu.copy() # current k boat position & orientation
    v_k = boat_state.v.copy() # current k boat linear & angular velocities
    dt = params.dt # time step

    M = np.diag([params.mass, params.mass, params.mass,
                params.Ix, params.Iy, params.Iz])
    
    # TODO include Damping by @(tau - D@V)

    # Velocity 
    v_k = np.linalg.inv(M) @ (tau)
    
    # Velocity Update
    v_k1 = v_k + v_k*dt

    # TODO implement rotation matrixes for integration instead
    # Linear Positions Update
    nu_k1 = nu_k.copy()
    nu_k1[0:3] += v_k1[0:3]*dt

    # Angular Update
    nu_k1[3:6] += v_k1[3:6]*dt

    return BoatState(nu_k1, v_k1)








