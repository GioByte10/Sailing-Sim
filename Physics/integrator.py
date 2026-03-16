import numpy as np
from boat_state import BoatState

def euler_integrate(boat_state, tau, params):
    """
    Integrates boat state forward in time.
    Uses sub-stepped Euler First Order for numerical stability 
    with stiff drag forces and large timesteps.
    """
    nu_k = boat_state.nu.copy() # current position & orientation (Earth frame)
    v_k = boat_state.v.copy()   # current linear & angular velocities (Body frame)
    
    # Sub-stepping to handle large dt (0.3s) without exploding
    sub_steps = params.sub_steps
    dt_sub = params.dt / sub_steps
    mass = params.mass

    surge = params.x_surge
    sway = params.y_sway
    heave = params.z_heave
    yaw_add = params.z_yaw_factor

    M = np.diag([mass*surge, mass*sway, mass*heave,
                params.Ix, params.Iy, params.Iz*yaw_add])
    C = np.eye(6)*params.damping*boat_state.v
    M_inv = np.linalg.inv(M)
   
    for _ in range(sub_steps):
        # Current Acceleration 
        d_surge = 50.0 + 100.0 * abs(v_k[0])
        d_sway  = 200.0 + 500.0 * abs(v_k[1])
        d_yaw   = params.yaw_damping + (params.yaw_damping * 0.5) * abs(v_k[5])
    
        # Simple diagonal damping matrix
        D = np.diag([d_surge, d_sway, 10.0, 50.0, 50.0, d_yaw])

        # 3. Calculate Acceleration: a = M^-1 * (Tau_ext - D*v)
        # This is the "Lagrangian" way to implement damping.
        a_k = np.linalg.inv(M) @ (tau - D @ v_k)
        
        # Velocity Update (Body Frame)
        v_k += a_k * dt_sub

        # --- FIX: Rotate body velocities into Earth frame ---
        yaw = nu_k[5]
        c, s = np.cos(yaw), np.sin(yaw)
        
        X_dot = v_k[0]*c - v_k[1]*s
        Y_dot = v_k[0]*s + v_k[1]*c
        Z_dot = v_k[2]
        
        # Linear Positions Update (Earth Frame)
        nu_k[0] += X_dot * dt_sub
        nu_k[1] += Y_dot * dt_sub
        nu_k[2] += Z_dot * dt_sub

        # Angular Position Update
        nu_k[3:6] += v_k[3:6] * dt_sub

    return BoatState(params, nu_k, v_k)