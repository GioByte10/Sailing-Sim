import numpy as np
from apparent_wind import compute_apparent_wind


#TODO add hydrodynamic, sail, keel, hull

# Tau = [X, Y, Z, K, M, N] for boat linear forces and rotational moments in boat frame


def compute_total_forces(boat_state, haptic_state, env, params):
    """
    Computes total forces moment vector Tau
    Tau = [X, Y, Z, K, M, N] for boat linear forces and moments
    
    Inputs: 
        boat_state
        haptic_state
        env
        params
    
    Outputs:
        tau_rudder: np.array({X,Y,Z,K,M,N}) of 6 element array of 
                    total boat forces and moments
  
    """

    tau_sail, sail_lift, sail_drag = compute_sail_forces(boat_state, haptic_state, env, params)
    
    
    tau_rudder = compute_rudder_forces(boat_state, haptic_state, env, params)


    # Summing Forces for net force tau
    tau = tau_sail + tau_rudder

    return tau, sail_lift, sail_drag

def compute_rudder_forces(boat_state, haptic_state, env, params):
    """
    Computes rudder forces and moments in body frame
    
    Inputs: 
        boat_state
        haptic_state
        params

    Outputs:
        tau_rudder: np.array({X,Y,Z,K,M,N}) of 6 element array of 
                    boat forces and moments by rudder
    """

    #TODO also add local water relative speeds from env

    tau = np.zeros(6)

    rudder_angle = haptic_state.wh[0] / params.steering_ratio

    u = boat_state.v[0]
    v = boat_state.v[1]

    V = np.sqrt(u**2 + v**2)

    rudder_SA = params.rudder_width * params.rudder_height
    rudder_AR = params.rudder_height**2 / rudder_SA

    CL = (2*np.pi * rudder_angle) / (1 + 2/rudder_AR)
    CD = CL**2 / (np.pi * rudder_AR)

    L = 0.5 * params.rho_h20 * V**2 * rudder_SA * CL
    D = 0.5 * params.rho_h20 * V**2 * rudder_SA * CD

    tau[1] += L
    tau[0] -= D



    tau[5] += L * params.rudder_arm

    return tau

def compute_sail_forces(boat_state, haptic_state, env, params):
    """
    Computes sail forces and moments in body frame
    
    Inputs: 
        boat_state
        haptic_state
        params

    Returns:
        tau_sail: np.array({X,Y,Z,K,M,N}) of 6 element array of 
                  boat forces and moments by sail
        lift: lift force
        drag: sail drag force
    """
    tau_sail = np.zeros(6)


    # Apparent Wind Magnitude v_aw and Angle beta_aw
    v_aw, beta_aw = compute_apparent_wind(boat_state.v[0:3],
                                          env,
                                          boat_state.nu[5])
    v_aw = np.linalg.norm(v_aw)
    beta_aw = float(beta_aw)
    # Calculating Aerodynamic Coefficients
    alpha = beta_aw-haptic_state.wi[0] # sail trim angle in boat frame
    CL= params.sailCLmax*np.sin(2*alpha)
    CD = params.sailCDmin + (params.sailCDmax - params.sailCDmin)*np.sin(alpha)**2 

    # Calculating Forces
    K = -0.5*params.rho_air*params.sailA*v_aw**2 # aerodynamic constant
    L = CL*K # lift force
    D = CD*K # drag force

    
    # Transforming and adding forces to body frame
    c, s = np.cos(beta_aw), np.sin(beta_aw)
    tau_sail[0] = L*c -D*s # Force in X
    tau_sail[1] = L*s + D*c # Force in Y
    tau_sail[5] = (L*s + D*c)*params.sailArm # Moment about Z - Weather Helm Arm

    return tau_sail, L, D

def compute_rudder_hinge_moment(boat_state, haptic_state, params):

    rho = params.rho_h20

    # rudder angle
    delta = haptic_state.wh[0] / params.steering_ratio

    # boat velocity
    u = boat_state.v[0]
    v = boat_state.v[1]

    V = np.sqrt(u**2 + v**2)

    # rudder geometry
    chord = params.rudder_width
    span = params.rudder_height
    area = chord * span

    # hinge moment coefficient slope
    Ch_alpha = params.rudder_Ch_alpha

    # hinge coefficient
    Ch = Ch_alpha * delta

    # hinge moment
    Mh = 0.5 * rho * V**2 * area * chord * Ch

    return Mh