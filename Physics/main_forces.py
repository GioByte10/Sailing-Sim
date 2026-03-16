import numpy as np
from apparent_wind import compute_apparent_wind


#TODO add hydrodynamic, sail, keel, hull

# Tau = [X, Y, Z, K, M, N] for boat linear forces and rotational moments in boat frame


def compute_total_forces(boat_state, haptic_state, control_state, env, params):
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
    # Compute forces
    tau_sail, sail_lift, sail_drag = compute_sail_forces(boat_state, haptic_state, control_state, env, params)
    
    tau_rudder = compute_rudder_forces(boat_state, haptic_state, control_state, env, params)

    tau_rudder_hinge_moment = compute_rudder_hinge_moment(boat_state, haptic_state, control_state, params)

    tau_hull = compute_hull_drag(boat_state, params)

    # Summing Forces for net force tau
    tau = tau_sail + tau_rudder + tau_rudder_hinge_moment + tau_hull

    return tau, sail_lift, sail_drag

def compute_rudder_forces(boat_state, haptic_state, control_state, env, params):
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

    tau_rud = np.zeros(6)

    # Calculating Velocity Magnitude
    u = boat_state.v[0] # - water env ue velocity
    v = boat_state.v[1] # - water env ve velocity
    V = np.sqrt((u)**2 + v**2) # velocity magnitude

    # Calculating effective rudder angle to water flow
    rudder_angle = control_state.rudder_angle 
    alpha_r = rudder_angle - np.arctan2(v,u)

    # Cacluating rudder geometry
    rudder_SA = params.rudder_width * params.rudder_height # surface area
    rudder_AR = params.rudder_height**2 / rudder_SA # aspect ratio

    # Calculating coefficients of lift and drag
    CL = (2*np.pi * alpha_r) / (1 + 2/rudder_AR)
    CLmax = params.rudder_CLmax
    CL = np.clip(CL, -CLmax, CLmax)
    CD = CL**2 / (np.pi * rudder_AR)

    # Calculating rudder forces
    K = 0.5 * params.rho_h20 * V**2 * rudder_SA 
    L = K*CL # lift force
    D = K*CD # drag force

    Vvec = np.array([u, v])
    V = np.linalg.norm(Vvec)

    if V < 1e-5:
        return tau_rud

    Vhat = Vvec/ V
    drag_dir = -Vhat
    lift_dir = np.array([-Vhat[1], Vhat[0]])
    F = L*lift_dir + D*drag_dir

    tau_rud[0] += F[0]
    tau_rud[1] += F[1]
    tau_rud[5] += F[1] * params.rudder_arm


    #print(f"Rudder Angle: {rudder_angle*180/np.pi}")
    #print(f"Vx: {u}")
    #print(f"Vy: {v}")
    #print(f"CL: {CL}")
    #print(f"CD: {CD}")
    # print(f"Lift: {L}")
    # print(f"Drag: {D}")
    #print(f"dynamic pressure: {K/rudder_SA}")



    return tau_rud

def compute_sail_forces(boat_state, haptic_state, control_state, env, params):
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
    V = v_aw[0:2]
    Vmag = np.linalg.norm(v_aw) # wind velocity magnitude

    # if wind is small, generate no forces
    if Vmag < 1e-5:
        boat_state.alpha = 0
        return tau_sail, 0, 0
    
    Vhat = V / Vmag # normalized wind

    # Calculating Sail Angle of Attack alpha
    beta_aw = float(beta_aw)
    alpha = beta_aw-control_state.sail_angle # relative wind minus sail angle relative to boat
   
    # Calculating Aerodynamic Coefficients
    CLmax = params.sailCLmax
    CL= np.clip(CLmax*np.sin(2*alpha),-1*CLmax, CLmax)
    CD = params.sailCDmin + (params.sailCDmax - params.sailCDmin)*np.sin(alpha)**2 

    # Calculating Forces
    K = 0.5*params.rho_air*params.sailA*Vmag**2 # aerodynamic coefficient
    L = CL*K # lift force
    D = CD*K # drag force


    #print(f"Aerodynamic Constant {K}")
    #print(f"Lift Force {L}")
    #print(f"Drag Force {D}")

    # Transforming and adding forces to body frame
    #c, s = np.cos(beta_aw), np.sin(beta_aw)
    #tau_sail[0] = L*c -D*s # Force in X
    #tau_sail[1] = L*s + D*c # Force in Y
    #tau_sail[5] = (L*s + D*c)*params.sailArm # Moment about Z - Weather Helm Arm

    drag_dir = -Vhat
    lift_dir = np.array([-Vhat[1], Vhat[0]])

    F = L*lift_dir + D*drag_dir
    tau_sail[0] = F[0]
    tau_sail[1] = F[1]
    tau_sail[5] = F[1]*params.sailArm
    

    # print(f"X Sail Force: {tau_sail[0]}")
    # print(f"Y Sail Force: {tau_sail[1]}")
    # print(f"Z Sail Moment: {tau_sail[5]}")
    # print(f"Yaw: {boat_state.nu[5]}")

    return tau_sail, L, D

def compute_rudder_hinge_moment(boat_state, haptic_state, control_state, params):

    tau_rudder_hinge = np.zeros(6)
    rho = params.rho_h20

    # rudder angle
    delta = control_state.rudder_angle

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

    tau_rudder_hinge[5] = Mh

    return tau_rudder_hinge


def compute_hull_drag(boat_state, params):
    """
    Computes hull hydrodynamic drag forces in body frame.

    Returns:
        tau_drag : np.array([X,Y,Z,K,M,N])
    """

    tau = np.zeros(6)

    rho = params.rho_h20

    # boat velocity in body frame
    u = boat_state.v[0]  # surge velocity
    v = boat_state.v[1]  # sway velocity
    r = boat_state.v[5]  # yaw rate

    # surge drag (forward resistance)
    X_drag = -0.5 * rho * params.hull_Cd_x * params.hull_Ax * u * abs(u)

    # sway drag (lateral resistance)
    Y_drag = -0.5 * rho * params.hull_Cd_y * params.hull_Ay * v * abs(v)

    # yaw damping (rotational resistance)
    N_drag = -params.yaw_damping * r * abs(r)

    tau[0] = X_drag
    tau[1] = Y_drag
    tau[5] = N_drag

    return tau