import numpy as np
from apparent_wind import compute_apparent_wind

def compute_total_forces(boat_state, haptic_state, control_state, env, params):
    """
    Computes total forces moment vector Tau
    Tau = [X, Y, Z, K, M, N] for boat linear forces and moments
    """
    # Compute individual forces
    tau_sail, sail_lift, sail_drag = compute_sail_forces(boat_state, haptic_state, control_state, env, params)
    tau_rudder = compute_rudder_forces(boat_state, haptic_state, control_state, env, params)
    tau_hull = compute_hull_drag(boat_state, params)

    # FIX: tau_rudder_hinge_moment is haptic only. It does not go into total boat forces.
    print(f"tau_sail: {tau_sail}")
    print(f"tau_rudder: {tau_rudder}")
    print(f"tau_hull: {tau_hull}")
    tau = tau_sail + tau_rudder + tau_hull

    print(f"X Sail Force: {tau[0]}")

    return tau, sail_lift, sail_drag

def compute_rudder_forces(boat_state, haptic_state, control_state, env, params):
    """
    Computes rudder forces and moments in body frame
    """
    tau_rud = np.zeros(6)

    # Boat velocities
    u = boat_state.v[0] 
    v = boat_state.v[1] 
    r = boat_state.v[5] # FIX: Boat yaw rate
    V = np.sqrt(u**2 + v**2) 

    rudder_angle = control_state.rudder_angle
    
    # FIX: Effective lateral velocity uses boat yaw rate (r), not wheel rotation
    v_eff = v + params.rudder_arm * r
    
    flow_angle = np.arctan2(v_eff, u) if u != 0 else 0
    flow_angle = np.mod(flow_angle + np.pi, 2*np.pi) - np.pi
    
    alpha_r = rudder_angle - flow_angle
    alpha_r = np.mod(alpha_r + np.pi, 2*np.pi) - np.pi

    # Calculating rudder geometry
    rudder_SA = params.rudder_width * params.rudder_height 
    rudder_AR = params.rudder_height**2 / rudder_SA 

    # Calculating coefficients of lift and drag
    CL = (2*np.pi * alpha_r) / (1 + 2/rudder_AR)
    CLmax = params.rudder_CLmax
    CL = np.clip(CL, -CLmax, CLmax)
    CD = CL**2 / (np.pi * rudder_AR)

    # Calculating rudder forces
    K = 0.5 * params.rho_h20 * V**2 * rudder_SA 
    L = K * CL 
    D = K * CD 

    Vvec = np.array([u, v])
    Vmag = np.linalg.norm(Vvec)

    if Vmag < 1e-5:
        return tau_rud

    Vhat = Vvec / Vmag
    drag_dir = -Vhat
    lift_dir = np.array([-Vhat[1], Vhat[0]])
    F = L * lift_dir + D * drag_dir

    tau_rud[0] += F[0]
    tau_rud[1] += F[1]
    tau_rud[5] += F[1] * params.rudder_arm

    print(f"X Rud Force: {F[0]}")

    return tau_rud

def compute_sail_forces(boat_state, haptic_state, control_state, env, params):
    """
    Computes sail forces and moments in body frame
    """
    tau_sail = np.zeros(6)

    # Apparent Wind Magnitude v_aw and Angle beta_aw
    v_aw, beta_aw = compute_apparent_wind(boat_state.v[0:3], env, boat_state.nu[5])
    V = v_aw[0:2]
    Vmag = np.linalg.norm(v_aw) 

    if Vmag < 1e-5:
        return tau_sail, 0, 0
    
    Vhat = V / Vmag 
    beta_aw = float(beta_aw)
    alpha = beta_aw - control_state.sail_angle 
   
    CLmax = params.sailCLmax
    CL= np.clip(CLmax*np.sin(2*alpha), -CLmax, CLmax)
    CD = params.sailCDmin + (params.sailCDmax - params.sailCDmin)*np.sin(alpha)**2 

    K = 0.5 * params.rho_air * params.sailA * Vmag**2 
    L = CL * K 
    D = CD * K 

    drag_dir = -Vhat
    lift_dir = np.array([-Vhat[1], Vhat[0]])

    F = L * lift_dir + D * drag_dir
    tau_sail[0] = F[0]
    tau_sail[1] = F[1]
    tau_sail[5] = F[1] * params.sailArm

    print(f"X Sail Force: {F[0]}")

    return tau_sail, L, D

def compute_hull_drag(boat_state, params):
    """
    Computes hull hydrodynamic drag forces in body frame.
    """
    tau = np.zeros(6)
    rho = params.rho_h20

    u = boat_state.v[0]  
    v = boat_state.v[1]  
    r = boat_state.v[5]  

    X_drag = -0.5 * rho * params.hull_Cd_x * params.hull_Ax * u * abs(u)
    Y_drag = -0.5 * rho * params.hull_Cd_y * params.hull_Ay * v * abs(v)
    
    # FIX: Added linear damping alongside quadratic damping to kill micro-oscillations near zero
    N_drag = -params.yaw_damping * r * abs(r) - (params.yaw_damping * 0.1) * r

    tau[0] = X_drag
    tau[1] = Y_drag
    tau[5] = N_drag

    print(f"X Hull Force: {X_drag}")

    return tau


def compute_rudder_hinge_moment(boat_state, haptic_state, control_state, params):
    """
    Computes the torque required to turn the rudder on its shaft (Haptic feedback only)
    """
    rho = params.rho_h20
    delta = control_state.rudder_angle
    u = boat_state.v[0]
    v = boat_state.v[1]
    V = np.sqrt(u**2 + v**2)

    chord = params.rudder_width
    span = params.rudder_height
    area = chord * span

    Ch_alpha = params.rudder_Ch_alpha
    Ch = Ch_alpha * delta
    Mh = 0.5 * rho * V**2 * area * chord * Ch
    #motor_command.wh_torque += Mh TODO
    return Mh
