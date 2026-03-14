import numpy as np
from main_forces import compute_total_forces
from integrator import euler_integrate
from motor_command_state import MotorCommand
from motor_torque import compute_motor_torques

def run_simulation(boat_state, haptic_state, env, params):
    """
    runs one timestep of sailboat simulation

    Inputs:
        boat_state    :current boat state
        haptic_state  :HapticState instance (current wheel/winch positions)
        env           :Environment instance (wind, current)
        params        :SimulatorParams instance
        dt            :timestep in seconds

    Output:
        new_boat_state :updated BoatState after dt
        tau_total     : forces & moments vector acting on boat
    """
    # Compute forces
    tau_total_k1, sail_lift, sail_drag = compute_total_forces(boat_state, haptic_state, env, params)
    
    # Integrate boat motion
    boat_state_k1 = euler_integrate(boat_state, tau_total_k1, params)

    # Compute motor torques
    motor_command = compute_motor_torques(tau_total_k1, sail_lift, sail_drag, haptic_state, params) 


    return boat_state_k1, tau_total_k1, motor_command