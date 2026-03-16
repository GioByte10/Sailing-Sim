import time
import numpy as np
from boat_state import BoatState
from haptic_state import HapticState
from environment_state import Environment 
from motor_command_state import MotorCommand
from control_state import ControlState
from params import Params
from simulate import run_simulation
#TODO import motor_interface
#TODO import graphics 

import matplotlib.pyplot as plt


def main():
    params = Params()
    boat_state = BoatState(params)
    haptic_state = HapticState()
    motor_command = MotorCommand()
    control_state = ControlState()
    env = Environment()

    dt = params.dt
    t = params.t_start
    t_end = params.t_end

    log_state = []
    log_haptic = []
    log_forces = []
    log_torque = []

    # For Plots:
    # Initialize storage lists
    time_data = []

    # boat_state.nu: [X, Y, Z, u, v, r] or similar
    nu_data = []  # store as list of vectors
    # boat_state.v: [Fx, Fy, Fz, Mx, My, Mz] or similar
    v_data = []

    # Graphics Start
    # last_draw_time = time.time()
    # draw_interval = 1/60 # for 60 HZ   

    while t < t_end:
        loop_start = time.time()
        print(t)
        # read motors 
        haptic_state.wh[0] = 0#.1*np.sin(2*np.pi*.02*t) # wheel position
        haptic_state.wh[1] = 0 # wheel velocity
        haptic_state.wh[2] = 0 # wheel acceleration

        haptic_state.wi[0] = 0 #2 *np.pi # devided by winch gear rati
        haptic_state.wi[1] = 0.0 # winch velocity
        haptic_state.wi[2] = 0.0 # winch acceleration

        # Updates boat control surfaces
        control_state.update(haptic_state, params)

        # simulation for single  step
        boat_state, tau_total, motor_command = run_simulation(boat_state, 
                                                              haptic_state, 
                                                              control_state,
                                                              env, params)

        # Send haptic torques to motors
        wheel_torque = motor_command.wh_torque #tau_total[5] / params.steering_ratio
        winch_torque = motor_command.wi_torque #motor_command.wi_torque

        #print(f"Wheel Torque:  {wheel_torque}")
        #print(f"Winch Torque: {winch_torque}")
        #print(f"Yaw Position: {boat_state.nu[5]*180/np.pi}")
        #print(f"Speed u: {boat_state.v[0]}")
        #print(f"Speed v: {boat_state.v[1]}")
        # motor1_interface.send_torques(wheel_torque)
        # motor2_interface.send_torques(winch_torque)

        # Update Graphics 
        # if (time.time() - last_draw_time) >= draw_interval:
            #graphics.update(boat_state)
            #last_draw_time = time.time()
        
 

        log_state.append(boat_state.as_vector())
        log_haptic.append(haptic_state.as_vector())
        log_forces.append(tau_total)
        log_torque.append([wheel_torque, winch_torque])

        # For plots
        time_data.append(t)
        nu_data.append(boat_state.nu)
        v_data.append(boat_state.v)

        t += dt # Update time
     

    
    log_state=np.array(log_state)
    log_haptic = np.array(log_haptic)
    log_forces = np.array(log_forces)
    log_torque = np.array(log_torque)

    print("Simulation complete")

    # Convert to NumPy arrays for easier slicing
    time_data = np.array(time_data)
    nu_data = np.array(nu_data)  # shape (N,6)
    v_data = np.array(v_data)    # shape (N,6)

    # ---- Plot boat_state.nu quantities ----
    plt.figure(figsize=(10, 6))
    plt.plot(time_data, nu_data[:,0], label='X Position', color='red')
    plt.plot(time_data, nu_data[:,1], label='Y Position', color='blue')
    plt.plot(time_data, np.rad2deg(nu_data[:,5]), label='Yaw', color='green')
    plt.xlabel('Time (s)')
    plt.ylabel('boat_state.nu')
    plt.title('Boat State Position vs Time')
    plt.legend()
    plt.grid(True)

    # ---- Plot boat_state.v quantities ----
    plt.figure(figsize=(10, 6))
    plt.plot(time_data, v_data[:,0], label='X Force', color='red')
    plt.plot(time_data, v_data[:,1], label='Y Force', color='blue')
    plt.plot(time_data, v_data[:,5], label='Z Moment', color='green')
    plt.xlabel('Time (s)')
    plt.ylabel('boat_state.v')
    plt.title('Boat State Forces & Moment vs Time')
    plt.legend()
    plt.grid(True)

    plt.show()




if __name__ == "__main__":
    main()
