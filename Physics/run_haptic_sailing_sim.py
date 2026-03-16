import time
import math
import socket

# --- 1. HARDWARE CONFIGURATION & MAPPING ---
class HardwareConfig:
    def __init__(self):
        # Physics Engine expected maximums (Raw theoretical values)
        self.winch_max_physics = 400.0
        self.wheel_max_physics = 100.0
        
        # Physical Hardware Limits (Set these to your motor's safe Nm ratings)
        self.winch_max_nm = 3.0
        self.wheel_max_nm = 7.0
        
        # Calculate the scaling ratios
        self.winch_scale = self.winch_max_nm / self.winch_max_physics
        self.wheel_scale = self.wheel_max_nm / self.wheel_max_physics

    def map_winch(self, raw_torque):
        """Scales and clamps the winch torque for hardware safety."""
        scaled = raw_torque * self.winch_scale
        return max(-self.winch_max_nm, min(scaled, self.winch_max_nm))

    def map_wheel(self, raw_torque):
        """Scales and clamps the wheel torque for hardware safety."""
        scaled = raw_torque * self.wheel_scale
        return max(-self.wheel_max_nm, min(scaled, self.wheel_max_nm))

# --- 2. PHYSICS & HEADING ENGINE ---
class DynamicBoatPhysics:
    def __init__(self):
        self.true_wind_speed = 15.0      
        self.true_wind_angle = 90.0 
        
        # Dynamic States
        self.boat_speed = 0.0       
        self.heading = 0.0          
        self.last_time = time.time()
        self.last_wheel_angle = 0.0
        self.pos_x = 0.0    
        self.pos_y = 0.0    
        
        # Boat Specs
        self.boat_mass = 1500.0     
        self.hull_drag_coeff = 25.0 
        self.max_thrust = 2000.0
        self.boat_length = 10.0     
        
        # Base Environmental Tuning
        self.k_winch = 1.2
        self.k_rudder = 0.8
        self.rudder_damping = 0.2
        self.weather_helm_factor = 0.005 

    def update_dynamics(self, sail_angle_deg, current_wheel_angle):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0: dt = 0.01 
        
        # --- Acceleration & Speed ---
        rel_wind = (self.true_wind_angle - self.heading) % 360
        if rel_wind > 180: rel_wind -= 360
        
        tw_rad = math.radians(rel_wind)
        aw_x = (self.true_wind_speed * math.cos(tw_rad)) - self.boat_speed
        aw_y = self.true_wind_speed * math.sin(tw_rad)
        
        aw_speed = math.sqrt(aw_x**2 + aw_y**2)
        aw_angle = math.degrees(math.atan2(aw_y, aw_x))
        
        angle_of_attack = aw_angle - sail_angle_deg
        
        efficiency = 0.0
        if 0 < angle_of_attack <= 15: efficiency = angle_of_attack / 15.0
        elif 15 < angle_of_attack <= 25: efficiency = 1.0 - ((angle_of_attack - 15.0) / 10.0)
        elif angle_of_attack > 25: efficiency = 0.1
            
        thrust = self.max_thrust * (aw_speed / self.true_wind_speed)**2 * efficiency
        drag = self.hull_drag_coeff * (self.boat_speed ** 2)
        
        self.boat_speed += ((thrust - drag) / self.boat_mass) * dt
        if self.boat_speed < 0: self.boat_speed = 0.0
            
        # --- Heading & Position Update ---
        turn_rate_rad = (self.boat_speed * math.sin(math.radians(current_wheel_angle))) / self.boat_length
        self.heading += math.degrees(turn_rate_rad) * dt
        self.heading %= 360.0 
        
        speed_ms = self.boat_speed * 0.514444
        heading_rad = math.radians(self.heading)
        self.pos_x += speed_ms * math.sin(heading_rad) * dt
        self.pos_y += speed_ms * math.cos(heading_rad) * dt
        
        # --- Base Forces ---
        wheel_velocity = (current_wheel_angle - self.last_wheel_angle) / dt
        rudder_rad = math.radians(current_wheel_angle)
        
        hydro_torque = self.k_rudder * (self.boat_speed ** 2) * math.sin(rudder_rad)
        damping_torque = -self.rudder_damping * wheel_velocity
        weather_helm_torque = thrust * self.weather_helm_factor
        
        base_wheel_torque = hydro_torque + damping_torque + weather_helm_torque
        aero_winch_pull = self.k_winch * (aw_speed ** 2) * math.sin(math.radians(max(0, angle_of_attack)))
        
        self.last_wheel_angle = current_wheel_angle
        self.last_time = current_time
        
        return aero_winch_pull, base_wheel_torque, wheel_velocity

# --- 3. HAPTIC ENGINES (Auto-Scaled) ---
class BreakawayWinch:
    def __init__(self, hardware_scale):
        # Configure the feel of your winch in actual Newton-meters (Nm)
        target_click_nm = 0.2
        target_breakaway_nm = 1.5
        target_slip_friction_nm = 0.3
        target_lock_stiffness_nm = 1.0 
        
        # Auto-scale up to raw physics space
        self.click_amplitude = target_click_nm / hardware_scale
        self.breakaway_torque = target_breakaway_nm / hardware_scale
        self.slip_friction = target_slip_friction_nm / hardware_scale
        self.lock_stiffness = target_lock_stiffness_nm / hardware_scale
        
        self.click_spacing = 15.0
        self.locked_angle = 0.0

    def calculate(self, current_angle, aero_force):
        angle_error = current_angle - self.locked_angle
        
        if angle_error > 0.5: 
            self.locked_angle = current_angle
            freq = (2 * math.pi) / self.click_spacing
            return aero_force - (self.click_amplitude * math.sin(freq * current_angle))
            
        elif angle_error < 0: 
            holding_torque = abs(angle_error) * self.lock_stiffness
            if holding_torque > self.breakaway_torque: 
                self.locked_angle = current_angle + (self.breakaway_torque / self.lock_stiffness)
                return aero_force + self.slip_friction
            else: 
                return aero_force + holding_torque
        return aero_force

class SteeringLimits:
    def __init__(self, hardware_scale):
        self.max_angle = 90.0 
        
        # Configure your virtual wall feel in actual Newton-meters (Nm)
        target_wall_stiffness_nm = 2.0  # 2 Nm of counter-force per degree past limit
        target_wall_damping_nm = 0.1    # 0.1 Nm of resistance per degree/sec of crash velocity
        
        # Auto-scale up to raw physics space
        self.k_p = target_wall_stiffness_nm / hardware_scale
        self.k_d = target_wall_damping_nm / hardware_scale
        
    def calculate(self, current_angle, velocity):
        if current_angle > self.max_angle:
            depth = current_angle - self.max_angle
            return -(self.k_p * depth) - (self.k_d * velocity)
        elif current_angle < -self.max_angle:
            depth = current_angle - (-self.max_angle)
            return -(self.k_p * depth) - (self.k_d * velocity)
        return 0.0

# --- 4. HARDWARE MOCKUP ---
class IntegratedMotorBus:
    def __init__(self):
        self.WHEEL_CMD_ID = 0x201
        self.WINCH_CMD_ID = 0x202
        self.mock_wheel = 0.0
        self.mock_winch = 0.0

    def update_incoming_messages(self):
        pass # Placeholder for CAN buffer sweep

    def get_wheel_position(self):
        return self.mock_wheel

    def get_winch_position(self):
        return self.mock_winch

    def set_torque(self, tx_id, torque):
        pass # Placeholder for sending CAN frame

# --- 5. MAIN EXECUTION LOOP ---
def run_simulator():
    # Initialize Core Systems
    motor_bus = IntegratedMotorBus()
    config = HardwareConfig()
    physics = DynamicBoatPhysics()
    
    # Initialize Haptics (Passing the hardware scales)
    winch_haptics = BreakawayWinch(config.winch_scale)
    wheel_limits = SteeringLimits(config.wheel_scale)
    
    # Initialize UDP Telemetry Broadcaster
    UDP_IP = "127.0.0.1" 
    UDP_PORT = 5005      
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    update_rate_hz = 100
    interval = 1.0 / update_rate_hz
    
    print("Simulation core online.")
    print(f"Hardware limits set to: Winch {config.winch_max_nm} Nm | Wheel {config.wheel_max_nm} Nm")
    print("Press Ctrl+C to stop simulation and engage E-Stop.")
    
    try:
        while True:
            start_time = time.time()
            
            # 1. READ SENSORS
            motor_bus.update_incoming_messages()
            wheel_ang = motor_bus.get_wheel_position()
            winch_ang = motor_bus.get_winch_position()
            
            # 2. CALCULATE PHYSICS
            aero_winch, base_wheel_tq, wheel_vel = physics.update_dynamics(winch_ang, wheel_ang)
            
            # 3. APPLY HAPTICS
            final_winch_tq_raw = winch_haptics.calculate(winch_ang, aero_winch)
            limit_tq = wheel_limits.calculate(wheel_ang, wheel_vel)
            final_wheel_tq_raw = base_wheel_tq + limit_tq
            
            # 4. MAP AND CLAMP FOR SAFETY
            safe_winch_tq = config.map_winch(final_winch_tq_raw)
            safe_wheel_tq = config.map_wheel(final_wheel_tq_raw)
            
            # 5. COMMAND MOTORS
            motor_bus.set_torque(motor_bus.WHEEL_CMD_ID, safe_wheel_tq)
            motor_bus.set_torque(motor_bus.WINCH_CMD_ID, safe_winch_tq)
            
            # 6. BROADCAST TELEMETRY
            telemetry = (
                f"X:{physics.pos_x:07.1f} | "
                f"Y:{physics.pos_y:07.1f} | "
                f"HDG:{physics.heading:05.1f} | "
                f"SPD:{physics.boat_speed:04.1f} | "
                f"WHEEL_ANG:{wheel_ang:+06.1f} | "
                f"WINCH_ANG:{winch_ang:+06.1f} || "
                f"WHEEL_TQ:{safe_wheel_tq:+06.2f} | "
                f"WINCH_TQ:{safe_winch_tq:+06.2f}"
            )
            print(telemetry, end='\r')
            
            sock.sendto(telemetry.encode('utf-8'), (UDP_IP, UDP_PORT))
            
            # 7. MAINTAIN TIMING
            elapsed = time.time() - start_time
            time.sleep(max(0, interval - elapsed))
            
    except KeyboardInterrupt:
        # Software E-Stop triggers on exit
        print("\n\nE-Stop Engaged. Safely zeroing motor torques...")
        motor_bus.set_torque(motor_bus.WHEEL_CMD_ID, 0.0)
        motor_bus.set_torque(motor_bus.WINCH_CMD_ID, 0.0)
        sock.close()

if __name__ == '__main__':
    run_simulator()