import math
import sys
import os
import time
import threading

import arcade
import numpy as np
import can

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper

WIDTH = 1200
HEIGHT = 800
SCREEN_TITLE = "Dynamic Haptic Sailing Simulation"

X = 0
Y = 1
STOP = 0

# --- THREAD-SAFE STATE BRIDGE ---
# This dictionary shares the physics data with the arcade graphics thread
sim_state = {
    'heading': 0.0,
    'pos_x': 0.0,
    'pos_y': 0.0,
    'wind_angle_deg': 0.0,
    'boat_speed_kts': 0.0
}

sail_ang = 0
rudder_ang = 0

# --- 1. NEW PHYSICS & HAPTIC ENGINES ---

class HardwareConfig:
    def __init__(self):
        self.winch_max_physics = 400.0
        self.wheel_max_physics = 100.0
        
        # Physical Hardware Limits (Nm)
        self.winch_max_nm = 3.0
        self.wheel_max_nm = 7.0
        
        self.winch_scale = self.winch_max_nm / self.winch_max_physics
        self.wheel_scale = self.wheel_max_nm / self.wheel_max_physics

    def map_winch(self, raw_torque):
        scaled = raw_torque * self.winch_scale
        return max(-self.winch_max_nm, min(scaled, self.winch_max_nm))

    def map_wheel(self, raw_torque):
        scaled = raw_torque * self.wheel_scale
        return max(-self.wheel_max_nm, min(scaled, self.wheel_max_nm))

class DynamicBoatPhysics:
    def __init__(self):
        self.true_wind_speed = 5.0
        self.true_wind_angle = 90.0
        
        self.boat_speed = 0.0       
        self.heading = 0.0          
        self.last_time = time.time()
        self.last_wheel_angle = 0.0
        self.pos_x = 0.0    
        self.pos_y = 0.0    
        
        self.boat_mass = 150.0
        self.hull_drag_coeff = 25.0 
        self.max_thrust = 4000.0
        self.boat_length = 10.0     
        
        self.k_winch = 1.2
        self.k_rudder = 2.5
        self.rudder_damping = 0.5
        self.weather_helm_factor = 0.02

    def update_dynamics(self, sail_angle_deg, current_wheel_angle):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0: dt = 0.01 
        
        # Acceleration & Speed
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
            
        # Heading & Position Update
        turn_rate_rad = (self.boat_speed * math.sin(math.radians(current_wheel_angle))) / self.boat_length
        self.heading += math.degrees(turn_rate_rad) * dt
        self.heading %= 360.0 
        
        speed_ms = self.boat_speed * 0.514444
        heading_rad = math.radians(self.heading)
        self.pos_x += speed_ms * math.sin(heading_rad) * dt
        self.pos_y += speed_ms * math.cos(heading_rad) * dt
        
        # Base Forces
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

class BreakawayWinch:
    def __init__(self, hardware_scale):
        target_click_nm = 0.2
        target_breakaway_nm = 1.5
        target_slip_friction_nm = 0.3
        target_lock_stiffness_nm = 1.0 
        
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
        target_wall_stiffness_nm = 2.0  
        target_wall_damping_nm = 0.1    
        
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

# --- 2. ARCADE GRAPHICS ---

def rotate(matrix, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return matrix @ R.T

class VectorField:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.spacingR = HEIGHT // self.rows
        self.spacingC = WIDTH // self.cols
        self.rows += 1
        self.cols += 1
        self.translations = np.zeros((self.rows * self.cols, 2), dtype=float)
        self.vectors = np.zeros((self.rows * self.cols, 2), dtype=float)

        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col
                self.translations[i, X] = 0.5 * self.spacingC + col * self.spacingC
                self.translations[i, Y] = 0.5 * self.spacingR + row * self.spacingR

        self.initialTranslations = self.translations.copy()

        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col
                r = 30
                theta = i * math.pi / 100
                self.vectors[i, X] = r * math.cos(theta)
                self.vectors[i, Y] = r * math.sin(theta)

    def point_to(self, theta_rad):
        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col
                r = 30
                self.vectors[i, X] = r * math.cos(theta_rad)
                self.vectors[i, Y] = r * math.sin(theta_rad)

    def update(self, offsetX, offsetY):
        offsetX = offsetX % self.spacingC
        offsetY = offsetY % self.spacingR
        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col
                self.translations[i, X] = self.initialTranslations[i, X] - offsetX
                self.translations[i, Y] = self.initialTranslations[i, Y] - offsetY

                if self.translations[i, X] < -self.spacingC:
                    self.translations[i, X] += WIDTH
                elif self.translations[i, X] > WIDTH + self.spacingC:
                    self.translations[i, X] -= WIDTH

                if self.translations[i, Y] < -self.spacingR / 2:
                    self.translations[i, Y] += HEIGHT
                elif self.translations[i, Y] > HEIGHT + self.spacingR / 2:
                    self.translations[i, Y] -= HEIGHT

class Canvas(arcade.Window):
    def __init__(self):
        super().__init__(WIDTH, HEIGHT, SCREEN_TITLE)

        arcade.set_background_color((0, 119, 190, 0))
        self.currentField = VectorField(8, 8)

        self.boat_sprite = arcade.Sprite('assets/boat.png', scale=0.6)
        self.compass_sprite = arcade.Sprite('assets/compass4.png', scale=0.15)
        self.rudder_sprite = arcade.Sprite('assets/rudder__.png', scale=0.2)

        self.compass_sprite.center_x = 70
        self.compass_sprite.center_y = HEIGHT - 70

        self.rudder_sprite.center_x = WIDTH - 120
        self.rudder_sprite.center_y = 120

        self.sprites = arcade.SpriteList()
        self.sprites.append(self.boat_sprite)
        self.sprites.append(self.compass_sprite)
        self.sprites.append(self.rudder_sprite)

        self.boat_path = []
        self.t = 0

        arcade.schedule(self.on_update, 1/60)

    def on_draw(self):
        self.clear()
        self.drawCurrentField()
        self.drawFrame()
        self.drawPath()
        self.sprites.draw()
        self.drawSail()
        self.drawRudderNeedle()
        
        # Overlay Speed HUD
        arcade.draw_text(f"Speed: {sim_state['boat_speed_kts']:.1f} kts", 10, HEIGHT - 30, arcade.color.WHITE, 16)
        arcade.draw_text(f"Heading: {sim_state['heading']:.0f}°", 10, HEIGHT - 55, arcade.color.WHITE, 16)

    def on_update(self, delta_time):
        offsetY = sim_state['pos_x']
        offsetX = sim_state['pos_y']

        self.boat_path.append([WIDTH / 2 - offsetY, HEIGHT / 2 - offsetX])
        self.currentField.update(offsetY, offsetX)
        self.updateBoat()

        theta_rad = math.radians(sim_state['wind_angle_deg'])
        self.currentField.point_to(theta_rad)
        self.t += 0.1

    @staticmethod
    def drawPolygon(points, tx, ty):
        n = points.shape[0]
        for i in range(n):
            arcade.draw_line(tx + points[i][0], ty + points[i][1], tx + points[(i + 1)  % n][0], ty + points[(i + 1) % n][1], color=arcade.color.BLACK)

    def updateBoat(self):
        self.boat_sprite.center_x = WIDTH / 2
        self.boat_sprite.center_y = HEIGHT / 2
        self.boat_sprite.angle = sim_state['heading']

    def drawCurrentField(self):
        for i in range(self.currentField.rows * self.currentField.cols):
            tx = self.currentField.translations[i, X]
            ty = self.currentField.translations[i, Y]
            x = self.currentField.vectors[i, X]
            y = self.currentField.vectors[i, Y]
            arcade.draw_line(tx, ty, tx + x, ty + y, arcade.color.BLACK)
            
            r = np.sqrt(np.square(x) + np.square(y))
            theta = np.atan2(y, x)
            tip = np.array([[r - 5, 5], [r, 0], [r - 5, -5]])
            rot_tip = rotate(tip, theta)
            
            arcade.draw_line(tx + rot_tip[0][0], ty + rot_tip[0][1], tx + rot_tip[1][0], ty + rot_tip[1][1], arcade.color.BLACK)
            arcade.draw_line(tx + rot_tip[1][0], ty + rot_tip[1][1], tx + rot_tip[2][0], ty + rot_tip[2][1], arcade.color.BLACK)


    def drawSail(self):
        arcade.draw_text("Sail angle: ", WIDTH -70, HEIGHT -20, arcade.color.BLACK, font_size=13, anchor_x="right", anchor_y="top")
        arcade.draw_text(f"{((sail_ang * 180 / math.pi) - 90) % 360:.2f}°", WIDTH -78, HEIGHT -40, arcade.color.BLACK, font_size=13, anchor_x="right", anchor_y="top")

        points = []
        start_angle = 30
        end_angle = 150

        width = 60
        height = 34
        center_x = WIDTH / 2
        center_y = HEIGHT / 2

        segments = 120

        start_rad = math.radians(start_angle)
        end_rad = math.radians(end_angle)
        tilt_rad = sail_ang

        for i in range(segments + 1):
            t = start_rad + (end_rad - start_rad) * i / segments

            x = (width / 2) * math.cos(t)
            y = (height / 2) * math.sin(t)

            rx = x * math.cos(tilt_rad) - y * math.sin(tilt_rad)
            ry = x * math.sin(tilt_rad) + y * math.cos(tilt_rad)

            points.append((center_x + rx, center_y + ry))

        arcade.draw_polygon_filled(points, arcade.color.WHITE)


    def drawFrame(self):
        arcade.draw_line(40, 40, 100, 40, arcade.color.WHITE, 2)
        arcade.draw_line(40, 40, 40, 100, arcade.color.WHITE, 2)

        arcade.draw_text("x", 40, 105, arcade.color.WHITE, font_size=16, anchor_x="center", anchor_y="bottom")
        arcade.draw_text("y", 110, 40, arcade.color.WHITE, font_size=16, anchor_x="left", anchor_y="center")


    def drawPath(self):
        dash_length = 10
        gap_length = 5
        width = 2
        draw = True
        remaining = dash_length

        for j in range(len(self.boat_path) - 1):
            i = len(self.boat_path) - 1 - j
            x1, y1 = self.boat_path[i]
            x2, y2 = self.boat_path[i - 1]

            dx = x2 - x1
            dy = y2 - y1
            segment_length = math.hypot(dx, dy)

            if segment_length == 0:
                continue

            dir_x = dx / segment_length
            dir_y = dy / segment_length

            dist = 0
            cx, cy = x1, y1

            while dist < segment_length:
                step = min(remaining, segment_length - dist)
                nx = cx + dir_x * step
                ny = cy + dir_y * step

                if draw:
                    arcade.draw_line(cx, cy, nx, ny, arcade.color.WHITE, width)

                cx, cy = nx, ny
                dist += step
                remaining -= step

                if remaining <= 0:
                    draw = not draw
                    remaining = dash_length if draw else gap_length

    def drawRudderNeedle(self):
        x = self.rudder_sprite.center_x
        y = self.rudder_sprite.center_y

        r = 40
        theta = rudder_ang * 2
        arcade.draw_line(x, y, x + r * np.sin(theta), y - r * np.cos(theta), arcade.color.WHITE, 2)


# --- 3. HARDWARE & PHYSICS LOOP ---

def end_motors(motors, notifier, can0):
    for motor in motors:
        motor.set_control_mode("torque", 0)
        motor.control()
    time.sleep(3)
    for motor in motors:
        motor.stop_all_tasks()
        motor.motor_off()
    notifier.stop()
    core.CANHelper.cleanup("can0")
    can0.shutdown()
    print("Exiting safely.")
    exit(0)

def run_motors():
    global STOP, sail_ang, rudder_ang

    # Initialize Hardware
    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    wheel = CanMotor(can0, motor_id=2, gear_ratio=6, name='wheel')
    winch = CanMotor(can0, motor_id=0, gear_ratio=6, name='winch')

    motors = [wheel, winch]
    motor_listener = MotorListener(motor_list=motors)
    notifier = can.Notifier(can0, [motor_listener])

    for motor in motors:
        motor.initialize_motor()
        motor.initialize_control_command()

    time.sleep(1)
    input("Press Enter to start physics loop...")

    # Pull initial offsets
    for i in range(3):
        for motor in motors:
            motor.read_status_once()
            time.sleep(0.02)
            motor.read_multiturn_once()
            time.sleep(0.02)
            motor.read_motor_state_once()
            time.sleep(0.02)

    wheel_offset = wheel.motor_data.multiturn_position
    winch_offset = winch.motor_data.multiturn_position
    
    # Initialize Physics and Haptics
    physics = DynamicBoatPhysics()
    config = HardwareConfig()
    winch_haptics = BreakawayWinch(config.winch_scale)
    wheel_limits = SteeringLimits(config.wheel_scale)

    try:
        while True:
            # Hardware Read (If your CAN library supports async buffer pulling, 
            # removing these sleep() calls will make haptics 10x smoother)
            wheel.read_status_once()
            time.sleep(0.01)
            wheel.read_multiturn_once()
            time.sleep(0.01)
            wheel.read_motor_state_once()
            time.sleep(0.01)

            winch.read_status_once()
            time.sleep(0.01)
            winch.read_multiturn_once()
            time.sleep(0.01)
            winch.read_motor_state_once()
            time.sleep(0.01)

            # Calculate precise hardware angles in degrees
            wheel_ang = (wheel.motor_data.multiturn_position - wheel_offset) / 10
            winch_ang = (winch.motor_data.multiturn_position - winch_offset) / 8

            rudder_ang = np.clip(wheel_ang, -0.69, 0.69)
            sail_ang = np.clip(winch_ang, -1.4, 1.4)
            
            # --- PHYSICS CALCULATION ---
            aero_winch, base_wheel_tq, wheel_vel = physics.update_dynamics(sail_ang * 180 / math.pi, rudder_ang * 180 / math.pi)

            # --- HAPTIC OVERLAYS ---
            final_winch_tq_raw = winch_haptics.calculate(winch_ang, aero_winch)
            limit_tq = wheel_limits.calculate(wheel_ang, wheel_vel)
            final_wheel_tq_raw = base_wheel_tq + limit_tq

            # --- MAP & CLAMP TO HARDWARE (Nm) ---
            safe_winch_tq = config.map_winch(final_winch_tq_raw)
            safe_wheel_tq = config.map_wheel(final_wheel_tq_raw)

            # --- WRITE TO UI STATE BRIDGE ---
            sim_state['heading'] = physics.heading
            sim_state['pos_x'] = physics.pos_x
            sim_state['pos_y'] = physics.pos_y
            sim_state['wind_angle_deg'] = physics.true_wind_angle
            sim_state['boat_speed_kts'] = physics.boat_speed

            # --- COMMAND MOTORS ---
            wheel.set_control_mode("torque", safe_wheel_tq)
            wheel.control()
            time.sleep(0.01)

            winch.set_control_mode("torque", 0)
            winch.control()
            time.sleep(0.01)

            if STOP:
                end_motors(motors, notifier, can0)

    except KeyboardInterrupt:
        end_motors(motors, notifier, can0)

if __name__ == "__main__":
    run_motors_thread = threading.Thread(target=run_motors, daemon=True)
    run_motors_thread.start()

    window = Canvas()

    try:
        arcade.run()
    except KeyboardInterrupt:
        STOP = 1