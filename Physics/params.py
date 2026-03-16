class Params:

    # Physical and simulation paramters

    def __init__(self):
        # Time Parameters
        self.dt = 0.04 # seconds
        self.t_start = 0.0
        self.t_end = 400.0
        self.sub_steps=1
        
        # Catalina 27 Parameters
        self.mass = 3100*2 # total mass kg
        self.beam =2.69 # boat width m
        self.lwl = 6.63 # boat length at waterline
        self.Ix = self.mass*(0.35*self.beam)**2
        self.Iy = self.mass*(0.25*self.lwl)**2 
        self.Iz = self.mass*(0.27*self.lwl)**2 
        self.x_surge = 1.1
        self.y_sway = 1.8
        self.z_heave = 1.5
        self.z_yaw_factor = 1.5

        self.rudder_angle_limit = 0.7 # radians = 40 degrees
        self.steering_ratio = 10.0 # wheel to rudder gear ratio

        self.winch_ratio = 8.0 # winch handle to winch gear ratio
        self.winch_radius = 0.028575 # for size 10 winch, radius = 1 1/8 inch

        self.hull_Ax = 2.5 # front wetted surface area m2
        self.hull_Ay = 7.0 # lateral wetted surface area m2
        self.hull_Cd_x = 0.4 
        self.hull_Cd_y = 1.8 
        self.yaw_damping = self.mass/3
        
        self.rudder_arm = -2.5 # m is negative because rudder is aft of CLR
        self.rudder_width = 0.25 # m
        self.rudder_height = 0.9 # m
        self.rudder_CLmax = 1.2 
        self.rudder_Ch_alpha = -.03 

        self.sailA = 31.6 # m^2 total sail area
        self.sailArm = -1.7 # m  between CE and CLR
        self.sailAngleMax = 1.4 # rad max sail angle
        self.sailCLmax = 1.2 # max coefficient of sail lift
        self.sailCDmax = 0.15 # max coefficient of sail drag
        self.sailCDmin = 0.05 # min coefficient of sail drag

        # Environment Parameters
        self.rho_air = 1.225 # kg/m^3 air density
        self.rho_h20 = 1030 # kg/m^3 water density

        self.motorsail_velocity = 0# m/s constant minimum forward x velocity  4 knots not used

        self.damping = self.mass


         