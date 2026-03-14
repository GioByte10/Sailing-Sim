class Params:

    # Physical and simulation paramters

    def __init__(self):
        # Time Parameters
        self.dt = 0.1 # seconds
        self.t_start = 0.0
        self.t_end = 10.0
        

        # Catalina 27 Parameters
        self.mass = 3100 # total mass kg
        self.beam =2.69 # boat width m
        self.lwl = 6.63 # boat length at waterline
        self.Ix = self.mass*(0.35*self.beam)**2
        self.Iy = self.mass*(0.25*self.lwl)**2 
        self.Iz = self.mass*(0.27*self.lwl)**2 

        self.steering_ratio = 10.0 # wheel to rudder gear ratio
        self.winch_ratio = 8.0 # winch handle to winch gear ratio
        self.winch_radius = 0.028575 # for size 10 winch, radius = 1 1/8 inch



        self.hull_surface_area = 0 #TODO
        self.hull_coeff_drag = 0 #TODO
        self.keel_surface_area = 0 #TODO
        
        self.rudder_arm = 4
        self.rudder_length = 0.45 # m
        self.rudder_width = 0.05 # m
        self.rudder_height = 0.9 # m
        self.rudder_Ch_alpha = -.03 

        self.sailA = 31.6 # m^2 total sail area
        self.sailArm = -1.7 # m  between CE and CLR
        self.sailCLmax = 1.2
        self.sailCDmax = 0.15
        self.sailCDmin = 0.05


        # Environment Parameters
        self.rho_air = 1.225 # kg/m^3 air density
        self.rho_h20 = 1030 # kg/m^3 water density


         