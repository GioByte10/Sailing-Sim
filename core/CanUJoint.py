from .CanMotor import CanMotor
import math

class CanUJoint(CanMotor):
    def __init__(self, bus, motor_id, GEAR_RATIO = 6, MIN_POS = -999 * 2 * math.pi, MAX_POS = 999 * 2 * math.pi):
        super(CanUJoint, self).__init__(bus, motor_id, GEAR_RATIO, MIN_POS = MIN_POS, MAX_POS = MAX_POS)
        self.multiturn_zero_pos = 0
        self.gear_ratio = GEAR_RATIO

    # Incoproate new boot-up procedure which will be ran when straight and gives
    # offset to multi-turn read position according to the single-turn 0
    # This way the snake only needs to be "roughly" straight to set it's 0
    def zero_motor(self):
        curPos = self.read_raw_position()
        curPos = self.read_raw_position() # For some reason 2 is necessary, or else it will only return 0 
        print(f"current Position {curPos}")
        self.setmultiTurnZeroOffset(curPos)

        '''
        if curPos < 0:
            self.multiTurnZeroOffset(abs(curPos))
        else: 
            self.multiTurnZeroOffset(abs(curPos), InvertDirection = 1)
        '''

        '''
        singleturn_zero_pos = super(CanUJoint, self).read_position()
        self.multiturn_zero_pos = super(CanUJoint, self).read_multiturn_position()# - singleturn_zero_pos
        return self.multiturn_zero_pos
        '''

    '''
    # Add position control that uses multi-turn set-point and needs to consider direction
    # We need to handle the direction by sending 2 commands since the multi-turn 
    # command does not let us set the direction
    # Incorporate speed/torque/acceleration limits
    def pos_ctrl(self, to_rad):
        to_rad -= self.multiturn_zero_pos # set # of radians to move based on zero pos
        
        # The least significant bit represents 0.01 degrees per second.
        to_deg = 100 * self.utils.radToDeg(to_rad) * self.gear_ratio
        byte1, byte2, byte3, byte4 = self.utils.toBytes(to_deg)

        # this does a multi-turn position control by moving to_rad # of radians
        # counterclockwise for +to_rad and clockwise for -to_rad
        super(CanUJoint, self).send([0xa4, 0x00, 0x00, 0x00, byte4, byte3, byte2, byte1])
    '''