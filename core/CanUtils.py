import math
import numpy as np

command_dict = {
    "0x30": "read_pid",
    "0x31": "write_pid_to_ram",
    "0x32": "write_pid_to_rom",
    "0x33": "read_acceleration",
    "0x34": "write_acceleration",
    "0x90": "read_encoder",
    "0x91": "write_encoder_offset",
    "0x19": "write_zero_pos",
    "0x92": "read_multiturn_pos",
    "0x94": "read_singleturn_pos",
    "0x80": "motor_off",
    "0x81": "motor_stop",
    "0x88": "motor_start",
    "0x9a": "read_temp_voltage_error",
    "0x9b": "clear_error_flag",
    "0x9c": "read_temp_torque_speed_encoder",
    "0x9d": "read_phase_currents",
    "0xa1": "torque_ctrl",
    "0xa2": "speed_ctrl",
    "0xa3": "multiturn_pos_ctrl",
    "0xa4": "multiturn_speed_limit_pos_ctrl",
    "0xa5": "singleturn_pos_ctrl",
    "0xa6": "singleturn_speed_limit_pos_ctrl",
}


class CanUtils:
    def readBytes(self, high_byte, low_byte):
        """Converts values from 2-byte hexadecimal to decimal value
        -
        """
        decimal_val = np.int16(np.uint16((high_byte << 8) | low_byte))
        return decimal_val 

    def readBytesList(self, byte_list):
        """Reads an array of bytes and returns an integer value
        -
        """        
        out = np.uint64(0)
        for byte in byte_list:
            # HACK: Numpy decided it does not want to bitshift when it is "unsafe" (aka overflow)
            # So using product and addition equivalent stuff instead
            # out = np.uint64((out << 8) | byte)
            out = (out * 2**8) + np.uint64(byte)

        # manually handling 2s compliment... This sucks  
        if out > 2**(8*len(byte_list) - 1) - 1:
            out = -np.int64(2**(8*len(byte_list)) - out) - 1
        else:
            out = np.int64(out)
        return out

    def int_to_bytes(self, value, length):
        """Converts integer value to x-byte hexadecimal
        -
        Args:
            value (int): Input value
            length (int): Number of bytes you want to convert the input value to 

        Returns:
            Array of bytes
            Ex: 
                byte1, byte2, byte3, byte4
                Where byte1 = High byte 
                and   byte4 = Low byte

        """        
        result = []
        for i in range(0, length):
            result.append(np.uint8(value >> (i * 8) & 0xff))
        result.reverse()
        return result

    def toDegrees(self, enc_position, enc_value_range):
        """Convert 14-bit encoder reading to degrees

        Args:
            enc_position (_type_): _description_

        Returns:
            _type_: _description_
        """
        return (enc_position*360/enc_value_range)

    def degToRad(self, in_deg):
        """Converts degrees to radians
        -
        """
        return math.pi * in_deg / 180

    def radToDeg(self, in_rad):
        """Converts radians to degrees
        -
        """        
        return 180 * in_rad / math.pi

    def encToAmp(self, in_enc):
        """Convert encoder bits to amps
        -
        """        
        return in_enc * 33 / 2048

    def ampToEnc(self, in_amp):
        """ Converts amps to encoder bits
        -
        """
        return in_amp * 2000 / 32
