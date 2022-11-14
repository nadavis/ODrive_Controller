"""
These configurations is based on Austin Owens implementation:
https://github.com/AustinOwens/robodog/blob/main/odrive/configs/odrive_hoverboard_config.py
That was based on the hoverboard tutorial found on the Odrive Robotics website.
AND
These configurations is based on Jorge Lamperez implementation:
https://github.com/jlamperez/karp/blob/main/karp_odrive/scripts/odrive_config.py
@author: Nadav Israel
@date: 1/11/2022
"""

import sys
import time
import odrive
from odrive.enums import *

class MotorConfig:

    MOTOR_KV = 500/24

    # Min/Max phase inductance of motor
    MIN_PHASE_INDUCTANCE = 0
    MAX_PHASE_INDUCTANCE = 0.002

    # Min/Max phase resistance of motor
    MIN_PHASE_RESISTANCE = 0
    MAX_PHASE_RESISTANCE = 1

    # Tolerance for encoder offset float
    ENCODER_OFFSET_FLOAT_TOLERANCE = 0.5

    def __init__(self, odrv, axis_num: int):
        self.axis_num = axis_num
        self.odrv = odrv
        self._find_odrive()
        # Connect to Odrive
    def _find_odrive(self):
        # connect to Odrive
        print("Looking for ODrive axis...")
        self.odrv_axis = getattr(self.odrv, "axis{}".format(self.axis_num))
        time.sleep(2)
        print("Found ODrive axis.")
    def set_odrive_parameters(self):
        print("Set odrive configuration parameters.")
        """Saves odrive axis, motor, encoder and controller parameters"""
        self.odrv_axis.motor.config.current_lim = 15
        self.odrv_axis.motor.config.pole_pairs = 15
        self.odrv_axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        self.odrv_axis.motor.config.resistance_calib_max_voltage = 10
        self.odrv_axis.motor.config.requested_current_range      = 25
        self.odrv_axis.motor.config.current_control_bandwidth    = 1000 #100
        self.odrv_axis.motor.config.torque_constant = 8.27/self.MOTOR_KV

        self.odrv_axis.encoder.config.mode = ENCODER_MODE_INCREMENTAL
        self.odrv_axis.encoder.config.cpr = 16384
        self.odrv_axis.encoder.config.calib_scan_distance = 150

        # Tuned values
        # self.odrv_axis.controller.config.pos_gain = 3
        # self.odrv_axis.controller.config.vel_gain = 1.3
        # self.odrv_axis.controller.config.vel_integrator_gain = 15
        # self.odrv_axis.controller.config.pos_gain = 1
        # self.odrv_axis.controller.config.vel_gain = 0.02 * self.odrv_axis.motor.config.torque_constant * self.odrv_axis.encoder.config.cpr
        # self.odrv_axis.controller.config.vel_integrator_gain = 0.1 * self.odrv_axis.motor.config.torque_constant * self.odrv_axis.encoder.config.cpr
        # self.odrv_axis.controller.config.vel_limit = 10
        self.odrv_axis.controller.config.vel_limit = 10

        # Set in position control mode so we can control the position of the
        # wheel
        self.odrv_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    def motor_calibration(self):
        print("Calibrating Odrive motor")
        self.odrv_axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION

        # Wait for calibration to take place
        time.sleep(5)

        if self.odrv_axis.motor.error != 0:
            print("Error: Odrive reported an error of {} while in the state "
            "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
            "debug:\n{}".format(self.odrv_axis.motor.error,
                                self.odrv_axis.motor))

            sys.exit(1)

        if self.odrv_axis.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE or \
        self.odrv_axis.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE:
            print("Error: After odrive motor calibration, the phase inductance "
            "is at {}, which is outside of the expected range. Either widen the "
            "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
            "is between {} and {} respectively) or debug/fix your setup. Printing "
            "out Odrive motor data for debug:\n{}".format(self.odrv_axis.motor.config.phase_inductance,
                                                          self.MIN_PHASE_INDUCTANCE,
                                                          self.MAX_PHASE_INDUCTANCE,
                                                          self.odrv_axis.motor))

            sys.exit(1)

        if self.odrv_axis.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE or \
        self.odrv_axis.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE:
            print("Error: After odrive motor calibration, the phase resistance "
            "is at {}, which is outside of the expected range. Either raise the "
            "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
            "debug/fix your setup. Printing out Odrive motor data for "
            "debug:\n{}".format(self.odrv_axis.motor.config.phase_resistance,
                                self.MIN_PHASE_RESISTANCE,
                                self.MAX_PHASE_RESISTANCE,
                                self.odrv_axis.motor))

            sys.exit(1)

        # If all looks good, then lets tell ODrive that saving this calibration
        # to persistent memory is OK
        # self.odrv_axis.motor.config.pre_calibrated = True



    def encoder_calibration(self):
        # Check the alignment between the motor and the encoder sensor.
        print("Calibrating Odrive encoder...")
        self.odrv_axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        # Wait for calibration to take place
        time.sleep(30)

        if self.odrv_axis.encoder.error != 0:
            print("Error: Odrive reported an error of {} while in the state "
            "AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
            "data for debug:\n{}".format(self.odrv_axis.encoder.error,
                                         self.odrv_axis.encoder))

            sys.exit(1)

        # If offset_float isn't 0.5 within some tolerance, or its not 1.5 within
        # some tolerance, raise an error
        if not ((abs(self.odrv_axis.encoder.config.offset_float) > 0.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        abs(self.odrv_axis.encoder.config.offset_float) < 0.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE) or \
        (abs(self.odrv_axis.encoder.config.offset_float) > 1.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        abs(self.odrv_axis.encoder.config.offset_float) < 1.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE)):
            print("Error: After odrive encoder calibration, the 'offset_float' "
            "is at {}, which is outside of the expected range. 'offset_float' "
            "should be close to 0.5 or 1.5 with a tolerance of {}. Either "
            "increase the tolerance or debug/fix your setup. Printing out "
            "Odrive encoder data for debug:\n{}".format(self.odrv_axis.encoder.config.offset_float,
                                                        self.ENCODER_OFFSET_FLOAT_TOLERANCE,
                                                        self.odrv_axis.encoder))

            sys.exit(1)

        # If all looks good, then lets tell ODrive that saving this calibration
        # to persistent memory is OK
        # self.odrv_axis.encoder.config.pre_calibrated = True
    def set_pre_calibrated(self):

        self.odrv_axis.encoder.config.use_index = True
        self.odrv_axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        self.odrv_axis.encoder.config.pre_calibrated = True
        self.odrv_axis.config.startup_encoder_index_search = True
        # self.odrv_axis.config.startup_encoder_offset_calibration = True
        self.odrv_axis.config.startup_closed_loop_control = True
        self.odrv_axis.motor.config.pre_calibrated = True

    def configure(self):
        self.set_odrive_parameters()
        # input("Make sure the motor is free to move, then press enter...")
        self.motor_calibration()
        self.encoder_calibration()
        print("Odrive configuration finished.")

    def full_calibration(self):
        """
        Make full calibration (MOTOR + ENCODER) of the motor.
        """

        print("Full calibration ")
        self.odrv_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        # Wait for calibration to take place
        time.sleep(40)

    def mode_idle(self):
        """
        Puts the motor in idle (i.e. can move freely).
        """

        self.odrv_axis.requested_state = AXIS_STATE_IDLE

    def mode_close_loop_control(self):
        """
        Puts the motor in closed loop control.
        """
        self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def move_input_pos(self, angle):
        """
        Puts the motor at a certain angle.

        :param angle: Angle you want the motor to move.
        :type angle: int or float
        """

        self.odrv_axis.controller.input_pos = angle/360.0
class ODriveConfig:
    def __init__(self):
        # Connect to Odrive
        self._find_odrive()

    def _find_odrive(self):
        # connect to Odrive
        print("Looking for ODrive...")
        self.odrv = odrive.find_any()
        self.motor_axis0_config = MotorConfig(self.odrv, axis_num=0)
        self.motor_axis1_config = MotorConfig(self.odrv, axis_num=1)
        time.sleep(3)
        print("Found ODrive.")

    def reboot(self):
        print("Reboot odrive.")
        try:
            self.odrv.reboot()
        except:
            pass

        time.sleep(3)
        self._find_odrive()

    def save_config(self):
        print("Saving manual configuration...")
        try:
            self.odrv.save_configuration()
        except:
            pass
        time.sleep(3)
        print("Manual configuration saved.")

    def erase_config(self):
        print("Erasing configuration.")
        try:
            self.odrv.erase_configuration()
        except:
            pass

        time.sleep(3)
        print("Erased configuration.")
    def configure(self):
        print("Starting Odrive configuration.")
        self.motor_axis0_config.configure()
        self.motor_axis1_config.configure()
        print("Odrive configuration finished.")
        # self.save_config()
        # self.reboot()
        # self._find_odrive()
    def set_odrive_close_loop(self):
        self.motor_axis0_config.mode_close_loop_control()
        self.motor_axis1_config.mode_close_loop_control()
    def odrive_test1(self):
        # Go from 0 to 360 degrees in increments of 30 degrees
        for angle in range(0, 360, 60):
            print("Setting motor to {} degrees.".format(angle))
            self.motor_axis0_config.move_input_pos(angle)
            time.sleep(2)
            self.motor_axis1_config.move_input_pos(angle)
            time.sleep(2)

    def odrive_test2(self):
        print("Test 2 - spinning once")
        self.motor_axis0_config.move_input_pos(360)
        time.sleep(2)
        self.motor_axis0_config.move_input_pos(0)
        time.sleep(2)
        self.motor_axis1_config.move_input_pos(360)
        time.sleep(2)
        self.motor_axis1_config.move_input_pos(0)
        time.sleep(2)

    def set_pre_calibrated(self):
        print("Starting set pre calibration.")
        self.motor_axis0_config.set_pre_calibrated()
        self.motor_axis1_config.set_pre_calibrated()
        print("Pre calibration. finished.")
        self.save_config()
        # self.reboot()
        self._find_odrive()

if __name__ == "__main__":
    odrv = ODriveConfig()
    # odrv.configure()
    # odrv.set_pre_calibrated()
    # odrv.set_odrive_close_loop()
    # # odrv.odrive_test1()
    # odrv.reboot()
    #
    # time.sleep(5)
    odrv.odrive_test2()

    # print("Placing motors in idle. If you move motor, motor will "
    #       "move freely")
    # motor_axis0_config.mode_idle()
    # motor_axis1_config.mode_idle()