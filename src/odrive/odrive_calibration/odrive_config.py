"""
These configurations is based on Austin Owens implementation:
https://github.com/AustinOwens/robodog/blob/main/odrive/configs/odrive_hoverboard_config.py

That was based on the hoverboard tutorial found on the Odrive Robotics website.

@author: Jorge Lamperez
@date: 04/02/2022
"""

import sys
import time
import odrive
from odrive.enums import *

class RBEMotorConfig:
    """
    Class for configuring an Odrive axis for a RBE-202024-003 motor.
    Only works with one Odrive at a time.
    """

    # Min/Max phase inductance of motor
    MIN_PHASE_INDUCTANCE = 0
    MAX_PHASE_INDUCTANCE = 0.002

    # Min/Max phase resistance of motor
    MIN_PHASE_RESISTANCE = 0
    MAX_PHASE_RESISTANCE = 1

    # Tolerance for encoder offset float
    ENCODER_OFFSET_FLOAT_TOLERANCE = 0.5

    def __init__(self, axis_num: int):
        """
        Initalizes RBEotorConfig class by finding odrive and grabbing specified
        axis object.

        :param axis_num: Which channel/motor on the odrive your referring to.
        :type axis_num: int (0 or 1)
        """

        self.axis_num = axis_num

        # Connect to Odrive
        print("Looking for ODrive...")
        self._find_odrive()
        print("Found ODrive.")

    def _find_odrive(self):
        # connect to Odrive
        self.odrv = odrive.find_any()
        self.odrv_axis = getattr(self.odrv, "axis{}".format(self.axis_num))

    def erase_config(self):
        """ Erase pre-exsisting configuration """

        print("Erasing pre-exsisting configuration...")
        try:
            self.odrv.erase_configuration()
        except:
            pass

    def set_odrive_parameters(self):
        self._find_odrive()
        """Saves odrive axis, motor, encoder and controller parameters"""
        self.odrv_axis.motor.config.pole_pairs = 15
        self.odrv_axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

        self.odrv_axis.motor.config.resistance_calib_max_voltage = 10
        self.odrv_axis.motor.config.requested_current_range      = 25
        self.odrv_axis.motor.config.current_control_bandwidth    = 100

        self.odrv_axis.motor.config.torque_constant = 24*8.27/500

        # We are going to use RBE-102024-003 Optical encoder AEDR-8300.
        self.odrv_axis.encoder.config.mode=ENCODER_MODE_INCREMENTAL
        self.odrv_axis.encoder.config.cpr=16384

        # Tuned values
        self.odrv_axis.controller.config.pos_gain = 3
        self.odrv_axis.controller.config.vel_gain = 1.3
        self.odrv_axis.controller.config.vel_integrator_gain = 15
        self.odrv_axis.controller.config.vel_limit = 10

        # Set in position control mode so we can control the position of the
        # wheel
        self.odrv_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        # In the next step we are going to start powering the motor and so we
        # want to make sure that some of the above settings that require a
        # reboot are applied first.
        print("Saving manual configuration and rebooting...")
        self.odrv.save_configuration()
        print("Manual configuration saved.")
        try:
            self.odrv.reboot()
        except:
            pass

    def motor_calibration(self):
        self._find_odrive()
        print("Calibrating Odrive for RBE-202024-003 motor (you should hear a "
        "beep)...")

        self.odrv_axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION

        # Wait for calibration to take place
        time.sleep(10)

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
        self.odrv_axis.motor.config.pre_calibrated = True

        print("Saving motor calibration configuration and rebooting...")
        self.odrv.save_configuration()
        print("Motor calibration configuration saved.")
        try:
            self.odrv.reboot()
        except:
            pass

    def encoder_calibration(self):
        self._find_odrive()
        # Check the alignment between the motor and the encoder sensor.
        print("Calibrating Odrive for encoder...")
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
        if not ((self.odrv_axis.encoder.config.offset_float > 0.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.odrv_axis.encoder.config.offset_float < 0.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE) or \
        (self.odrv_axis.encoder.config.offset_float > 1.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.odrv_axis.encoder.config.offset_float < 1.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE)):
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
        self.odrv_axis.encoder.config.pre_calibrated = True

    def configure(self):
        """
        Configures the odrive device for RBE-1020-24-003 motor.
        """
        self.set_odrive_parameters()
        # input("Make sure the motor is free to move, then press enter...")
        self.motor_calibration()
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

if __name__ == "__main__":
    rbe_motor_axis0_config = RBEMotorConfig(axis_num = 0)
    rbe_motor_axi1_config = RBEMotorConfig(axis_num = 1)
    rbe_motor_axis0_config.erase_config()

    rbe_motor_axis0_config.configure()
    rbe_motor_axi1_config.configure()

    rbe_motor_axis0_config.encoder_calibration()
    rbe_motor_axi1_config.encoder_calibration()

    print("CONDUCTING MOTOR TEST")
    print("Placing motors in close loop control. If you move motor, motor will "
          "resist you.")
    rbe_motor_axis0_config.mode_close_loop_control()
    rbe_motor_axi1_config.mode_close_loop_control()

    # Go from 0 to 360 degrees in increments of 30 degrees
    for angle in range(0, 390, 30):
        print("Setting motor to {} degrees.".format(angle))
        rbe_motor_axis0_config.move_input_pos(angle)
        rbe_motor_axi1_config.move_input_pos(-angle)
        time.sleep(2)

    print("Placing motors in idle. If you move motor, motor will "
          "move freely")
    rbe_motor_axis0_config.mode_idle()
    rbe_motor_axi1_config.mode_idle()