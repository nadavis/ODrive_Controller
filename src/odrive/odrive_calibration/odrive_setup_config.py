
import odrive
from odrive.enums import *
import time
import math

print("finding an odrive...")
my_drive = odrive.find_any()
print("erase odrive configuration...")
try:
    my_drive.erase_configuration()
except:
    pass
time.sleep(2)
print("finding an odrive...")
my_drive = odrive.find_any()
print("setting odrive motor0...")
# my_drive.axis0.motor.config.pre_calibrated = False
my_drive.axis0.controller.config.vel_limit = 5
my_drive.axis0.motor.config.current_lim = 10
my_drive.axis0.motor.config.pole_pairs = 15
my_drive.axis0.motor.config.torque_constant = 24*8.27/500
my_drive.axis0.motor.config.current_control_bandwidth = 1000
my_drive.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
# my_drive.axis0.motor.config.direction = 1
my_drive.axis0.encoder.config.cpr = 16384
# my_drive.axis0.encoder.config.bandwidth = 100
# my_drive.axis0.controller.config.pos_gain = 1
my_drive.axis0.encoder.config.calib_scan_distance = 150

# my_drive.axis0.controller.config.vel_gain = 0.02 * my_drive.axis0.motor.config.torque_constant * my_drive.axis0.encoder.config.cpr
# my_drive.axis0.controller.config.vel_integrator_gain = 0.1 * my_drive.axis0.motor.config.torque_constant * my_drive.axis0.encoder.config.cpr
# my_drive.axis0.controller.config.vel_limit = 36
# my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
print("saving an odrive motor0 config...")
my_drive.save_configuration()
time.sleep(1)

# Calibrate motor and wait for it to finish
print("starting calibration motor0...")
my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while my_drive.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
print('Error motor 0: ', my_drive.axis0.motor.error)
print('Error encoder 0: ', my_drive.axis0.encoder.error)

time.sleep(2)
# my_drive.axis0.config.startup_closed_loop_control = True
# my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
# my_drive.axis0.controller.input_pos = 2
# my_drive.axis0.controller.input_vel = 2
# my_drive.axis0.motor.config.pre_calibrated = False
time.sleep(3)

print("setting odrive motor1...")
# my_drive.axis1.motor.config.pre_calibrated = False
my_drive.axis1.controller.config.vel_limit = 5
my_drive.axis1.motor.config.current_lim = 10
my_drive.axis1.motor.config.pole_pairs = 15
my_drive.axis1.motor.config.torque_constant = 24*8.27/500
my_drive.axis1.motor.config.current_control_bandwidth = 1000
my_drive.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
# my_drive.axis1.motor.config.direction = -1
my_drive.axis1.encoder.config.cpr = 16384
my_drive.axis1.encoder.config.calib_scan_distance = 150

# my_drive.axis1.encoder.config.bandwidth = 100
# my_drive.axis1.motor.config.direction = -1
# my_drive.axis1.controller.config.pos_gain = 1
# my_drive.axis1.controller.config.vel_gain = 0.02 * my_drive.axis1.motor.config.torque_constant * my_drive.axis1.encoder.config.cpr
# my_drive.axis1.controller.config.vel_integrator_gain = 0.1 * my_drive.axis1.motor.config.torque_constant * my_drive.axis1.encoder.config.cpr
# my_drive.axis1.controller.config.vel_limit = 36
# my_drive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

print("saving an odrive motor1 config...")
my_drive.save_configuration()
time.sleep(1)

# Calibrate motor and wait for it to finish
print("starting calibration motor1...")
my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while my_drive.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
print('Error motor 1: ', my_drive.axis1.motor.error)
print('Error encoder 1: ', my_drive.axis1.encoder.error)
time.sleep(2)
# my_drive.axis1.config.startup_closed_loop_control = True
# my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# my_drive.axis1.controller.input_pos = 2
# my_drive.axis1.motor.config.pre_calibrated = True
# time.sleep(2)
print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

# my_drive.axis1.encoder.config.pre_calibrated = True
# my_drive.axis1.motor.config.pre_calibrated = True
#
# my_drive.axis0.encoder.config.pre_calibrated = True
# my_drive.axis0.motor.config.pre_calibrated = True

# my_drive.save_configuration()
#


