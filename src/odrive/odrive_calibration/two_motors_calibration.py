import odrive
from odrive.enums import *
import time

print("Finding an odrive...")
odrv = odrive.find_any()
time.sleep(3)
print('voltage', odrv.vbus_voltage)
axes = [odrv.axis0, odrv.axis1]
# axes = [odrv.axis1];


for ax in axes:
    print("Start Calibrate...")
    ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    time.sleep(40)
    print("Close loop...")
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    print("Moving one cycle...")
    ax.controller.input_pos = 1
    time.sleep(3)
    print("Back to zero")
    ax.controller.input_pos = 0
    time.sleep(3)
    # ax.encoder.config.pre_calibrated = True
    # ax.motor.config.pre_calibrated = True

# odrv.save_configuration()
#
# try:
#     odrv.reboot()
# except:
#     pass
# tmp = odrv.dump_errors(odrv)
