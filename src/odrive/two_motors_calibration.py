import odrive
from odrive.enums import *
import time

print("Finding an odrive...")
odrv = odrive.find_any()

axes = [odrv.axis0, odrv.axis1];
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

# tmp = odrv.dump_errors(odrv)
