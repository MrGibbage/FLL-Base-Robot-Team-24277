# LEGO type:standard slot:0 autostart

import base_robot
import sys
from spike.control import wait_for_seconds, wait_until, Timer
from spike.operator import greater_than, greater_than_or_equal_to, \
    less_than, less_than_or_equal_to, equal_to, not_equal_to

br = base_robot.BaseRobot()

#br.AccelGyroDriveForward(40) #go forward 40 cm
br.driveMotors.move(.5, "seconds", 0, -10)
br.hub.motion_sensor.reset_yaw_angle()
br.AccelGyroDriveForward(55)
#wait_for_seconds(2)
br.TurnRightAndDriveOnHeadingWithPid(87, 45, 5, 1, 0)
#br.GyroTurn(90)
#print(br._version)


#raise SystemExit
sys.exit(1)