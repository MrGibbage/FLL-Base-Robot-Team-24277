# LEGO type:standard slot:0 autostart

import base_robot
import sys
from spike.control import wait_for_seconds, wait_until, Timer
from spike.operator import greater_than, greater_than_or_equal_to, \
    less_than, less_than_or_equal_to, equal_to, not_equal_to


br = base_robot.BaseRobot()

#back up gently to make sure the robot is square in the jig
#then reset the gyro
br.driveMotors.move(.5, "seconds", 0, -10)
br.hub.motion_sensor.reset_yaw_angle()

br.AccelGyroDriveFwd()
br.TurnRightAndDriveOnHeading(85, 50)
br.GyroDriveOnHeading()
br.foo()
br.GyroTurn()

#raise SystemExit
sys.exit(1)

