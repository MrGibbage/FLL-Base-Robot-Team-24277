
from spike import PrimeHub, LightMatrix, Button, StatusLight, \
    ForceSensor, MotionSensor, Speaker, ColorSensor, App, \
    DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from spike.operator import greater_than, greater_than_or_equal_to, \
    less_than, less_than_or_equal_to, equal_to, not_equal_to
import math
import sys


class BaseRobot():
    """
    A collection of methods and Spike Prime objects for FLL Team 24277. \
    The BaseRobot has two drive motors as a MotorPair, two medium motors \
    for moving attachments, and all of the base methods available for \
    Spike Prime sensors and motors.

    Example
    ---------

    import base_robot

    import sys

    from spike.control import wait_for_seconds, wait_until, Timer

    from spike.operator import greater_than, greater_than_or_equal_to, \
        less_than, less_than_or_equal_to, equal_to, not_equal_to

    from spike import wait_until

    br = base_robot.BaseRobot()

    br.AccelGyroDriveForward(40)

    br.driveMotors.move_tank(25, 'cm', 50, 50)

    """

    def __init__(self):
        self._version = "1.0 (20 May 2022)"

        self.timer = Timer()
        self._leftDriveMotorPort = 'A'
        self._rightDriveMotorPort = 'C'
        self._leftMediumMotorPort = 'F'
        self._rightMediumMotorPort = 'B'
        #self._colorSensorPort = NULL
        #self._touchSensorPort = NULL

        self._tireDiameter = 5.6 #cm
        self._tireCircum = self._tireDiameter * math.pi #cm

        self._errorArray = [0, 0, 0, 0, 0]

        self.hub = PrimeHub()

        self.driveMotors = MotorPair(self._leftDriveMotorPort, self._rightDriveMotorPort)
        self.driveMotors.set_motor_rotation(amount = self._tireCircum, unit = 'cm')

        self.leftMedMotor = Motor(self._leftMediumMotorPort)
        self.rightMedMotor = Motor(self._rightMediumMotorPort)

    def GetVersion(self):
        return self._version

    def AccelGyroDriveForward(self, desiredDistance):
        """
        Drives the robot very straight, using the acceleration \
        and the gyro. Accelerates to prevent wheel slipping. \
        Gyro keeps the robot pointing on the same heading.

        Minimum distance that this will work for is about 16cm. \
        If you need to go a very short distance, use move_tank.

        Parameter
        -------------

        desiredDistance: How far the robot should go in cm

        type: float

        values: any value above 16.0. You can enter smaller numbers, but the robot will still go 16cm

        default: no default value

        Example
        -------------

        import base_robot

        br = base_robot.BaseRobot()

        br.AccelGyroDriveForward(40)

        """
        maxSpeed = 75 #rpm
        #halfSpeed = 40 #rpm
        minSpeed = 5 #rpm
        loopDelay = 0.02 #seconds
        #minProportionFactor = 4 #how much do we correct for every degree off course
        #maxProportionFactor = 20
        baseYaw = self.hub.motion_sensor.get_yaw_angle()
        testMotor = Motor(self._rightDriveMotorPort)
        testMotor.set_degrees_counted(0)
        totalDegreesNeeded = desiredDistance / self._tireCircum * 360

        #First accelerate up to maxSpeed
        for curSpeed in range(minSpeed, maxSpeed):
            error = baseYaw - self.hub.motion_sensor.get_yaw_angle()
            proportionFactor = self.calulateProportionFactor(curSpeed)
            print("error = " + str(error) + "; curSpeed = " + str(curSpeed) + "; propFactor = " + str(proportionFactor) + "; steering = " + str(error * proportionFactor) + "; current heading = " + str(self.hub.motion_sensor.get_yaw_angle()))
            self.driveMotors.start(error * proportionFactor, curSpeed)
            wait_for_seconds(loopDelay)

        #we are now driving at maxSpeed

        #How much farther do we need to go?
        #totalDegreesNeeded -= testMotor.get_degrees_counted()

        curSpeed = maxSpeed #just to be sure
        while testMotor.get_degrees_counted() < totalDegreesNeeded:
            error = baseYaw - self.hub.motion_sensor.get_yaw_angle()
            totalDegreesRemaining = totalDegreesNeeded - testMotor.get_degrees_counted()
            curSpeed = minSpeed + ((maxSpeed - minSpeed) * min(totalDegreesRemaining, 540) / 540)
            proportionFactor = self.calulateProportionFactor(curSpeed)
            print("error = " + str(error) + "; curSpeed = " + str(curSpeed) + "; propFactor = " + str(proportionFactor) + "; steering = " + str(error * proportionFactor) + "; current heading = " + str(self.hub.motion_sensor.get_yaw_angle()))
            self.driveMotors.start(error * proportionFactor, int(curSpeed))
            #wait_for_seconds(loopDelay)

        self.driveMotors.set_stop_action('brake')
        self.driveMotors.stop()
        print("Stopped. Drove " + str(testMotor.get_degrees_counted()) + " of " + str(totalDegreesNeeded) + " degrees")

    @staticmethod
    def calulateProportionFactor(speed):
        if speed > 40:
            return 6
        return int(speed * -0.4 + 20)

    def GyroTurn(self, desiredDegrees):
        """
        Turns the robot the specified number of degrees. Positive numbers turn to the \
        right, negative numbers turn the robot to the left. Note that when the robot \
        makes the turn, it will always overshoot by about seven degrees. In other words \
        if you need a +90 degree turn, you will probably end up commanding something \
        around +83 degrees. You may also want to put a wait_for_seconds(0.2) or \
        something like that after a gyro turn. Just to make sure the robot has stopped \
        moving before continuing with more instructions.

        Parameter
        -------------

        desiredDegrees: How many degrees should the robot turn. Positive values turn the \
        robot to the right, negative values turn to the left.

        type: float

        values: any. Best to keep the numbers less than 180, just so the robot doesn't \
        more than necessary.

        default: no default value

        """
        self.hub.motion_sensor.reset_yaw_angle()
        speed = 15

        if desiredDegrees < 0:
            while self.hub.motion_sensor.get_yaw_angle() > desiredDegrees:
                self.driveMotors.start_tank(-speed, speed)
        else:
            while self.hub.motion_sensor.get_yaw_angle() < desiredDegrees:
                self.driveMotors.start_tank(speed, -speed)
        self.driveMotors.stop()
        wait_for_seconds(0.25)


    def GyroTurnSlow(self, desiredDegrees):
        """
        Just like GyroTurn, but uses less power, making turns that are slightly more \
        accurate. This usually only overshoots by a degree or two. Probably best for \
        smaller turns, or turns that have to be very accurate.
        
        Turns the robot the specified number of degrees. Positive numbers turn to the \
        right, negative numbers turn the robot to the left. Note that when the robot \
        makes the turn, it will always overshoot by about seven degrees. In other words \
        if you need a +90 degree turn, you will probably end up commanding something \
        around +83 degrees.

        Parameter
        -------------

        desiredDegrees: How many degrees should the robot turn. Positive values turn the \
        robot to the right, negative values turn to the left.

        type: float

        values: any. Best to keep the numbers less than 180, just so the robot doesn't \
        more than necessary.

        default: no default value

        """
        self.hub.motion_sensor.reset_yaw_angle()
        speed = 5

        if desiredDegrees < 0:
            while self.hub.motion_sensor.get_yaw_angle() > desiredDegrees:
                self.driveMotors.start_tank(-speed, speed)
        else:
            while self.hub.motion_sensor.get_yaw_angle() < desiredDegrees:
                self.driveMotors.start_tank(speed, -speed)
        self.driveMotors.stop()


    def GyroDriveOnHeading(self, desiredHeading, desiredDistance):
        """
        Drives the robot very straight on a given heading for a \
        given distance, using the acceleration and the gyro. \
        Accelerates to prevent wheel slipping. \
        Gyro keeps the robot pointing on the desired heading.

        Minimum distance that this will work for is about 16cm.

        If you need to go a very short distance, use move_tank.

        This method works best with the TurnRightAndDriveOnHeading \
        and TurnLeftAndDriveOnHeading functions

        Parameter
        -------------

        desiredHeading: On what heading should the robot drive

        type: float

        values: any. Best if the desired heading is close to the current heading. Unpredictable robot movement may occur for large heading differences.

        default: no default value

        desiredDistance: How far the robot should go in cm

        type: float

        values: any value above 16.0. You can enter smaller numbers, but the robot will still go 16cm

        default: no default value

        Example
        -------------

        import base_robot

        br = base_robot.BaseRobot()

        br.GyroDriveOnHeading(90, 40) #drive on heading 90 for 40 cm

        """
        print("GyroDriveOnHeading("+ str(desiredHeading) + ", " + str(desiredDistance) + ")")
        print("Current heading = " + str(self.hub.motion_sensor.get_yaw_angle()))
        maxSpeed = 75 #rpm
        halfSpeed = 40 #rpm
        minSpeed = 5 #rpm
        loopDelay = 0.02 #seconds
        #proportionFactor = 4 #how much do we correct for every degree off course
        testMotor = Motor(self._rightDriveMotorPort)
        testMotor.set_degrees_counted(0)
        totalDegreesNeeded = desiredDistance / self._tireCircum * 360
        self.driveMotors.start(0, minSpeed)
        #First accelerate up to maxSpeed
        print("Current heading = " + str(self.hub.motion_sensor.get_yaw_angle()))
        for curSpeed in range(minSpeed, maxSpeed):
            error = desiredHeading - self.hub.motion_sensor.get_yaw_angle()
            proportionFactor = self.calulateProportionFactor(curSpeed)
            print("error = " + str(error) + "; curSpeed = " + str(curSpeed) + "; propFactor = " + str(proportionFactor) + "; steering = " + str(error * proportionFactor) + "; current heading = " + str(self.hub.motion_sensor.get_yaw_angle()))
            self.driveMotors.start(error * proportionFactor, curSpeed)
            wait_for_seconds(loopDelay)

        #we are now driving at maxSpeed

        curSpeed = maxSpeed
        while testMotor.get_degrees_counted() < totalDegreesNeeded:
            error = desiredHeading - self.hub.motion_sensor.get_yaw_angle()
            totalDegreesRemaining = totalDegreesNeeded - testMotor.get_degrees_counted()
            curSpeed = minSpeed + ((maxSpeed - minSpeed) * min(totalDegreesRemaining, 540) / 540)
            proportionFactor = self.calulateProportionFactor(curSpeed)
            print("error = " + str(error) + "; curSpeed = " + str(curSpeed) + "; propFactor = " + str(proportionFactor) + "; steering = " + str(error * proportionFactor) + "; current heading = " + str(self.hub.motion_sensor.get_yaw_angle()))
            self.driveMotors.start(error * proportionFactor, int(curSpeed))
            #wait_for_seconds(loopDelay)

        self.driveMotors.set_stop_action('brake')
        self.driveMotors.stop()
        print("Stopped. Drove " + str(testMotor.get_degrees_counted()) + " of " + str(totalDegreesNeeded) + " degrees")


    def TurnRightAndDriveOnHeading(self, desiredHeading, desiredDistance):
        """
        Turns the robot to the right until the desiredHeading \
        is reached. Then drives on the desiredHeading until \
        the desiredDistance has been reached.

        Minimum distance that this will work for is about 16cm. \
        If you need to go a very short distance, use GyroTurn and \
        move_tank.

        Parameter
        -------------

        desiredHeading: On what heading should the robot drive

        type: float

        values: any. However, it must be a heading larger than the current heading (that is, to the right). If a heading is entered that is less than the current heading, the program will exit.

        default: no default value

        desiredDistance: How far the robot should go in cm

        type: float

        values: any value above 16.0. You can enter smaller numbers, but the robot will still go 16cm

        default: no default value

        Example
        -------------

        import base_robot

        br = base_robot.BaseRobot()

        br.TurnRightAndDriveOnHeading(90, 40) #drive on heading 90 for 40 cm

        """
        print("TurnRight")
        if desiredHeading < self.hub.motion_sensor.get_yaw_angle():
            print("Cannot turn right to a heading that is smaller than the current heading. Exiting.")
            sys.exit(1)

        overshoot = 7 #how many degrees does gyroturn normally overshoot by

        currentHeading = int(self.hub.motion_sensor.get_yaw_angle())
        self.GyroTurn(desiredHeading - overshoot - currentHeading)
        self.GyroDriveOnHeading(desiredHeading, desiredDistance)

    def TurnLeftAndDriveOnHeading(self, desiredHeading, desiredDistance):
        """
        Turns the robot to the left until the desiredHeading \
        is reached. Then drives on the desiredHeading until \
        the desiredDistance has been reached.

        Minimum distance that this will work for is about 16cm. \
        If you need to go a very short distance, use GyroTurn and \
        move_tank.

        Parameter
        -------------

        desiredHeading: On what heading should the robot drive

        type: float

        values: any. However, it must be a heading smaller than the current heading (that is, to the left). If a heading is entered that is greater than the current heading, the program will exit.

        default: no default value

        desiredDistance: How far the robot should go in cm

        type: float

        values: any value above 16.0. You can enter smaller numbers, but the robot will still go 16cm

        default: no default value

        Example
        -------------

        import base_robot

        br = base_robot.BaseRobot()

        br.TurnLeftAndDriveOnHeading(90, 40) #drive on heading 90 for 40 cm

        """

        if desiredHeading > self.hub.motion_sensor.get_yaw_angle():
            print("Cannot turn left to a heading that is greater than the current heading. Exiting.")
            sys.exit(1)

        overshoot = 7 #how many degrees does gyroturn normally overshoot by

        currentHeading = int(self.hub.motion_sensor.get_yaw_angle())
        self.GyroTurn(-(currentHeading - desiredHeading - overshoot))
        self.GyroDriveOnHeading(desiredHeading, desiredDistance)
