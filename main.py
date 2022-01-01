#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Direction, Button, Stop
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import ImageFile, SoundFile


# Initialize the EV3 brick.
ev3 = EV3Brick()

# Configure 2 motors on Ports A and B.  Set the motor directions to
# clockwise, so that positive speed values make the robot move
# forward.  These will be the left and right motors of the Tank Bot.
left_motor = Motor(Port.B, Direction.CLOCKWISE)
right_motor = Motor(Port.C, Direction.CLOCKWISE)

# The wheel diameter of the Bot in mm.
WHEEL_DIAMETER = 54

# The axle track is the distance between the centers of each of the
# wheels.  This is about 200 mm for the Tank Bot.
AXLE_TRACK = 125

# The Driving Base is comprised of 2 motors.  There is a wheel on each
# motor.  The wheel diameter and axle track values are used to make the
# motors move at the correct speed when you give a drive command.
#robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)


# Sets up the robot's drive base
def setupTank():
    global robot
    robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)

    # Set up the robot speed
    robot.stop
    # settings(straight_speed, straight_acceleration, turn_rate, turn_acceleration)
    robot.settings(800, 5000, 40000, 100000)


# Sets up the tank for the first time
setupTank()


# Initialize the steering and overshoot variables.
steering = 60
overshoot = 5

# Set up the Gyro Sensor.  It is used to measure the angle of the robot.
# Keep the Gyro Sensor and EV3 steady when connecting the cable and
# during start-up of the EV3.
gyroSensor = GyroSensor(Port.S1)

#Set up the fork lift
liftMotor = Motor(Port.D, Direction.CLOCKWISE, [8, 12, 28])


# Functions are under this line
#-------------------------------------------------------------------------


def rightTurnLoop(inputAngle):
    gyroSensor.reset_angle(0)
    print("Sub target angle " + str(inputAngle - 10))

    # Turn quickly only if the angle is bigger than 10
    if (inputAngle > 10):
        robot.turn(inputAngle - 10)
    print(gyroSensor.angle())

    print("Target angle " + str(inputAngle))
    print("Loop condition " + str(gyroSensor.angle() < inputAngle))
    if (gyroSensor.angle() < inputAngle):
        while (gyroSensor.angle() < inputAngle):
            robot.turn(1)
            print(gyroSensor.angle())

    if (gyroSensor.angle() > inputAngle):
        while (gyroSensor.angle() > inputAngle):
            robot.turn(-1)
            print(gyroSensor.angle())


def leftTurnLoop(inputAngle):
    gyroSensor.reset_angle(0)

    print("Target angle " + str(-inputAngle))
    print("Sub target angle " + str((-inputAngle) + 10))

    # Turn quickly only if the angle is bigger than 10
    if (inputAngle > 10):
        robot.turn((-inputAngle) + 10)

    print(gyroSensor.angle())
    if (gyroSensor.angle() > -inputAngle):
        while (gyroSensor.angle() > -inputAngle):
            robot.turn(-1)
            print(gyroSensor.angle())

    if (gyroSensor.angle() < -inputAngle):
        while (gyroSensor.angle() < -inputAngle):
            robot.turn(1)
            print(gyroSensor.angle())


# torque; only for run_until_stalled, limit is the torque of the motor
# height; max height to lift to
def grab(torque, height):
    # Opens up the arm
    liftMotor.run_target(400, -450)

    # move in to grab by length of the black pins on the fork
    forkLength = 30
    robot.straight(forkLength)

    # alternative lift function
    liftMotor.run_target(250, height)


def letGo():
    liftMotor.run_target(400, -450)
    liftMotor.hold()

    # back out by length of the black pins on the fork
    forkLenght = 30
    constant = 10
    robot.straight(-forkLenght -constant)

    # restore fork position
    liftMotor.run_target(400, 0)


# Moves the fork up
# Must start with the fork positioned like a forklift
def liftUp():
    # Reset the position of the fork to 0
    liftMotor.run_target(400, 0)
    # lift it up
    liftMotor.run_target(400, 450)


# Moves the fork down
def liftDown():
   liftMotor.run_target(400, 0)


def ram():
    # Record the angle while moving for better accuracy
    robot.straight(500)
    gyroSensor.reset_angle(0)
    angleBefore = gyroSensor.angle()
    robot.straight(270)

    ramDistance = 20
    index = 0
    indexMax = 20
    while (index < indexMax):
        wait(100)
        robot.straight(-ramDistance)
        print("Back distance " + str(-ramDistance))
        wait(100)
        # The distance changes for each push
        robot.straight(ramDistance + 30)
        print("Forward distance " + str(ramDistance + 30))

        index += 1

    angleAfter = gyroSensor.angle()
    print("Angle in " + str(angleBefore))
    print("Angle after " + str(angleAfter))

    # Go back a bit
    robot.straight(-50)


    if (0 > angleAfter):
        rightTurnLoop(abs(angleAfter))
    else:
        restoreAngle(angleBefore, angleAfter)

    # Go back to start
    robot.straight(-1200)


# Restores the previous angle of the bot
def restoreAngle(angleBefore, angleAfter):
    if (angleBefore != angleAfter):
        if (angleAfter < angleBefore):
            rightTurnLoop(abs(angleAfter - angleBefore))
        if (angleAfter > angleBefore):
            leftTurnLoop(abs(angleAfter - angleBefore))


# Goes under the pull up bar
def goUnder():
    robot.straight(800)
    leftTurnLoop(90)
    robot.straight(640)
    leftTurnLoop(65)
    robot.straight(350)
    dance()


def dance():
    gyroSensor.reset_angle(0)

    robot.turn(180)
    robot.straight(-10)
    robot.turn(360)
    robot.straight(10)
    robot.turn(360 * 2)


# Takes an string input
# Makes the robot say the input out loud in Norwegian
def speakNor(whatToSay):
    ev3.speaker.set_speech_options(language='no', voice='f1', speed=200, pitch=70)
    ev3.speaker.say(whatToSay)


# Pulls the arms down from the locked position
def releaseArms():
    liftMotor.run_target(400, -1050)
    liftMotor.run_target(400, 180)
    liftMotor.run_target(400, 0)


def wallSmash():
    robot.straight(280)
    leftTurnLoop(90)
    robot.straight(1000)
    robot.straight(-170)
    gyroSensor.reset_angle(90)
    leftTurnLoop(90)
    robot.straight(800)


def treadmill_complete():
    robot.straight(1000)
    gyroSensor.reset_angle(0)
    robot.straight(650)
    treadmill()

    angleAfter = gyroSensor.angle()
    robot.straight(-200)
    restoreAngle(0, angleAfter)
    rightTurnLoop(180)
    robot.straight(1700)


def treadmill():
    # Disable the tank
    robot.stop()

    index = 0
    while index < 20:
        right_motor.dc(100)
        wait(100)

        index += 1

    # Set up the tank again
    setupTank()


#-------------------------------------------------------------------------


# checks which button is pressed and does methods that are being called upon.
def buttonPress():

    if Button.UP in ev3.buttons.pressed():   # Checks if button UP is pressed.
        print("You pressed the up button")
        ev3.speaker.beep()
        wait(400)
        ram()
        speakNor('Er du sikker på at du trykket rett knapp?')

    elif Button.RIGHT in ev3.buttons.pressed():
        print("You pressed the right button")
        ev3.speaker.beep()
        wait(400)
        wallSmash()
        speakNor('Tror ikke denne knappen gjør noe heller...')

    elif Button.DOWN in ev3.buttons.pressed():    # Checks if button right is pressed.
        print("You pressed the down button")
        ev3.speaker.beep()
        wait(400)
        goUnder()
        speakNor('Ja, ja, ja, dette var gøy')

    elif Button.LEFT in ev3.buttons.pressed():    # Checks if button down is pressed.
        print("You pressed the left button")
        ev3.speaker.beep()
        wait(400)
        treadmill_complete()
        speakNor('Tror du mente å trykke høyre knapp.')


#-------------------------------------------------------------------------


ev3.speaker.beep()
print('I am ready!')

# Main loop, makes buttonPres() run for ever!
while True:
    buttonPress()
    wait(0.01)

