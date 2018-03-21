from time import sleep
from ev3dev.auto import *
import time
import sys
from collections import deque
import importlib
import json

def Read(file):
    file.seek(0)    
    return(int(file.read()))

def Write(file,value):
    file.truncate(0)
    file.write(str(int(value)))
    file.flush()    

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

def ControlAction(action):
    wholeAction = int(round(action * voltageCompensation))
    if wholeAction > 0:
        wholeAction = min(100, wholeAction + frictionOffset)
    elif wholeAction < 0:
        wholeAction = max(-100, wholeAction - frictionOffset)
    Write(motorDutyCycleLeft, wholeAction)
    Write(motorDutyCycleRight, wholeAction)

powerSupply = PowerSupply()
buttons = Button()

gyroSensorValueRaw  = open(GyroSensor()._path + "/value0", "r")
motorLeft  = LargeMotor('outC')
motorRight = LargeMotor('outB')
voltageNominal = 8.0
frictionOffsetNominal = 3
degPerRawMotorUnit = 1
RPMperPerPercentSpeed = 1.7  
GyroAngle = 1300
GyroRate = 120   
MotorAngle = 7
MotorAngularSpeed = 9
MotorAngleErrorAccumulated = 3
loopTimeMiliSec = 30
motorAngleHistoryLength = 5
gyroDriftCompensationFactor = 0.05 

radiansPerDegree = 3.14159/180                                                # The number of radians in a degree.
radiansPerSecondPerRawGyroUnit = radiansPerDegree # Rate in radians/sec per gyro output unit
radiansPerRawMotorUnit = degPerRawMotorUnit*radiansPerDegree        # Angle in radians per motor encoder unit
radPerSecPerPercentSpeed = RPMperPerPercentSpeed*6*radiansPerDegree   # Actual speed in radians/sec per unit of motor speed    

# Read battery voltage
voltageIdle = powerSupply.measured_volts
voltageCompensation = voltageNominal/voltageIdle

# Offset to limit friction deadlock
frictionOffset = int(round(frictionOffsetNominal*voltageCompensation))

#Timing settings for the program
loopTimeSec = loopTimeMiliSec/1000  # Time of each loop, measured in seconds.
loopCount = 0                            # Loop counter, starting at 0

# A deque (a fifo array) which we'll use to keep track of previous motor positions, which we can use to calculate the rate of change (speed)
motorAngleHistory = deque([0], motorAngleHistoryLength)
gyroDriftCompensationRate = gyroDriftCompensationFactor*loopTimeSec*radiansPerSecondPerRawGyroUnit
motorLeft.reset()
motorRight.reset()
motorLeft.run_direct()              
motorRight.run_direct()
motorEncoderLeft    = open(motorLeft._path + "/position", "r")    
motorEncoderRight   = open(motorRight._path + "/position", "r")           
motorDutyCycleLeft = open(motorLeft._path + "/duty_cycle_sp", "w")
motorDutyCycleRight= open(motorRight._path + "/duty_cycle_sp", "w")  
                    
# Reset variables representing physical signals
motorAngleRaw              = 0 # The angle of "the motor", measured in raw units (degrees for the EV3). We will take the average of both motor positions as "the motor" angle, wich is essentially how far the middle of the robot has traveled.
motorAngle                 = 0 # The angle of the motor, converted to radians (2*pi radians equals 360 degrees).
motorAngleReference        = 0 # The reference angle of the motor. The robot will attempt to drive forward or backward, such that its measured position equals this reference (or close enough).
motorAngleError            = 0 # The error: the deviation of the measured motor angle from the reference. The robot attempts to make this zero, by driving toward the reference.
motorAngleErrorAccumulated = 0 # We add up all of the motor angle error in time. If this value gets out of hand, we can use it to drive the robot back to the reference position a bit quicker.
motorAngularSpeed          = 0 # The motor speed, estimated by how far the motor has turned in a given amount of time
motorAngularSpeedReference = 0 # The reference speed during manouvers: how fast we would like to drive, measured in radians per second.
motorAngularSpeedError     = 0 # The error: the deviation of the motor speed from the reference speed.
motorDutyCycle             = 0 # The 'voltage' signal we send to the motor. We calulate a new value each time, just right to keep the robot upright.
gyroRateRaw                = 0 # The raw value from the gyro sensor in rate mode.
gyroRate                   = 0 # The angular rate of the robot (how fast it is falling forward or backward), measured in radians per second.
gyroEstimatedAngle         = 0 # The gyro doesn't measure the angle of the robot, but we can estimate this angle by keeping track of the gyroRate value in time
gyroOffset                 = 0 # Over time, the gyro rate value can drift. This causes the sensor to think it is moving even when it is perfectly still. We keep track of this offset.
eprint("Start")
while not buttons.any():
    time.sleep(0.01)
gyroRateCalibrateCount = 100
for i in range(gyroRateCalibrateCount):
    gyroOffset = gyroOffset + Read(gyroSensorValueRaw)
    time.sleep(0.01)
gyroOffset = gyroOffset/gyroRateCalibrateCount
tProgramStart = time.time()
while not buttons.any():
    tLoopStart = time.time() - tProgramStart
    speed = 0
    current = Read(gyroSensorValueRaw)
    
    gyroRateRaw = Read(gyroSensorValueRaw)
    gyroRate = (gyroRateRaw - gyroOffset)*radiansPerSecondPerRawGyroUnit
    motorAngleRaw = (Read(motorEncoderLeft) + Read(motorEncoderRight))/2
    motorAngle = motorAngleRaw*radiansPerRawMotorUnit
    motorAngularSpeedReference = speed*radPerSecPerPercentSpeed
    motorAngleReference = motorAngleReference + motorAngularSpeedReference*loopTimeSec
    motorAngleError = motorAngle - motorAngleReference    
    motorAngularSpeed = (motorAngle - motorAngleHistory[0])/(motorAngleHistoryLength*loopTimeSec)
    motorAngularSpeedError = motorAngularSpeed
    motorAngleHistory.append(motorAngle)

    motorDutyCycle =( GyroAngle  * gyroEstimatedAngle
                    + GyroRate   * gyroRate
                    + MotorAngle * motorAngleError
                    + MotorAngularSpeed * motorAngularSpeedError
                    + MotorAngleErrorAccumulated * motorAngleErrorAccumulated)
                    
    #eprint("----------------",balanceAngle,"\n",currentAngle,"\n",gError,"\n",motorDutyCycle)
    ControlAction(motorDutyCycle)
    if motorDutyCycle > 400 or motorDutyCycle < -400:
        break

    gyroEstimatedAngle = gyroEstimatedAngle + gyroRate*loopTimeSec
    gyroOffset = (1-gyroDriftCompensationRate)*gyroOffset+gyroDriftCompensationRate*gyroRateRaw
    motorAngleErrorAccumulated = motorAngleErrorAccumulated + motorAngleError*loopTimeSec
    while(time.time()-tProgramStart - tLoopStart <  loopTimeSec):
        time.sleep(0.0001)
tProgramEnd = time.time()
ControlAction(0)