import servo
import time


# PID Gain
KP = 80
KI = 0.1
KD = 600

# Servo offset
adcMAX = 2900.0
adcMIN = 300.0
angleMAX = 90
angleMIN = -90

# Maximum servo output (0~999)
MaxOutput = 999

# Low-pass filter coefficient for sensor (0~1)
alpha = 1

# Low-pass filter coefficient for servo output (0~1)
beta = 1



# Instantiate the servo
Servo = servo.ServoP(0x10)

# Configure the servo
Servo.set_PID(KP, KI, KD)

# Calibrate the servo
Servo.CalibrateADCMotorAngle(angleMIN, angleMAX, adcMIN, adcMAX)

# Set the maximum output for the servo
Servo.set_MaxOutput(MaxOutput)

# Set the low-pass filter coefficient for sensor values of each servo
Servo.set_lowpass_filter_alpha(alpha)

# Set the low-pass filter coefficient for the sensor values of the servo
Servo.set_lowpass_filter_beta(beta)

# Set the current angle of the servo to the initial target angle
Servo.set_Init()

# Start the servo motor
Servo.motor_on()



# Move the servo to the target angle
"""
count = 0
t = 0.5
while(count < 3):
    # Move to 0 degrees
    goal_angle = 0
    Servo.set_GoalAngle(goal_angle)
    time.sleep(t)

    # Move to 90 degrees
    goal_angle = 90
    Servo.set_GoalAngle(goal_angle)
    time.sleep(t)

    # Move to 0 degrees
    goal_angle = 0
    Servo.set_GoalAngle(goal_angle)
    time.sleep(t)

    # Move to -90 degrees
    goal_angle = -90
    Servo.set_GoalAngle(goal_angle)
    time.sleep(t)
    count += 1

# Shut down the servo
Servo.close_servo()
"""



# Move the servo using speed control
"""
count = 0
t = 0.5
while(count < 3):
    # Move to 0 degrees over 1 second
    goal_angle = 0
    movement_time = 1
    Servo.move_to_Target_angle(goal_angle, movement_time)
    time.sleep(t)

    # Move to 90 degrees over 1 second
    goal_angle = 90
    movement_time = 1
    Servo.move_to_Target_angle(goal_angle, movement_time)
    time.sleep(t)

    # Move to 0 degrees over 1 second
    goal_angle = 0
    movement_time = 1
    Servo.move_to_Target_angle(goal_angle, movement_time)
    time.sleep(t)

    # Move to -90 degrees over 1 second
    goal_angle = -90
    movement_time = 1
    Servo.move_to_Target_angle(goal_angle, movement_time)
    time.sleep(t)

Servo.close_servo()
"""


# Retrieve angle information from the servo
Servo.motor_off()
while(1):
    NowAngle = Servo.get_Angle()
    print(f"Angle: {NowAngle}")
    time.sleep(0.01)


 