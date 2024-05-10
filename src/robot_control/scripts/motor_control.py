#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)  # Use BCM numbering

# Motor A (Left Wheel)
motorA_IN1 = 27
motorA_IN2 = 17
motorA_PWM = 12

# Motor B (Right Wheel)
motorB_IN1 = 5
motorB_IN2 = 6
motorB_PWM = 13
STBY = 22

# Setup the GPIO pins
GPIO.setup(motorA_IN1, GPIO.OUT)
GPIO.setup(motorA_IN2, GPIO.OUT)
GPIO.setup(motorA_PWM, GPIO.OUT)
GPIO.setup(motorB_IN1, GPIO.OUT)
GPIO.setup(motorB_IN2, GPIO.OUT)
GPIO.setup(motorB_PWM, GPIO.OUT)
GPIO.setup(STBY, GPIO.OUT)

# Initialize PWM
pwmA = GPIO.PWM(motorA_PWM, 100)  # Set PWM frequency to 100 Hz
pwmB = GPIO.PWM(motorB_PWM, 100)
pwmA.start(0)
pwmB.start(0)

def set_motor_speed(motor, speed):
    in1, in2, pwm = motor
    if speed > 0:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(min(speed, 100))
        #print("AAAAAAAAA")
        #pwm.ChangeDutyCycle(100)
    elif speed < 0:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        pwm.ChangeDutyCycle(min(-speed, 100))
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(0)

def cmd_vel_callback(data):
    rospy.loginfo("Received cmd_vel")
    linear_speed = data.linear.x  # Typically meters/sec
    angular_speed = data.angular.z  * 1.3 # Typically radians/sec

    GPIO.output(STBY, GPIO.HIGH)

    # Calculate wheel speeds in percentage, adjusting the sign for angular speed
    wheel_radius = 0.34  # meters
    robot_base = 0.5    # meters
    v_left = (linear_speed + angular_speed * robot_base / 2) * 100 / wheel_radius
    v_right = (linear_speed - angular_speed * robot_base / 2) * 100 / wheel_radius

    # Max speed for safety or hardware limits
    max_speed = 50  # Adjust this value based on your motor specs
    left_wheel_speed = max(min(v_left, max_speed), -max_speed)
    right_wheel_speed = max(min(v_right, max_speed), -max_speed)

    # Set motor speeds
    set_motor_speed((motorA_IN1, motorA_IN2, pwmA), left_wheel_speed)
    set_motor_speed((motorB_IN1, motorB_IN2, pwmB), right_wheel_speed)



def motor_control_node():
    rospy.init_node('motor_control_node')
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.loginfo("Motor control node is running")
    rospy.spin()

    # Cleanup GPIO when node is stopped
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        motor_control_node()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
