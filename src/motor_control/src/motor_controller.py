#!/usr/bin/env python3

import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import String
from sensor_msgs.msg import Joy

# GPIO Setup
PWM_LEFT = 32
PWM_RIGHT = 33
LEN = 37
REN = 38

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PWM_LEFT, GPIO.OUT)
GPIO.setup(PWM_RIGHT, GPIO.OUT)
GPIO.setup(REN, GPIO.OUT)
GPIO.setup(LEN, GPIO.OUT)
# Set up PWM with frequency (e.g., 1000 Hz)
pwm_left = GPIO.PWM(PWM_LEFT, 255)
pwm_right = GPIO.PWM(PWM_RIGHT, 255)
r_en = GPIO.output(37, GPIO.LOW)
l_en = GPIO.output(38, GPIO.LOW)

pwm_left.start(0)  # Start with 0% duty cycle (stopped)
pwm_right.start(0)

# Global variables for motor speed and direction
left_speed = 0
right_speed = 0

def set_motor_speed(left, right):
    """Updates motor speeds using PWM"""
    global left_speed, right_speed
    
    left_speed = max(0, min(100, left))   # Limit duty cycle to 0-100%
    right_speed = max(0, min(100, right))
    # If we have reverse
    if left<0:
        l_en=GPIO.output(38, GPIO.HIGH)
    else:
        l_en=GPIO.output(38, GPIO.LOW)

    if right<0:
        r_en=GPIO.output(37, GPIO.HIGH)
    else:
        r_en=GPIO.output(37, GPIO.LOW)
    pwm_left.ChangeDutyCycle(abs(left_speed))
    pwm_right.ChangeDutyCycle(abs(right_speed))

def keyboard_callback(msg):
    """Handles keyboard input for movement"""
    key = msg.data.lower()
    rospy.loginfo(f"Keyboard Input: {key}")

    if key == "up":  # Forward
        set_motor_speed(80, 80)
    elif key == "down":  # Backward
        set_motor_speed(-80, -80)
    elif key == "left":  # Turn left
        set_motor_speed(-50, 50)
    elif key == "right":  # Turn right
        set_motor_speed(50, -50)
    elif key == "q":  # Stop
        set_motor_speed(0, 0)

def gamepad_callback(msg):
    """Handles joystick input"""
    axes = msg.axes
    buttons = msg.buttons
    
    linear = axes[1]  # Forward/Backward (Assuming left stick Y-axis)
    angular = axes[0]  # Left/Right (Assuming left stick X-axis)

    left_motor = (linear + angular) * 100  # Differential drive
    right_motor = (linear - angular) * 100

    set_motor_speed(left_motor, right_motor)

def shutdown():
    """Stops motors on shutdown"""
    rospy.loginfo("Shutting down, stopping motors")
    set_motor_speed(0, 0)
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()

if __name__ == "__main__":
    rospy.init_node("motor_controller", anonymous=True)

    rospy.Subscriber("/keyboard_topic", String, keyboard_callback)
    rospy.Subscriber("/gamepad_topic", Joy, gamepad_callback)

    rospy.on_shutdown(shutdown)
    rospy.loginfo("Motor Controller Node Running")
    rospy.spin()