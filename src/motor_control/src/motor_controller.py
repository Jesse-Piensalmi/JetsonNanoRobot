#!/usr/bin/env python3

import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import signal
import sys

# Check priviliges
import os
import grp

group_names = [grp.getgrgid(g).gr_name for g in os.getgroups()]
print("Effective user ID:", os.geteuid())
print("Groups:", group_names)
print("Can write to /sys/class/gpio/export:", os.access("/sys/class/export", os.W_OK))
# GPIO Setup
PWM_LEFT = 33
PWM_RIGHT = 32

GPIO.setmode(GPIO.BOARD)
#GPIO.setup(35,GPIO.OUT) #PWMR
#GPIO.setup(36,GPIO.OUT) #PWML
GPIO.setup(37,GPIO.OUT) #PWMR
GPIO.setup(38,GPIO.OUT) #PWML
GPIO.setup(PWM_LEFT,GPIO.OUT, initial=GPIO.LOW) #L_EN
GPIO.setup(PWM_RIGHT,GPIO.OUT, initial=GPIO.LOW) #R_EN
GPIO.setup(36,GPIO.OUT)
# Set up PWM with frequency (e.g., 1000 Hz)
pwm_left = GPIO.PWM(PWM_LEFT, 1000)
pwm_right = GPIO.PWM(PWM_RIGHT, 1000)
r_en = 37
l_en = 38

pwm_left.start(0)  # Start with 0% duty cycle (stopped)
pwm_right.start(0)

# Global variables for motor speed and direction
left_speed = 0
right_speed = 0

def set_motor_speed(left, right):
    """Updates motor speeds using PWM"""
    global left_speed, right_speed
    
    
    # If we have reverse
    if left<0:
        GPIO.output(38, GPIO.HIGH)
        GPIO.output(37, GPIO.LOW)
        rospy.loginfo(f"Motor control: Backing up")
        
        pwm_left.ChangeDutyCycle(abs(left))
    else:
        GPIO.output(38, GPIO.LOW)
        GPIO.output(37, GPIO.HIGH)
        rospy.loginfo(f"Motor control: Backing up")
        
        pwm_left.ChangeDutyCycle(abs(left))
    if right<0:
        #GPIO.output(35, GPIO.HIGH)
        #GPIO.output(36, GPIO.LOW)
        rospy.loginfo(f"Motor control: Backing up")
        pwm_right.ChangeDutyCycle(abs(right))
        
    else:
        #GPIO.output(35, GPIO.LOW)
        #GPIO.output(36, GPIO.HIGH)
        rospy.loginfo(f"Motor control: CHAAARGE duty cycle {left_speed}")
        pwm_right.ChangeDutyCycle(abs(right))
        

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

    # release Pins in case the robot shits the bed
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    rospy.loginfo("Motor Controller Node Running")
    rospy.spin()
