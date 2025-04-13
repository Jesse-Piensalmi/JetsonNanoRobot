#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DataloggerNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('datalogger_node', anonymous=True)

        # Initialize a CvBridge for image conversion
        self.bridge = CvBridge()

        # Define state variables
        self.folder_name = None  # Folder name is None until a valid recording starts
        self.is_recording = False
        # Define the desktop directory
        desktop_path = os.path.expanduser("~/Desktop/data_gathering")

        # Create the folder if it doesn't exist
        if not os.path.exists(desktop_path):
            os.makedirs(desktop_path)
        # Define the base directory for saving images
        self.base_directory = desktop_path
        if not os.path.exists(self.base_directory):
            os.makedirs(self.base_directory)

        # Subscribers
        rospy.Subscriber('/recording_topic', String, self.recording_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

    def recording_callback(self, msg):
        rospy.loginfo(str(msg))
        if "Recording started" in msg.data:
            if self.is_recording:
                rospy.logwarn("Recording already in progress! Ignoring new start message.")
                return  # Ignore new starts if already recording

            # Extract the folder name after "Message:"
            self.folder_name = msg.data.split(" ")[4].strip()
            self.folder_path = os.path.join(self.base_directory, self.folder_name)

            # Create the new folder
            if not os.path.exists(self.folder_path):
                os.makedirs(self.folder_path)
                rospy.loginfo(f"Created folder: {self.folder_path}")

            # Start recording
            self.is_recording = True
            rospy.loginfo(f"Recording started. Saving images to {self.folder_path}")

        elif "Recording stopped" in msg.data:
            # Stop recording and reset the folder name
            self.is_recording = False
            self.folder_name = None
            rospy.loginfo("Recording stopped.")

    def image_callback(self, msg):
        if self.is_recording and self.folder_path:
            try:
                # Convert ROS Image to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                # Generate filename using timestamp
                image_filename = os.path.join(self.folder_path, f"{rospy.get_time()}.png")

                # Save the image
                cv2.imwrite(image_filename, cv_image)
                rospy.loginfo(f"Saved image: {image_filename}")

            except Exception as e:
                rospy.logerr(f"Error while saving image: {str(e)}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        datalogger_node = DataloggerNode()
        datalogger_node.run()
    except rospy.ROSInterruptException:
        pass