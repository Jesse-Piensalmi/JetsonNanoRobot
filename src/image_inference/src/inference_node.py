#!/usr/bin/env python3

import rospy
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import torchvision.models as models
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from PIL import Image as PILImage
import time
import datetime
import os
import csv

# Model Weights Paths
MODEL_PATHS = {
    "res_net": "/home/jetson/ws/src/image_inference/weights/best_resnet.pth",
    "mobile_net": "/home/jetson/ws/src/image_inference/weights_mobile/best_mobilenet.pth",
    "efficient_net": "/home/jetson/ws/src/image_inference/weights_efficient/best_efficient.pth"
}

# Device setup (GPU if available)
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Global variables
bridge = CvBridge()
is_processing = False
current_model_name = "res_net"  # Default model
model = None
environments = ["Buildings","Forest","Mountains","Glacier","Street","Sea"]

# Logging
is_recording = False
log_file = None
csv_writer = None
log_folder_path = None

# Preprocessing Transform (matches training setup)
transform = transforms.Compose([
    transforms.Resize((150, 150)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

def recording_callback(msg):
    global is_recording, log_file, csv_writer, log_folder_path

    if "Recording started" in msg.data:
        is_recording = True
        folder_name = msg.data.split(" ")[4].strip()
        log_folder_path = os.path.join("/home/jetson/Desktop/data_gathering", folder_name)
        os.makedirs(log_folder_path, exist_ok=True)

        log_file = open(os.path.join(log_folder_path, "inference_log.csv"), mode='w', newline='')
        csv_writer = csv.writer(log_file)
        csv_writer.writerow(["Timestamp", "Inference Result", "Inference Time (s)"])
        rospy.loginfo(f"Inference logging started in: {log_folder_path}")

    elif "Recording stopped" in msg.data:
        is_recording = False
        if log_file:
            log_file.close()
            log_file = None
            csv_writer = None
            log_folder_path = None
            rospy.loginfo("Inference logging stopped.")

def load_model(model_name):
    """Loads a model architecture and weights dynamically."""
    global model, current_model_name

    if model_name == "res_net":
        model = models.resnet18(pretrained=False)
        model.fc = nn.Linear(model.fc.in_features, 6)

    elif model_name == "mobile_net":
        model = models.mobilenet_v2(pretrained=False)
        model.classifier[1] = nn.Linear(model.classifier[1].in_features, 6)

    elif model_name == "efficient_net":
        model = models.efficientnet_b0(pretrained=False)
        model.classifier[1] = nn.Linear(model.classifier[1].in_features, 6)

    else:
        rospy.logerr(f"Invalid model name: {model_name}")
        return

    # Load model weights
    model.load_state_dict(torch.load(MODEL_PATHS[model_name], map_location=device))
    model = model.to(device)
    model.eval()
    
    current_model_name = model_name
    rospy.loginfo(f"Loaded model: {model_name}")

def image_callback(msg):
    """Processes incoming images and performs inference."""
    global is_processing
    if is_processing:
        rospy.logwarn("Skipping image, still processing...")
        return  # Skip if inference is running

    is_processing = True
    try:
        # Convert ROS Image to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # Crop and Downsample
        downsampled_cv2_image = crop_center_and_downsample(cv_image)
        # Display cropped and downsampled image
        #cv2.imshow("Downsampled_image feed", downsampled_cv2_image)
        #cv2.waitKey(1)

        # Convert OpenCV image to PIL for transformation
        pil_image = PILImage.fromarray(cv2.cvtColor(downsampled_cv2_image, cv2.COLOR_BGR2RGB))
        input_tensor = transform(pil_image).unsqueeze(0).to(device)
        
        start_time = time.time()
        # Perform inference
        with torch.no_grad():
            output = model(input_tensor)
        inference_time = time.time() - start_time
        # Post-process result
        result = postprocess(output)
        pub_result.publish(result)

        rospy.loginfo(f"Model: {current_model_name}, Inference: {result}, Time: {inference_time:.4f}s")

        if is_recording and csv_writer:
            csv_writer.writerow([
                datetime.datetime.now().isoformat(),
                result,
                f"{inference_time:.4f}"
            ])

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

    torch.cuda.empty_cache()
    
    is_processing = False  # Unlock inference

def crop_center_and_downsample(image, crop_size=(1080,1080), target_size=(150,150)):
    height,width,_ = image.shape
    start_x = (width-crop_size[1]) // 2
    start_y = (height-crop_size[0]) // 2
    cropped_image = image[start_y:start_y+crop_size[0],start_x:start_x+crop_size[1]]

    downsample = cv2.resize(cropped_image, target_size, interpolation=cv2.INTER_AREA)
    print(downsample.shape)
    return downsample

def postprocess(output):
    """Extracts the class prediction from the model output."""
    _, predicted = torch.max(output, 1)  # Get the class index
    return str(environments[int(predicted.item())])

def model_switch_callback(msg):
    """Handles model switching requests from Flask (via ROSBridge)."""
    model_name = msg.data.strip()
    if model_name in MODEL_PATHS and model_name != current_model_name:
        rospy.loginfo(f"Switching model to: {model_name}")
        load_model(model_name)
    else:
        rospy.logwarn(f"Invalid or already active model: {model_name}")

if __name__ == "__main__":
    rospy.init_node("image_inference_node")

    # Load the default model
    load_model(current_model_name)

    # Subscribe to camera topic
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback, queue_size=1)
    rospy.Subscriber('/recording_topic', String, recording_callback)
    # Publisher for inference results
    pub_result = rospy.Publisher("/inference/result", String, queue_size=1)
    
    # Subscribe to model selection topic (from Flask)
    rospy.Subscriber("/model/select", String, model_switch_callback)

    rospy.loginfo("Inference node started!")
    rospy.spin()