from flask import Flask, render_template, request, jsonify, Response
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import os
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


app = Flask(__name__, template_folder=os.path.join(os.path.dirname(__file__), "templates"))

# Initialize ROS node
rospy.init_node('flask_ros_interface', anonymous=True)

# Publishers for different topics
keyboard_pub = rospy.Publisher('/keyboard_topic', String, queue_size=10)
gamepad_pub = rospy.Publisher('/gamepad_topic', Joy, queue_size=10)
recording_pub = rospy.Publisher('/recording_topic', String, queue_size=10)
# Image conversion
bridge = CvBridge()
latest_frame = None

def image_callback(msg):
    global latest_frame
    try:
        # Convert ROS image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        _, jpeg = cv2.imencode('.jpg', cv_image)  # Convert to JPEG
        latest_frame = jpeg.tobytes()
    except Exception as e:
        rospy.logerr(f"Failed to convert image: {e}")

# Subscribe to camera topic (e.g., /camera/image_raw)
rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

def generate_frames():
    """Generator function to stream frames to frontend"""
    while True:
        if latest_frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
        rospy.sleep(0.1)  # Slow down the loop to reduce CPU usage

@app.route('/')
def index():
    return render_template('index.html')

# Endpoint to send keyboard input to ROS
@app.route('/send_keyboard', methods=['POST'])
def send_keyboard():
    data = request.json  # Expecting JSON data from the frontend
    key = data.get('key', '')
    if key:
        rospy.loginfo(f"Keyboard key pressed: {key}")
        keyboard_pub.publish(key)  # Send the key press to the /keyboard_topic
        return jsonify({'status': 'success', 'key': key})
    return jsonify({'status': 'error', 'message': 'No key received'}), 400

# Endpoint to send gamepad input to ROS
@app.route('/send_gamepad', methods=['POST'])
def send_gamepad():
    data = request.json  # Expecting JSON data from the frontend
    button_data = data.get('buttons', [])
    joystick_data = data.get('joystick', [0, 0])

    if button_data and joystick_data:
        rospy.loginfo(f"Gamepad buttons: {button_data}, Joystick: {joystick_data}")
        
        joy_msg = Joy()
        joy_msg.buttons = button_data
        joy_msg.axes = joystick_data

        gamepad_pub.publish(joy_msg)  # Send gamepad data to /gamepad_topic
        return jsonify({'status': 'success', 'buttons': button_data, 'joystick': joystick_data})
    return jsonify({'status': 'error', 'message': 'Incomplete gamepad data'}), 400

# Endpoint to send recording status and text input to ROS
@app.route('/send_recording', methods=['POST'])
def send_recording():
    data = request.json  # Expecting JSON data from the frontend
    message = data.get('message', '')
    recording_status = data.get('record', False)

    if message or recording_status is not None:
        rospy.loginfo(f"Recording {'started' if recording_status else 'stopped'}, Message: {message}")
        recording_pub.publish(f"Recording {'started' if recording_status else 'stopped'} with message: {message}")
        return jsonify({'status': 'success', 'recording': recording_status, 'message': message})
    return jsonify({'status': 'error', 'message': 'No recording data received'}), 400

@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    try:
        return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
    except Exception as e:
        rospy.logerr(f"Error in video feed: {e}")
        return "Error generating video feed", 500



if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)