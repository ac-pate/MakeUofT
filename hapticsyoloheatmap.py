import pyrealsense2 as rs
import numpy as np
import cv2
import serial
from ultralytics import YOLO

# Initialize Serial Communication with Arduino
arduino = serial.Serial('COM14', 115200, timeout=1)  # Adjust the port as needed

# Load YOLOv8 model
model = YOLO("yolov8n.pt")

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Align depth to color
align = rs.align(rs.stream.color)

def get_depth_at_pixel(depth_frame, x, y):
    """Retrieve depth in meters at a specific pixel."""
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_value = depth_image[y, x] * 0.001  # Convert mm to meters
    return depth_value

def send_to_arduino(left_value, right_value):
    """Send left and right command values to the Arduino."""
    command = f"{left_value},{right_value}\n"
    arduino.write(command.encode())

# Parameters for mapping depth to a command value
d_min = 0.2   # Object is considered “close” at 0.2m (buzzing stops)
d_max = 2.0   # Maximum depth for full feedback
value_min = 200   # Command value at close range (<=STOP_DISTANCE so no buzz)
value_max = 1000  # Command value when far away (slow buzz)

def map_depth_to_value(depth):
    """Map the measured depth (in meters) to a command value for haptic feedback."""
    if depth <= d_min:
        return value_min  # When too close, send value to stop buzzing
    # Linearly map depth from [d_min, d_max] to [value_min, value_max]
    mapped = int((depth - d_min) / (d_max - d_min) * (value_max - value_min) + value_min)
    return max(value_min, min(value_max, mapped))

# Define a threshold (in pixels) for the object to be considered centered
center_threshold = 50

# User input for object detection
target_object = input("Enter the object to find (e.g., Cup): ").strip().lower()

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Get color image and frame dimensions
        color_image = np.asanyarray(color_frame.get_data())
        height, width, _ = color_image.shape
        center_x, center_y = width // 2, height // 2

        # Create a heat map from the depth frame (using COLORMAP_JET)
        depth_image = np.asanyarray(depth_frame.get_data())
        # Convert depth image to 8-bit (adjust scaling factor as needed)
        depth_colormap = cv2.convertScaleAbs(depth_image, alpha=0.09)
        depth_heatmap = cv2.applyColorMap(depth_colormap, cv2.COLORMAP_JET)

        # Run object detection on the color image
        results = model(color_image)
        
        nearest_object = None
        min_depth = float('inf')
        
        for r in results:
            for box, conf, cls in zip(r.boxes.xyxy, r.boxes.conf, r.boxes.cls):
                label = model.names[int(cls)].lower()
                if label == target_object:
                    x1, y1, x2, y2 = map(int, box[:4])
                    obj_center_x, obj_center_y = (x1 + x2) // 2, (y1 + y2) // 2
                    depth_value = get_depth_at_pixel(depth_frame, obj_center_x, obj_center_y)
                    
                    if depth_value < min_depth:
                        min_depth = depth_value
                        nearest_object = (x1, y1, x2, y2, label, conf, depth_value, obj_center_x, obj_center_y)
        
        if nearest_object:
            x1, y1, x2, y2, label, conf, depth_value, obj_center_x, obj_center_y = nearest_object
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            text = f"{label} {conf:.2f} Depth {depth_value:.2f}m"
            cv2.putText(color_image, text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Compute the command value based solely on depth
            command_value = map_depth_to_value(depth_value)
            
            # Determine horizontal offset from the camera center
            offset = obj_center_x - center_x
            
            if abs(offset) < center_threshold:
                # Object is centered: both motors buzz in sync
                left_command = right_command = command_value
            elif offset < 0:
                # Object is to the left: only the left motor buzzes
                left_command = command_value
                right_command = 0
            else:
                # Object is to the right: only the right motor buzzes
                right_command = command_value
                left_command = 0
            
            send_to_arduino(left_command, right_command)
            
            # Draw an arrow from the object to the center for visual guidance
            cv2.arrowedLine(color_image, (obj_center_x, obj_center_y), (center_x, center_y), (0, 0, 255), 2, tipLength=0.2)
        
        # Show the normal view with annotations and the depth heat map
        cv2.imshow("Navigation Guide", color_image)
        cv2.imshow("Heat Map (Jet)", depth_heatmap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    arduino.close()
