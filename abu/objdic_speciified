import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

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

        color_image = np.asanyarray(color_frame.get_data())
        results = model(color_image)
        
        nearest_object = None
        min_depth = float('inf')
        
        for r in results:
            for box, conf, cls in zip(r.boxes.xyxy, r.boxes.conf, r.boxes.cls):
                label = model.names[int(cls)].lower()
                if label == target_object:
                    x1, y1, x2, y2 = map(int, box[:4])
                    center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                    depth_value = get_depth_at_pixel(depth_frame, center_x, center_y)
                    
                    if depth_value < min_depth:
                        min_depth = depth_value
                        nearest_object = (x1, y1, x2, y2, label, conf, depth_value)
        
        if nearest_object:
            x1, y1, x2, y2, label, conf, depth_value = nearest_object
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            text = f"{label} {conf:.2f} Depth {depth_value:.2f}m"
            cv2.putText(color_image, text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow("Detected Object", color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
