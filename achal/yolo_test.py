import cv2
from ultralytics import YOLO

# Load model
model = YOLO("yolov8n.pt")

# Set model parameters
model.overrides['conf'] = 0.25  # NMS confidence threshold
model.overrides['iou'] = 0.45  # NMS IoU threshold
model.overrides['agnostic_nms'] = False  # NMS class-agnostic
model.overrides['max_det'] = 1000  # maximum number of detections per imag
# model.export(format="ncnn", imgsz=640)  # creates 'yolov8n_ncnn_model'

# Initialize webcam
cam = cv2.VideoCapture(1)  # Use 0 for default webcam, or try other indices (1, 2, etc.) if you have multiple cameras

# Check if the webcam is opened correctly
if not cam.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Loop through the webcam frames
while True:
    # Read a frame from the webcam
    success, frame = cam.read()

    if success:
        # Run YOLOv8 inference on the frame
        results = model(frame)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Display the annotated frame
        cv2.imshow("YOLOv8 Realtime Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        print("Error: Failed to capture frame from webcam.")
        break

# Release the webcam and close all OpenCV windows
cam.release()
cv2.destroyAllWindows()