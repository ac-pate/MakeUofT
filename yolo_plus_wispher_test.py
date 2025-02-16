import whisper
import pyaudio
import numpy as np
import threading
import queue
import time
import cv2
from datetime import datetime
import re
from ultralytics import YOLO
import supervision as sv 

# Load Whisper model for speech recognition
whisper_model = whisper.load_model("base")

# Load YOLO-World model for open-vocabulary detection
yolo_model = YOLO("yolov8s-worldv2.pt")

# Audio parameters
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100 
CHUNK = 1024
RECORD_SECONDS = 5  # Shorter chunks for more responsive wake word detection
LANGUAGE = "en"

# Initialize PyAudio
audio = pyaudio.PyAudio()

# Create queues and flags
audio_queue = queue.Queue()
is_recording = True
current_object_to_find = None

def record_audio():
    """Continuously record audio and put chunks in the queue"""

    mic_index = 0  # Change this to match your USB mic index (use `pyaudio.PyAudio().get_device_info_by_index(i)`)
    
    stream = audio.open(format=FORMAT, channels=CHANNELS,
                        rate=RATE, input=True,
                        input_device_index=mic_index,
                        frames_per_buffer=CHUNK)
    
    print("* Recording started - say 'Hey Glasses, can you find me a [object]'")
    
    while is_recording:
        data = stream.read(CHUNK)
        audio_queue.put(data)
    
    stream.stop_stream()
    stream.close()
    audio.terminate()
    print("* Recording stopped")

def process_audio():
    """Process audio chunks from the queue and transcribe"""
    global current_object_to_find
    
    while is_recording:
        if not audio_queue.empty():
            # Collect chunks for RECORD_SECONDS
            frames = []

            for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
                if audio_queue.empty():
                    break
                frames.append(audio_queue.get())
            
            if frames:
                # Convert frames to numpy array
                audio_data = np.frombuffer(b''.join(frames), dtype=np.int16).astype(np.float32) / 32768.0
                
                
                # Transcribe with Whisper
                result = whisper_model.transcribe(audio_data, language=LANGUAGE)
                text = result["text"].strip()
                
                if text:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] {text}")
                    
                    # Extract object name using regex
                    match = re.search(r'find (?:me |)(?:a |an |the |)(.+)', text)
                    if match:
                        object_to_find = match.group(1)
                        print(f"Looking for: {object_to_find}")
                        current_object_to_find = object_to_find  # Update global variable

def run_object_detection():
    """Run YOLO-World detection on webcam feed"""
    cap = cv2.VideoCapture(2)

    # Create a Supervision Box Annotator instance
    box_annotator = sv.BoxAnnotator()
    
    while is_recording:
        success, frame = cap.read()
        if not success:
            break
            
        if current_object_to_find:
            results = yolo_model.predict(frame, classes=[current_object_to_find])  # Remove `classes`
            
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates
                    label = result.names[int(box.cls[0])]   # Get label
                    score = box.conf[0].item()              # Get confidence score

                    if score > 0.3:  # Threshold for valid detections
                        # Use Supervision to annotate the bounding boxes
                        frame = box_annotator.annotate(
                            scene=frame, 
                            boxes=[(x1, y1, x2, y2)], 
                            labels=[f"{label}: {score:.2f}"], 
                            color=(0, 255, 0), 
                            thickness=2
                        )
            
            # Show the object we're looking for
            cv2.putText(frame, f"Looking for: {current_object_to_find}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Display the resulting frame
        cv2.imshow('YOLO-World Object Detection', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

# Start recording in a separate thread
recording_thread = threading.Thread(target=record_audio)
recording_thread.start()

# Start audio processing in another thread
processing_thread = threading.Thread(target=process_audio)
processing_thread.start()

# Run object detection in the main thread
try:
    run_object_detection()
except KeyboardInterrupt:
    print("Stopping...")
    is_recording = False
    recording_thread.join()
    processing_thread.join()
    audio.terminate()