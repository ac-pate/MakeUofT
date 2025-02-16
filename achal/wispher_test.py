import whisper
import serial
import numpy as np
import queue
import threading
import time
from datetime import datetime

# Load Whisper model
model = whisper.load_model("base", device="cpu")

# Audio parameters
RATE = 16000      # Must match Arduino's sample rate
CHUNK = 256       # Match Arduino's buffer size
RECORD_SECONDS = 1  # Process audio every second
LANGUAGE = "en"    # Set to None for auto-detection

# Open Serial connection to Arduino (adjust port as needed)
SERIAL_PORT = "COM6"  # Change to "COMX" on Windows
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Queue for audio chunks
audio_queue = queue.Queue()
is_recording = True

def read_serial_audio():
    """ Continuously read audio data from Arduino over Serial. """
    print("* Listening from Arduino Microphone...")

    while is_recording:
        try:
            # Read PCM data from Serial (size = CHUNK * 2 bytes per sample)
            raw_data = ser.read(CHUNK * 2)

            if len(raw_data) == CHUNK * 2:
                # Convert raw bytes to NumPy int16 array
                audio_data = np.frombuffer(raw_data, dtype=np.int16).astype(np.float32) / 32768.0
                audio_queue.put(audio_data)
        except Exception as e:
            print(f"Serial Read Error: {e}")

    print("* Stopping audio capture from Arduino.")

def process_audio():
    """ Process audio chunks from queue and transcribe """
    while is_recording:
        if not audio_queue.empty():
            frames = []
            for _ in range(int(RATE / CHUNK * RECORD_SECONDS)):
                if audio_queue.empty():
                    break
                frames.append(audio_queue.get())

            if frames:
                audio_data = np.concatenate(frames)  # Flatten array

                # Transcribe with Whisper
                result = model.transcribe(audio_data, language=LANGUAGE)

                if result["text"].strip():
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] {result['text']}")

# Start Serial reading in a separate thread
serial_thread = threading.Thread(target=read_serial_audio)
serial_thread.start()

# Start processing in the main thread
try:
    process_audio()
except KeyboardInterrupt:
    print("Stopping...")
    is_recording = False
    serial_thread.join()
    ser.close()
