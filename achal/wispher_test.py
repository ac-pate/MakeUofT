import whisper
import pyaudio
import numpy as np
import threading
import queue
import time
from datetime import datetime

# Load Whisper model
model = whisper.load_model("base", device="cpu")

# Audio parameters
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024
RECORD_SECONDS = 1  # Process audio in 5-second chunks
LANGUAGE = "en"  # Set to None for automatic language detection

# Initialize PyAudio
audio = pyaudio.PyAudio()

# Create a queue to hold audio chunks
audio_queue = queue.Queue()
is_recording = True

def record_audio():
    """Continuously record audio and put chunks in the queue"""
    stream = audio.open(format=FORMAT, channels=CHANNELS,
                        rate=RATE, input=True,
                        input_device_index=3,

                        frames_per_buffer=CHUNK)
    
    print("* Recording started")
    
    while is_recording:
        for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            if not is_recording:
                break
            data = stream.read(CHUNK)
            audio_queue.put(data)
    
    stream.stop_stream()
    stream.close()
    print("* Recording stopped")

def process_audio():
    """Process audio chunks from the queue and transcribe"""
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
                result = model.transcribe(audio_data, language=LANGUAGE)
                
                if result["text"].strip():
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] {result['text']}")

# Start recording in a separate thread
recording_thread = threading.Thread(target=record_audio)
recording_thread.start()

# Start processing in the main thread
try:
    process_audio()
except KeyboardInterrupt:
    print("Stopping...")
    is_recording = False
    recording_thread.join()
    audio.terminate()