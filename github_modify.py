import torch
import pyaudio
import numpy as np
from transformers import pipeline
import threading
import queue
import time


# Initialize the ASR pipeline
pipe = pipeline("automatic-speech-recognition",
                "openai/whisper-small",
                torch_dtype=torch.float16,
                device="cuda:0" if torch.cuda.is_available() else "cpu")

# Audio parameters
CHUNK = 16000  # 1 second of audio at 16kHz
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
SILENCE_THRESHOLD = 100  # Adjust based on your environment

# Initialize PyAudio
p = pyaudio.PyAudio()

# Audio queue and processing flag
audio_queue = queue.Queue()
is_running = True

def find_usb_mic():
    for i in range(p.get_device_count()):
        dev_info = p.get_device_info_by_index(i)
        if (dev_info.get('maxInputChannels') > 0 and 
            "USB" in dev_info.get('name', '')):
            return i
    # Fall back to default if no USB mic found
    return p.get_default_input_device_info()['index']

def audio_callback(in_data, frame_count, time_info, status):
    audio_data = np.frombuffer(in_data, dtype=np.int16)
    audio_level = np.abs(audio_data).mean()
    print(f"Audio level: {audio_level}")  # Debug print
    if audio_level > SILENCE_THRESHOLD:
        audio_queue.put(audio_data)
    return (in_data, pyaudio.paContinue)

def process_audio():
    buffer = []
    buffer_seconds = 2  # Process 2 seconds of audio at a time
    
    while is_running:
        try:
            data = audio_queue.get(timeout=1)
            print("Received audio chunk") 
            buffer.append(data)
            
            # Process when we have enough data
            if len(buffer) >= buffer_seconds:
                audio_segment = np.concatenate(buffer)
                
                # Convert to float32 and normalize
                float_data = audio_segment.astype(np.float32) / 32768.0
                
                # Test the pipeline with a dummy input
                dummy_input = np.zeros(16000, dtype=np.float32)
                test_result = pipe({"raw": dummy_input, "sampling_rate": RATE})
                print(f"Model test: {test_result}")

                # Transcribe
                result = pipe({"raw": float_data, "sampling_rate": RATE})
                if result["text"].strip():
                    print(f"Transcription: {result['text']}")
                
                # Slide the window (keep the last second)
                buffer = buffer[1:]
        except queue.Empty:
            pass

# Start stream
device_index = find_usb_mic()
device_index = 0
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=CHUNK,
                stream_callback=audio_callback)

print(f"Using microphone: {p.get_device_info_by_index(device_index)['name']}")
print("Listening... (Press Ctrl+C to stop)")

# Start processing thread
processing_thread = threading.Thread(target=process_audio)
processing_thread.daemon = True
processing_thread.start()

# Keep main thread alive
try:
    stream.start_stream()
    while stream.is_active():
        torch.cuda.empty_cache()  # Help prevent memory buildup
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    is_running = False
    stream.stop_stream()
    stream.close()
    p.terminate()
    processing_thread.join()
    print("\nStopped listening.")