import serial
import torch
import numpy as np
import io
import whisper

# Initialize Whisper model
model = whisper.load_model("base")  # Adjust based on the desired model size

# Serial setup for receiving data from Arduino
SERIAL_PORT = "COM6"  # Adjust the serial port as needed
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# Set up a buffer to hold the incoming data
buffer_size = 256  # Buffer size based on Arduino output
audio_buffer = np.zeros(buffer_size, dtype=np.int16)

def transcribe_audio_data(audio_data):
    """
    Process and transcribe the incoming audio data using Whisper.
    """
    # Convert the raw audio buffer to a format that Whisper can process
    audio_data = np.array(audio_data, dtype=np.float32) / 32768.0  # Normalize 16-bit to float
    audio_tensor = torch.from_numpy(audio_data).unsqueeze(0)  # Convert to a tensor
    
    # Transcribe using Whisper
    result = model.transcribe(audio_tensor)
    
    print(f"Transcription: {result['text']}")

def process_serial_data():
    """
    Continuously read serial data from the Arduino and process it.
    """
    while True:
        # Read a chunk of data from the serial connection
        if ser.in_waiting > 0:
            data = ser.readline().strip()
            try:
                # Parse the received data as comma-separated integers
                audio_samples = np.fromstring(data.decode(), sep=',', dtype=np.int16)
                
                # Pass the audio data to Whisper for transcription
                transcribe_audio_data(audio_samples)
            except ValueError:
                # Handle any parsing errors (e.g., incomplete or corrupt data)
                print("Error processing incoming data")

if __name__ == "__main__":
    process_serial_data()
