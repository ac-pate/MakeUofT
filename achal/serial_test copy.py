import serial
import numpy as np
import wave
import time
from scipy.signal import butter, lfilter

# configure serial connection with arduino
ser = serial.Serial('COM6', 9600)  # replace COM6 with your Arduino's serial port
time.sleep(2)  # allow time for the serial connection to initialize

# audio configuration
CHANNELS = 1
SAMPLE_WIDTH = 2  # 16-bit samples
SAMPLE_RATE = 16000  # 16 kHz sample rate (ensure Arduino is sending at this rate)
CHUNK_SIZE = 1024  # number of bytes to read at a time
RECORD_DURATION = 10  # duration to record in seconds
EXPECTED_SAMPLES = SAMPLE_RATE * RECORD_DURATION  # expected total samples

# butterworth filter parameters
CUTOFF_LOW = 5000  # low-pass cutoff frequency in Hz (remove high-frequency noise)
ORDER_LOW = 4  # low-pass filter order
CUTOFF_HIGH = 200  # high-pass cutoff frequency in Hz (remove low-frequency hum)
ORDER_HIGH = 2  # high-pass filter order

# noise gating threshold
NOISE_GATE_THRESHOLD = 300

# create butterworth filters for high-pass and low-pass filtering
nyquist_freq = 0.5 * SAMPLE_RATE
b_low, a_low = butter(ORDER_LOW, CUTOFF_LOW / nyquist_freq, btype='low')
b_high, a_high = butter(ORDER_HIGH, CUTOFF_HIGH / nyquist_freq, btype='high')

# function to amplify the audio data
def amplify(data, gain):
    return np.clip(data * gain, -32768, 32767)  # prevent overflow

# set up the output wav file
output_filename = "enhanced_recorded_audio.wav"
output_file = wave.open(output_filename, 'wb')
output_file.setnchannels(CHANNELS)
output_file.setsampwidth(SAMPLE_WIDTH)
output_file.setframerate(SAMPLE_RATE)

# start recording
print(f"Recording for {RECORD_DURATION} seconds...")

start_time = time.time()
buffer = []

while time.time() - start_time < RECORD_DURATION:
    if ser.in_waiting >= CHUNK_SIZE:  # ensure we have enough data
        raw_data = ser.read(CHUNK_SIZE)  # read chunk from serial
        samples = np.frombuffer(raw_data, dtype=np.int16)  # convert to numpy array

        # apply filters
        filtered_samples = lfilter(b_low, a_low, samples)
        filtered_samples = lfilter(b_high, a_high, filtered_samples)

        # apply noise gating
        filtered_samples[np.abs(filtered_samples) < NOISE_GATE_THRESHOLD] = 0

        # amplify
        amplified_samples = amplify(filtered_samples, 2)  # adjust gain

        # store processed samples in buffer
        buffer.extend(amplified_samples)

# ensure correct number of samples
buffer = np.array(buffer, dtype=np.int16)
if len(buffer) < EXPECTED_SAMPLES:
    # pad with silence if samples are missing
    buffer = np.pad(buffer, (0, EXPECTED_SAMPLES - len(buffer)), 'constant')
elif len(buffer) > EXPECTED_SAMPLES:
    # trim extra samples if necessary
    buffer = buffer[:EXPECTED_SAMPLES]

# write final buffer to wav file
output_file.writeframes(buffer.tobytes())
output_file.close()

print(f"Recording complete. Audio saved as {output_filename}")
