import argparse
import wave
import struct
import os
import serial
import serial.tools.list_ports
import time
import numpy as np
from scipy.signal import butter, sosfilt, lfilter, freqz

# Default values
DEFAULT_BAUD = 9600
DEFAULT_LABEL = 'audio'
SAMPLE_RATE = 16000  # Adjust as needed
CHUNK_DURATION = 10  # Duration of each audio chunk in seconds

# Bandpass filter parameters
LOWCUT = 3000  # Lower cutoff frequency (Hz)
HIGHCUT = 5000  # Higher cutoff frequency (Hz)
ORDER = 4  # Filter order

# Normalize cutoff frequencies
LOWCUT_NORM = LOWCUT / (SAMPLE_RATE / 2)
HIGHCUT_NORM = HIGHCUT / (SAMPLE_RATE / 2)
sos = butter(ORDER, [LOWCUT_NORM, HIGHCUT_NORM], btype='band', output='sos')

# Parse arguments
parser = argparse.ArgumentParser(description="Serial Audio Data Collection")
parser.add_argument('-p', '--port', dest='port', type=str, required=True, help="Serial port to connect to")
parser.add_argument('-b', '--baud', dest='baud', type=int, default=DEFAULT_BAUD, help="Baud rate (default = " + str(DEFAULT_BAUD) + ")")
parser.add_argument('-d', '--directory', dest='directory', type=str, default=".", help="Output directory for files (default =.)")
parser.add_argument('-l', '--label', dest='label', type=str, default=DEFAULT_LABEL, help="Label for files (default = " + DEFAULT_LABEL + ")")

args = parser.parse_args()
port = args.port
baud = args.baud
out_dir = args.directory
label = args.label

# Spectral subtraction parameters
OVERSUBTRACTION_FACTOR = 1.0  # Adjust as needed
NOISE_SAMPLE_LENGTH = 1000  # Length of the noise sample for noise spectrum estimation

# Notch filter parameters
NOTCH_FREQ = 3.0  # Frequency to be notched (e.g., 60 Hz for power line noise)
NOTCH_Q = 1.0  # Quality factor for the notch filter

# Wiener filter parameters
WIENER_WINDOW_LENGTH = 256  # Length of the window for Wiener filtering

def spectral_subtraction(signal, noise_mean):
    """
    Apply spectral subtraction to a signal using a given noise spectrum.

    Args:
        signal (np.ndarray): Input signal.
        noise_mean (np.ndarray): Noise spectrum estimate.

    Returns:
        np.ndarray: Signal with noise reduced by spectral subtraction.
    """
    signal_spectrum = np.fft.fft(signal)
    signal_magnitude = np.abs(signal_spectrum)

    # Spectral subtraction
    subtracted_magnitude = signal_magnitude**2 - OVERSUBTRACTION_FACTOR * noise_mean**2
    subtracted_magnitude = np.maximum(subtracted_magnitude, 0)  # Ensure non-negative values
    subtracted_magnitude = np.sqrt(subtracted_magnitude)

    # Reconstruct the signal with the subtracted spectrum
    subtracted_spectrum = subtracted_magnitude * np.exp(1j * np.angle(signal_spectrum))
    enhanced_signal = np.fft.ifft(subtracted_spectrum).real

    return enhanced_signal

def notch_filter(signal, notch_freq, notch_q, sample_rate=SAMPLE_RATE):
    """
    Apply a notch filter to remove a specific frequency component from the signal.

    Args:
        signal (np.ndarray): Input signal.
        notch_freq (float): Frequency to be notched (in Hz).
        notch_q (float): Quality factor for the notch filter.
        sample_rate (int): Sample rate of the signal (default: SAMPLE_RATE).

    Returns:
        np.ndarray: Signal with the notched frequency component removed.
    """
    notch_freq_norm = notch_freq / (sample_rate / 2)  # Normalize frequency
    notch_bandwidth = notch_freq_norm / notch_q  # Calculate the bandwidth
    low_cutoff = notch_freq_norm - notch_bandwidth / 2
    high_cutoff = notch_freq_norm + notch_bandwidth / 2
    b_notch, a_notch = butter(ORDER, [low_cutoff, high_cutoff], btype='bandstop', analog=False, output='ba')

    filtered_signal = lfilter(b_notch, a_notch, signal)

    return filtered_signal

def wiener_filter(signal, window_length=WIENER_WINDOW_LENGTH):
    """
    Apply a Wiener filter to the signal for noise reduction.

    Args:
        signal (np.ndarray): Input signal.
        window_length (int): Length of the window for Wiener filtering (default: WIENER_WINDOW_LENGTH).

    Returns:
        np.ndarray: Signal with noise reduced by Wiener filtering.
    """
    window_length = min(window_length, len(signal))
    padded_signal = np.pad(signal, (window_length // 2, window_length // 2), mode='constant')

    filtered_signal = np.zeros_like(signal)
    for i in range(len(signal)):
        window = padded_signal[i:i + window_length]
        window_mean = np.mean(window)
        filtered_signal[i] = window_mean

    return filtered_signal

# Print out available serial ports
print()
print("Available serial ports:")
available_ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(available_ports):
    print(" {} : {} [{}]".format(port, desc, hwid))

# Parse arguments
args = parser.parse_args()
port = args.port
baud = args.baud
out_dir = args.directory
label = args.label

print(f"Connected to: {port} successfully")
# Configure serial port
ser = serial.Serial()
ser.port = port
ser.baudrate = baud

# Attempt to connect to the serial port
try:
    ser.open()
except Exception as e:
    print("ERROR:", e)
    exit()

# Make output directory
try:
    os.makedirs(out_dir)
except FileExistsError:
    pass

# Audio configuration
CHANNELS = 1
SAMPLE_WIDTH = 2  # 2 bytes per sample (16-bit)

try:
    while True:
        # Read data from the serial port for 10 seconds
        data_list = []
        start_time = time.time()
        while time.time() - start_time < CHUNK_DURATION:
            data_str = ser.readline().decode('utf-8').strip()
            if data_str:
                data_list.extend([int(x) for x in data_str.split(',') if x])

        # Apply bandpass filter
        data_filtered = sosfilt(sos, data_list)

        # Normalize the audio data
        data_normalized = (data_filtered - np.min(data_filtered)) / (np.max(data_filtered) - np.min(data_filtered))

        # Estimate the noise spectrum from the first few samples
        noise_sample = data_normalized[:NOISE_SAMPLE_LENGTH]
        noise_mean = np.mean(np.abs(np.fft.fft(noise_sample))**2)

        # Apply spectral subtraction
        data_enhanced = spectral_subtraction(data_normalized, noise_mean)

        # Apply notch filter
        data_enhanced = notch_filter(data_enhanced, NOTCH_FREQ, NOTCH_Q)

        # Apply Wiener filter
        #data_enhanced = wiener_filter(data_enhanced)

        # Pack the enhanced data into a byte string
        #data_bytes = struct.pack(f'<{len(data_enhanced)}h', *(int(sample * 32767) for sample in data_enhanced))

        # Clip the enhanced data to the valid range
        data_enhanced = np.clip(data_enhanced, -1, 1)

        # Pack the clipped data into a byte string
        data_bytes = struct.pack(f'<{len(data_enhanced)}h', *(int(sample * 32767) for sample in data_enhanced))

        # Get the current timestamp
        import datetime
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        # Create a unique filename for the WAV file
        import uuid
        uid = str(uuid.uuid4())[-12:]
        filename = f"{label}.{uid}.{timestamp}.wav"
        audio_path = os.path.join(out_dir, filename)

        # Open the WAV file and write the data
        with wave.open(audio_path, 'w') as wav_file:
            wav_file.setparams((CHANNELS, SAMPLE_WIDTH, SAMPLE_RATE, len(data_normalized), 'NONE', 'NONE'))
            wav_file.writeframes(data_bytes)

        print(f'Audio data saved to {audio_path}')

        # Calculate elapsed time for this recording
        elapsed_time = time.time() - start_time
        if elapsed_time < CHUNK_DURATION:
            time.sleep(CHUNK_DURATION - elapsed_time)  # Wait for remaining time if needed

except KeyboardInterrupt:
    pass

finally:
    # Close the serial port
    ser.close()