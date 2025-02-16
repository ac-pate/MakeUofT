import serial

# Open the serial port (COM6 in this case)
ser = serial.Serial('COM6', 115200, timeout=1)

# Continuously read and print data from the Arduino
while True:
    line = ser.readline()  # Read a line of data from the Arduino
    if line:
        print(line.decode('utf-8').strip())  # Print the data, stripping any extra spaces or newlines
