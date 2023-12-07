import serial
import time

# Specify the serial port and baud rate
serial_port = '/dev/ttyACM0'  # Update this with the correct port for your device
baud_rate = 115200  # Update this with the baud rate of your device

while True:
    try:
        # Open the serial port
        ser = serial.Serial(serial_port, baud_rate)
        break  # If successful, exit the loop

    except serial.SerialException as e:
        print(f"Error: {e}")
        print("Retrying in 1 second...")
        time.sleep(1)

try:
    # Adding a delay after opening the port
    time.sleep(2)

    while True:
        # Read a line from the serial port
        line = ser.readline().decode('utf-8').strip()

        # Print the received data
        print(f"Received data: {line}")

except KeyboardInterrupt:
    # Close the serial port on keyboard interrupt (Ctrl+C)
    ser.close()
    print("Serial port closed.")
