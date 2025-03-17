import serial
import time

def send_pose_serial(serial_port, baud_rate, x, y, z, qx, qy, qz, qw):
    """
    Sends pose data over a serial port.

    Args:
        serial_port (str): The serial port to use (e.g., '/dev/ttyUSB0' or 'COM3').
        baud_rate (int): The baud rate of the serial connection.
        x (float): Position x.
        y (float): Position y.
        z (float): Position z.
        qx (float): Quaternion x.
        qy (float): Quaternion y.
        qz (float): Quaternion z.
        qw (float): Quaternion w.
    """
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        data_string = f"{x},{y},{z},{qx},{qy},{qz},{qw}\n"
        ser.write(data_string.encode('utf-8'))
        print(f"Sent: {data_string.strip()}")
        ser.close() #close the serial port after sending data.

    except serial.SerialException as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    serial_port = '/dev/ttyUSB0'  # Replace with your ESP32's serial port
    baud_rate = 115200

    # Example loop (replace with your data source)
    try:
        while True:
            # Example data (replace with your actual pose data)
            x = 1.0
            y = 2.0
            z = 3.0
            qx = 0.0
            qy = 0.707
            qz = 0.0
            qw = 0.707

            send_pose_serial(serial_port, baud_rate, x, y, z, qx, qy, qz, qw)
            time.sleep(0.1)  # Adjust the delay as needed

    except KeyboardInterrupt:
        print("Serial communication stopped by user.")
