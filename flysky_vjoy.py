import serial
import time
import pyvjoy

# Set up Serial connection (change COM port as needed)
ser = serial.Serial("COM5", 115200, timeout=1)  # Change COM5 to your actual port

# Initialize VJoy
j = pyvjoy.VJoyDevice(1)

def map_range(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

while True:
    try:
        line = ser.readline().decode("utf-8").strip()
        if line.startswith("CH1:"):
            parts = line.split("|")
            ch1 = int(parts[0].split(":")[1].strip())  # Aileron
            ch2 = int(parts[1].split(":")[1].strip())  # Elevator
            ch3 = int(parts[2].split(":")[1].strip())  # Throttle
            ch4 = int(parts[3].split(":")[1].strip())  # Rudder

            # Map iBUS values (1000-2000) to VJoy range (1-32767)
            x = map_range(ch1, 1000, 2000, 1, 32767)
            y = map_range(ch2, 1000, 2000, 1, 32767)
            throttle = map_range(ch3, 1000, 2000, 1, 32767)
            rudder = map_range(ch4, 1000, 2000, 1, 32767)

            # Send to VJoy
            j.data.wAxisX = x  # Aileron
            j.data.wAxisY = y  # Elevator
            j.data.wAxisZ = throttle  # Throttle
            j.data.wAxisXRot = rudder  # Rudder
            j.update()

            print(f"Aileron: {x}, Elevator: {y}, Throttle: {throttle}, Rudder: {rudder}")

    except Exception as e:
        print("Error:", e)
        time.sleep(0.1)
