import serial
import time
import pyvjoy

# Set up Serial connection
# Note: Double check your COM port in Device Manager!
ser = serial.Serial("COM5", 115200, timeout=1)  

# Initialize VJoy Device 1
j = pyvjoy.VJoyDevice(1)

def map_range(value, in_min, in_max, out_min, out_max):
    try:
        # Constrain input to avoid errors if radio sends weird spikes <1000 or >2000
        if value < in_min: value = in_min
        if value > in_max: value = in_max
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    except:
        return out_min

print("Starting 6-Channel FlySky Bridge...")

while True:
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors='ignore').strip()
            
            if line.startswith("CH1:"):
                parts = line.split("|")
                
                # Ensure we received the full data packet before parsing
                if len(parts) >= 6:
                    # Parse Channels
                    ch1 = int(parts[0].split(":")[1].strip())  # Aileron
                    ch2 = int(parts[1].split(":")[1].strip())  # Elevator
                    ch3 = int(parts[2].split(":")[1].strip())  # Throttle
                    ch4 = int(parts[3].split(":")[1].strip())  # Rudder
                    ch5 = int(parts[4].split(":")[1].strip())  # Aux 1 (Switch)
                    ch6 = int(parts[5].split(":")[1].strip())  # Aux 2 (Switch)

                    # Map iBUS values (1000-2000) to VJoy range (1-32767)
                    x = map_range(ch1, 1000, 2000, 1, 32767)
                    y = map_range(ch2, 1000, 2000, 1, 32767)
                    throttle = map_range(ch3, 1000, 2000, 1, 32767)
                    rudder = map_range(ch4, 1000, 2000, 1, 32767)
                    aux1 = map_range(ch5, 1000, 2000, 1, 32767)
                    aux2 = map_range(ch6, 1000, 2000, 1, 32767)

                    # Send to VJoy
                    j.data.wAxisX = x          # Axis X
                    j.data.wAxisY = y          # Axis Y
                    j.data.wAxisZ = throttle   # Axis Z
                    j.data.wAxisXRot = rudder  # Axis Rx
                    j.data.wAxisYRot = aux1    # Axis Ry (New)
                    j.data.wAxisZRot = aux2    # Axis Rz (New)
                    
                    j.update()

                    # Print for debugging (Optional: comment out for speed)
                    print(f"A: {x} E: {y} T: {throttle} R: {rudder} S1: {aux1} S2: {aux2}")

    except Exception as e:
        print("Error:", e)
        time.sleep(0.1)
