import asyncio
import struct
import math
import time
from bleak import BleakClient, BleakScanner

# BLE UUIDs given by eSense
SERVICE_UUID = "0000ff06-0000-1000-8000-00805f9b34fb"
CHAR_UUID = "0000ff08-0000-1000-8000-00805f9b34fb"
SAMPLE_UUID = "0000ff07-0000-1000-8000-00805f9b34fb"

# Device ID (using our specific eSense device)
DEVICE_NAME = "eSense-0718"
DEVICE_ADDRESS = "AD794E13-2F43-B11B-B88C-74AEFA6956EE"

# Calibration and filter state
set_init_angles = False
is_calibrated = False
counter = 0
angle_pitchz = angle_rollx = 0
angle_pitchz_offset = angle_rollx_offset = 0

# Complementary filter constants
f = 0.16
to = 0.02
rc = 1 / (2 * math.pi * f)
alpha = rc / (rc + to)

# Tilt detection configuration
SMALL_TILT_THRESHOLD = 5  # degrees for small tilt
BIG_TILT_THRESHOLD = 10   # degrees for significant tilt

class TiltDetector:
    def __init__(self):
        self.last_small_tilt = None
        self.last_big_tilt = None
        self.last_small_tilt_time = 0
        self.last_big_tilt_time = 0
        self.min_small_tilt_interval = 0.2  # Minimum time between small tilts
        self.min_big_tilt_interval = 0.5   # Minimum time between big tilts
        self.tilt_history = []  # Store recent tilt events

    def detect_tilt(self, roll, pitch):
        current_time = time.time()
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        tilt_result = None
        
        # Big left tilt
        if roll > BIG_TILT_THRESHOLD:
            if current_time - self.last_big_tilt_time >= self.min_big_tilt_interval:
                print(f"[{timestamp}] 👈 Big Left Tilt (Roll: {roll:.2f}°)")
                self.last_big_tilt = 'left'
                self.last_big_tilt_time = current_time
                tilt_result = 'big_left'

        # Big right tilt
        elif roll < -BIG_TILT_THRESHOLD:
            if current_time - self.last_big_tilt_time >= self.min_big_tilt_interval:
                print(f"[{timestamp}] 👉 Big Right Tilt (Roll: {roll:.2f}°)")
                self.last_big_tilt = 'right'
                self.last_big_tilt_time = current_time
                tilt_result = 'big_right'

        # Small left tilt
        elif SMALL_TILT_THRESHOLD <= roll <= BIG_TILT_THRESHOLD:
            if current_time - self.last_small_tilt_time >= self.min_small_tilt_interval:
                if self.last_small_tilt != 'left_small':
                    print(f"[{timestamp}] ➡️ Small Left Tilt (Roll: {roll:.2f}°)")
                    self.last_small_tilt = 'left_small'
                    self.last_small_tilt_time = current_time
                    tilt_result = 'small_left'

        # Small right tilt
        elif -BIG_TILT_THRESHOLD <= roll <= -SMALL_TILT_THRESHOLD:
            if current_time - self.last_small_tilt_time >= self.min_small_tilt_interval:
                if self.last_small_tilt != 'right_small':
                    print(f"[{timestamp}] ⬅️ Small Right Tilt (Roll: {roll:.2f}°)")
                    self.last_small_tilt = 'right_small'
                    self.last_small_tilt_time = current_time
                    tilt_result = 'small_right'

        # Reset when back to neutral
        if abs(roll) < SMALL_TILT_THRESHOLD:
            if self.last_small_tilt is not None:
                print(f"[{timestamp}] 📊 Neutral position")
            self.last_small_tilt = None

        # Record tilt in history if detected
        if tilt_result:
            self.tilt_history.append((tilt_result, current_time))
            # Keep only the last 10 tilt events
            if len(self.tilt_history) > 10:
                self.tilt_history.pop(0)
                
        return tilt_result

# Global tilt detector
tilt_detector = TiltDetector()

def process_imu(data):
    global set_init_angles, is_calibrated, counter
    global angle_pitchz, angle_rollx, angle_pitchz_offset, angle_rollx_offset

    # Parse 6 signed 16-bit ints from offset 4 (gyro & accel)
    gyrox, gyroy, gyroz, accx, accy, accz = struct.unpack_from(">hhhhhh", data, 4)

    # Convert to physical units
    accx /= 8192.0
    accy /= 8192.0
    accz /= 8192.0

    # Re-orient coordinates
    accx, accy = accy, -accx

    # Gyro to angles
    angle_roll_gyro = gyroy / 65.5
    angle_pitch_gyro = gyroz / 65.5

    # Accelerometer to angles
    angle_pitch_accz = -math.atan2(-accx, math.sqrt(accy ** 2 + accz ** 2)) * 57.296
    angle_roll_accx = -math.atan2(accz, accy) * 57.296

    if set_init_angles:
        angle_pitchz = ((angle_pitchz + angle_pitch_gyro * to) * alpha) + angle_pitch_accz * (1 - alpha)
        angle_rollx = ((angle_rollx + angle_roll_gyro * to) * alpha) + angle_roll_accx * (1 - alpha)
    else:
        angle_pitchz = angle_pitch_accz
        angle_rollx = angle_roll_accx
        set_init_angles = True

    if is_calibrated:
        pitch = angle_pitchz - angle_pitchz_offset
        roll = angle_rollx - angle_rollx_offset

        # Display current angles every 50 samples (for debugging)
        #if counter % 50 == 0:
           # print(f"📊 Roll: {roll:.2f}°, Pitch: {pitch:.2f}°")
            
        # Tilt detection
        tilt_detector.detect_tilt(roll, pitch)
        
        # Increment counter for periodic printing
        counter += 1
    else:
        angle_pitchz_offset += angle_pitchz
        angle_rollx_offset += angle_rollx
        counter += 1

        if counter % 50 == 0:
            print(f"⚙️ Calibrating... {counter}/250")

        if counter >= 250:
            angle_pitchz_offset /= counter
            angle_rollx_offset /= counter
            is_calibrated = True
            counter = 0  # Reset counter for the periodic display
            print("✅ Calibration complete")
            print(f"Pitch offset: {angle_pitchz_offset:.2f}°, Roll offset: {angle_rollx_offset:.2f}°")
            print("------------------------------------------")
            print("🚀 Tilt detection active! Keep your device still for neutral position.")
            print("------------------------------------------")

def imu_notify_handler(sender, data):
    process_imu(data)

async def connect_to_device():
    """Connect to the eSense device using the specific device address"""
    timestamp = time.strftime("%H:%M:%S", time.localtime()) 
    print(f"[{timestamp}] 🔍 Looking for {DEVICE_NAME} ({DEVICE_ADDRESS})...")
    
    # Try direct connection first
    try:
        print(f"[{timestamp}] 🔗 Attempting direct connection to {DEVICE_NAME}...")
        client = BleakClient(DEVICE_ADDRESS)
        await client.connect()
        print(f"[{timestamp}] ✅ Connected to {DEVICE_NAME}")
        return client
    except Exception as e:
        print(f"[{timestamp}] ❗ Direct connection failed: {e}")
    
    # Fall back to scanning
    print(f"[{timestamp}] 🔄 Scanning for nearby eSense devices...")
    devices = await BleakScanner.discover()
    target = None
    
    for d in devices:
        if DEVICE_NAME.lower() in d.name.lower() if d.name else False:
            print(f"[{timestamp}] 📡 Found device: {d.name} ({d.address})")
            target = d
            break
            
    if not target:
        # Try again with service UUID
        for d in devices:
            if SERVICE_UUID.lower() in [s.lower() for s in d.metadata.get("uuids", [])]:
                print(f"[{timestamp}] 📡 Found eSense device by service UUID: {d.name or 'Unknown'} ({d.address})")
                target = d
                break
    
    if not target:
        raise Exception("❌ No eSense device found.")
    
    client = BleakClient(target.address)
    await client.connect()
    print(f"[{timestamp}] ✅ Connected to {target.name or target.address}")
    return client

async def main():
    try:
        client = await connect_to_device()
        
        async with client:
            timestamp = time.strftime("%H:%M:%S", time.localtime())
            print(f"[{timestamp}] 🔌 Connection established")
            
            # Start IMU notifications
            await client.start_notify(CHAR_UUID, imu_notify_handler)
            print(f"[{timestamp}] 📊 Starting IMU data stream...")
            
            # Start IMU data stream - send command multiple times to ensure it's received
            sample_on = bytearray([0x53, 0x35, 0x02, 0x01, 0x32])
            for _ in range(10):
                await client.write_gatt_char(SAMPLE_UUID, sample_on)
                await asyncio.sleep(0.05)

            print(f"[{timestamp}] 📡 Streaming IMU data... (Ctrl+C to stop)")
            print(f"[{timestamp}] ⚙️ Starting calibration - keep device still...")
            
            # Keep the connection alive
            while True:
                await asyncio.sleep(0.1)
                
    except Exception as e:
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        print(f"[{timestamp}] ❌ Error: {e}")

if __name__ == "__main__":
    try:
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        print(f"[{timestamp}] 🎧 Starting eSense tilt detection program...")
        asyncio.run(main())
    except KeyboardInterrupt:
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        print(f"[{timestamp}] 🛑 Program stopped by user.")