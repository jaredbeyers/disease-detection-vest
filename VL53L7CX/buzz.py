import time
import board
import busio
import adafruit_mlx90640
from picamera2 import Picamera2
import os
import RPi.GPIO as GPIO

# ---------------- Buzzer setup ----------------
BUZZER_PIN = 12  # BCM pin number

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

def buzz(duration=0.3, frequency=2000):
    """
    Make the buzzer sound for 'duration' seconds at roughly 'frequency' Hz.
    Works for a passive piezo by toggling the pin quickly.
    """
    period = 1.0 / frequency
    half_period = period / 2.0
    end_time = time.time() + duration

    while time.time() < end_time:
        GPIO.output(BUZZER_PIN, GPIO.HIGH)
        time.sleep(half_period)
        GPIO.output(BUZZER_PIN, GPIO.LOW)
        time.sleep(half_period)

# ---------------- Thermal sensor setup ----------------
# I2C setup for thermal sensor
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)  # warning about freq is OK

mlx = adafruit_mlx90640.MLX90640(i2c)

print("Found MLX90640 with serial:", [hex(i) for i in mlx.serial_number])

# Start with a modest refresh rate to reduce "Too many retries" errors
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ  # 2 frames per second


# Sensor is 24 rows x 32 columns = 768 pixels
HEIGHT = 24
WIDTH = 32
PIXELS = HEIGHT * WIDTH
frame = [0.0] * PIXELS

# --- Human detection tuning parameters ---
HUMAN_MIN_TEMP = 28.0   # °C - lower bound for "warm body" pixels
HUMAN_MAX_TEMP = 40.0   # °C - upper bound to ignore very hot objects
MIN_WARM_PIXELS = 10    # how many warm pixels needed to call it a human (tune this)

center_index = (HEIGHT // 2) * WIDTH + (WIDTH // 2)

# ---------------- Camera setup ----------------
output_dir = "Images"
os.makedirs(output_dir, exist_ok=True)

picam2 = Picamera2()
config = picam2.create_still_configuration(
    main={"size": (1280, 720)}  # you can change this
)
picam2.configure(config)
picam2.start()
time.sleep(1)  # let the camera warm up

print("System initialized. Starting thermal sensing...")

def capture_photos(num_photos=15, fps=3):
    """Capture a sequence of photos at specified FPS"""
    interval = 1.0 / fps
    
    print(f"Person detected! Starting capture: {num_photos} photos at ~{fps} FPS")
    start_time = time.time()
    
    for i in range(num_photos):
        t0 = time.time()
        filename = os.path.join(output_dir, f"frame_{i:04d}.jpg")
        picam2.capture_file(filename)
        print(f"Captured {filename}")
        
        # Keep roughly consistent timing
        elapsed = time.time() - t0
        sleep_time = interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    total_time = time.time() - start_time
    print(f"Done. Captured {num_photos} frames in {total_time:.2f} seconds.")
    print("Returning to thermal sensing...\n")

try:
    # Main loop
    while True:
        try:
            # Grab one full frame of temperatures
            mlx.getFrame(frame)
            
            # Basic stats
            max_temp = max(frame)
            min_temp = min(frame)
            avg_temp = sum(frame) / len(frame)
            
            # Count pixels in "human-ish" range
            warm_pixels = [
                t for t in frame
                if HUMAN_MIN_TEMP <= t <= HUMAN_MAX_TEMP
            ]
            warm_count = len(warm_pixels)
            
            # Simple decision rule
            human_detected = (
                warm_count >= MIN_WARM_PIXELS and
                HUMAN_MIN_TEMP <= max_temp <= HUMAN_MAX_TEMP
            )
            
            # Print some debug info
            print(
                f"Center: {frame[center_index]:5.2f} C | "
                f"Min: {min_temp:5.2f} C  Max: {max_temp:5.2f} C  Avg: {avg_temp:5.2f} C | "
                f"Warm pixels: {warm_count:3d} | "
                f"{'HUMAN DETECTED' if human_detected else 'no human'}"
            )
            
            # If human detected, buzz (and optionally take photos)
            if human_detected:
                buzz(duration=0.4, frequency=2500)
                capture_photos()
                time.sleep(0.5)  # small delay before resuming thermal sensing
            
        except (RuntimeError, ValueError) as e:
            # MLX90640 sometimes glitches; just skip that frame
            print("Frame error:", e)
            time.sleep(0.2)
            continue
        
        # Slightly longer than 1 / 2 Hz = 0.5 s
        time.sleep(0.6)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    GPIO.cleanup()
