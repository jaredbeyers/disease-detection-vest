import time
import board
import busio
import adafruit_mlx90640
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput
import os
import RPi.GPIO as GPIO


# ---------------- Buzzer setup ----------------
BUZZER_PIN = 12  # BCM pin number

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)


def buzz(duration=0.3, frequency=2000):
    """
    Sound buzzer for 'duration' seconds at approx 'frequency' Hz.
    Works for a passive piezo by manually toggling.
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
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
mlx = adafruit_mlx90640.MLX90640(i2c)

print("Found MLX90640 with serial:", [hex(i) for i in mlx.serial_number])

mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ

HEIGHT = 24
WIDTH = 32
PIXELS = HEIGHT * WIDTH
frame = [0.0] * PIXELS
center_index = (HEIGHT // 2) * WIDTH + (WIDTH // 2)


# ---------------- Human detection tuning ----------------
HUMAN_MIN_TEMP = 28.0   # °C
HUMAN_MAX_TEMP = 40.0
MIN_WARM_PIXELS = 10    # minimum for human presence


# ---------------- Camera setup ----------------
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration())
picam2.start()


def capture_photos(num_photos=12, fps=3):
    """
    Capture a sequence of photos using Picam2.

    Filenames are deterministic and reset every capture:
      /home/jared/OVImages/img_0000.jpg
      /home/jared/OVImages/img_0001.jpg
      ...
    Each new detection will overwrite the previous set.
    """
    save_dir = "/home/jared/OVImages"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        print(f"[thermal_module] Created folder {save_dir}")

    delay = 1.0 / fps
    print(f"[thermal_module] Capturing {num_photos} photos at {fps} fps...")

    for i in range(num_photos):
        filename = os.path.join(save_dir, f"img_{i:04d}.jpg")
        picam2.capture_file(filename)
        print(f"[thermal_module] Saved {filename}")
        time.sleep(delay)


def capture_video(duration=5, fps=10):
    """
    Record a single video clip of `duration` seconds.

    Saves to:
      /home/jared/OVVideos/human_detected.mp4

    The file is overwritten on each new detection.
    """
    save_dir = "/home/jared/OVVideos"
    os.makedirs(save_dir, exist_ok=True)
    filepath = os.path.join(save_dir, "human_detected.mp4")

    print(f"[thermal_module] Recording video: {filepath} ({duration}s @ ~{fps} fps)")

    # Stop still pipeline before reconfiguring
    try:
        picam2.stop()
    except Exception:
        # If not started yet, ignore
        pass

    # Configure for video
    video_config = picam2.create_video_configuration()
    picam2.configure(video_config)

    encoder = H264Encoder(bitrate=5_000_000)
    output = FfmpegOutput(filepath)

    picam2.start_recording(encoder, output)
    time.sleep(duration)
    picam2.stop_recording()

    # Reconfigure back to stills for future photo captures
    picam2.configure(picam2.create_still_configuration())
    picam2.start()

    print(f"[thermal_module] Finished recording video: {filepath}")


# --------------------------------------------------------
# MAIN FUNCTION YOU WILL CALL FROM PYTHON
# --------------------------------------------------------
def detect_human_and_capture(timeout=10.0, mode="photo"):
    """
    Watches thermal camera for up to `timeout` seconds.
    If human detected:
       - buzz
       - capture sequence of photos OR a video, depending on `mode`.

    mode:
        "photo" -> capture_photos(...)
        "video" -> capture_video(...)

    Returns: True if human found, False otherwise
    """
    print(f"[thermal_module] Scanning for human presence (mode={mode})...")

    end_time = time.time() + timeout

    while time.time() < end_time:
        try:
            mlx.getFrame(frame)

            max_temp = max(frame)
            min_temp = min(frame)
            avg_temp = sum(frame) / len(frame)

            warm_pixels = [
                t for t in frame
                if HUMAN_MIN_TEMP <= t <= HUMAN_MAX_TEMP
            ]
            warm_count = len(warm_pixels)

            human_detected = (
                warm_count >= MIN_WARM_PIXELS
                and HUMAN_MIN_TEMP <= max_temp <= HUMAN_MAX_TEMP
            )

            print(
                f"[thermal] Center: {frame[center_index]:.2f} C | "
                f"Min: {min_temp:.2f} C  Max: {max_temp:.2f} C  "
                f"Avg: {avg_temp:.2f} C | Warm px: {warm_count} | "
                f"{'HUMAN DETECTED' if human_detected else '...'}"
            )

            if human_detected:
                print(f"[thermal_module] Human detected! Buzzing + capturing ({mode})...")
                buzz(duration=0.4, frequency=2500)

                if mode == "photo":
                    capture_photos(num_photos=9, fps=3)
                elif mode == "video":
                    # adjust duration/fps as you like
                    capture_video(duration=3, fps=3)
                else:
                    print(f"[thermal_module] Unknown mode '{mode}', defaulting to photos.")
                    capture_photos(num_photos=9, fps=3)

                return True

        except (RuntimeError, ValueError) as e:
            print("[thermal_module] Thermal read error:", e)
            time.sleep(0.2)
            continue

        time.sleep(0.5)

    print("[thermal_module] No human detected within timeout.")
    return False


# --------------------------------------------------------
# ALLOW STANDALONE TESTING (optional)
# --------------------------------------------------------
if __name__ == "__main__":
    try:
        # Example: change mode to "video" to test video capture
        detect_human_and_capture(timeout=20, mode="photo")
    except KeyboardInterrupt:
        print("Exiting thermal module...")
    finally:
        GPIO.cleanup()
