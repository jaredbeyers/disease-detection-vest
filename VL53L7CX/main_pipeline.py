# main_pipeline.py
import time
from tof_wrapper import get_distance_m, shutdown as tof_shutdown
import thermal_module  # thermal_module.py handles thermal + buzzer + camera

THRESHOLD_M = 2.0  # meters

# Set this to "photo" or "video" to choose what happens on human detection
CAPTURE_MODE = "video"   # change to "video" when you want video clips instead


def main():
    try:
        while True:
            # 1) Get distance from the ToF sensor in meters
            dist = get_distance_m()

            if dist is None:
                print("[main] No valid ToF reading")
            else:
                print(f"[main] Avg distance: {dist:.3f} m")

                # 2) If something is closer than 2 meters, run the thermal check
                if dist < THRESHOLD_M:
                    print(
                        f"[main] Object closer than {THRESHOLD_M} m -> "
                        f"running thermal check (mode={CAPTURE_MODE})"
                    )

                    # Pass the selected mode down to the thermal module
                    human = thermal_module.detect_human_and_capture(
                        timeout=10.0,
                        mode=CAPTURE_MODE
                    )

                    if human:
                        if CAPTURE_MODE == "photo":
                            print("[main] Human detected and photo(s) captured.")
                        elif CAPTURE_MODE == "video":
                            print("[main] Human detected and video captured.")
                        else:
                            print("[main] Human detected and capture completed.")
                    else:
                        print("[main] No human detected within timeout.")

            # 3) Don't hammer the sensor too fast
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("[main] Exiting main loop...")

    finally:
        # Cleanly shut down ToF sensor
        tof_shutdown()
        # No GPIO cleanup here; thermal_module handles it only when run directly.


if __name__ == "__main__":
    main()
