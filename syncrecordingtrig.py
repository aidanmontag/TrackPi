import time
import threading
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from datetime import datetime
import os
import cv2

# GPIO setup
BUTTON_PIN = 16
LED_PIN = 5
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LED_PIN, GPIO.OUT)

# Events to control threading
capture_event = threading.Event()
stop_event = threading.Event()  # Shared stop signal

# Create a directory for saving the images
output_dir = "/home/aidan/Documents/PiTrack/output"
os.makedirs(output_dir, exist_ok=True)

# Shared variables for frame counting
frame_count_lock = threading.Lock()
total_frames = 0  # Total frames captured
start_time = None  # Start time for recording


class TimeTriggerThread(threading.Thread):
    """
    A thread that sends a trigger event every 1/24th of a second.
    """

    def __init__(self, trigger_event, stop_event):
        super().__init__()
        self.trigger_event = trigger_event
        self.stop_event = stop_event

    def run(self):
        while not self.stop_event.is_set():
            self.trigger_event.set()
            #print('trigger')
            time.sleep(1 / 24)
            self.trigger_event.clear()


class CameraCaptureThread(threading.Thread):
    """
    A thread that waits for a trigger event and captures a frame when the event is set.
    """

    def __init__(self, camera, trigger_event, stop_event, output_dir, camera_name):
        super().__init__()
        self.camera = camera
        self.trigger_event = trigger_event
        self.stop_event = stop_event
        self.output_dir = output_dir
        self.camera_name = camera_name

    def run(self):
        global total_frames
        frame_count = 0  # Thread-specific frame count
        while not self.stop_event.is_set():
            if self.trigger_event.wait(timeout=0.1):  # Timeout ensures periodic checking for stop_event
                #timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                frame = self.camera.capture_array()
                #yuv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
                #y_channel, u_channel, v_channel = cv2.split(yuv_frame)
                filename = os.path.join(
                    self.output_dir, f"{self.camera_name}_{frame_count:04d}.jpg"
                )
                cv2.imwrite(filename, frame)
                frame_count += 1
                with frame_count_lock:  # Safely update the global frame count
                    total_frames += 1
                #print('succesful captre', self.camera_name)
                self.trigger_event.clear()


def main():
    global total_frames, start_time

    # Create directories for output
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    left_dir = os.path.join(output_dir, timestamp, "left")
    right_dir = os.path.join(output_dir, timestamp, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    # Initialize cameras
    camera1 = Picamera2(camera_num=0)
    camera2 = Picamera2(camera_num=1)
    # Set the resolution to quarter resolution (1640x1232)
    resolution = (1640, 1232)
    camera1.configure(camera1.create_video_configuration(main={"size": resolution, "format": "RGB888"}))
    camera2.configure(camera2.create_video_configuration(main={"size": resolution, "format": "RGB888"}))

    camera_controls = {
        "ExposureTime": 10000,
        "AnalogueGain": 4.0,
        "AeEnable": False,
        "AwbEnable": False,
    }
    camera1.set_controls(camera_controls)
    camera2.set_controls(camera_controls)
    GPIO.output(LED_PIN, GPIO.LOW)
    

    print("Waiting for button press...")
    while GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        time.sleep(0.1)
    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
        time.sleep(0.1)

    # Start recording
    GPIO.output(LED_PIN, GPIO.HIGH)
    camera1.start()
    camera2.start()
    print('begin recording')

    time.sleep(1)

    # Start threads
    time_thread = TimeTriggerThread(capture_event, stop_event)
    capture_thread1 = CameraCaptureThread(camera1, capture_event, stop_event, left_dir, "left")
    capture_thread2 = CameraCaptureThread(camera2, capture_event, stop_event, right_dir, "right")
    start_time = time.perf_counter()
    capture_thread1.start()
    capture_thread2.start()
    time.sleep(.1)
    time_thread.start()

    # Wait for button release
    while GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        time.sleep(0.1)

    print("Button released, stopping...")
    stop_event.set()  # Signal threads to stop
    end_time = time.perf_counter()
    GPIO.output(LED_PIN, GPIO.LOW)

    time_thread.join()
    capture_thread1.join()
    capture_thread2.join()

    camera1.stop()
    camera2.stop()
    GPIO.cleanup()

    # Calculate average framerate
    total_time = end_time - start_time
    average_fps = (total_frames / 2) / total_time if total_time > 0 else 0

    print(f"Recording finished.")
    print(f"Total frames recorded: {total_frames}")
    print(f"Total recording time: {total_time:.2f} seconds")
    print(f"Average framerate: {average_fps:.2f} FPS")


if __name__ == "__main__":
    main()