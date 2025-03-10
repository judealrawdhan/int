#!/usr/bin/env python3
import threading
import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from picamera2.devices import IMX500
import numpy as np
import os

# ====== CONFIGURATION ======
PINS = {'red': 17, 'yellow': 27, 'green': 22}
FIRMWARE_PATH = "/usr/share/imx500-models/imx500_network_mobilenet_v2.rpk"
DETECTION_THRESHOLD = 0.7  # 70% confidence

# ====== SHARED STATE ======
ambulance_detected = False
system_lock = threading.Lock()

# ====== GPIO INITIALIZATION ======
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in PINS.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# ====== CAMERA FUNCTIONS ======
def initialize_camera():
    if not os.path.exists(FIRMWARE_PATH):
        print(f"❌ Missing firmware: {FIRMWARE_PATH}")
        print("Install with: sudo apt install imx500-firmware")
        return None, None
    
    try:
        imx500 = IMX500(FIRMWARE_PATH)
        picam2 = Picamera2(imx500.camera_num)
        config = picam2.create_preview_configuration()
        picam2.configure(config)
        return imx500, picam2
    except Exception as e:
        print(f"❌ Camera error: {str(e)}")
        return None, None

def detect_ambulance(imx500, picam2):
    global ambulance_detected
    picam2.start()
    try:
        while True:
            request = picam2.capture_request()
            outputs = imx500.get_outputs(request.get_metadata())
            
            if outputs:
                output = outputs[0]
                top_indices = np.argpartition(-output, 3)[:3]
                
                for idx in top_indices:
                    label = imx500.network_intrinsics.labels[idx].lower()
                    if "ambulance" in label and output[idx] > DETECTION_THRESHOLD:
                        with system_lock:
                            ambulance_detected = True
            
            request.release()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()

# ====== TRAFFIC LIGHT FUNCTIONS ======
def traffic_control():
    global ambulance_detected
    try:
        while True:
            with system_lock:
                current_state = ambulance_detected
                ambulance_detected = False  # Reset after checking
                
            if current_state:
                # Emergency sequence
                GPIO.output(PINS['green'], GPIO.HIGH)
                GPIO.output(PINS['yellow'], GPIO.LOW)
                GPIO.output(PINS['red'], GPIO.LOW)
                print("🚑 AMBULANCE PRIORITY: GREEN LIGHT")
                time.sleep(15)  # Hold green for 15 seconds
                
                # Transition back to normal
                GPIO.output(PINS['green'], GPIO.LOW)
                GPIO.output(PINS['yellow'], GPIO.HIGH)
                time.sleep(3)
                GPIO.output(PINS['yellow'], GPIO.LOW)
                GPIO.output(PINS['red'], GPIO.HIGH)
                time.sleep(15)  # Red phase before normal cycle
            else:
                # Normal cycle
                GPIO.output(PINS['green'], GPIO.HIGH)
                time.sleep(15)
                GPIO.output(PINS['green'], GPIO.LOW)
                
                GPIO.output(PINS['yellow'], GPIO.HIGH)
                time.sleep(3)
                GPIO.output(PINS['yellow'], GPIO.LOW)
                
                GPIO.output(PINS['red'], GPIO.HIGH)
                time.sleep(15)
                GPIO.output(PINS['red'], GPIO.LOW)
                
    except KeyboardInterrupt:
        GPIO.cleanup()

# ====== MAIN EXECUTION ======
if __name__ == "__main__":
    imx500, picam2 = initialize_camera()
    if not imx500:
        exit(1)

    # Start threads
    cam_thread = threading.Thread(target=detect_ambulance, args=(imx500, picam2), daemon=True)
    traffic_thread = threading.Thread(target=traffic_control, daemon=True)
    
    cam_thread.start()
    traffic_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🔴 Shutting down...")
        picam2.stop()
        GPIO.cleanup()
