import json
import time
import queue
from threading import Thread

from ball_detection import BallDetector
from pid_controller import PIDController
from stewart_kinematics import StewartKinematics
from stewart_gui import StewartGUI

class App:
    def __init__(self, config_file="config.json"):
        with open(config_file, 'r') as f:
            self.config = json.load(f)

        self.position_queue = queue.Queue(maxsize=1)
        self.running = [False]
        self.start_time = [None]

        self.ball_detector = BallDetector(config_file)
        self.pid = PIDController(self.config)
        self.kinematics = StewartKinematics()
        self.gui = StewartGUI(self.pid, self.config, self.stop, self.running)

    def stop(self):
        self.running[0] = False

    def run(self):
        print("[INFO] Starting Stewart Platform PID Controller")
        print("Use sliders to tune PID gains in real-time")
        print("Close camera window or click Stop to exit")
        self.running[0] = True
        self.start_time[0] = time.time()

        cam_thread = Thread(target=self.ball_detector.camera_thread, args=(self.position_queue, self.running), daemon=True)
        ctrl_thread = Thread(target=self.pid.control_thread, args=(self.position_queue, self.running, self.kinematics, self.gui, self.start_time), daemon=True)
        cam_thread.start()
        ctrl_thread.start()

        self.gui.create_gui()
        self.gui.run()

        self.running[0] = False
        print("[INFO] Controller stopped")

if __name__ == "__main__":
    try:
        app = App()
        app.run()
    except FileNotFoundError:
        print("[ERROR] config.json not found. Run simple_cal.py first.")
    except Exception as e:
        print(f"[ERROR] {e}")
