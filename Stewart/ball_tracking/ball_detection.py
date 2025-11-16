import cv2
import numpy as np
import json
import os
import time

class BallDetector:
    def __init__(self, config_file="config.json"):
        self.lower_hsv = np.array([5, 150, 150], dtype=np.uint8)
        self.upper_hsv = np.array([20, 255, 255], dtype=np.uint8)
        self.scale_factor_x = 1.0
        self.scale_factor_y = 1.0
        self.config = None
        
        if os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    self.config = json.load(f)

                if 'ball_detection' in self.config:
                    if self.config['ball_detection']['lower_hsv']:
                        self.lower_hsv = np.array(self.config['ball_detection']['lower_hsv'], dtype=np.uint8)
                    if self.config['ball_detection']['upper_hsv']:
                        self.upper_hsv = np.array(self.config['ball_detection']['upper_hsv'], dtype=np.uint8)

                if 'calibration' in self.config:
                    self.scale_factor_x = self.config['calibration']['pixel_to_meter_ratio_x'] * self.config['camera']['frame_width'] / 2
                    self.scale_factor_y = self.config['calibration']['pixel_to_meter_ratio_y'] * self.config['camera']['frame_height'] / 2

                print(f"[BALL_DETECT] Loaded HSV bounds: {self.lower_hsv} to {self.upper_hsv}")
                print(f"[BALL_DETECT] Scale factors: X={self.scale_factor_x:.6f}, Y={self.scale_factor_y:.6f}")

            except Exception as e:
                print(f"[BALL_DETECT] Config load error: {e}, using defaults")
        else:
            print("[BALL_DETECT] No config file found, using default HSV bounds")

    def detect_ball(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return False, None, None, (0.0, 0.0)

        largest_contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

        if radius < 5 or radius > 100:
            return False, None, None, (0.0, 0.0)

        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2
        normalized_x = (x - center_x) / center_x
        normalized_y = (y - center_y) / center_y

        position_x_m = normalized_x * self.scale_factor_x
        position_y_m = normalized_y * self.scale_factor_y

        return True, (int(x), int(y)), radius, (position_x_m, position_y_m)

    def draw_detection(self, frame, show_info=True):
        found, center, radius, position_m = self.detect_ball(frame)
        overlay = frame.copy()
        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2

        cv2.line(overlay, (center_x, 0), (center_x, height), (255, 255, 255), 1)
        cv2.line(overlay, (0, center_y), (width, center_y), (255, 255, 255), 1)
        cv2.putText(overlay, "Center", (center_x + 5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if found:
            cv2.circle(overlay, center, int(radius), (0, 255, 0), 2)
            cv2.circle(overlay, center, 3, (0, 255, 0), -1)
            cv2.line(overlay, (center_x, center_y), center, (0, 255, 255), 1)
            
            if show_info:
                x_m, y_m = position_m
                cv2.putText(overlay, f"px: ({center[0]}, {center[1]})", (center[0] - 50, center[1] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                cv2.putText(overlay, f"x: {x_m:.4f}m", (center[0] - 40, center[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(overlay, f"y: {y_m:.4f}m", (center[0] - 40, center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                distance = np.sqrt(x_m**2 + y_m**2)
                cv2.putText(overlay, f"dist: {distance:.4f}m", (center[0] - 45, center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        return overlay, found, position_m

    def detect_ball_xy(self, frame):
        vis_frame, found, position_m = self.draw_detection(frame)

        if found:
            x_m, y_m = position_m
            x_normalized = x_m / self.scale_factor_x if self.scale_factor_x != 0 else 0.0
            y_normalized = y_m / self.scale_factor_y if self.scale_factor_y != 0 else 0.0
            x_normalized = np.clip(x_normalized, -1.0, 1.0)
            y_normalized = np.clip(y_normalized, -1.0, 1.0)
        else:
            x_normalized = 0.0
            y_normalized = 0.0

        return found, x_normalized, y_normalized, vis_frame

    def camera_thread(self, position_queue, running_flag):
        cap = cv2.VideoCapture(self.config['camera']['index'], cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        while running_flag[0]:
            ret, frame = cap.read()
            if not ret:
                continue

            frame = cv2.resize(frame, (320, 240))
            found, x_normalized, y_normalized, vis_frame = self.detect_ball_xy(frame)

            if found:
                curr_time = time.time()
                position_mx = x_normalized * self.scale_factor_x
                position_my = y_normalized * self.scale_factor_y
                position_m = [position_mx, position_my]

                try:
                    if position_queue.full():
                        position_queue.get_nowait()
                    position_queue.put_nowait((position_m, curr_time))
                except Exception:
                    pass

            cv2.imshow("Ball Tracking", vis_frame)
            if cv2.waitKey(1) & 0xFF == 27:
                running_flag[0] = False
                break

        cap.release()
        cv2.destroyAllWindows()