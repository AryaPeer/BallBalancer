import cv2
import numpy as np
import json
import serial
import time
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from threading import Thread
import queue
from ball_detection import detect_ball_xy
import math
import struct

MIN_SERVO_ANGLE = -30  # Min allowable sent servo angle
MAX_SERVO_ANGLE = 30   # Max allowable sent servo angle

DISPLAY_SIZE = (320, 240)  # Size for display window

class BasicPIDController:
    def __init__(self, config_file="config.json"):
        """Initialize controller, load config, set defaults and queues."""
        # Load experiment and hardware config from JSON file
        with open(config_file, 'r') as f:
            self.config = json.load(f)
        # PID gains (controlled by sliders in GUI)
        self.Kp = 6.0
        self.Ki = 1.5
        self.Kd = 10.0
        # Scale factor for converting from pixels to meters
        self.scale_factor_x = self.config['calibration']['pixel_to_meter_ratio'] * self.config['camera']['frame_width'] / 2
        self.scale_factor_y = self.config['calibration']['pixel_to_meter_ratio'] * self.config['camera']['frame_width'] / 2

        # Servo port name and center angle
        self.servo_port = self.config['servo']['port']
        self.neutral_angle = self.config['servo']['neutral_angle']
        self.servo = None
        # Controller-internal state
        self.setpoint = (0.0, 0.0)
        self.integral = [0.0, 0.0]
        self.prev_error = [0.0, 0.0]
        self.L = [0.15, 0.094, 0.080, 0.05] #[m]
        self.Pz = 0.0954
        # Data logs for plotting results
        self.time_log = []
        self.position_log = []
        self.setpoint_log = []
        self.control_log = []
        self.start_time = None
        # Thread-safe queue for most recent ball position measurement
        self.position_queue = queue.Queue(maxsize=1)
        self.running = False    # Main run flag for clean shutdown

    #----------------------------------------------------------------------------------
    def connect_servo(self):
        """Try to open serial connection to servo, return True if success."""
        try:
            self.servo = serial.Serial(self.servo_port, 115200)
            time.sleep(2)
            print("[SERVO] Connected")
            return True
        except Exception as e:
            print(f"[SERVO] Failed: {e}")
            return False

    #----------------------------------------------------------------------------------
    def send_servo_angle(self, angle1, angle2, angle3):
        """Send angle command to servo motor (clipped for safety)."""
        if self.servo:
            try:
                angle1 = int(np.clip(angle1, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE))
                angle2 = int(np.clip(angle2, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE))
                angle3 = int(np.clip(angle3, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE))

                checksum = (angle1 + angle2 + angle3) & 0xFF

                packet = struct.pack('BbbbB', 0xAA, angle1, angle2, angle3, checksum)
                self.servo.write(packet)
            except Exception:
                print("[SERVO] Send failed")

    #----------------------------------------------------------------------------------
    def update_pid(self, position, dt=0.033):
        """Perform PID calculation and return control output."""

        ### from ball position/error, find desired roll and pitch

        scale = 20

        error_x = (self.setpoint - position[0])*scale  # Compute x-error
        error_y = (self.setpoint - position[1])*scale  # Compute y-error

        # Proportional term
        Px = self.Kp * error_x
        Py = self.Kp * error_y

        # Integral term accumulation
        self.integral[0] += error_x * dt
        self.integral[1] += error_y * dt

        Ix = self.Ki * self.integral[0]
        Iy = self.Ki * self.integral[1]

        # Derivative term calculation
        derivative_x = (error_x - self.prev_error[0]) / dt
        derivative_y = (error_y - self.prev_error[1]) / dt

        Dx = self.Kd * derivative_x
        Dy = self.Kd * derivative_y

        self.prev_error = [error_x, error_y]

        # PID output (limit to safe beam range)
        output = [Px + Ix + Dx, Py+ Iy+ Dy]
        # output[0] = np.clip(output[0], -15, 15)
        # output[1] = np.clip(output[1], -15, 15)

        theta = math.degrees(math.atan2(output[1], output[0]))
        if theta < 0:
            theta += 360
        phi = math.sqrt(output[0]**2 + output[1]**2)

        print(error_x)
        print(error_y)

        return theta, phi

    #----------------------------------------------------------------------------------
    def camera_thread(self):
        """Dedicated thread for video capture and ball detection."""
        cap = cv2.VideoCapture(self.config['camera']['index'], cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        while self.running:
            ret, frame = cap.read()

            if not ret:
                continue

            frame = cv2.resize(frame, DISPLAY_SIZE)

            # Detect ball position in frame
            found, x_normalized, y_normalized, vis_frame = detect_ball_xy(frame)

            if found:
                # Convert normalized to meters using scale
                position_mx = x_normalized * self.scale_factor
                position_my = y_normalized * self.scale_factor

                position_norm = [position_mx, position_my]
                # Always keep latest measurement only
                try:
                    if self.position_queue.full():
                        self.position_queue.get_nowait()
                    self.position_queue.put_nowait(position_norm)
                except Exception:
                    pass

            # Show processed video with overlays
            cv2.imshow("Ball Tracking", vis_frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC exits
                self.running = False
                break

        cap.release()
        cv2.destroyAllWindows()

    #----------------------------------------------------------------------------------
    def control_thread(self):
        """Runs PID control loop in parallel with GUI and camera."""
        if not self.connect_servo():
            print("[ERROR] No servo - running in simulation mode")

        self.start_time = time.time()

        while self.running:
            try:
                # Wait for latest ball position from camera
                position_norm = self.position_queue.get(timeout=0.1)
                # Compute control output using PID

                theta, phi = self.update_pid(position_norm)


                # may need to place inverse kinematics function call/eqns here from control output to motor angles

                motor_angle1, motor_angle2, motor_angle3 = self.inverse_kinematics(theta, phi, self.Pz)

                # Send control command to servo (real or simulated)
                self.send_servo_angle(motor_angle1, motor_angle2, motor_angle3) # change for 3 motors and 2 outputs (x and y)
                # Log results for plotting
                current_time = time.time() - self.start_time

            except queue.Empty:
                continue
            
            except Exception as e:
                print(f"[CONTROL] Error: {e}")
                break

        if self.servo:
            # Return to neutral on exit
            self.send_servo_angle(0)
            self.servo.close()

    def solve_quadratic(self, C, D, E, sign=1):
        discriminant = D**2 - 4*C*E
        if discriminant < 0:
            raise ValueError("Negative discriminant")
        return (-D + sign * math.sqrt(discriminant)) / (2*C)

    def compute_marker(self, n, Pz, L, coeffs):
        denom = math.sqrt(n[0]**2 + coeffs[0]*n[1]**2 + coeffs[1]*n[2]**2 + coeffs[2]*n[0]*n[1])
        scale = L[3] / denom
        x = scale * coeffs[3] * n[2]
        y = scale * coeffs[4] * n[2]
        z = Pz + scale * (coeffs[5]*n[1] + coeffs[6]*n[0])
        return [x, y, z]

    def compute_theta(self, marker, L, Pmz, sign_y=1):
        A = -(marker[0] + sign_y * math.sqrt(3) * marker[1] + 2*L[0]) / marker[2]
        B = (sum(m**2 for m in marker) + L[1]**2 - L[2]**2 - L[0]**2) / (2 * marker[2])
        C = A**2 + 4
        D = 2*A*B + 4*L[0]
        E = B**2 + L[0]**2 - L[1]**2
        x = self.solve_quadratic(C, D, E, sign=-1)
        y = sign_y * math.sqrt(3) * x
        z = math.sqrt(L[1]**2 - 4*x**2 - 4*L[0]*x - L[0]**2)
        if marker[2] < Pmz:
            z = -z
        return 90 - math.degrees(math.atan2(math.sqrt(x**2 + y**2) - L[0], z))


    def inverse_kinematics(self, theta, phi, Pz):
        # inverse kinematic equations for converting platform tilt into motor angles
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r*math.cos(math.radians(theta))
        y = r*math.sin(math.radians(theta))
        n = [x, y, z]
        L = self.L
        L01 = L[0] + L[1]
        # Base point
        A = L01 / Pz
        B = (Pz**2 + L[2]**2 - L01**2 - L[3]**2) / (2 * Pz)
        C = A**2 + 1
        D = 2 * (A * B - L01)
        E = B**2 + L01**2 - L[2]**2
        Pmx = self.solve_quadratic(C, D, E, sign=-1)
        Pmz = math.sqrt(L[2]**2 - Pmx**2 + 2 * L01 * Pmx - L01**2)

        # A marker
        a_m = self.compute_marker(n, Pz, L, [0, 1, 0, 1, 0, 0, -1])
        A = (L[0] - a_m[0]) / a_m[2]
        B = (sum(m**2 for m in a_m) - L[2]**2 - L[0]**2 + L[1]**2) / (2 * a_m[2])
        C = A**2 + 1
        D = 2 * (A * B - L[0])
        E = B**2 + L[0]**2 - L[1]**2
        ax = self.solve_quadratic(C, D, E, sign=-1)
        az = math.sqrt(L[1]**2 - ax**2 + 2 * L[0] * ax - L[0]**2)
        if a_m[2] < Pmz:
            az = -az
        motor_angle1 = 90 - math.degrees(math.atan2(ax - L[0], az))

        # B and C markers
        b_m = self.compute_marker(n, Pz, L, [3, 4, 2 * math.sqrt(3), -1, -math.sqrt(3), math.sqrt(3), 1])
        motor_angle2 = self.compute_theta(b_m, L, Pmz, sign_y=1)

        c_m = self.compute_marker(n, Pz, L, [3, 4, -2 * math.sqrt(3), -1, math.sqrt(3), -math.sqrt(3), 1])
        motor_angle3 = self.compute_theta(c_m, L, Pmz, sign_y=-1)

        print("Motor angles:")
        print(motor_angle1)
        print(motor_angle2)
        print(motor_angle3)

        return motor_angle1, motor_angle2, motor_angle3

    #---------------------------------------------------------------------------------- 
    def create_gui(self):

        """Build Tkinter GUI with large sliders and labeled controls."""
        self.root = tk.Tk()
        self.root.title("Basic PID Controller")
        self.root.geometry("520x400")

        # Title label
        ttk.Label(self.root, text="X Axis PID Gains", font=("Arial", 16, "bold")).pack(pady=10)

        # Kp slider
        ttk.Label(self.root, text="Kp_x (Proportional)", font=("Arial", 11)).pack()
        self.kp_x_var = tk.DoubleVar(value=self.Kp_x)
        kp_x_slider = ttk.Scale(self.root, from_=0, to=100, variable=self.kp_x_var,
                                orient=tk.HORIZONTAL, length=500)
        kp_x_slider.pack(pady=3)
        self.kp_x_label = ttk.Label(self.root, text=f"Kp_x: {self.Kp_x:.1f}")
        self.kp_x_label.pack()

        # X axis Ki
        ttk.Label(self.root, text="Ki_x (Integral)", font=("Arial", 11)).pack()
        self.ki_x_var = tk.DoubleVar(value=self.Ki_x)
        ki_x_slider = ttk.Scale(self.root, from_=0, to=10, variable=self.ki_x_var,
                                orient=tk.HORIZONTAL, length=500)
        ki_x_slider.pack(pady=3)
        self.ki_x_label = ttk.Label(self.root, text=f"Ki_x: {self.Ki_x:.1f}")
        self.ki_x_label.pack()

        # X axis Kd
        ttk.Label(self.root, text="Kd_x (Derivative)", font=("Arial", 11)).pack()
        self.kd_x_var = tk.DoubleVar(value=self.Kd_x)
        kd_x_slider = ttk.Scale(self.root, from_=0, to=20, variable=self.kd_x_var,
                                orient=tk.HORIZONTAL, length=500)
        kd_x_slider.pack(pady=3)
        self.kd_x_label = ttk.Label(self.root, text=f"Kd_x: {self.Kd_x:.1f}")
        self.kd_x_label.pack()

        # Separator
        ttk.Separator(self.root, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        # Y axis title
        ttk.Label(self.root, text="Y Axis PID Gains", font=("Arial", 16, "bold")).pack(pady=10)

        # Y axis Kp
        ttk.Label(self.root, text="Kp_y (Proportional)", font=("Arial", 11)).pack()
        self.kp_y_var = tk.DoubleVar(value=self.Kp_y)
        kp_y_slider = ttk.Scale(self.root, from_=0, to=100, variable=self.kp_y_var,
                                orient=tk.HORIZONTAL, length=500)
        kp_y_slider.pack(pady=3)
        self.kp_y_label = ttk.Label(self.root, text=f"Kp_y: {self.Kp_y:.1f}")
        self.kp_y_label.pack()

        # Y axis Ki
        ttk.Label(self.root, text="Ki_y (Integral)", font=("Arial", 11)).pack()
        self.ki_y_var = tk.DoubleVar(value=self.Ki_y)
        ki_y_slider = ttk.Scale(self.root, from_=0, to=10, variable=self.ki_y_var,
                                orient=tk.HORIZONTAL, length=500)
        ki_y_slider.pack(pady=3)
        self.ki_y_label = ttk.Label(self.root, text=f"Ki_y: {self.Ki_y:.1f}")
        self.ki_y_label.pack()

        # Y axis Kd
        ttk.Label(self.root, text="Kd_y (Derivative)", font=("Arial", 11)).pack()
        self.kd_y_var = tk.DoubleVar(value=self.Kd_y)
        kd_y_slider = ttk.Scale(self.root, from_=0, to=20, variable=self.kd_y_var,
                                orient=tk.HORIZONTAL, length=500)
        kd_y_slider.pack(pady=3)
        self.kd_y_label = ttk.Label(self.root, text=f"Kd_y: {self.Kd_y:.1f}")
        self.kd_y_label.pack()

        # Setpoints
        ttk.Separator(self.root, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        pos_min = self.config['calibration']['position_min_m']
        pos_max = self.config['calibration']['position_max_m']

        # X setpoint
        ttk.Label(self.root, text="Setpoint X (meters)", font=("Arial", 11)).pack()
        self.setpoint_x_var = tk.DoubleVar(value=self.setpoint_x)
        setpoint_x_slider = ttk.Scale(self.root, from_=pos_min, to=pos_max,
                                     variable=self.setpoint_x_var,
                                     orient=tk.HORIZONTAL, length=500)
        setpoint_x_slider.pack(pady=3)
        self.setpoint_x_label = ttk.Label(self.root, text=f"X: {self.setpoint_x:.3f}m")
        self.setpoint_x_label.pack()

        # Y setpoint
        ttk.Label(self.root, text="Setpoint Y (meters)", font=("Arial", 11)).pack()
        self.setpoint_y_var = tk.DoubleVar(value=self.setpoint_y)
        setpoint_y_slider = ttk.Scale(self.root, from_=pos_min, to=pos_max,
                                     variable=self.setpoint_y_var,
                                     orient=tk.HORIZONTAL, length=500)
        setpoint_y_slider.pack(pady=3)
        self.setpoint_y_label = ttk.Label(self.root, text=f"Y: {self.setpoint_y:.3f}m")
        self.setpoint_y_label.pack()

        # Buttons
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=15)
        ttk.Button(button_frame, text="Reset Integrals",
                   command=self.reset_integral).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Plot Results",
                   command=self.plot_results).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Stop",
                   command=self.stop).pack(side=tk.LEFT, padx=5)

        self.update_gui()
        
    #----------------------------------------------------------------------------------
    def update_gui(self):
        """Reflect latest values from sliders into program and update display."""
        if self.running:
            # PID parameters
            self.Kp = self.kp_x_var.get()
            self.Ki = self.ki_x_var.get()
            self.Kd = self.kd_x_var.get()
            self.setpoint = self.setpoint_var.get()

            # Y axis gains
            self.Kp_y = self.kp_y_var.get()
            self.Ki_y = self.ki_y_var.get()
            self.Kd_y = self.kd_y_var.get()
            
            # Setpoints
            self.setpoint_x = self.setpoint_x_var.get()
            self.setpoint_y = self.setpoint_y_var.get()
            
            # Update labels
            self.kp_x_label.config(text=f"Kp_x: {self.Kp_x:.1f}")
            self.ki_x_label.config(text=f"Ki_x: {self.Ki_x:.1f}")
            self.kd_x_label.config(text=f"Kd_x: {self.Kd_x:.1f}")
            self.kp_y_label.config(text=f"Kp_y: {self.Kp_y:.1f}")
            self.ki_y_label.config(text=f"Ki_y: {self.Ki_y:.1f}")
            self.kd_y_label.config(text=f"Kd_y: {self.Kd_y:.1f}")
            self.setpoint_x_label.config(text=f"X: {self.setpoint_x:.3f}m")
            self.setpoint_y_label.config(text=f"Y: {self.setpoint_y:.3f}m")
            
            self.root.after(50, self.update_gui)

    #----------------------------------------------------------------------------------
    def reset_integral(self):
        """Clear integral error in PID (button handler)."""
        self.integral = [0.0, 0.0]
        print("[RESET] Integral term reset")

    #----------------------------------------------------------------------------------
    def plot_results(self):
        """Show matplotlib plots of position and control logs."""
        if not self.time_log:
            print("[PLOT] No data to plot")
            return
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

        # Ball position trace
        ax1.plot(self.time_log, self.position_log, label="Ball Position", linewidth=2)
        ax1.plot(self.time_log, self.setpoint_log, label="Setpoint",
                 linestyle="--", linewidth=2)
        ax1.set_ylabel("Position (m)")
        ax1.set_title(f"Basic PID Control (Kp={self.Kp:.1f}, Ki={self.Ki:.1f}, Kd={self.Kd:.1f})")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Control output trace
        ax2.plot(self.time_log, self.control_log, label="Control Output",
                 color="orange", linewidth=2)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Beam Angle (degrees)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()

    #----------------------------------------------------------------------------------
    def stop(self):
        """Stop everything and clean up threads and GUI."""
        self.running = False

        # Try to safely close all windows/resources
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    #----------------------------------------------------------------------------------
    def run(self):

        """Entry point: starts threads, launches GUI mainloop."""
        print("[INFO] Starting Basic PID Controller")
        print("Use sliders to tune PID gains in real-time")
        print("Close camera window or click Stop to exit")
        self.running = True

        # Start camera and control threads, mark as daemon for exit
        cam_thread = Thread(target=self.camera_thread, daemon=True)
        ctrl_thread = Thread(target=self.control_thread, daemon=True)
        cam_thread.start()
        ctrl_thread.start()

        # Build and run GUI in main thread
        self.create_gui()
        self.root.mainloop()

        # After GUI ends, stop everything
        self.running = False
        print("[INFO] Controller stopped")

#---------------------------------- MAIN ----------------------------------#
if __name__ == "__main__":
    try:
        controller = BasicPIDController()
        controller.run()

    except FileNotFoundError:
        print("[ERROR] config.json not found. Run simple_autocal.py first.")
        
    except Exception as e:
        print(f"[ERROR] {e}")
