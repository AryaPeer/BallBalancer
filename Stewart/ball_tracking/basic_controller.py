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
        
        # PID gains for X axis (controlled by sliders in GUI)
        self.Kp_x = 1.5
        self.Ki_x = 0.4
        self.Kd_x = 2.5
        
        # PID gains for Y axis
        self.Kp_y = 1.5
        self.Ki_y = 0.4
        self.Kd_y = 2.5

        # Scale factors for converting from pixels to meters
        self.scale_factor_x = self.config['calibration']['pixel_to_meter_ratio_x'] * self.config['camera']['frame_width'] / 2
        self.scale_factor_y = self.config['calibration']['pixel_to_meter_ratio_y'] * self.config['camera']['frame_height'] / 2

        # Servo port name
        self.servo_port = "COM5"
        self.servo = None
        
        # Controller-internal state
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.integral = [0.0, 0.0]
        self.prev_error = [0.0, 0.0]
        
        # Stewart platform kinematics parameters
        self.L = [0.050, 0.080, 0.1085, 0.14056] 
        self.Pz = 0.0824
        
        # Data logs for plotting results
        self.time_log = []
        self.position_x_log = []
        self.position_y_log = []
        self.setpoint_x_log = []
        self.setpoint_y_log = []
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
            time.sleep(1)
            print("[SERVO] Connected")
            return True
        except Exception as e:
            SystemExit()
            return False

    #----------------------------------------------------------------------------------
    def send_servo_angle(self, angle1, angle2, angle3):
        """Send angle command to servo motor (clipped for safety)."""
        if self.servo:
            try:
                # Clip angles to safe range
                angle1 = int(np.clip(angle1, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE))
                angle2 = int(np.clip(angle2, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE))
                angle3 = int(np.clip(angle3, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE))

                # Calculate checksum
                checksum = (angle1 + angle2 + angle3) & 0xFF

                packet = struct.pack('BbbbB', 0xAA, angle1, angle2, angle3, checksum)
                self.servo.write(packet)

                print(f"[SERVO] Sent: {packet}")

            except Exception as e:
                print(f"[SERVO] Send failed: {e}")

    #----------------------------------------------------------------------------------
    def update_pid(self, position, dt=0.033):
        """Perform PID calculation and return control output (theta, phi)."""
        
        # Compute errors
        error_x = (position[0] - self.setpoint_x)
        error_y = (position[1] - self.setpoint_y)
        print ("Actual Errors:", error_x, error_y)

        error_x = 10.0 * error_x
        error_y = 10.0 * error_y

        # Clamp errors to prevent impossible positions
        MAX_ERROR = 0.5
        error_x = np.clip(error_x, -MAX_ERROR, MAX_ERROR)
        error_y = np.clip(error_y, -MAX_ERROR, MAX_ERROR)

        # X-axis PID
        Px = self.Kp_x * error_x

        self.integral[0] += error_x * dt
        Ix = self.Ki_x * self.integral[0]

        derivative_x = (error_x - self.prev_error[0]) / dt
        Dx = self.Kd_x * derivative_x
        
        # Y-axis PID (same fixes)
        Py = self.Kp_y * error_y
        
        self.integral[1] += error_y * dt
        Iy = self.Ki_y * self.integral[1]
        
        derivative_y = (error_y - self.prev_error[1]) / dt
        Dy = self.Kd_y * derivative_y
        
        # Update previous error
        self.prev_error = [error_x, error_y]

        # PID outputs
        output_x = Px + Ix + Dx
        output_y = Py + Iy + Dy
        print("PID Outputs (x,y):", output_x, output_y)

        theta = math.degrees(math.atan2(output_y, output_x))

        if theta < 0:
            theta += 360
        print ("Theta (degrees):", theta)

        phi = math.sqrt(output_x**2 + output_y**2)
        print("Raw phi:", phi)
        phi = np.clip(phi, 0, 15) 
        print("Clipped phi:", phi)

     #   print(f"Err: ({error_x:.2f}, {error_y:.2f}) | Out: ({output_x:.2f}, {output_y:.2f}) | θ={theta:.1f}°, φ={phi:.2f}°")

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
                position_mx = x_normalized * self.scale_factor_x
                position_my = y_normalized * self.scale_factor_y

                position_m = [position_mx, position_my]
                
                # Always keep latest measurement only
                try:
                    if self.position_queue.full():
                        self.position_queue.get_nowait()
                    self.position_queue.put_nowait(position_m)
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
                position_m = self.position_queue.get(timeout=0.1)
                
                print("This is the position of the ball (X,Y):", position_m)
                
                # Compute control output using PID
                theta, phi = self.update_pid(position_m)

                print("Done updating PID")

                # Convert to motor angles using inverse kinematics
                try:
                    motor_angle1, motor_angle2, motor_angle3 = self.inverse_kinematics(theta, phi, self.Pz)
                    
                    # Send control command to servo
                    self.send_servo_angle(motor_angle1, motor_angle2, motor_angle3)
                    
                except ValueError as e:
                    print(f"[IK] Kinematics error: {e}")
                    continue
                
                # Log results for plotting
                current_time = time.time() - self.start_time
                self.time_log.append(current_time)
                self.position_x_log.append(position_m[0])
                self.position_y_log.append(position_m[1])
                self.setpoint_x_log.append(self.setpoint_x)
                self.setpoint_y_log.append(self.setpoint_y)
                self.control_log.append([motor_angle1, motor_angle2, motor_angle3])

            except queue.Empty:
                continue
            
            except Exception as e:
                print(f"[CONTROL] Error: {e}")
                break

        if self.servo:
            # Return to neutral on exit
            self.send_servo_angle(0, 0, 0)
            self.servo.close()

    #----------------------------------------------------------------------------------
    def solve_quadratic(self, C, D, E, sign=1):
        """Solve quadratic equation Cx^2 + Dx + E = 0"""
        discriminant = D**2 - 4*C*E
        if discriminant < 0:
            raise ValueError(f"Negative discriminant: {discriminant}")
        return (-D + sign * math.sqrt(discriminant)) / (2*C)

#----------------------------------------------------------------------------------
    def compute_marker(self, n, Pz, L, leg):
        if leg == 'A':
            denom = math.sqrt(n[0]**2 + n[2]**2)
            x = (L[3] / denom) * n[2]
            y = 0
            z = Pz + (L[3] / denom) * (-n[0])
        elif leg == 'B':
            denom = math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*math.sqrt(3)*n[0]*n[1])
            x = (L[3] / denom) * (-n[2])
            y = (L[3] / denom) * (-math.sqrt(3) * n[2])
            z = Pz + (L[3] / denom) * (math.sqrt(3)*n[1] + n[0])
        elif leg == 'C':
            denom = math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*math.sqrt(3)*n[0]*n[1])
            x = (L[3] / denom) * (-n[2])
            y = (L[3] / denom) * (math.sqrt(3) * n[2])
            z = Pz + (L[3] / denom) * (-math.sqrt(3)*n[1] + n[0])
        else:
            raise ValueError("Invalid leg ID. Use 'A', 'B', or 'C'.")
        return [x, y, z]

    #----------------------------------------------------------------------------------
    def compute_theta(self, marker, L, Pmz, sign_y=1):
        """Compute theta angle for given marker position"""
        A = -(marker[0] + sign_y * math.sqrt(3) * marker[1] + 2*L[0]) / marker[2]
        B = (sum(m**2 for m in marker) + L[1]**2 - L[2]**2 - L[0]**2) / (2 * marker[2])
        C = A**2 + 4
        D = 2*A*B + 4*L[0]
        E = B**2 + L[0]**2 - L[1]**2
        
        x = self.solve_quadratic(C, D, E, sign=-1)
        y = sign_y * math.sqrt(3) * x
        
        disc_z = L[1]**2 - 4*x**2 - 4*L[0]*x - L[0]**2
        if disc_z < 0:
            raise ValueError(f"Invalid z discriminant: {disc_z}")

        z = math.sqrt(disc_z)
        if marker[2] < Pmz:
            z = -z

        theta = 90 - math.degrees(math.atan2(math.sqrt(x**2 + y**2) - L[0], z))
        return theta

    #----------------------------------------------------------------------------------
    def inverse_kinematics(self, theta, phi, Pz):
        """Inverse kinematic equations for converting platform tilt into motor angles"""
        
        # Convert angles to normal vector
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))
        n = [x, y, z]
        L = self.L
        L01 = L[0] + L[1]
        
        # Base point calculation
        A = L01 / Pz
        B = (Pz**2 + L[2]**2 - L01**2 - L[3]**2) / (2 * Pz)
        C = A**2 + 1
        D = 2 * (A * B - L01)
        E = B**2 + L01**2 - L[2]**2
        Pmx = self.solve_quadratic(C, D, E, sign=+1)
        
        discriminant = L[2]**2 - Pmx**2 + 2 * L01 * Pmx - L01**2
        if discriminant < 0:
            raise ValueError(f"Invalid Pmz discriminant: {discriminant}")
        Pmz = math.sqrt(discriminant)

        # Motor 1 (A marker)
        a_m = self.compute_marker(n, Pz, L, 'A')
        A = (L[0] - a_m[0]) / a_m[2]
        B = (sum(m**2 for m in a_m) - L[2]**2 - L[0]**2 + L[1]**2) / (2 * a_m[2])
        C = A**2 + 1
        D = 2 * (A * B - L[0])
        E = B**2 + L[0]**2 - L[1]**2
        
        ax = self.solve_quadratic(C, D, E, sign=1)
        discriminant = L[1]**2 - ax**2 + 2 * L[0] * ax - L[0]**2
        if discriminant < 0:
            raise ValueError(f"Invalid az discriminant: {discriminant}")
        
        az = math.sqrt(discriminant)
        if a_m[2] < Pmz:
            az = -az
        motor_angle1 = 90 - math.degrees(math.atan2(ax - L[0], az))

        # Motor 2 (B marker)
        b_m = self.compute_marker(n, Pz, L, 'B')
        motor_angle2 = self.compute_theta(b_m, L, Pmz, sign_y=1)

        # Motor 3 (C marker)
        c_m = self.compute_marker(n, Pz, L, 'C')
        motor_angle3 = self.compute_theta(c_m, L, Pmz, sign_y=-1)

        print(f"Motor angles: [{motor_angle1:.1f}°, {motor_angle2:.1f}°, {motor_angle3:.1f}°]")

        return motor_angle1, motor_angle2, motor_angle3


    #---------------------------------------------------------------------------------- 
    def create_gui(self):
        """Build Tkinter GUI with large sliders and labeled controls."""
        self.root = tk.Tk()
        self.root.title("Stewart Platform PID Controller")
        self.root.geometry("520x650")

        # X axis title
        ttk.Label(self.root, text="X Axis PID Gains", font=("Arial", 14, "bold")).pack(pady=5)

        # Kp_x slider
        ttk.Label(self.root, text="Kp_x (Proportional)", font=("Arial", 10)).pack()
        self.kp_x_var = tk.DoubleVar(value=self.Kp_x)
        kp_x_slider = ttk.Scale(self.root, from_=0, to=50, variable=self.kp_x_var,
                                orient=tk.HORIZONTAL, length=500)
        kp_x_slider.pack(pady=2)
        self.kp_x_label = ttk.Label(self.root, text=f"Kp_x: {self.Kp_x:.1f}")
        self.kp_x_label.pack()

        # Ki_x slider
        ttk.Label(self.root, text="Ki_x (Integral)", font=("Arial", 10)).pack()
        self.ki_x_var = tk.DoubleVar(value=self.Ki_x)
        ki_x_slider = ttk.Scale(self.root, from_=0, to=10, variable=self.ki_x_var,
                                orient=tk.HORIZONTAL, length=500)
        ki_x_slider.pack(pady=2)
        self.ki_x_label = ttk.Label(self.root, text=f"Ki_x: {self.Ki_x:.1f}")
        self.ki_x_label.pack()

        # Kd_x slider
        ttk.Label(self.root, text="Kd_x (Derivative)", font=("Arial", 10)).pack()
        self.kd_x_var = tk.DoubleVar(value=self.Kd_x)
        kd_x_slider = ttk.Scale(self.root, from_=0, to=20, variable=self.kd_x_var,
                                orient=tk.HORIZONTAL, length=500)
        kd_x_slider.pack(pady=2)
        self.kd_x_label = ttk.Label(self.root, text=f"Kd_x: {self.Kd_x:.1f}")
        self.kd_x_label.pack()

        # Separator
        ttk.Separator(self.root, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)
        
        # Y axis title
        ttk.Label(self.root, text="Y Axis PID Gains", font=("Arial", 14, "bold")).pack(pady=5)

        # Kp_y slider
        ttk.Label(self.root, text="Kp_y (Proportional)", font=("Arial", 10)).pack()
        self.kp_y_var = tk.DoubleVar(value=self.Kp_y)
        kp_y_slider = ttk.Scale(self.root, from_=0, to=50, variable=self.kp_y_var,
                                orient=tk.HORIZONTAL, length=500)
        kp_y_slider.pack(pady=2)
        self.kp_y_label = ttk.Label(self.root, text=f"Kp_y: {self.Kp_y:.1f}")
        self.kp_y_label.pack()

        # Ki_y slider
        ttk.Label(self.root, text="Ki_y (Integral)", font=("Arial", 10)).pack()
        self.ki_y_var = tk.DoubleVar(value=self.Ki_y)
        ki_y_slider = ttk.Scale(self.root, from_=0, to=10, variable=self.ki_y_var,
                                orient=tk.HORIZONTAL, length=500)
        ki_y_slider.pack(pady=2)
        self.ki_y_label = ttk.Label(self.root, text=f"Ki_y: {self.Ki_y:.1f}")
        self.ki_y_label.pack()

        # Kd_y slider
        ttk.Label(self.root, text="Kd_y (Derivative)", font=("Arial", 10)).pack()
        self.kd_y_var = tk.DoubleVar(value=self.Kd_y)
        kd_y_slider = ttk.Scale(self.root, from_=0, to=20, variable=self.kd_y_var,
                                orient=tk.HORIZONTAL, length=500)
        kd_y_slider.pack(pady=2)
        self.kd_y_label = ttk.Label(self.root, text=f"Kd_y: {self.Kd_y:.1f}")
        self.kd_y_label.pack()

        # Setpoints
        ttk.Separator(self.root, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)
        ttk.Label(self.root, text="Setpoints", font=("Arial", 14, "bold")).pack(pady=5)
        
        pos_min = -self.config.get('platform_diameter_m', 0.3) / 2
        pos_max = self.config.get('platform_diameter_m', 0.3) / 2

        # X setpoint
        ttk.Label(self.root, text="Setpoint X (meters)", font=("Arial", 10)).pack()
        self.setpoint_x_var = tk.DoubleVar(value=self.setpoint_x)
        setpoint_x_slider = ttk.Scale(self.root, from_=pos_min, to=pos_max,
                                     variable=self.setpoint_x_var,
                                     orient=tk.HORIZONTAL, length=500)
        setpoint_x_slider.pack(pady=2)
        self.setpoint_x_label = ttk.Label(self.root, text=f"X: {self.setpoint_x:.3f}m")
        self.setpoint_x_label.pack()

        # Y setpoint
        ttk.Label(self.root, text="Setpoint Y (meters)", font=("Arial", 10)).pack()
        self.setpoint_y_var = tk.DoubleVar(value=self.setpoint_y)
        setpoint_y_slider = ttk.Scale(self.root, from_=pos_min, to=pos_max,
                                     variable=self.setpoint_y_var,
                                     orient=tk.HORIZONTAL, length=500)
        setpoint_y_slider.pack(pady=2)
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
            # X axis gains
            self.Kp_x = self.kp_x_var.get()
            self.Ki_x = self.ki_x_var.get()
            self.Kd_x = self.kd_x_var.get()

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
        print("[RESET] Integral terms reset")

    #----------------------------------------------------------------------------------
    def plot_results(self):
        """Show matplotlib plots of position and control logs."""
        if not self.time_log:
            print("[PLOT] No data to plot")
            return
        
        fig, axes = plt.subplots(3, 1, figsize=(10, 10))

        # X position trace
        axes[0].plot(self.time_log, self.position_x_log, label="X Position", linewidth=2)
        axes[0].plot(self.time_log, self.setpoint_x_log, label="X Setpoint",
                     linestyle="--", linewidth=2)
        axes[0].set_ylabel("X Position (m)")
        axes[0].set_title("Stewart Platform PID Control")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # Y position trace
        axes[1].plot(self.time_log, self.position_y_log, label="Y Position", 
                     color='green', linewidth=2)
        axes[1].plot(self.time_log, self.setpoint_y_log, label="Y Setpoint",
                     linestyle="--", color='lime', linewidth=2)
        axes[1].set_ylabel("Y Position (m)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # Motor angles
        if self.control_log:
            control_array = np.array(self.control_log)
            axes[2].plot(self.time_log, control_array[:, 0], label="Motor 1", linewidth=2)
            axes[2].plot(self.time_log, control_array[:, 1], label="Motor 2", linewidth=2)
            axes[2].plot(self.time_log, control_array[:, 2], label="Motor 3", linewidth=2)
            axes[2].set_xlabel("Time (s)")
            axes[2].set_ylabel("Motor Angle (degrees)")
            axes[2].legend()
            axes[2].grid(True, alpha=0.3)
        
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
        print("[INFO] Starting Stewart Platform PID Controller")
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
        print("[ERROR] config.json not found. Run simple_cal.py first.")
        
    except Exception as e:
        print(f"[ERROR] {e}")
