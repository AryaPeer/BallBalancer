import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
import numpy as np

class StewartGUI:
    def __init__(self, pid_controller, config, on_stop_callback):
        self.pid = pid_controller
        self.config = config
        self.on_stop_callback = on_stop_callback
        self.running = True

        self.time_log = []
        self.position_x_log = []
        self.position_y_log = []
        self.setpoint_x_log = []
        self.setpoint_y_log = []
        self.control_log = []

        self.root = None

    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("Stewart Platform PID Controller")
        self.root.geometry("520x650")

        ttk.Label(self.root, text="X Axis PID Gains", font=("Arial", 14, "bold")).pack(pady=5)

        ttk.Label(self.root, text="Kp_x (Proportional)", font=("Arial", 10)).pack()
        self.kp_x_var = tk.DoubleVar(value=self.pid.Kp_x)
        kp_x_slider = ttk.Scale(self.root, from_=0, to=10, variable=self.kp_x_var,
                                orient=tk.HORIZONTAL, length=500)
        kp_x_slider.pack(pady=2)
        self.kp_x_label = ttk.Label(self.root, text=f"Kp_x: {self.pid.Kp_x:.1f}")
        self.kp_x_label.pack()

        ttk.Label(self.root, text="Ki_x (Integral)", font=("Arial", 10)).pack()
        self.ki_x_var = tk.DoubleVar(value=self.pid.Ki_x)
        ki_x_slider = ttk.Scale(self.root, from_=0, to=15, variable=self.ki_x_var,
                                orient=tk.HORIZONTAL, length=500)
        ki_x_slider.pack(pady=2)
        self.ki_x_label = ttk.Label(self.root, text=f"Ki_x: {self.pid.Ki_x:.1f}")
        self.ki_x_label.pack()

        ttk.Label(self.root, text="Kd_x (Derivative)", font=("Arial", 10)).pack()
        self.kd_x_var = tk.DoubleVar(value=self.pid.Kd_x)
        kd_x_slider = ttk.Scale(self.root, from_=0, to=20, variable=self.kd_x_var,
                                orient=tk.HORIZONTAL, length=500)
        kd_x_slider.pack(pady=2)
        self.kd_x_label = ttk.Label(self.root, text=f"Kd_x: {self.pid.Kd_x:.1f}")
        self.kd_x_label.pack()

        ttk.Separator(self.root, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)

        ttk.Label(self.root, text="Y Axis PID Gains", font=("Arial", 14, "bold")).pack(pady=5)

        ttk.Label(self.root, text="Kp_y (Proportional)", font=("Arial", 10)).pack()
        self.kp_y_var = tk.DoubleVar(value=self.pid.Kp_y)
        kp_y_slider = ttk.Scale(self.root, from_=0, to=15, variable=self.kp_y_var,
                                orient=tk.HORIZONTAL, length=500)
        kp_y_slider.pack(pady=2)
        self.kp_y_label = ttk.Label(self.root, text=f"Kp_y: {self.pid.Kp_y:.1f}")
        self.kp_y_label.pack()

        ttk.Label(self.root, text="Ki_y (Integral)", font=("Arial", 10)).pack()
        self.ki_y_var = tk.DoubleVar(value=self.pid.Ki_y)
        ki_y_slider = ttk.Scale(self.root, from_=0, to=10, variable=self.ki_y_var,
                                orient=tk.HORIZONTAL, length=500)
        ki_y_slider.pack(pady=2)
        self.ki_y_label = ttk.Label(self.root, text=f"Ki_y: {self.pid.Ki_y:.1f}")
        self.ki_y_label.pack()

        ttk.Label(self.root, text="Kd_y (Derivative)", font=("Arial", 10)).pack()
        self.kd_y_var = tk.DoubleVar(value=self.pid.Kd_y)
        kd_y_slider = ttk.Scale(self.root, from_=0, to=20, variable=self.kd_y_var,
                                orient=tk.HORIZONTAL, length=500)
        kd_y_slider.pack(pady=2)
        self.kd_y_label = ttk.Label(self.root, text=f"Kd_y: {self.pid.Kd_y:.1f}")
        self.kd_y_label.pack()

        ttk.Separator(self.root, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)
        ttk.Label(self.root, text="Setpoints", font=("Arial", 14, "bold")).pack(pady=5)

        pos_min = -self.config.get('platform_diameter_m', 0.3) / 2
        pos_max = self.config.get('platform_diameter_m', 0.3) / 2

        ttk.Label(self.root, text="Setpoint X (meters)", font=("Arial", 10)).pack()
        self.setpoint_x_var = tk.DoubleVar(value=self.pid.setpoint_x)
        setpoint_x_slider = ttk.Scale(self.root, from_=pos_min, to=pos_max,
                                     variable=self.setpoint_x_var,
                                     orient=tk.HORIZONTAL, length=500)
        setpoint_x_slider.pack(pady=2)
        self.setpoint_x_label = ttk.Label(self.root, text=f"X: {self.pid.setpoint_x:.3f}m")
        self.setpoint_x_label.pack()

        ttk.Label(self.root, text="Setpoint Y (meters)", font=("Arial", 10)).pack()
        self.setpoint_y_var = tk.DoubleVar(value=self.pid.setpoint_y)
        setpoint_y_slider = ttk.Scale(self.root, from_=pos_min, to=pos_max,
                                     variable=self.setpoint_y_var,
                                     orient=tk.HORIZONTAL, length=500)
        setpoint_y_slider.pack(pady=2)
        self.setpoint_y_label = ttk.Label(self.root, text=f"Y: {self.pid.setpoint_y:.3f}m")
        self.setpoint_y_label.pack()

        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=15)
        ttk.Button(button_frame, text="Reset Integrals",
                   command=self.reset_integral).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Plot Results",
                   command=self.plot_results).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Stop",
                   command=self.stop).pack(side=tk.LEFT, padx=5)

        self.update_gui()

    def update_gui(self):
        if self.running:
            self.pid.Kp_x = self.kp_x_var.get()
            self.pid.Ki_x = self.ki_x_var.get()
            self.pid.Kd_x = self.kd_x_var.get()

            self.pid.Kp_y = self.kp_y_var.get()
            self.pid.Ki_y = self.ki_y_var.get()
            self.pid.Kd_y = self.kd_y_var.get()

            self.pid.setpoint_x = self.setpoint_x_var.get()
            self.pid.setpoint_y = self.setpoint_y_var.get()

            self.kp_x_label.config(text=f"Kp_x: {self.pid.Kp_x:.1f}")
            self.ki_x_label.config(text=f"Ki_x: {self.pid.Ki_x:.1f}")
            self.kd_x_label.config(text=f"Kd_x: {self.pid.Kd_x:.1f}")
            self.kp_y_label.config(text=f"Kp_y: {self.pid.Kp_y:.1f}")
            self.ki_y_label.config(text=f"Ki_y: {self.pid.Ki_y:.1f}")
            self.kd_y_label.config(text=f"Kd_y: {self.pid.Kd_y:.1f}")
            self.setpoint_x_label.config(text=f"X: {self.pid.setpoint_x:.3f}m")
            self.setpoint_y_label.config(text=f"Y: {self.pid.setpoint_y:.3f}m")

            self.root.after(50, self.update_gui)

    def reset_integral(self):
        self.pid.reset_integral()

    def plot_results(self):
        if not self.time_log:
            print("[PLOT] No data to plot")
            return

        fig, axes = plt.subplots(3, 1, figsize=(10, 10))

        axes[0].plot(self.time_log, self.position_x_log, label="X Position", linewidth=2)
        axes[0].plot(self.time_log, self.setpoint_x_log, label="X Setpoint",
                     linestyle="--", linewidth=2)
        axes[0].set_ylabel("X Position (m)")
        axes[0].set_title("Stewart Platform PID Control")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(self.time_log, self.position_y_log, label="Y Position",
                     color='green', linewidth=2)
        axes[1].plot(self.time_log, self.setpoint_y_log, label="Y Setpoint",
                     linestyle="--", color='lime', linewidth=2)
        axes[1].set_ylabel("Y Position (m)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

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

    def stop(self):
        self.running = False
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass
        if self.on_stop_callback:
            self.on_stop_callback()

    def run(self):
        self.root.mainloop()

    def log_data(self, current_time, position_m, motor_angles):
        self.time_log.append(current_time)
        self.position_x_log.append(position_m[0])
        self.position_y_log.append(position_m[1])
        self.setpoint_x_log.append(self.pid.setpoint_x)
        self.setpoint_y_log.append(self.pid.setpoint_y)
        self.control_log.append(motor_angles)
