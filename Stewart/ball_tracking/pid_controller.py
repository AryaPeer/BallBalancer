import numpy as np
import math
import serial
import time
import struct
import queue

class PIDController:
    def __init__(self, config):
        self.config = config

        self.Kp_x = 1.5
        self.Ki_x = 0.4
        self.Kd_x = 2.5

        self.Kp_y = 1.5
        self.Ki_y = 0.4
        self.Kd_y = 2.5

        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.integral = [0.0, 0.0]
        self.prev_error = [0.0, 0.0]

        self.servo_port = "COM5"
        self.servo = None
        self.min_servo_angle = -30
        self.max_servo_angle = 30

    def update(self, position, dt):
        error_x = (position[0] - self.setpoint_x)
        error_y = (position[1] - self.setpoint_y)
        print("Actual Errors:", error_x, error_y)

        error_x = 10.0 * error_x
        error_y = 10.0 * error_y

        MAX_ERROR = 0.5
        error_x = np.clip(error_x, -MAX_ERROR, MAX_ERROR)
        error_y = np.clip(error_y, -MAX_ERROR, MAX_ERROR)

        Px = self.Kp_x * error_x
        self.integral[0] += error_x * dt
        Ix = self.Ki_x * self.integral[0]
        derivative_x = (error_x - self.prev_error[0]) / dt
        Dx = self.Kd_x * derivative_x

        Py = self.Kp_y * error_y
        self.integral[1] += error_y * dt
        Iy = self.Ki_y * self.integral[1]
        derivative_y = (error_y - self.prev_error[1]) / dt
        Dy = self.Kd_y * derivative_y

        self.prev_error = [error_x, error_y]

        output_x = Px + Ix + Dx
        output_y = Py + Iy + Dy
        print("PID Outputs (x,y):", output_x, output_y)

        theta = math.degrees(math.atan2(output_y, output_x))
        if theta < 0:
            theta += 360
        print("Theta (degrees):", theta)

        phi = math.sqrt(output_x**2 + output_y**2)
        print("Raw phi:", phi)
        phi = np.clip(phi, 0, 15)
        print("Clipped phi:", phi)

        return theta, phi

    def reset_integral(self):
        self.integral = [0.0, 0.0]
        print("[RESET] Integral terms reset")

    def connect_servo(self):
        try:
            self.servo = serial.Serial(self.servo_port, 115200)
            time.sleep(1)
            print("[SERVO] Connected")
            return True
        except Exception as e:
            raise SystemExit("Servo connection failed: " + str(e))

    def send_servo_angle(self, angle1, angle2, angle3):
        if self.servo:
            try:
                angle1 = int(np.clip(angle1, self.min_servo_angle, self.max_servo_angle))
                angle2 = int(np.clip(angle2, self.min_servo_angle, self.max_servo_angle))
                angle3 = int(np.clip(angle3, self.min_servo_angle, self.max_servo_angle))

                checksum = (angle1 + angle2 + angle3) & 0xFF
                packet = struct.pack('BbbbB', 0xAA, angle1, angle2, angle3, checksum)
                self.servo.write(packet)

                print(f"[SERVO] Sent: {packet}")

            except Exception as e:
                print(f"[SERVO] Send failed: {e}")

    def control_thread(self, position_queue, running_flag, kinematics, gui, start_time):
        if not self.connect_servo():
            print("[ERROR] No servo - running in simulation mode")

        last_time = time.time() 

        while running_flag[0]:
            try:
                position_m = position_queue.get(timeout=0.1)

                print("This is the position of the ball (X,Y):", position_m)

                curr_time = time.time()
                dt = curr_time - last_time
                last_time = curr_time

                theta, phi = self.update(position_m, dt)

                print("Done updating PID")

                try:
                    motor_angle1, motor_angle2, motor_angle3 = kinematics.inverse_kinematics(theta, phi, kinematics.Pz)
                    self.send_servo_angle(motor_angle1, motor_angle2, motor_angle3)

                except ValueError as e:
                    print(f"[IK] Kinematics error: {e}")
                    continue

                current_time = time.time() - start_time[0]
                gui.log_data(current_time, position_m, [motor_angle1, motor_angle2, motor_angle3])

            except queue.Empty:
                continue
            except Exception as e:
                print(f"[CONTROL] Error: {e}")
                break

        if self.servo:
            self.send_servo_angle(0, 0, 0)
            self.servo.close()
