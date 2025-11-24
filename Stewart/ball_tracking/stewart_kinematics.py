import math

class StewartKinematics:
    def __init__(self):
        self.L = [0.050, 0.080, 0.1085, 0.14056]
        self.Pz = 0.1124

    def solve_quadratic(self, C, D, E, sign=1):
        discriminant = D**2 - 4*C*E
        if discriminant < 0:
            raise ValueError(f"Negative discriminant: {discriminant}")
        return (-D + sign * math.sqrt(discriminant)) / (2*C)

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

    def compute_theta(self, marker, L, Pmz, sign_y=1):
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

    def inverse_kinematics(self, theta, phi, Pz):
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))
        n = [x, y, z]
        L = self.L
        L01 = L[0] + L[1]

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

        b_m = self.compute_marker(n, Pz, L, 'B')
        motor_angle2 = self.compute_theta(b_m, L, Pmz, sign_y=1)

        c_m = self.compute_marker(n, Pz, L, 'C')
        motor_angle3 = self.compute_theta(c_m, L, Pmz, sign_y=-1)

        print(f"Motor angles: [{motor_angle1:.1f}°, {motor_angle2:.1f}°, {motor_angle3:.1f}°]")

        return motor_angle1, motor_angle2, motor_angle3
