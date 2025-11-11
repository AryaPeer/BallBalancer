# Simple Auto Calibration System for stewart
# Interactive calibration tool for color detection and geometry
# Generates config.json file for use with ball tracking controller

import cv2
import numpy as np
import json
import math
from datetime import datetime

class SimpleAutoCalibrator:
    """Interactive calibration system for ball and beam control setup."""
    
    #----------------------------------------------------------------------------------
    def __init__(self):
        """Initialize calibration parameters and default values."""
        # Physical system parameters
        self.PLATFORM_DIAMETER = 0.3  # Known platform diameter in meters
        
        # Camera configuration
        self.CAM_INDEX = 0 # Default camera index (1 for external USB camera)
        self.FRAME_W, self.FRAME_H = 700, 480  # Frame dimensions
        
        # Calibration state tracking
        self.current_frame = None  # Current video frame
        self.phase = "color"  # Current phase: "color", "geometry", "complete"
        
        # Color calibration data
        self.hsv_samples = []  # Collected HSV color samples
        self.lower_hsv = None  # Lower HSV bound for ball detection
        self.upper_hsv = None  # Upper HSV bound for ball detection
        
        # Geometry calibration data
        self.peg_points = []  # Platform endpoint pixel coordinates
        self.pixel_to_meter_ratio_x = None  # Conversion ratio from pixels to meters (x-axis)
        self.pixel_to_meter_ratio_y = None  # Conversion ratio from pixels to meters (y-axis)

    #----------------------------------------------------------------------------------
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse click events for interactive calibration.
        
        Args:
            event: OpenCV mouse event type
            x, y: Mouse click coordinates
            flags: Additional event flags
            param: User data (unused)
        """

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.phase == "color":
                # Color sampling phase - collect HSV samples at click point
                self.sample_color(x, y)
            elif self.phase == "geometry" and len(self.peg_points) < 4:
                # Geometry phase - collect platform x-y coordinates
                self.peg_points.append((x, y))
                print(f"[GEO] Peg {len(self.peg_points)} selected")
                if len(self.peg_points) == 4:
                    self.calculate_geometry()

    #----------------------------------------------------------------------------------
    def sample_color(self, x, y):
        """Sample HSV color values in a 5x5 region around click point.
        
        Args:
            x, y: Center coordinates for color sampling
        """
        if self.current_frame is None:
            return
        
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
        
        # Sample 5x5 region around click point
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                px, py = x + dx, y + dy
                # Check bounds and collect valid samples
                if 0 <= px < hsv.shape[1] and 0 <= py < hsv.shape[0]:
                    self.hsv_samples.append(hsv[py, px])
        
        # Update HSV bounds based on collected samples
        if self.hsv_samples:
            samples = np.array(self.hsv_samples)
            
            # Calculate adaptive margins for each HSV channel
            h_margin = max(5, (np.max(samples[:, 0]) - np.min(samples[:, 0])) * 0.1)
            s_margin = max(10, (np.max(samples[:, 1]) - np.min(samples[:, 1])) * 0.15)
            v_margin = max(10, (np.max(samples[:, 2]) - np.min(samples[:, 2])) * 0.15)
            
            # Set lower bounds with margin
            self.lower_hsv = [
                max(0, np.min(samples[:, 0]) - h_margin),
                max(0, np.min(samples[:, 1]) - s_margin),
                max(0, np.min(samples[:, 2]) - v_margin)
            ]
            
            # Set upper bounds with margin
            self.upper_hsv = [
                min(179, np.max(samples[:, 0]) + h_margin),
                min(255, np.max(samples[:, 1]) + s_margin),
                min(255, np.max(samples[:, 2]) + v_margin)
            ]
            
            print(f"[COLOR] Samples: {len(self.hsv_samples)}")

    #----------------------------------------------------------------------------------
    def calculate_geometry(self):
        """Calculate pixel-to-meter conversion ratio from platform endpoint coordinates."""
        p1, p2, p3, p4 = self.peg_points
        
        # Calculate pixel distance between platform endpoints
        # X-axis endpoints (p1 to p2)
        pixel_distance_x = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        # Y-axis endpoints (p3 to p4)
        pixel_distance_y = math.sqrt((p4[0] - p3[0])**2 + (p4[1] - p3[1])**2)

        
        # Convert to meters using known platform diameter
        self.pixel_to_meter_ratio_x = (self.PLATFORM_DIAMETER/2)/ pixel_distance_x
        self.pixel_to_meter_ratio_y = (self.PLATFORM_DIAMETER/2) / pixel_distance_y
        print(f"[GEO] Pixel-to-meter ratio: x = {self.pixel_to_meter_ratio_x:.6f}, y = {self.pixel_to_meter_ratio_y:.6f}")
        
        # Advance to complete phase
        self.phase = "complete"

    #----------------------------------------------------------------------------------
    def detect_ball_position(self, frame):
        """Detect ball in frame and return position in meters from center.
        
        Args:
            frame: Input BGR image frame

        Returns:
            list or None: [x_offset, y_offset] in meters from center, None if not detected
        """
        if not self.lower_hsv:
            return None
        
        # Convert to HSV and create color mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(self.lower_hsv, dtype=np.uint8)
        upper = np.array(self.upper_hsv, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        
        # Clean up mask with morphological operations
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find contours in mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        
        # Get largest contour (assumed to be ball)
        largest = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest)
        
        # Filter out very small detections
        if radius < 5:
            return None
        
        # Convert pixel position to meters from center
        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2

        pixel_offset_x = x - center_x
        pixel_offset_y = y - center_y

        meters_offset_x = pixel_offset_x * self.pixel_to_meter_ratio_x
        meters_offset_y = pixel_offset_y * self.pixel_to_meter_ratio_y

        meters_offset = [meters_offset_x, meters_offset_y]
        
        return meters_offset

    #----------------------------------------------------------------------------------
    def save_config(self):
        """Save all calibration results to config.json file."""
        config = {
            "timestamp": datetime.now().isoformat(),
            "platform_diameter_m": float(self.PLATFORM_DIAMETER),
            "camera": {
                "index": int(self.CAM_INDEX),
                "frame_width": int(self.FRAME_W),
                "frame_height": int(self.FRAME_H)
            },
            "ball_detection": {
                "lower_hsv": [float(x) for x in self.lower_hsv] if self.lower_hsv else None,
                "upper_hsv": [float(x) for x in self.upper_hsv] if self.upper_hsv else None
            },
            "calibration": {
                "pixel_to_meter_ratio_x": float(self.pixel_to_meter_ratio_x) if self.pixel_to_meter_ratio_x else None,
                "pixel_to_meter_ratio_y": float(self.pixel_to_meter_ratio_y) if self.pixel_to_meter_ratio_y else None,
                "peg_points": self.peg_points if self.peg_points else None
            }
        }
        
        # Write configuration to JSON file
        with open("config.json", "w") as f:
            json.dump(config, f, indent=2)
        print("[SAVE] Configuration saved to config.json")

    #----------------------------------------------------------------------------------
    def draw_overlay(self, frame):
        """Draw calibration status and instructions overlay on frame.
        
        Args:
            frame: Input BGR image frame
            
        Returns:
            numpy.ndarray: Frame with overlay graphics and text
        """
        overlay = frame.copy()
        
        # Phase-specific instruction text
        phase_text = {
            "color": "Click on ball to sample colors. Press 'c' when done.",
            "geometry": "Click on platform x endpoints and y endpoints (4 points)",
            "complete": "Calibration complete! Press 's' to save"
        }
        
        # Draw current phase and instructions
        cv2.putText(overlay, f"Phase: {self.phase}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, phase_text[self.phase], (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show color calibration progress
        if self.hsv_samples:
            cv2.putText(overlay, f"Color samples: {len(self.hsv_samples)}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Show geometry calibration points
        for i, peg in enumerate(self.peg_points):
            cv2.circle(overlay, peg, 8, (0, 255, 0), -1)
            cv2.putText(overlay, f"Peg {i+1}", (peg[0]+10, peg[1]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw lines between endpoints if all are selected
        if len(self.peg_points) == 4:
            cv2.line(overlay, self.peg_points[0], self.peg_points[1], (255, 0, 0), 2)
            cv2.line(overlay, self.peg_points[2], self.peg_points[3], (255, 0, 0), 2)

        
        # Show real-time ball detection if color calibration is complete
        if self.lower_hsv:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array(self.lower_hsv, dtype=np.uint8)
            upper = np.array(self.upper_hsv, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            
            # Clean up mask
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            # Find and draw detected ball
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest)
                if radius > 5:
                    # Draw detection circle
                    cv2.circle(overlay, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(overlay, (int(x), int(y)), 3, (0, 255, 255), -1)
                    
                    # Show position if geometry calibration is complete
                    if self.pixel_to_meter_ratio_x and self.pixel_to_meter_ratio_y:
                        pos = self.detect_ball_position(frame)
                        if pos is not None:
                            cv2.putText(overlay, f"X: {pos[0]:.4f}m, Y: {pos[1]:.4f}m",
                                       (int(x)+20, int(y)+20),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Show calibration ratios if complete
        if self.pixel_to_meter_ratio_x is not None and self.pixel_to_meter_ratio_y is not None:
            cv2.putText(overlay, f"Ratios - X: {self.pixel_to_meter_ratio_x:.6f}, Y: {self.pixel_to_meter_ratio_y:.6f}",
                       (10, overlay.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        return overlay

    #----------------------------------------------------------------------------------
    def run(self):
        """Main calibration loop with interactive GUI."""
        # Initialize camera capture
        self.cap = cv2.VideoCapture(self.CAM_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
        
        # Setup OpenCV window and mouse callback
        cv2.namedWindow("Auto Calibration")
        cv2.setMouseCallback("Auto Calibration", self.mouse_callback)
        
        # Display instructions
        print("[INFO] Simple Auto Calibration")
        print("Phase 1: Click on ball to sample colors, press 'c' when done")
        print("Phase 2: Click on platform endpoints (4 points)")
        print("Press 's' to save, 'q' to quit")
        
        # Main calibration loop
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            self.current_frame = frame
            
            # Draw overlay and display frame
            display = self.draw_overlay(frame)
            cv2.imshow("Auto Calibration", display)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                # Quit calibration
                break
            elif key == ord('c') and self.phase == "color":
                # Complete color calibration phase
                if self.hsv_samples:
                    self.phase = "geometry"
                    print("[INFO] Color calibration complete. Click on platform endpoints.")
            elif key == ord('s') and self.phase == "complete":
                # Save configuration and exit
                self.save_config()
                break
        
        # Clean up resources
        self.cap.release()
        cv2.destroyAllWindows()
            
#---------------------------------- MAIN ----------------------------------#
if __name__ == "__main__":
    """Run calibration when script is executed directly."""
    calibrator = SimpleAutoCalibrator()
    calibrator.run()