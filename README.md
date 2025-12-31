# Stewart Platform Ball Balancer

A closed-loop control system that balances a ball on a 3-DOF Stewart platform using computer vision and PID control.

## How It Works
- Camera tracks ball position using HSV color detection
- Dual-axis PID controllers compute tilt corrections
- Inverse kinematics converts tilt angles to servo commands
- Three servos actuate the platform to keep the ball centered

## Quick Start
1. Run `python simple_cal.py` to calibrate color and geometry
2. Run `python main.py` to start the controller
3. Tune PID gains in real-time using the GUI sliders
