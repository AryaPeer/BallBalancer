import cv2
from ball_detection import BallDetector

def main():
    detector = BallDetector()
    cap = cv2.VideoCapture(0)
    
    print("2D Ball Detection Test")
    print("Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        frame = cv2.resize(frame, (640, 480))
        vis_frame, found, position_m = detector.draw_detection(frame)
        
        if found:
            x_m, y_m = position_m
            print(f"Ball at ({x_m:.4f}m, {y_m:.4f}m)")
        
        cv2.imshow("2D Ball Detection Test", vis_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
