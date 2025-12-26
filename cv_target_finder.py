import cv2
import numpy as np

# Global variable for camera setup (usually only done once)
CAP = None
FRAME_WIDTH = 640  # Placeholder: replace with your actual camera resolution
FRAME_HEIGHT = 480 # Placeholder: replace with your actual camera resolution

def initialize_camera(camera_index=0):
    """Initializes the camera capture object."""
    global CAP, FRAME_WIDTH, FRAME_HEIGHT
    CAP = cv2.VideoCapture(camera_index)
    if not CAP.isOpened():
        print("Error: Cannot access camera.")
        return False
    FRAME_WIDTH = int(CAP.get(cv2.CAP_PROP_FRAME_WIDTH))
    FRAME_HEIGHT = int(CAP.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return True

def find_target_center(expected_radius=40, tolerance=15):
    """
    Finds the center of the largest red circular target.
    
    Returns: (center_x, center_y) in pixels, or None if no target is found.
    """
    if CAP is None or not CAP.isOpened():
        print("Error: Camera not initialized.")
        return None

    ret, frame = CAP.read()
    if not ret:
        print("Error: Can't receive frame.")
        return None

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red mask (same as your original code)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    blurred = cv2.GaussianBlur(mask, (9, 9), 2, 2)

    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
        param1=120, param2=30, minRadius=15, maxRadius=100
    )

    best_circle = None
    if circles is not None:
        circles = np.uint16(np.around(circles))[0, :]
        for x, y, r in circles:
            # Apply size and circularity checks
            if abs(r - expected_radius) > tolerance:
                continue

            # Your circularity check logic goes here (simplified for brevity)
            # ... (omitted circularity check for a cleaner example, assume size check is sufficient for now)
            
            # If a suitable target is found, record it (you might want to choose the largest)
            if best_circle is None or r > best_circle[2]:
                best_circle = (x, y, r)

    # Release resources or draw on frame if needed (for debugging)
    # cv2.imshow("Detection", frame)
    # cv2.waitKey(1)
    
    if best_circle:
        x, y, r = best_circle
        # Return the target's center coordinates
        return x, y
    else:
        return None

def get_frame_center():
    """Returns the center of the camera frame."""
    return FRAME_WIDTH / 2, FRAME_HEIGHT / 2

def release_camera():
    global CAP
    if CAP is not None:
        CAP.release()
