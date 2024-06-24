import cv2
import numpy as np

def detect_triangles(frame, lower_bound, upper_bound):
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Threshold the HSV image to get only specified colors
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Bitwise-AND mask and original image to extract specified areas
    color_extract = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Convert extracted image to grayscale then to binary
    gray = cv2.cvtColor(color_extract, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
    
    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Minimum contour area to filter small objects
    min_contour_area = 500
    object_centers = []
    # Process each contour
    for contour in contours:
        if cv2.contourArea(contour) >= min_contour_area:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
                object_centers.append((center_x, center_y))
    #print(len(object_centers))
    return binary, object_centers

def main():
    ip_camera_url = 'http://192.168.137.15:8080/video'
    cap = cv2.VideoCapture(ip_camera_url)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        #print("Error: Could not open video stream.")
        return
    _whiteLower = np.array([0, 0, 200])
    _whiteUpper = np.array([255, 50, 255])
    # Define different color bounds for each window
    bounds = [
        #(np.array([140, 150, 50]), np.array([160, 255, 255])),  # Purple
        #(np.array([150, 75, 125]), np.array([170, 255, 255])),  # Rosa
        #(np.array([40, 100, 100]), np.array([80, 255, 255]))  # Green
        (_whiteLower, _whiteUpper) # white

    ]
    
    window_titles = ['white']
    #window_titles = ['Purple']
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                #print("Error: Failed to capture image.")
                break

            for (lower, upper), title in zip(bounds, window_titles):
                processed_binary, triangles = detect_triangles(frame, lower, upper)
                for triangle in triangles:
                    ""
                    #print(triangle)

                cv2.imshow(title, processed_binary)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
