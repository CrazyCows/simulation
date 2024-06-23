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
    
    # List to store center coordinates of triangles
    triangle_centers = []
    
    # Minimum contour area to filter small objects
    min_contour_area = 500
    
    # Process each contour
    for contour in contours:
        if cv2.contourArea(contour) > min_contour_area:
            epsilon = 0.03 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            if len(approx) == 3:  # Check for triangles
                center_x = sum([point[0][0] for point in approx]) // 3
                center_y = sum([point[0][1] for point in approx]) // 3
                triangle_centers.append((center_x, center_y))
    
    return binary, triangle_centers

def main():
    ip_camera_url = 'http://10.209.177.243:8080/video'
    cap = cv2.VideoCapture(ip_camera_url)

    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return

    # Define different color bounds for each window
    bounds = [
        (np.array([140, 150, 50]), np.array([160, 255, 255])),  # Purple
        (np.array([150, 75, 125]), np.array([170, 255, 255])),  # Rosa
        (np.array([40, 100, 100]), np.array([80, 255, 255]))  # Green
    ]
    
    window_titles = ['Purple', 'Rosa', 'Green']

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture image.")
                break

            for (lower, upper), title in zip(bounds, window_titles):
                processed_binary, triangles = detect_triangles(frame, lower, upper)
                for triangle in triangles:
                    print(triangle)

                cv2.imshow(title, processed_binary)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
