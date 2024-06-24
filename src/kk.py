import cv2
import numpy as np

def detect_triangles(frame):
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define the color bounds in HSV
    lower_bound = np.array([40, 150, 150])  # Example lower bound for a color
    upper_bound = np.array([80, 255, 255])  # Example upper bound for a color

    
    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Bitwise-AND mask and original image to extract green areas
    green_extract = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Convert extracted green image to grayscale then to binary
    gray = cv2.cvtColor(green_extract, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
    
    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # List to store center coordinates of triangles
    triangle_centers = []
    
    # Minimum contour area to filter small objects
    min_contour_area = 500  # Adjust this value based on your specific needs
    
    # Process each contour
    for contour in contours:
        if cv2.contourArea(contour) > min_contour_area:
            # Approximate contour to reduce the number of points
            epsilon = 0.03 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Check if the approximated contour has three points (triangle)
            if len(approx) == 3:
                # Calculate the center of the triangle
                center_x = sum([point[0][0] for point in approx]) // 3
                center_y = sum([point[0][1] for point in approx]) // 3
                triangle_centers.append((center_x, center_y))
                
                # Optional: draw bounding rectangle around triangle for visualization
                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(binary, (x, y), (x + w, y + h), 255, 2)
    
    return binary, triangle_centers

def main():
    ip_camera_url = 'http://10.209.177.243:8080/video'  # Change this to your IP camera's stream URL
    cap = cv2.VideoCapture(2)

    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return

    try:
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture image.")
                break


            # Detect triangles and draw bounding boxes on binary image
            processed_binary, triangles = detect_triangles(frame)
            
            for triangle in triangles:
                ""
                #print(triangle)

            # Display the resulting binary frame
            cv2.imshow('Frame', processed_binary)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # When everything is done, release the capture
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

