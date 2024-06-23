import cv2
import numpy as np

# Initialize the video capture with DroidCam's virtual webcam
cap = cv2.VideoCapture(0)  # Use 0 as the index for DroidCam's virtual webcam

if not cap.isOpened():
    print("Error: Could not open DroidCam.")
else:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Failed to capture image.")
            break
        
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Use Canny edge detector to find edges in the image
        edges = cv2.Canny(gray, 100, 200)

        # Find contours in the edges
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Process each contour
        for contour in contours:
            # Approximate contour with accuracy proportional to the contour perimeter
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if the approximated contour has 3 points (triangle)
            if len(approx) == 3:
                # Compute the area of the triangle
                area = cv2.contourArea(contour)

                # Define a minimum area for triangles to be considered
                min_area = 10  # you can adjust this value based on your needs

                if area > min_area:
                    # Compute the bounding rectangle for the triangle
                    x, y, w, h = cv2.boundingRect(approx)
                    center_x, center_y = x + w // 2, y + h // 2

                    # Draw rectangle around the triangle
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    
                    # Draw circle at the center
                    cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

                    # Display the center coordinates and area of the triangle
                    text = f"Center: ({center_x}, {center_y}) | Area: {area}"
                    cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('DroidCam Video', frame)

        # Press 'q' to quit the video stream
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything is done, release the capture
    cap.release()
    cv2.destroyAllWindows()
