import cv2

# List to store the coordinates of the clicked points
points = []

def click_event(event, x, y, flags, param):
    # If the left mouse button is clicked
    if event == cv2.EVENT_LBUTTONDOWN:
        # Append the coordinates to the points list
        points.append((x, y))
        # Display the clicked point on the frame
        cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
        cv2.imshow('Webcam Feed', frame)
        # Print the coordinates
        print(f"Point {len(points)}: ({x}, {y})")
        # If 12 points have been clicked, close the window
        if len(points) == 12:
            cv2.destroyAllWindows()

def main():
    global frame
    # Capture the video feed from the webcam
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    cv2.namedWindow('Webcam Feed')
    cv2.setMouseCallback('Webcam Feed', click_event)

    while True:
        # Read a frame from the webcam feed
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        # Display the frame
        cv2.imshow('Webcam Feed', frame)

        # Wait for 1 ms and check if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

    # Print the collected points
    print("Collected points:", points)

if __name__ == "__main__":
    main()