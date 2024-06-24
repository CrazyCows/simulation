import cv2
import numpy as np

# Initialize variables to store the points
points = []

# Define the callback function that will be called when the user clicks on the image
def click_event(event, x, y, flags, params):
    global points

    # Check for left mouse button click event
    if event == cv2.EVENT_LBUTTONDOWN:
        # Append the point to the list
        points.append((x, y))

        # Draw a circle at the clicked point
        cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
        cv2.imshow('DroidCam', frame)

        # If we have two points, calculate the distance
        if len(points) == 2:
            point1, point2 = points
            distance = np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
            print(f'Distance: {distance} pixels')

            # Clear the points after calculating the distance
            points = []

# Connect to the DroidCam feed (assuming it is accessible via IP address)
droidcam_url = "http://your_droidcam_ip:4747/video"
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

# Create a window to display the video feed
cv2.namedWindow('DroidCam')
cv2.setMouseCallback('DroidCam', click_event)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Display the frame
    cv2.imshow('DroidCam', frame)

    # Wait for 'q' key to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
