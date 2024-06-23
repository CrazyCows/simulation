import cv2

# Replace with your phone's IP address and port number
phone_ip = '192.168.0.206'
port = '8080'
url = f'http://{phone_ip}:{port}/video'

# Initialize the video capture with the URL
cap = cv2.VideoCapture(url)

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

while True:
    ret, frame = cap.read()
    if not ret:
        break  # Stop the loop if there are no frames to read

    # Convert the frame to grayscale for the face detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

    # Draw rectangles around the faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

    # Display the resulting frame
    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
        break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()
