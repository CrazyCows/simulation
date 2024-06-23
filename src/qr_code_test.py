import cv2
from pyzbar import pyzbar

# Replace with your phone's IP address and port number
phone_ip = '192.168.0.206'
port = '8080'
url = f'http://{phone_ip}:{port}/video'

# Initialize the video capture with the URL
cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("Error: Could not open video stream.")
else:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Failed to capture image.")
            break

        # Detect QR Codes in the image
        qr_codes = pyzbar.decode(frame)

        # Process detected QR Codes
        for qr in qr_codes:
            # Extract data and rectangle corners of the QR code
            qr_data = qr.data.decode('utf-8')
            (x, y, w, h) = qr.rect
            center_x, center_y = x + w // 2, y + h // 2
            
            # Draw rectangle around the QR code
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Draw circle at the center
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

            # Display the QR code data
            text = f"Data: {qr_data} | Center: ({center_x}, {center_y})"
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('DroidCam Video', frame)

        # Press 'q' to quit the video stream
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything is done, release the capture
    cap.release()
    cv2.destroyAllWindows()
