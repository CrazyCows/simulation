import cv2
import numpy as np

def display_color_bounds(lower_bound, upper_bound):
    # Create an image of solid color for the lower bound
    lower_image = np.zeros((100, 100, 3), dtype=np.uint8)
    lower_image[:] = cv2.cvtColor(np.uint8([[lower_bound]]), cv2.COLOR_HSV2BGR)

    # Create an image of solid color for the upper bound
    upper_image = np.zeros((100, 100, 3), dtype=np.uint8)
    upper_image[:] = cv2.cvtColor(np.uint8([[upper_bound]]), cv2.COLOR_HSV2BGR)

    # Display the images
    cv2.imshow('Lower Bound Color', lower_image)
    cv2.imshow('Upper Bound Color', upper_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Define the color bounds in HSV
lower_bound = np.array([140, 150, 150])  # Example lower bound for a color
upper_bound = np.array([170, 255, 255])  # Example upper bound for a color

# Display the color bounds
display_color_bounds(lower_bound, upper_bound)
