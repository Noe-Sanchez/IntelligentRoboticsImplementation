import cv2
import numpy as np

# Function to handle trackbar events
def on_trackbar(val):
    # Get the current trackbar values
    h_min = cv2.getTrackbarPos('Hue Min', 'Thresholding')
    h_max = cv2.getTrackbarPos('Hue Max', 'Thresholding')
    s_min = cv2.getTrackbarPos('Saturation Min', 'Thresholding')
    s_max = cv2.getTrackbarPos('Saturation Max', 'Thresholding')
    v_min = cv2.getTrackbarPos('Value Min', 'Thresholding')
    v_max = cv2.getTrackbarPos('Value Max', 'Thresholding')

    # Define the lower and upper threshold values for HSV
    lower_threshold = np.array([h_min, s_min, v_min])
    upper_threshold = np.array([h_max, s_max, v_max])

    # Apply the thresholding
    thresholded_image = cv2.inRange(hsv_image, lower_threshold, upper_threshold)

    # Display the thresholded image
    cv2.imshow('Thresholded Image', thresholded_image)

# Load the image
image = cv2.imread('image.png')
print("loaded image")
# Convert the image to HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Create a window to display the thresholding result
cv2.namedWindow('Thresholding')

# Create trackbars for adjusting the HSV filter
cv2.createTrackbar('Hue Min', 'Thresholding', 0, 255, on_trackbar)
cv2.createTrackbar('Hue Max', 'Thresholding', 255, 255, on_trackbar)
cv2.createTrackbar('Saturation Min', 'Thresholding', 0, 255, on_trackbar)
cv2.createTrackbar('Saturation Max', 'Thresholding', 255, 255, on_trackbar)
cv2.createTrackbar('Value Min', 'Thresholding', 0, 255, on_trackbar)
cv2.createTrackbar('Value Max', 'Thresholding', 255, 255, on_trackbar)

# Initialize the trackbar values
on_trackbar(0)

# Wait for a key press to exit
cv2.waitKey(0)
cv2.destroyAllWindows()