import cv2
import numpy as np

# Read the image (replace with your image file path)
image = cv2.imread('/home/alakh/shot.png')

# Convert the image to HSV format for color detection
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Mouse callback function to extract HSV values on mouse click
def mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        h = hsv_image[y, x, 0]
        s = hsv_image[y, x, 1]
        v = hsv_image[y, x, 2]
        print(f"H: {h}, S: {s}, V: {v}")

# Set up mouse callback to show HSV values on click
cv2.namedWindow('HSV Image')
cv2.setMouseCallback('HSV Image', mouse)

# Display the HSV converted image for clicking
cv2.imshow("HSV Image", hsv_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Use the correct HSV range values for ball detection
# Replace these with your updated HSV values after extracting them
light_ball = np.array([115, 100, 100])  # Lower HSV range for ball color
dark_ball = np.array([125, 260, 260])  # Upper HSV range for ball color

# Create a mask to detect the ball based on HSV values
mask = cv2.inRange(hsv_image, light_ball, dark_ball)

# Display the mask to see if the ball is detected
cv2.imshow('Mask', mask)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Use contours to detect the ball based on the mask
contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Check if any contours were found
if len(contours) > 0:
    # Find the largest contour (assuming the ball is the largest object)
    largest_contour = max(contours, key=cv2.contourArea)
    (x, y), radius = cv2.minEnclosingCircle(largest_contour)

    # Only proceed if the radius is large enough to be a valid ball
    if radius > 1:
        center = (int(x), int(y))
        frame_mid = center[0]  # x-coordinate of the ball's center
        mid_point = image.shape[1] // 2  # Midpoint of the image width

        # Draw the detected ball on the image
        cv2.circle(image, center, int(radius), (0, 255, 255), 2)

        # Calculate the error between the ball's center and the image midpoint
        error = mid_point - frame_mid

        # Determine action based on the error
        action = "Go Right" if error < 0 else "Go Left"

        # Print and display the error and action
        print(f"Error: {error}, Action: {action}")

        # Annotate the action on the image
        f_image = cv2.putText(image, action, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.imshow('Final Image', f_image)
        cv2.waitKey(0)

# Clean up all windows
cv2.destroyAllWindows()
