import cv2
import numpy as np

# read image 
rgb_image = cv2.imread('color_test.jpg')
# to hsv
hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
# bounds
blue_low = np.array([110,50,50])
blue_high = np.array([130,255,255])
# hsv threshold
blue_mask = cv2.inRange(hsv_image, blue_low, blue_high)
# find largest contour
contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if len(contours) > 0:
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    # draw rectangle
    cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 0, 255), 2)

# show image
cv2.imshow('blue_mask', rgb_image)
cv2.waitKey(0)