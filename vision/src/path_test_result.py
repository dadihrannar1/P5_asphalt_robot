import cv2
import numpy as np

image_path = r"path_test_images"

# Load the two images
white_image = cv2.imread(f'{image_path}/Path_test.png')
colored_image = cv2.imread(f'{image_path}/result.png')

# Scale down the images
scale_factor = 0.2
white_image = cv2.resize(white_image, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)
colored_image = cv2.resize(colored_image, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)


# Convert the images to grayscale
gray_white_image = cv2.cvtColor(white_image, cv2.COLOR_BGR2GRAY)
gray_colored_image = cv2.cvtColor(colored_image, cv2.COLOR_BGR2GRAY)

# Threshold the grayscale images to create binary images
ret, white_threshold = cv2.threshold(gray_white_image, 0, 255, cv2.THRESH_BINARY)
# retrieve the oppisite
black_threshold = cv2.bitwise_not(white_threshold)

ret, colored_threshold = cv2.threshold(gray_colored_image, 250, 255, cv2.THRESH_BINARY)

# Create a mask for the red pixels in the colored image
colored_mask = cv2.inRange(colored_image, (0, 0, 100), (100, 100, 255))

# Use bitwise_and to intersect the white pixels and the red pixels
overlap = cv2.bitwise_and(colored_mask, white_threshold)
overlap2 = cv2.bitwise_and(colored_mask, black_threshold)

# Count the number of non-zero pixels in the overlap image
non_zero_pixels = cv2.countNonZero(overlap)
non_zero_pixels2 = cv2.countNonZero(overlap2)

# Calculate the percentage overlap by dividing the number of non-zero pixels by the total number of pixels in the white image
total_pixels = white_image.shape[0] * white_image.shape[1]
overlap_percentage = (non_zero_pixels / total_pixels) * 100
miss_percentage = (non_zero_pixels2 / total_pixels) * 100

print(f'The red pixels overlap the white pixels by {overlap_percentage:.2f}%')
print(f'The red pixels missed the white pixels by {miss_percentage:.2f}%')