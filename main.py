"""Given an image or stream, outputs the coordinates of the centres of the cells"""

import cv2
import numpy as np
import sys

# Area mask:            50:1530, 180:1810   (full zone)
#                       50:1530, 1020:1810  (danger zone)
# Cell parameters:      (160, 80, 50), (255, 190, 130) and 7x7
# Robot parameters:     (120, 190, 230), (160, 240, 255)    (orange)
#                       (170, 130, 200), (210, 170, 240)    (purple)
#                       (120, 140, 60), (160, 180, 100)     (green; not used since too close to safe zone colour)

# User inputs
streaming = False
camera = 0
image_name = "test_data/colour_1.jpg"

# Parameters
cell_boundaries = ((160, 80, 50), (255, 190, 130))
cell_boundaries = np.array(cell_boundaries, dtype="uint8")

robot_boundaries = ((170, 130, 200), (210, 170, 240))
robot_boundaries = np.array(robot_boundaries, dtype="uint8")

cell_thresholds = [4, 4]
robot_thresholds = [4, 4]


def read_colour(image, boundaries, thresholds):
    """Given an image, return image with bounding boxes and a colour mask for a given colour boundary set."""
    coordinates = np.array([])
    images_mask = np.zeros(image.shape, dtype="uint8")
    # Isolate pixels within BGR boundaries
    for boundary in boundaries:
        mask = cv2.inRange(image, *boundary)
        image_mask = cv2.bitwise_and(image, image, mask=mask)
        images_mask = cv2.bitwise_or(images_mask, image_mask)

        # Find contours
        gray = cv2.cvtColor(image_mask, cv2.COLOR_BGR2GRAY)
        contours, hierarchy = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes and get coordinates
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if [w, h] > thresholds:
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 0), 2)
                coordinates = np.concatenate((coordinates, [x + w / 2, y + h / 2]))
    coordinates = np.reshape(coordinates, (-1, 2))
    coordinates = np.array(coordinates, dtype="int16")

    return images_mask, coordinates


def read_image(image):
    """Given an image, return image with bounding boxes and a colour mask."""
    image = image[50:1530, 180:1810]

    cell_mask, cell_coordinates = read_colour(image, cell_boundaries, cell_thresholds)
    robot_mask, robot_coordinates = read_colour(image, robot_boundaries, robot_thresholds)

    if robot_coordinates.shape[0] == 3:
        print("Write the code")

    else:
        print("{} robot markers detected".format(robot_coordinates.shape[0]))

    return cell_coordinates, robot_coordinates, image, cell_mask, robot_mask


if streaming:
    # Start stream
    stream = cv2.VideoCapture(camera)

    # Check for stream
    if not stream.isOpened():
        print("Camera {} not detected".format(camera))
        stream.release()
        sys.exit()

    print("Camera detected\nStreaming...")

    # Main loop for streaming
    is_on = True
    while is_on:
        is_on, image = stream.read()

        # Stop if ESC pressed or if camera disconnected
        key = cv2.waitKey(20)
        if key == 27 or not is_on:
            break

        # Get data from the image
        cell_coordinates, robot_coordinates, image, cell_mask, robot_mask = read_image(image)

        # Show boxes and mask
        cv2.imshow("colour_masking", np.hstack((image, cell_mask, robot_mask)))

    stream.release()

else:
    image = cv2.imread(image_name)
    cell_coordinates, robot_coordinates, image, cell_mask, robot_mask = read_image(image)

    # Show boxes and mask
    print("Number of cells: {}".format(cell_coordinates.shape[0]))
    cv2.imwrite("colour_masking.jpg", np.hstack((image, cell_mask, robot_mask)))
    cv2.waitKey(0)  # Close if ESC pressed

# Clear windows
cv2.destroyAllWindows()
