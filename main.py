"""Main file for offboard processing.
Robot Name: Mildred
"""

import cv2
import numpy as np
import keyboard
import sys
import serial

# Area mask:            50:1530, 180:1810   (full zone)
#                       50:1530, 1020:1810  (danger zone)
# Cell parameters:      (160, 80, 50), (255, 190, 130)
# Mildred parameters:   (120, 190, 230), (160, 240, 255)    (orange)
#                       (170, 130, 200), (210, 170, 240)    (purple)

# User inputs
streaming = True
controlled = True
camera = 2
port = 'COM3'
image_name = "test_data/mildred_1.jpg"

# Parameters
cell_boundaries = (((160, 80, 50), (255, 190, 130)),
                   )
cell_boundaries = np.array(cell_boundaries, dtype="uint8")

mildred_boundaries = (((120, 190, 230), (160, 240, 255)),
                      ((170, 130, 200), (210, 170, 240)))
mildred_boundaries = np.array(mildred_boundaries, dtype="uint8")

if streaming:
    cell_thresholds = (3, 3)
    mildred_thresholds = (3, 3)
    image_boundaries = ((0, 640), (0, 550))

else:
    cell_thresholds = (12, 12)
    mildred_thresholds = (5, 5)
    image_boundaries = ((50, 1530), (180, 1810))


def startStream(camera):
    """Given a camera returns a stream"""
    # Start stream
    stream = cv2.VideoCapture(camera)

    # Check for stream
    if not stream.isOpened():
        print("Mildred can't find her reading glasses".format(camera))
        stream.release()
        sys.exit()
    print("Mildred put on her reading glasses")

    return stream


def startSerial(port):
    """Given a port, tries to start a serial stream and returns it"""
    try:
        mildred = serial.Serial(port, 115200, timeout=.1)
        print("Mildred turned on her hearing aids")
    except serial.serialutil.SerialException:
        print("Mildred can't find her hearing aids")
        sys.exit()

    return mildred


def readColour(image, boundaries, thresholds):
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
            if (w, h) > thresholds:
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 0), 2)
                coordinates = np.concatenate((coordinates, [x + w / 2, y + h / 2]))
    coordinates = np.reshape(coordinates, (-1, 2))
    coordinates = np.array(coordinates, dtype="int16")

    return images_mask, coordinates


def readImage(image):
    """Given an image, return image with bounding boxes and a colour mask."""
    # Crop image to size of operating zone
    image = image[image_boundaries[0][0]:image_boundaries[0][1], image_boundaries[1][0]:image_boundaries[1][1]]

    # Get data from cells and mildred colour masks
    cell_mask, cell_coordinates = readColour(image, cell_boundaries, cell_thresholds)
    mildred_mask, mildred_coordinates = readColour(image, mildred_boundaries, mildred_thresholds)

    # Combine colour masks
    mask = cv2.bitwise_or(cell_mask, mildred_mask)

    # Find centre and direction
    if mildred_coordinates.shape[0] == 2:
        mildred_centre = np.average(mildred_coordinates.T, axis=1)
        mildred_direction = np.subtract(*mildred_coordinates)

    else:
        # print("{} Mildred markers".format(mildred_coordinates.shape[0]))
        mildred_centre = [0, 0]
        mildred_direction = [0, 0]

    # Draw arrowed line for mildred
    drawArrow(image, mildred_centre, mildred_direction)

    return cell_coordinates, (mildred_centre, mildred_direction), image, mask


def findNearestCell(image, mildred_centre, cell_coordinates):
    """Given the current positions of everything, return the coordinates of the closest cell"""
    if cell_coordinates.shape[0] and mildred_centre[0]:
        distances = np.subtract(cell_coordinates, mildred_centre)
        distances = np.linalg.norm(distances, axis=1)
        min_distance = np.amin(distances)
        min_index = distances.tolist().index(min_distance)
        min_coordinates = cell_coordinates[min_index]
        min_coordinates = np.array(min_coordinates, dtype="int16")
        cv2.circle(image, tuple(min_coordinates), 12, (0, 0, 255), 2)

    else:
        min_coordinates = [0, 0]
        min_distance = 0

    return image, min_coordinates, min_distance


def processInputs(image, mildred):
    """Checks for inputs."""
    # Check for directional input
    if keyboard.is_pressed('up'):
        mildred.write('u')
        arrow_direction = (0, -1)
    elif keyboard.is_pressed('down'):
        mildred.write('d')
        arrow_direction = (0, 1)
    elif keyboard.is_pressed('left'):
        mildred.write('l')
        arrow_direction = (-1, 0)
    elif keyboard.is_pressed('right'):
        mildred.write('r')
        arrow_direction = (1, 0)
    else:
        mildred.write('s')
        arrow_direction = (0, 0)

    arrow_start = np.subtract(image.shape[:-1], (50, 100))
    arrow_direction = 10 * np.array(arrow_direction)
    drawArrow(image, arrow_start, arrow_direction)

    return image


def drawArrow(image, start, direction):
    """Given an image, a point and a direction, draws a line on the image."""
    start = np.array(start, dtype="int16")
    direction = np.array(direction, dtype="int16")
    end = start + 3 * direction
    start = tuple(start)
    end = tuple(end)
    cv2.arrowedLine(image, start, end, (0, 0, 255), 2)


if streaming and controlled:
    stream = startStream(camera)
    mildred = startSerial(port)

    # Main loop for streaming
    is_on = True
    while is_on:
        # Get image from stream
        is_on, image = stream.read()

        # Stop if ESC pressed or if camera disconnected
        key = cv2.waitKey(20)
        if key == 27 or not is_on:
            break

        cell_coordinates, (mildred_centre, mildred_direction), image, mask = readImage(image)

        image, min_coordinates, min_distance = findNearestCell(image, mildred_centre, cell_coordinates)

        image = processInputs(image, mildred)

        cv2.imshow("Mildred Vision", np.hstack((image, mask)))

    stream.release()

elif streaming:
    stream = startStream(camera)

    # Main loop for streaming
    is_on = True
    while is_on:
        # Get image from stream
        is_on, image = stream.read()

        # Stop if ESC pressed or if camera disconnected
        key = cv2.waitKey(20)
        if key == 27 or not is_on:
            break

        cell_coordinates, (mildred_centre, mildred_direction), image, mask = readImage(image)

        image, min_coordinates, min_distance = findNearestCell(image, mildred_centre, cell_coordinates)

        cv2.imshow("Mildred Vision", np.hstack((image, mask)))

    stream.release()

else:
    image = cv2.imread(image_name)

    cell_coordinates, (mildred_centre, mildred_direction), image, mask = readImage(image)

    image, min_coordinates, min_distance = findNearestCell(image, mildred_centre, cell_coordinates)

    cv2.imwrite("mildred_vision.jpg", np.hstack((image, mask)))

cv2.destroyAllWindows()
