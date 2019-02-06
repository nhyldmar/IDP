"""Main file for off board processing.
Orange marker on front."""

import cv2
import numpy as np
import keyboard
import sys
import serial
import struct

# Area mask:            50:1530, 180:1810   (image)
#                       0:640, 0:550        (streaming)
# Cell parameters:      (160, 80, 50), (255, 190, 130)
# Robot parameters:     (120, 190, 230), (160, 240, 255)    (orange)
#                       (170, 130, 200), (210, 170, 240)    (purple)
#                       (50, 85, 230), (180, 210, 255)      (red)
#                       (130, 140, 20), (200, 220, 130)     (green)

# User inputs
streaming = False
controlled = True
camera = 2
image_name = "test_data/stream.jpg"

# Parameters
cell_boundaries = (((160, 80, 50), (255, 190, 130)),
                   )
cell_boundaries = np.array(cell_boundaries, dtype="uint8")

robot_boundaries = (((120, 190, 230), (195, 250, 255)),
                    ((170, 130, 190), (230, 205, 240)))
robot_boundaries = np.array(robot_boundaries, dtype="uint8")

cell_thresholds = ((4, 4), (10, 10))
robot_thresholds = ((20, 20), (40, 40))
image_boundaries = ((0, 640), (0, 550))

distance_offset = 35


def startStream(camera):
    """Given a camera returns a stream"""
    # Start stream
    stream = cv2.VideoCapture(camera)

    # Check for stream
    if not stream.isOpened():
        print("Camera {} not found".format(camera))
        stream.release()
        sys.exit()
    print("Streaming...")

    return stream


def startSerial():
    """Given a port, tries to start a serial stream and returns it"""
    ports = ["COM{}".format(port) for port in range(255)]
    active_ports = []
    board = None
    for port in ports:
        try:
            board = serial.Serial(port, timeout=1)
            active_ports.append(port)
        except serial.serialutil.SerialException:
            pass

    if len(active_ports) == 0:
        print("No COM ports open")
        sys.exit()
    elif len(active_ports) > 1:
        board.close()
        active_ports = input("COM ports {} are open. Please select one:".format(active_ports))
        active_ports = ["COM{}".format(active_ports)]
        board = serial.Serial(*active_ports, timeout=1)

    print("Sending to {}...".format(*active_ports))

    return board


def readColour(boundaries, thresholds):
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
            if thresholds[0][0] < w < thresholds[1][0] and thresholds[0][1] < h < thresholds[1][1]:
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 0), 2)
                coordinates = np.concatenate((coordinates, [x + w / 2, y + h / 2]))
    coordinates = np.reshape(coordinates, (-1, 2))
    coordinates = np.array(coordinates, dtype="int16")

    return images_mask, coordinates


def readImage(image):
    """Given an image, return image with bounding boxes and a colour mask."""
    # Crop image to size of operating zone
    image = image[image_boundaries[0][0]:image_boundaries[0][1], image_boundaries[1][0]:image_boundaries[1][1]]

    # Get data from cells and robot colour masks
    cell_mask, cell_coordinates = readColour(cell_boundaries, cell_thresholds)
    robot_mask, robot_coordinates = readColour(robot_boundaries, robot_thresholds)

    # Combine colour masks
    mask = cv2.bitwise_or(cell_mask, robot_mask)

    # Find centre and direction
    if robot_coordinates.shape[0] == 2:
        robot_centre = np.average(robot_coordinates.T, axis=1)
        robot_direction = np.subtract(*robot_coordinates)
        robot_direction = robot_direction / np.linalg.norm(robot_direction)
        robot_centre += robot_direction * distance_offset
    else:
        robot_centre = [0, 0]
        robot_direction = [0, 0]

    # Draw arrowed line for robot
    drawArrow(image, robot_centre, robot_direction)

    return image, cell_coordinates, (robot_centre, robot_direction), mask


def findNearestCell(robot_centre, robot_direction, cell_coordinates):
    """Given the current positions of everything, return the coordinates of and distance and angle to the closest
    cell"""
    if cell_coordinates.shape[0] and robot_centre[0]:
        distances = np.subtract(cell_coordinates, robot_centre)
        distances = np.linalg.norm(distances, axis=1)
        min_distance = np.amin(distances)
        min_index = distances.tolist().index(min_distance)
        min_coordinates = cell_coordinates[min_index]
        min_coordinates = np.array(min_coordinates, dtype="int16")
        min_direction = np.subtract(min_coordinates, robot_centre)
        min_direction /= np.linalg.norm(min_direction)
        min_angle = np.arccos(np.clip(np.dot(robot_direction, min_direction), -1, 1))
        determinant = np.linalg.det((robot_direction, min_direction))
        if determinant < 0:
            min_angle = -min_angle
        drawArrow(image, robot_centre, min_direction)

    else:
        min_coordinates = [0, 0]
        min_distance = 0
        min_angle = 0

    return min_coordinates, min_distance, min_angle


def approachCell(min_distance, min_angle):
    """Given a distance and angle to a cell, returns motor speeds."""
    if np.abs(min_angle) > 0.1:
        scaled_angle = np.clip(min_angle, -1, 1)
        motor_speeds = (scaled_angle, -scaled_angle)
    else:
        scaled_distance = np.clip(min_distance, -1, 1)
        motor_speeds = (scaled_distance, scaled_distance)

    return motor_speeds


def sendSpeeds(motor_speeds):
    """Given motor speeds, sends them to the board."""
    if board.read() == b'r':
        motor_speeds = [speed * 127 for speed in motor_speeds]
        motor_speeds = np.array(motor_speeds, dtype="int8")
        buffer = bytearray(struct.pack('cbb', b's', *motor_speeds))
        board.write(buffer)

        sent = struct.unpack('cbb', buffer)[1:]
        received = board.readline().decode()
        print("Sent: {}{}  Received: {}".format(*sent, received))


def processPlayer():
    """Checks for keyboard inputs."""
    # Check for directional input
    if keyboard.is_pressed('up'):
        motor_speeds = (1, 1)
        arrow_direction = (0, -1)
    elif keyboard.is_pressed('down'):
        motor_speeds = (-1, -1)
        arrow_direction = (0, 1)
    elif keyboard.is_pressed('left'):
        motor_speeds = (-1, 1)
        arrow_direction = (-1, 0)
    elif keyboard.is_pressed('right'):
        motor_speeds = (1, -1)
        arrow_direction = (1, 0)
    else:
        motor_speeds = (0, 0)
        arrow_direction = (0, 0)

    if image:
        arrow_start = np.subtract(image.shape[:-1], (50, 100))
        arrow_direction = 10 * np.array(arrow_direction)
        drawArrow(image, arrow_start, arrow_direction)

    return motor_speeds


def drawArrow(image, start, direction):
    """Given an image, a point and a direction, draws a line on the image."""
    start = np.array(start, dtype="int16")
    direction = np.array(direction)
    end = start + 30 * direction
    end = np.array(end, dtype="int16")
    start = tuple(start)
    end = tuple(end)
    cv2.arrowedLine(image, start, end, (0, 0, 255), 2)


if streaming and controlled:
    stream = startStream(camera)
    board = startSerial()

    # Main loop for streaming
    is_on = True
    while is_on:
        # Get image from stream
        is_on, image = stream.read()

        # Stop if ESC pressed or if camera disconnected
        key = cv2.waitKey(20)
        if key == 27 or not is_on:
            break

        image, cell_coordinates, (robot_centre, robot_direction), mask = readImage(image)

        min_coordinates, min_distance, min_angle = findNearestCell(robot_centre, robot_direction,
                                                                   cell_coordinates)

        motor_speeds = processPlayer()

        sendSpeeds(motor_speeds)

        cv2.imshow("Stream", np.hstack((image, mask)))

    stream.release()
    board.close()

elif streaming:
    stream = startStream(camera)

    board = startSerial()

    # Main loop for streaming
    is_on = True
    while is_on:
        # Get image from stream
        is_on, image = stream.read()

        # Stop if ESC pressed or if camera disconnected
        key = cv2.waitKey(20)
        if key == 27 or not is_on:
            break

        image, cell_coordinates, (robot_centre, robot_direction), mask = readImage(image)

        min_coordinates, min_distance, min_angle = findNearestCell(robot_centre, robot_direction,
                                                                   cell_coordinates)

        motor_speeds = approachCell(min_distance, min_angle)

        sendSpeeds(motor_speeds)

        cv2.imshow("Stream", np.hstack((image, mask)))

    stream.release()
    board.close()

elif controlled:
    board = startSerial()
    image = False
    while True:
        motor_speeds = processPlayer()
        sendSpeeds(motor_speeds)
        if keyboard.is_pressed('esc'):
            break
    board.close()

else:
    image = cv2.imread(image_name)

    image, cell_coordinates, (robot_centre, robot_direction), mask = readImage(image)

    min_coordinates, min_distance, min_angle = findNearestCell(robot_centre, robot_direction,
                                                               cell_coordinates)

    motor_speeds = approachCell(min_distance, min_angle)

    cv2.imwrite("vision.jpg", np.hstack((image, mask)))

cv2.destroyAllWindows()

print("Done")
