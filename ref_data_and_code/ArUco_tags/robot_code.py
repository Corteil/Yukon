#!/usr/bin/env python

'''
Welcome to the ArUco Marker Detector!

This program:
  - Detects ArUco markers using OpenCV and Python

uses code from
Project: ArUco Marker Detector
Date created: 12/18/2021
Python version: 3.11.2
Reference: https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
'''

import time

import cv2 # Import the OpenCV library

# setup pi camera

from picamera2 import Picamera2
from libcamera import controls, Transform

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": 'XRGB8888', "size": (1144, 1080)}, transform=Transform(hflip=True, vflip=True)))

desired_aruco_dictionary = "DICT_4X4_1000"

# The different ArUco dictionaries built into the OpenCV library.
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# colours
RED = (0, 0, 255)
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# display constants
MARKER_BOX_THICKNESS = 2
MARKER_CENTER_RADIUS = 4
GATE_LINE_THICKNESS = 5
GATE_LINE_HALF_HEIGHT = 50
FONT = cv2.FONT_HERSHEY_SIMPLEX
MARKER_FONT_SCALE = 0.5
MARKER_FONT_THICKNESS = 2
FPS_FONT_SCALE = 2
FPS_FONT_THICKNESS = 2
FPS_POSITION = (100, 100)
GATE_LABEL_OFFSET = 55


def main():
  """
  Main method of the program.
  """
  # Build gate pairs list
  number_of_gates = 100
  gates = []
  odd = 1
  for _ in range(number_of_gates):
    gates.append([odd, odd + 1])
    odd += 2
  print(gates)

  # Check that we have a valid ArUco marker
  if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(desired_aruco_dictionary))
    return

  # Load the ArUco dictionary
  print("[INFO] detecting '{}' markers...".format(desired_aruco_dictionary))
  this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
  this_aruco_parameters = cv2.aruco.DetectorParameters_create()

  # Start the video stream
  picam2.start()

  while True:
    # time stamps
    start_frame_time = time.time()

    # reset visible_tags
    visible_tags = {}

    # Capture frame-by-frame
    frame = picam2.capture_array()
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers in the video frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(grey, this_aruco_dictionary, parameters=this_aruco_parameters)

    # Check that at least two ArUco markers was detected
    if len(corners) > 1:
      # Flatten the ArUco IDs list
      ids = ids.flatten()

      # Loop over the detected ArUco corners
      for (marker_corner, marker_id) in zip(corners, ids):

        # Extract the marker corners (use a distinct name to avoid shadowing outer corners)
        marker_corners = marker_corner.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = marker_corners

        # Convert the (x,y) coordinate pairs to integers
        top_right = (int(top_right[0]), int(top_right[1]))
        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
        top_left = (int(top_left[0]), int(top_left[1]))

        # Draw the bounding box of the ArUco detection
        cv2.line(frame, top_left, top_right, GREEN, MARKER_BOX_THICKNESS)
        cv2.line(frame, top_right, bottom_right, GREEN, MARKER_BOX_THICKNESS)
        cv2.line(frame, bottom_right, bottom_left, GREEN, MARKER_BOX_THICKNESS)
        cv2.line(frame, bottom_left, top_left, GREEN, MARKER_BOX_THICKNESS)

        # Calculate and draw the center of the ArUco marker
        center_x = int((top_left[0] + bottom_right[0]) / 2.0)
        center_y = int((top_left[1] + bottom_right[1]) / 2.0)
        cv2.circle(frame, (center_x, center_y), MARKER_CENTER_RADIUS, RED, -1)

        # Draw the ArUco marker ID on the video frame
        cv2.putText(frame, str(marker_id),
          (top_left[0], top_left[1] - 15),
          FONT, MARKER_FONT_SCALE, GREEN, MARKER_FONT_THICKNESS)

        # add ArUco marker to visible_tags
        tag_area = abs(top_left[0] - bottom_right[0]) * abs(top_left[1] - bottom_right[1])
        visible_tags[marker_id] = [center_x, center_y, tag_area]

    if visible_tags:
      sorted_tags = sorted(visible_tags.keys())
      print(f"sorted Tags: {sorted_tags}")
      for tag in sorted_tags:
        if tag % 2 != 0 and tag + 1 in sorted_tags:
          gate_pair = [tag, tag + 1]
          if gate_pair in gates:
            gate_id = gates.index(gate_pair)
            print(f"gate id: {gate_id} gate pair {gate_pair[0]}, {gate_pair[1]}")
            gate_odd_x = visible_tags[gate_pair[0]][0]
            gate_even_x = visible_tags[gate_pair[1]][0]
            gate_odd_y = visible_tags[gate_pair[0]][1]
            gate_even_y = visible_tags[gate_pair[1]][1]
            if gate_odd_x > gate_even_x:
              print("odd tag is on the right")
              colour = RED
            else:
              colour = BLUE
            gate_centre_x = gate_odd_x + int((gate_even_x - gate_odd_x) / 2)
            gate_centre_y = (gate_odd_y + gate_even_y) // 2
            print(f"odd X: {gate_odd_x} even X: {gate_even_x} Gate centre: {gate_centre_x}")
            cv2.line(frame,
              (gate_centre_x, gate_centre_y - GATE_LINE_HALF_HEIGHT),
              (gate_centre_x, gate_centre_y),
              colour, GATE_LINE_THICKNESS)
            cv2.putText(frame, str(gate_id),
              (gate_centre_x, gate_centre_y - GATE_LABEL_OFFSET),
              FONT, MARKER_FONT_SCALE, colour, MARKER_FONT_THICKNESS)

    finish_time = time.time()
    time_taken = finish_time - start_frame_time
    ftp = 1 / time_taken if time_taken > 0 else float('inf')

    cv2.putText(frame, f"FPS: {round(ftp, 3)}", FPS_POSITION, FONT, FPS_FONT_SCALE, WHITE, FPS_FONT_THICKNESS)

    # Display the resulting frame
    cv2.imshow('frame', frame)

    # If "q" is pressed on the keyboard, exit this loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  # Close down the video stream
  picam2.stop()
  cv2.destroyAllWindows()

if __name__ == '__main__':
  print(__doc__)
  main()
