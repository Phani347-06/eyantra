import argparse
import os
import sys

import cv2 as cv
import numpy as np

def get_markers(gray_img):
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)

    if hasattr(cv.aruco, "ArucoDetector"):
        parameters = cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

        corners, ids, rejected = detector.detectMarkers(gray_img)
    else:
        parameters = cv.aruco.DetectorParameters_create()

        corners, ids, rejected = cv.aruco.detectMarkers(gray_img, aruco_dict, parameters = parameters)

    return corners, ids

def get_inner_corners(corners, ids):
    given_ids = {80, 85, 90, 95}
    marker_dict = {}

    for i in range(len(ids)):
        marker_id = int(ids.ravel()[i])
        if marker_id in given_ids:
            marker_dict[marker_id] = corners[i][0]

    if len(marker_dict) != 4:
        raise ValueError("Could not find all four required markers")

    marker_centers = []
    for pts in marker_dict.values():
        center = np.mean(pts, axis = 0)
        marker_centers.append(center)

    arena_center = np.mean(marker_centers, axis = 0)

    inner_pts = []
    for pts in marker_dict.values():
        dist = np.linalg.norm(pts - arena_center, axis = 1)
        inner_pts.append(pts[np.argmin(dist)])

    ordered_pts = sorted(inner_pts, key = lambda pt: pt[1])
    top_two = sorted(ordered_pts[:2], key = lambda pt: pt[0])
    btm_two = sorted(ordered_pts[2:], key = lambda pt: pt[0])

    res = np.float32([top_two[0], top_two[1], btm_two[1], btm_two[0]])

    return res

def get_grid_points():
    grid = {}
    cell_size = 75
    cols = "ABCDEFGHIJK"

    for col_idx, col_char in enumerate(cols):
        x = (col_idx + 1) * cell_size

        for row in range(1, 12):
            y = row * cell_size
            label = f"{col_char}{row}"
            grid[label] = (x, y)

    return grid

def detect_objects(warped_img, grid_map):
    hsv = cv.cvtColor(warped_img, cv.COLOR_BGR2HSV)

    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)
    red_mask = cv.inRange(hsv, lower_red1, upper_red1) | cv.inRange(hsv, lower_red2, upper_red2)

    kernel = np.ones((3, 3), np.uint8)

    yellow_mask = cv.morphologyEx(yellow_mask, cv.MORPH_OPEN, kernel)
    yellow_mask = cv.morphologyEx(yellow_mask, cv.MORPH_CLOSE, kernel)

    red_mask = cv.morphologyEx(red_mask, cv.MORPH_OPEN, kernel)
    red_mask = cv.morphologyEx(red_mask, cv.MORPH_CLOSE, kernel)

    detected_objects = []

    def process_contours(mask, shape_name):
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv.contourArea(cnt)
            if 80 <= area <= 8000:
                M = cv.moments(cnt)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    point = np.array((cx, cy), dtype=np.float32)

                    closest_grid = min(grid_map.keys(),key = lambda k: np.linalg.norm(np.array(grid_map[k], dtype=np.float32) - point))

                    detected_objects.append({"shape": shape_name, "pixel_center": (cx, cy), "grid_location": closest_grid})

    process_contours(yellow_mask, "Yellow Circle")
    process_contours(red_mask, "Red Triangle")

    return detected_objects

def write_results(marker_ids = None, critical = None, stable = None):
    if marker_ids is None:
        marker_ids = []

    if critical is None:
        critical = []

    if stable is None:
        stable = []

    with open(op_filename, "w") as f:
        f.write(f"Detected marker IDs: {marker_ids}\n")
        f.write(f"Critical Survivors: {', '.join(critical)}\n")
        f.write(f"Stable Survivors: {', '.join(stable)}\n")

parser = argparse.ArgumentParser(description="Task 1A")
parser.add_argument("--image", required=True, type=str, help="Path to the image")
args = parser.parse_args()

img_stem = os.path.splitext(os.path.basename(args.image))[0]
op_filename = os.path.join(os.path.dirname(args.image), f"{img_stem}_results.txt")

img = cv.imread(args.image)

if img is None:
    print(f"Could not read the image: {args.image}")
    write_results()
    sys.exit(1)

gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

corners, ids = get_markers(gray_img)

req_ids = {80, 85, 90, 95}
if ids is None:
    print("No markers found")
    write_results()
    sys.exit(1)

detected_ids = [int(marker_id) for marker_id in ids.ravel()]

missing_ids = req_ids - set(detected_ids)

if missing_ids:
    print("Missing required markers")
    write_results(sorted(detected_ids))
    sys.exit(1)

try:
    src_pts = get_inner_corners(corners, ids)
    dst_pts = np.float32([[0, 0], [899, 0], [899, 899], [0, 899]])

    mtx = cv.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv.warpPerspective(img, mtx, (900, 900))

except(ValueError, cv.error) as error:
    print(f"Could not straighten the arena: {error}")
    write_results(sorted(detected_ids))
    sys.exit(1)

grid_map = get_grid_points()
detected_objects = detect_objects(warped, grid_map)

marker_ids = sorted(detected_ids)

critical = []
stable = []

for person in detected_objects:
    if person['shape'] == "Red Triangle":
        critical.append(person['grid_location'])
critical.sort()

for person in detected_objects:
    if person['shape'] == "Yellow Circle":
        stable.append(person['grid_location'])
stable.sort()

write_results(marker_ids, critical, stable)

print(f"results saved at {op_filename}")
