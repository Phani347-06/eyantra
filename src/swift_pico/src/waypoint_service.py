#!/usr/bin/env python3

import json
import os
import heapq
import math
from itertools import permutations

import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from waypoint_navigation.srv import GetWaypoints
from ament_index_python.packages import get_package_share_directory

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


class WayPoints(Node):

    def __init__(self):
        super().__init__('waypoints_service')
        self.srv = None
        self.waypoints = []
        self.waypoint_labels = []
        self.marker_ids = []

        self.arena_image = None
        self.rectified_image = None
        self.critical_survivors = []
        self.survivor_coordinates = []
        self.planned_order = []
        self.planned_route = []
        self.route_distance = 0.0
        self.turn_penalty = 0.15
        self.max_route_extra = 0.02
        coordinates_file = os.path.join(
            get_package_share_directory('swift_pico'), 'scripts', 'grid_whycode_coordinates.json'
        )
        with open(coordinates_file) as file:
            self.grid_coordinates = json.load(file)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, qos)

    def image_callback(self, msg):
        self.arena_image = msg
        self.get_logger().info(f'Arena image received: {msg.width} x {msg.height}')
        try:
            img = self.get_cv_image(msg)
            gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            corners, ids = get_markers(gray_img)

            if ids is None:
                raise ValueError("No arena markers found")

            src_pts = get_inner_corners(corners, ids)
            dst_pts = np.float32([[0, 0], [899, 0], [899, 899], [0, 899]])
            mtx = cv.getPerspectiveTransform(src_pts, dst_pts)
            warped = cv.warpPerspective(img, mtx, (900, 900))

            grid_map = get_grid_points()
            detected_objects = detect_objects(warped, grid_map)
            critical = []
            for person in detected_objects:
                if person['shape'] == "Red Triangle":
                    critical.append(person['grid_location'])
            critical.sort()

            if len(critical) != 4 or len(set(critical)) != 4:
                raise ValueError(f"Expected four critical survivors, found: {critical}")

            coordinates = self.get_survivor_coordinates(critical)
            order, route, distance = self.plan_route(critical)
            waypoint_labels = self.get_navigation_route(route, critical)
            # F6 is the starting position, unless a survivor there needs a stop.
            if 'F6' not in critical:
                waypoint_labels = waypoint_labels[1:]
            waypoints = self.get_survivor_coordinates(waypoint_labels)
            marker_ids = []
            visited = set()
            for label in waypoint_labels:
                if label in critical and label not in visited:
                    marker_ids.append(f'survivor:{label}')
                    visited.add(label)
                else:
                    marker_ids.append(f'transit:{label}')
            self.rectified_image = warped
            self.critical_survivors = critical
            self.survivor_coordinates = coordinates
            self.planned_order = order
            self.planned_route = route
            self.route_distance = distance
            self.waypoint_labels = waypoint_labels
            self.waypoints = waypoints
            self.marker_ids = marker_ids
            self.get_logger().info(f'Critical survivors: {critical}')
            for label, point in zip(critical, coordinates):
                self.get_logger().info(f'{label}: {point}')
            self.get_logger().info(f'Visiting order: {order}')
            self.get_logger().info(f'Grid route: {route}')
            self.get_logger().info(f'Navigation goals: {waypoint_labels}')
            self.get_logger().info(f'Route length: {distance:.2f} WhyCode units')
            try:
                self.save_checkpoints()
            except (OSError, cv.error) as error:
                self.get_logger().warning(f'Could not save checkpoint images: {error}')
            self.srv = self.create_service(GetWaypoints, 'waypoints', self.waypoint_callback)
            self.get_logger().info(f'Waypoint service ready with {len(self.waypoints)} goals')
        except (ValueError, cv.error) as error:
            self.get_logger().error(f'Could not prepare waypoints: {error}')
        finally:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None

    def save_checkpoints(self):
        output_dir = os.path.join(os.getcwd(), 'task2c_output')
        os.makedirs(output_dir, exist_ok=True)
        grid_map = get_grid_points()
        detected_image = self.rectified_image.copy()
        font = cv.FONT_HERSHEY_SIMPLEX

        for label in self.critical_survivors:
            x, y = grid_map[label]
            point = self.grid_coordinates[label]
            text = f"{label} ({point['x']:.2f}, {point['y']:.2f})"
            text_width = cv.getTextSize(text, font, 0.45, 1)[0][0]
            text_x = max(5, min(x - 50, detected_image.shape[1] - text_width - 5))
            cv.circle(detected_image, (x, y), 23, (0, 255, 255), 2)
            cv.putText(detected_image, text, (text_x, y - 30), font, 0.45, (0, 0, 0), 3)
            cv.putText(detected_image, text, (text_x, y - 30), font, 0.45, (0, 255, 255), 1)

        route_image = detected_image.copy()
        for start, end in zip(self.planned_route, self.planned_route[1:]):
            cv.arrowedLine(route_image, grid_map[start], grid_map[end], (0, 165, 255), 2,
                           tipLength=0.2)
        for label in self.planned_route:
            cv.circle(route_image, grid_map[label], 4, (0, 165, 255), -1)
        for number, label in enumerate(self.planned_order, 1):
            x, y = grid_map[label]
            cv.putText(route_image, str(number), (x - 8, y + 8), font, 0.7, (0, 0, 0), 4)
            cv.putText(route_image, str(number), (x - 8, y + 8), font, 0.7, (255, 255, 255), 2)
        x, y = grid_map['F6']
        cv.putText(route_image, 'START F6', (x + 10, y + 22), font, 0.5, (0, 0, 0), 3)
        cv.putText(route_image, 'START F6', (x + 10, y + 22), font, 0.5, (255, 255, 255), 1)

        for filename, image in [('detected_survivors.png', detected_image),
                                ('planned_route.png', route_image)]:
            if not cv.imwrite(os.path.join(output_dir, filename), image):
                raise OSError(f'Could not write {filename}')
        self.get_logger().info(f'Checkpoint images saved in {output_dir}')

    def get_survivor_coordinates(self, labels):
        coordinates = []
        for label in labels:
            if label not in self.grid_coordinates:
                raise ValueError(f'No coordinates found for {label}')
            point = self.grid_coordinates[label]
            coordinates.append([float(point['x']), float(point['y']), 13.0])
        return coordinates

    def shortest_grid_path(self, start, end, turn_penalty=0.0):
        if start not in self.grid_coordinates or end not in self.grid_coordinates:
            raise ValueError(f'Missing grid coordinates for {start} or {end}')

        # Track arrival direction so a turn has a cost without changing grid edges.
        initial = (start, 0, 0)
        costs = {initial: 0.0}
        previous = {}
        queue = [(0.0, initial)]
        while queue:
            cost, state = heapq.heappop(queue)
            if cost > costs[state]:
                continue
            current, previous_dc, previous_dr = state
            if current == end:
                break

            col = ord(current[0]) - ord('A')
            row = int(current[1:])
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    if dc == 0 and dr == 0:
                        continue
                    if not (0 <= col + dc < 11 and 1 <= row + dr <= 11):
                        continue
                    neighbor = f'{chr(ord("A") + col + dc)}{row + dr}'
                    if neighbor not in self.grid_coordinates:
                        continue

                    point = self.grid_coordinates[current]
                    next_point = self.grid_coordinates[neighbor]
                    step = math.hypot(next_point['x'] - point['x'], next_point['y'] - point['y'])
                    changed_direction = (
                        (previous_dc != 0 or previous_dr != 0)
                        and (dc, dr) != (previous_dc, previous_dr)
                    )
                    candidate = cost + step + turn_penalty * changed_direction
                    next_state = (neighbor, dc, dr)
                    if candidate < costs.get(next_state, math.inf):
                        costs[next_state] = candidate
                        previous[next_state] = state
                        heapq.heappush(queue, (candidate, next_state))

        if current != end:
            raise ValueError(f'No grid path from {start} to {end}')
        path = [end]
        while state != initial:
            state = previous[state]
            path.append(state[0])
        path.reverse()
        distance = 0.0
        for first, second in zip(path, path[1:]):
            point = self.grid_coordinates[first]
            next_point = self.grid_coordinates[second]
            distance += math.hypot(next_point['x'] - point['x'], next_point['y'] - point['y'])
        return path, distance

    def plan_route(self, survivors):
        if len(survivors) != 4 or len(set(survivors)) != 4:
            raise ValueError('Route planning needs four distinct survivors')

        # Compare all visiting orders using shortest and turn-aware grid paths.
        candidates = []
        for penalty in (0.0, self.turn_penalty):
            paths = {}
            for start in dict.fromkeys(['F6'] + survivors):
                for end in survivors:
                    paths[(start, end)] = self.shortest_grid_path(start, end, penalty)

            for order in permutations(sorted(survivors)):
                route = ['F6']
                for label in order:
                    route.extend(paths[(route[-1], label)][0][1:])

                # Finish once all four have been encountered, including F6.
                visited = set()
                for index, label in enumerate(route):
                    if label in survivors:
                        visited.add(label)
                    if len(visited) == len(survivors):
                        route = route[:index + 1]
                        break

                distance = 0.0
                for first, second in zip(route, route[1:]):
                    point = self.grid_coordinates[first]
                    next_point = self.grid_coordinates[second]
                    distance += math.hypot(next_point['x'] - point['x'], next_point['y'] - point['y'])
                goal_count = len(self.get_navigation_route(route, survivors))
                candidates.append((goal_count, distance, route))

        shortest_distance = min(distance for _, distance, _ in candidates)
        distance_limit = shortest_distance * (1.0 + self.max_route_extra) + 1e-9
        _, distance, route = min(
            candidate for candidate in candidates if candidate[1] <= distance_limit
        )

        # A grid path can pass another survivor before its selected stop.
        visit_order = []
        for label in route:
            if label in survivors and label not in visit_order:
                visit_order.append(label)
        return visit_order, route, distance

    def get_navigation_route(self, route, survivors):
        # Keep turns and survivor stops; pass straight through other grid points.
        navigation_route = route[:1]
        for previous, current, following in zip(route, route[1:], route[2:]):
            incoming = (ord(current[0]) - ord(previous[0]),
                        int(current[1:]) - int(previous[1:]))
            outgoing = (ord(following[0]) - ord(current[0]),
                        int(following[1:]) - int(current[1:]))
            if current in survivors or incoming != outgoing:
                navigation_route.append(current)
        if len(route) > 1:
            navigation_route.append(route[-1])
        return navigation_route

    def get_cv_image(self, msg):
        # Read the camera pixels directly; cv_bridge here uses an older NumPy ABI.
        if msg.encoding not in ('rgb8', 'bgr8'):
            raise ValueError(f'Unsupported image encoding: {msg.encoding}')
        if (msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * 3
                or len(msg.data) < msg.height * msg.step):
            raise ValueError('Invalid camera image data')

        pixels = np.frombuffer(msg.data, dtype=np.uint8, count=msg.height * msg.step)
        rows = pixels.reshape(msg.height, msg.step)
        img = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
        if msg.encoding == 'rgb8':
            img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
        return img

    def waypoint_callback(self, request, response):
        if request.get_waypoints == True:
            response.waypoints.poses = [Pose() for _ in range(len(self.waypoints))]
            for i in range(len(self.waypoints)):
                response.waypoints.poses[i].position.x = float(self.waypoints[i][0])
                response.waypoints.poses[i].position.y = float(self.waypoints[i][1])
                response.waypoints.poses[i].position.z = float(self.waypoints[i][2])
            response.marker_ids = self.marker_ids[:]
            self.get_logger().info("Incoming request for Waypoints")
            return response
        else:
            self.get_logger().info("Request rejected")
            return response

def main():
    rclpy.init()
    waypoints = WayPoints()

    try:
        rclpy.spin(waypoints)
    except KeyboardInterrupt:
        waypoints.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoints.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
