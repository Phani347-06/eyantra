#!/usr/bin/env python3

import time
import math
try:
    from tf_transformations import euler_from_quaternion
except ImportError:
    def euler_from_quaternion(q):
        x, y, z, w = q
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from waypoint_navigation.action import NavToWaypoint
from swift_msgs.msg import SwiftMsgs
from std_msgs.msg import Header
from whycode_interfaces.msg import MarkerArray
from error_msg.msg import Error
from controller_msg.msg import PIDTune
from nav_msgs.msg import Odometry


class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.pid_or_lqr_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()
        self.odometry_callback_group = ReentrantCallbackGroup()

        self.time_inside_sphere = 0
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.duration = 0

        self.yaw = 0.0
        self.desired_yaw = -0.5
        self.curr_state = [0.0, 0.0, 0.0]
        self.desired_state = [0.0, 0.0, 13.0]
        self.drone_position = [0.0, 0.0, 0.0]
        self.xyz = [0.0, 0.0, 0.0, 0.0]
        self.dtime = 0

        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 1000

        # PID parameters [roll, pitch, throttle]
        self.Kp = [9.0, 9.0, 82.5]
        self.Ki = [0.0, 0.0, 0.2]
        self.Kd = [20.0, 20.0, 60.0]

        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]
        self.filtered_der = [0.0, 0.0, 0.0]
        self.der_alpha = 0.2

        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]

        self.neutral = 1500
        self.i_limit = 5.0
        self.sample_time = 0.033
        self.sign = [1.0, 1.0, -1.0]

        self.use_gui = False
        self.gui_scale = [
            [30.0, 1.0, 1.7],
            [30.0, 1.0, 1.7],
            [50.0, 1.0, 0.7]
        ]

        self.marker_rec = False
        self.first_cycle = True
        self.reset_xy_derivative = False
        self.armed = False

        self.last_pid_time = None
        self.last_marker_time = None
        self.marker_timeout = 0.5
        self.tracking_reset_count = 0
        self.pos_error = Error()

        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)
        self.heartbeat_pub = self.create_publisher(Header, '/controller_heartbeat', 10)

        self.create_subscription(
            MarkerArray, '/whycode_node/markers', self.whycode_callback, 1,
            callback_group=self.odometry_callback_group
        )
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)
        self.create_subscription(
            Odometry, '/rotors/odometry', self.odometry_callback, 10,
            callback_group=self.odometry_callback_group
        )

        self._action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            self.execute_callback,
            callback_group=self.action_callback_group
        )

        self.publish_heartbeat()
        self.create_timer(0.1, self.publish_heartbeat, callback_group=self.odometry_callback_group)

        time.sleep(0.5)
        self.arm()

        self.timer = self.create_timer(
            self.sample_time, self.pid, callback_group=self.pid_or_lqr_callback_group
        )

    def publish_heartbeat(self):
        hb = Header()
        hb.stamp = self.get_clock().now().to_msg()
        hb.frame_id = 'pico_controller_active'
        self.heartbeat_pub.publish(hb)

    def disarm(self):
        self.armed = False
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        self.disarm()
        self.error_sum = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.filtered_der = [0.0, 0.0, 0.0]
        self.first_cycle = True
        self.last_pid_time = None

        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)
        self.armed = True
        self.get_logger().info('Drone armed.')

    def whycode_callback(self, msg):
        if not msg.markers:
            self.marker_rec = False
            self.tracking_reset_count += 1
            self.first_cycle = True
            self.error_sum = [0.0, 0.0, 0.0]
            self.filtered_der = [0.0, 0.0, 0.0]
            return

        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_marker_time is not None and (
            now - self.last_marker_time > self.marker_timeout
            or stamp - self.dtime > self.marker_timeout
            or stamp < self.dtime
        ):
            self.tracking_reset_count += 1

        marker = msg.markers[0]
        position = marker.position.position
        self.curr_state = [position.x, position.y, position.z]
        self.drone_position = self.curr_state[:]

        self.yaw = math.degrees(marker.rotation.z)
        self.dtime = stamp

        self.marker_rec = True
        self.last_marker_time = now

    def set_gains(self, axis, msg):
        if not self.use_gui:
            return
        self.Kp[axis] = (msg.kp * self.gui_scale[axis][0])
        self.Ki[axis] = (msg.ki * self.gui_scale[axis][1])
        self.Kd[axis] = (msg.kd * self.gui_scale[axis][2])

    def altitude_set_pid(self, alt):
        self.set_gains(2, alt)

    def pitch_set_pid(self, pitch):
        self.set_gains(1, pitch)

    def roll_set_pid(self, roll):
        self.set_gains(0, roll)

    def odometry_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        self.roll_deg = math.degrees(roll)
        self.pitch_deg = math.degrees(pitch)
        self.yaw_deg = math.degrees(yaw)

    def pid(self):
        now = time.monotonic()

        if not self.armed:
            return

        marker_stale = (self.last_marker_time is None or now - self.last_marker_time > self.marker_timeout)

        if not self.marker_rec or marker_stale:
            self.marker_rec = False
            self.first_cycle = True
            self.error_sum = [0.0, 0.0, 0.0]
            self.filtered_der = [0.0, 0.0, 0.0]
            self.last_pid_time = None

            self.cmd.rc_pitch = self.neutral
            self.cmd.rc_roll = self.neutral
            self.cmd.rc_throttle = self.neutral
            self.command_pub.publish(self.cmd)
            return

        if self.last_pid_time is None:
            dt = self.sample_time
        else:
            dt = now - self.last_pid_time

        if dt <= 0.0 or dt > 0.15:
            dt = self.sample_time
            self.first_cycle = True

        self.last_pid_time = now

        for i in range(3):
            self.error[i] = (self.desired_state[i] - self.curr_state[i])

        if self.first_cycle:
            self.prev_error = self.error.copy()
            self.filtered_der = [0.0, 0.0, 0.0]
            self.first_cycle = False
        elif self.reset_xy_derivative:
            self.prev_error[:2] = self.error[:2]
            self.filtered_der[:2] = [0.0, 0.0]
        self.reset_xy_derivative = False

        gain_idx = [1, 0, 2]
        cmds = []

        for i in range(3):
            g = gain_idx[i]
            propp = (self.Kp[g] * self.error[i])
            raw_der = (self.error[i] - self.prev_error[i]) / dt
            self.filtered_der[i] = (self.der_alpha * raw_der + (1.0 - self.der_alpha) * self.filtered_der[i])
            der = (self.Kd[g] * self.filtered_der[i])
            cand_integral = (self.error_sum[i] + self.error[i] * dt)
            cand_integral = max(-self.i_limit, min(self.i_limit, cand_integral))

            cand_output = (propp + self.Ki[g] * cand_integral + der)
            cand_cmd = (self.neutral + self.sign[i] * cand_output)

            integral_chg = (self.sign[i] * self.Ki[g] * (cand_integral - self.error_sum[i]))
            pushes_up = (cand_cmd > self.max_values[i] and integral_chg > 0.0)
            pushes_low = (cand_cmd < self.min_values[i] and integral_chg < 0.0)

            if not (pushes_up or pushes_low):
                self.error_sum[i] = cand_integral

            integral = (self.Ki[g] * self.error_sum[i])
            output = (propp + integral + der)

            command = self.neutral + self.sign[i] * output
            command = max(self.min_values[i], min(self.max_values[i], command))
            cmds.append(int(round(command)))

            self.prev_error[i] = self.error[i]

        self.cmd.rc_pitch = cmds[0]
        self.cmd.rc_roll = cmds[1]
        self.cmd.rc_throttle = cmds[2]
        self.cmd.rc_yaw = 1500

        self.pos_error.pitch_error = self.error[0]
        self.pos_error.roll_error = self.error[1]
        self.pos_error.throttle_error = self.error[2]

        self.command_pub.publish(self.cmd)
        self.pos_error_pub.publish(self.pos_error)

    def reset_hover_time(self):
        self.point_in_sphere_start_time = None
        self.time_inside_sphere = 0.0
        self.max_time_inside_sphere = 0.0

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        previous_z = self.desired_state[2]
        self.desired_state[0] = goal_handle.request.waypoint.position.x
        self.desired_state[1] = goal_handle.request.waypoint.position.y
        self.desired_state[2] = goal_handle.request.waypoint.position.z

        # Keep vertical damping when only the horizontal target changes.
        if self.desired_state[2] != previous_z:
            self.first_cycle = True
        else:
            self.reset_xy_derivative = True

        self.get_logger().info(f'New Waypoint Set: {self.desired_state}')
        self.reset_hover_time()
        tracking_reset_count = self.tracking_reset_count
        self.duration = self.dtime

        feedback_msg = NavToWaypoint.Feedback()

        while rclpy.ok():
            if tracking_reset_count != self.tracking_reset_count:
                self.reset_hover_time()
                tracking_reset_count = self.tracking_reset_count
            feedback_msg.current_waypoint.pose.position.x = self.curr_state[0]
            feedback_msg.current_waypoint.pose.position.y = self.curr_state[1]
            feedback_msg.current_waypoint.pose.position.z = self.curr_state[2]
            feedback_msg.current_waypoint.header.stamp.sec = int(self.max_time_inside_sphere)

            goal_handle.publish_feedback(feedback_msg)

            marker_fresh = (
                self.marker_rec and self.last_marker_time is not None
                and time.monotonic() - self.last_marker_time <= self.marker_timeout
            )
            drone_is_in_box = marker_fresh and self.is_drone_in_box(self.drone_position, goal_handle, 0.4)

            if drone_is_in_box and not goal_handle.request.hover:
                break

            if not drone_is_in_box:
                self.reset_hover_time()
            elif self.point_in_sphere_start_time is None:
                self.point_in_sphere_start_time = self.dtime
                self.get_logger().info('Drone entered waypoint tolerance')
            else:
                self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time

            if self.time_inside_sphere > self.max_time_inside_sphere:
                self.max_time_inside_sphere = self.time_inside_sphere

            if self.time_inside_sphere >= 3.0:
                break

            time.sleep(0.033)

        goal_handle.succeed()

        result = NavToWaypoint.Result()
        result.hov_time = int(self.dtime - self.duration)
        return result

    def is_drone_in_box(self, drone_pos, goal_handle, tolerance):
        tolerance += 1e-9  # Allow floating point roundoff at the boundary.
        return (
            abs(drone_pos[0] - goal_handle.request.waypoint.position.x) <= tolerance
            and abs(drone_pos[1] - goal_handle.request.waypoint.position.y) <= tolerance
            and abs(drone_pos[2] - goal_handle.request.waypoint.position.z) <= tolerance
        )


def main(args=None):
    rclpy.init(args=args)

    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_server.disarm()
        waypoint_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
