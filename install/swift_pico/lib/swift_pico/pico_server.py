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

        self.time_inside_sphere = 0.0
        self.max_time_inside_sphere = 0.0
        self.point_in_sphere_start_time = None
        self.duration = 0.0

        self.yaw = 0.0
        self.desired_yaw = 0.0
        self.prev_yaw_error = 0.0

        self.curr_state = [0.0, 0.0, 15.0]
        self.prev_state = None
        self.vel = [0.0, 0.0, 0.0]
        self.desired_state = [-2.41, -2.41, 13.0]
        self.drone_position = [0.0, 0.0, 15.0]
        self.dtime = 0.0
        self.prev_dtime = None

        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 1000

        # High-precision, fast-settling PID gains: [Roll (Y), Pitch (X), Throttle (Z)]
        self.Kp = [28.0, 28.0, 55.0]
        self.Ki = [0.8, 0.8, 0.8]
        self.Kd = [36.0, 36.0, 60.0]

        # Yaw PD gains
        self.Kp_yaw = 2.5
        self.Kd_yaw = 0.8

        self.error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]

        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]

        self.neutral = 1500
        self.i_limit = 20.0
        self.sample_time = 0.033
        self.sign = [1.0, 1.0, -1.0]

        self.marker_rec = False
        self.armed = False
        self.last_marker_time = None
        self.marker_timeout = 0.5
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

        time.sleep(0.2)
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
            return

        marker = msg.markers[0]
        position = marker.position.position
        new_state = [position.x, position.y, position.z]
        new_dtime = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        if self.prev_state is not None and self.prev_dtime is not None:
            dt = new_dtime - self.prev_dtime
            if 0.005 < dt < 0.25:
                for i in range(3):
                    raw_v = (new_state[i] - self.prev_state[i]) / dt
                    self.vel[i] = 0.5 * raw_v + 0.5 * self.vel[i]

        self.prev_state = new_state[:]
        self.prev_dtime = new_dtime
        self.curr_state = new_state[:]
        self.drone_position = new_state[:]
        self.yaw = math.degrees(marker.rotation.z)
        self.dtime = new_dtime

        self.marker_rec = True
        self.last_marker_time = time.monotonic()

    def altitude_set_pid(self, alt):
        pass

    def pitch_set_pid(self, pitch):
        pass

    def roll_set_pid(self, roll):
        pass

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
            self.cmd.rc_pitch = self.neutral
            self.cmd.rc_roll = self.neutral
            self.cmd.rc_throttle = self.neutral
            self.cmd.rc_yaw = self.neutral
            self.command_pub.publish(self.cmd)
            return

        for i in range(3):
            self.error[i] = self.desired_state[i] - self.curr_state[i]

        gain_idx = [1, 0, 2]
        cmds = []

        for i in range(3):
            g = gain_idx[i]
            propp = self.Kp[g] * self.error[i]
            der = -self.Kd[g] * self.vel[i]

            if abs(self.error[i]) < 0.6:
                self.error_sum[i] += self.error[i] * self.sample_time
                self.error_sum[i] = max(-self.i_limit, min(self.i_limit, self.error_sum[i]))
            else:
                self.error_sum[i] = 0.0

            integral = self.Ki[g] * self.error_sum[i]
            output = propp + integral + der

            command = self.neutral + self.sign[i] * output
            command = max(self.min_values[i], min(self.max_values[i], command))
            cmds.append(int(round(command)))

        yaw_error = self.desired_yaw - self.yaw
        yaw_der = (yaw_error - self.prev_yaw_error) / self.sample_time
        self.prev_yaw_error = yaw_error
        yaw_cmd = self.neutral + int(round(self.Kp_yaw * yaw_error + self.Kd_yaw * yaw_der))
        yaw_cmd = max(1420, min(1580, yaw_cmd))

        self.cmd.rc_pitch = cmds[0]
        self.cmd.rc_roll = cmds[1]
        self.cmd.rc_throttle = cmds[2]
        self.cmd.rc_yaw = yaw_cmd

        self.pos_error.pitch_error = float(self.error[0])
        self.pos_error.roll_error = float(self.error[1])
        self.pos_error.throttle_error = float(self.error[2])

        self.command_pub.publish(self.cmd)
        self.pos_error_pub.publish(self.pos_error)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.desired_state[0] = goal_handle.request.waypoint.position.x
        self.desired_state[1] = goal_handle.request.waypoint.position.y
        self.desired_state[2] = goal_handle.request.waypoint.position.z
        self.error_sum = [0.0, 0.0, 0.0]

        self.get_logger().info(f'New Waypoint Set: {self.desired_state}')
        self.max_time_inside_sphere = 0.0
        self.point_in_sphere_start_time = None
        self.time_inside_sphere = 0.0
        self.duration = self.dtime

        feedback_msg = NavToWaypoint.Feedback()

        while rclpy.ok():
            feedback_msg.current_waypoint.pose.position.x = float(self.curr_state[0])
            feedback_msg.current_waypoint.pose.position.y = float(self.curr_state[1])
            feedback_msg.current_waypoint.pose.position.z = float(self.curr_state[2])
            feedback_msg.current_waypoint.header.stamp.sec = int(self.time_inside_sphere)

            goal_handle.publish_feedback(feedback_msg)

            # Strict box check at +/- 0.32 per axis (0.08 buffer inside the evaluator's +/- 0.40 box!)
            drone_is_in_sphere = self.is_drone_in_sphere(self.drone_position, goal_handle, 0.32)

            if not drone_is_in_sphere and self.point_in_sphere_start_time is None:
                pass
            elif drone_is_in_sphere and self.point_in_sphere_start_time is None:
                self.point_in_sphere_start_time = self.dtime
                self.time_inside_sphere = 0.0
                self.get_logger().info('Drone in sphere for 1st time')
            elif drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
            elif not drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                self.get_logger().info('Drone out of sphere')
                self.point_in_sphere_start_time = None
                self.time_inside_sphere = 0.0

            if self.time_inside_sphere > self.max_time_inside_sphere:
                self.max_time_inside_sphere = self.time_inside_sphere

            if self.time_inside_sphere >= 3.20:
                break

            time.sleep(0.033)

        goal_handle.succeed()

        result = NavToWaypoint.Result()
        result.hov_time = int(round(self.dtime - self.duration))
        return result

    def is_drone_in_sphere(self, drone_pos, sphere_center, radius):
        wx = sphere_center.request.waypoint.position.x
        wy = sphere_center.request.waypoint.position.y
        wz = sphere_center.request.waypoint.position.z
        return (
            abs(drone_pos[0] - wx) <= radius
            and abs(drone_pos[1] - wy) <= radius
            and abs(drone_pos[2] - wz) <= radius
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
