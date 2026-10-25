#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from whycode_interfaces.msg import MarkerArray
from swift_msgs.msg import SwiftMsgs
from error_msg.msg import Error
from controller_msg.msg import PIDTune

class SwiftPicoPID(Node):
    def __init__(self):
        super().__init__('pico_controller_PID')

        # Task 2A Target Setpoint: (X, Y, Z) = (1.5, -1.5, 13.0)
        self.setpoint = [1.5, -1.5, 13.0]
        self.drone_position = [0.0, 0.0, 15.0]
        self.marker_detected = False

        # Controller states: [roll (Y), pitch (X), throttle (Z)]
        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]

        # Initial PID gains [Roll (Y), Pitch (X), Throttle (Z)]
        self.Kp = [0.66, 0.66, 2.55]
        self.Ki = [0.016, 0.016, 0.064]
        self.Kd = [7.2, 7.2, 9.6]

        self.max_error_sum = [200.0, 200.0, 150.0]
        self.sample_time = 0.033  # ~30 Hz

        # Arming handshake step counter
        self.arm_step = 0
        self.is_armed = False

        # Explicit QoS matching mujoco_bridge (RELIABLE + VOLATILE)
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', cmd_qos)
        self.error_pub = self.create_publisher(Error, '/pos_error', 10)

        self.create_subscription(MarkerArray, '/whycode_node/markers', self.whycode_callback, 10)
        self.create_subscription(PIDTune, '/pid_tuning_altitude', self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, '/pid_tuning_pitch', self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, '/pid_tuning_roll', self.roll_set_pid, 1)

        self.cmd_msg = SwiftMsgs()
        self.timer = self.create_timer(self.sample_time, self.pid_loop)
        self.get_logger().info('Task 2A Controller Initialized. Starting DISARM -> ARM sequence...')

    def altitude_set_pid(self, msg: PIDTune):
        self.Kp[2] = msg.kp * 0.03
        self.Ki[2] = msg.ki * 0.008
        self.Kd[2] = msg.kd * 0.6

    def pitch_set_pid(self, msg: PIDTune):
        self.Kp[1] = msg.kp * 0.03
        self.Ki[1] = msg.ki * 0.008
        self.Kd[1] = msg.kd * 0.6

    def roll_set_pid(self, msg: PIDTune):
        self.Kp[0] = msg.kp * 0.03
        self.Ki[0] = msg.ki * 0.008
        self.Kd[0] = msg.kd * 0.6

    def whycode_callback(self, msg: MarkerArray):
        if len(msg.markers) > 0:
            marker = msg.markers[0]
            if hasattr(marker, 'position') and hasattr(marker.position, 'position'):
                pos = marker.position.position
            elif hasattr(marker, 'pose') and hasattr(marker.pose, 'position'):
                pos = marker.pose.position
            else:
                pos = marker.position

            self.drone_position[0] = float(pos.x)
            self.drone_position[1] = float(pos.y)
            self.drone_position[2] = float(pos.z)
            self.marker_detected = True

    def pid_loop(self):
        # Phase 1: Explicit DISARM for 30 cycles (~1.0s)
        if self.arm_step < 30:
            self.cmd_msg.rc_roll = 1500
            self.cmd_msg.rc_pitch = 1500
            self.cmd_msg.rc_yaw = 1500
            self.cmd_msg.rc_throttle = 1000
            self.cmd_msg.rc_aux1 = 1500
            self.cmd_msg.rc_aux2 = 1500
            self.cmd_msg.rc_aux3 = 1500
            self.cmd_msg.rc_aux4 = 1000  # DISARMED
            self.command_pub.publish(self.cmd_msg)
            self.arm_step += 1
            return

        # Phase 2: Send ARM pulse for 30 cycles (~1.0s) with zero throttle
        if self.arm_step < 60:
            self.cmd_msg.rc_roll = 1500
            self.cmd_msg.rc_pitch = 1500
            self.cmd_msg.rc_yaw = 1500
            self.cmd_msg.rc_throttle = 1000
            self.cmd_msg.rc_aux1 = 1500
            self.cmd_msg.rc_aux2 = 1500
            self.cmd_msg.rc_aux3 = 1500
            self.cmd_msg.rc_aux4 = 2000  # ARMED (> 1500)
            self.command_pub.publish(self.cmd_msg)
            self.arm_step += 1
            if self.arm_step == 60:
                self.is_armed = True
                self.get_logger().info('Arm handshake complete. ARM signal locked at rc_aux4=2000!')
            return

        # Phase 3: PID Calculation & Active Flight
        pitch_error = self.setpoint[0] - self.drone_position[0]
        roll_error = self.setpoint[1] - self.drone_position[1]
        
        # INVERTED Z: In WhyCon frame, floor is ~15.0 and target hover is 13.0
        throttle_error = self.drone_position[2] - self.setpoint[2]

        self.error[0] = roll_error
        self.error[1] = pitch_error
        self.error[2] = throttle_error

        err_msg = Error()
        err_msg.pitch_error = float(pitch_error)
        err_msg.roll_error = float(roll_error)
        err_msg.throttle_error = float(throttle_error)
        self.error_pub.publish(err_msg)

        # Roll PID (Y)
        self.error_sum[0] = max(min(self.error_sum[0] + self.error[0], self.max_error_sum[0]), -self.max_error_sum[0])
        d_roll = self.error[0] - self.prev_error[0]
        out_roll = (self.Kp[0] * self.error[0]) + (self.Ki[0] * self.error_sum[0]) + (self.Kd[0] * d_roll)

        # Pitch PID (X)
        self.error_sum[1] = max(min(self.error_sum[1] + self.error[1], self.max_error_sum[1]), -self.max_error_sum[1])
        d_pitch = self.error[1] - self.prev_error[1]
        out_pitch = (self.Kp[1] * self.error[1]) + (self.Ki[1] * self.error_sum[1]) + (self.Kd[1] * d_pitch)

        # Throttle PID (Z)
        self.error_sum[2] = max(min(self.error_sum[2] + self.error[2], self.max_error_sum[2]), -self.max_error_sum[2])
        d_throttle = self.error[2] - self.prev_error[2]
        out_throttle = (self.Kp[2] * self.error[2]) + (self.Ki[2] * self.error_sum[2]) + (self.Kd[2] * d_throttle)

        # Map to RC PWM limits [1000, 2000]
        self.cmd_msg.rc_roll = int(max(min(1500 - out_roll, 2000), 1000))
        self.cmd_msg.rc_pitch = int(max(min(1500 + out_pitch, 2000), 1000))
        self.cmd_msg.rc_throttle = int(max(min(1500 + out_throttle, 2000), 1000))
        self.cmd_msg.rc_yaw = 1500
        self.cmd_msg.rc_aux1 = 1500
        self.cmd_msg.rc_aux2 = 1500
        self.cmd_msg.rc_aux3 = 1500
        self.cmd_msg.rc_aux4 = 2000  # Maintain ARMED state
        self.command_pub.publish(self.cmd_msg)

        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]

def main(args=None):
    rclpy.init(args=args)
    node = SwiftPicoPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
