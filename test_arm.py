#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from swift_msgs.msg import SwiftMsgs

class ArmTest(Node):
    def __init__(self):
        super().__init__('arm_test_node')
        
        # Explicitly match the subscribers' QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.pub = self.create_publisher(SwiftMsgs, '/drone_command', qos)
        self.msg = SwiftMsgs()
        self.count = 0
        self.timer = self.create_timer(0.033, self.timer_cb) # 30 Hz
        self.get_logger().info("Arm test started. Sending DISARM (1000) for 1 second...")

    def timer_cb(self):
        self.msg.rc_roll = 1500
        self.msg.rc_pitch = 1500
        self.msg.rc_yaw = 1500
        self.msg.rc_throttle = 1000

        # Phase 1: Send DISARM for ~1 second (30 ticks)
        if self.count < 30:
            self.msg.rc_aux4 = 1000
            self.pub.publish(self.msg)
            self.count += 1
            return

        # Phase 2: Send ARM for ~1 second (30 ticks) with idle throttle
        if self.count < 60:
            self.msg.rc_aux4 = 1500
            self.pub.publish(self.msg)
            self.count += 1
            if self.count == 60:
                self.get_logger().info("ARM edge sent! Now applying test throttle (1300)...")
            return

        # Phase 3: Spin motors up
        self.msg.rc_aux4 = 1500
        self.msg.rc_throttle = 1300
        self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = ArmTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
