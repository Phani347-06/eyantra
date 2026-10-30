#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given drone.
This node publishes and subsribes the following topics:

       PUBLICATIONS         SUBSCRIPTIONS
       /drone_command       /whycode_node/markers
       /pos_error          /throttle_pid
                         /pitch_pid
                         /roll_pid

Rather than using different variables, use list. eg : self.desired_state = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_desired_state = 1, self.y_desired_state = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''
from operator import truediv

# Importing the required libraries

from swift_msgs.msg import SwiftMsgs
from whycode_interfaces.msg import MarkerArray
from controller_msg.msg import PIDTune
from error_msg.msg import Error
from std_msgs.msg import Header
import time
import rclpy
from rclpy.node import Node


class Swift_Pico(Node):
    def __init__(self):
        super().__init__('pico_controller')  # initializing ros node with name pico_controller

        # This corresponds to your current position of drone. This value must be updated in your whycode callback
        # [x,y,z]
        self.current_state = [0.0, 0.0, 0.0]

        # This corresponds to the setpoint you want the drone to reach or hold
        # [x_desired_state, y_desired_state, z_desired_state]
        self.desired_state = [1.5, -1.5,
                              13.0]  # whycode marker at the position of the drone given in the scene. Make the whycode marker associated with position_to_hold drone renderable and make changes accordingly

        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 1000

        # initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        # after tuning and computing corresponding PID parameters, change the parameters

        self.Kp = [4.5, 4.5, 82.5]
        self.Ki = [0.0, 0.0, 0.2]
        self.Kd = [0.0, 0.0, 21.0]

        # -----------------------Add other required variables for pid here ----------------------------------------------

        # Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]       #      Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
        #                                       self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
        #                                                   You can change the upper limit and lower limit accordingly.
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit.

        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]

        self.filtered_der = [0.0, 0.0, 0.0]
        self.der_alpha = 0.2

        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]

        self.neutral = 1500
        self.i_limit = 5.0

        self.sample_time = 0.033  # in seconds

        self.sign = [1.0, 1.0, -1.0]

        self.use_gui = True

        self.gui_scale = [
            [30.0, 1.0, 1.7],
            [30.0, 1.0, 1.7],
            [50.0, 1.0, 0.7]
        ]

        self.marker_rec = False
        self.first_cycle = True
        self.armed = False

        self.last_pid_time = None
        self.last_marker_time = None

        self.marker_timeout = 0.5
        self.pos_error = Error()

        # Publishing /drone_command, /pos_error
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)
        self.heartbeat_pub = self.create_publisher(Header, '/controller_heartbeat', 10)

        # ------------------------Add other ROS 2 Publishers here-----------------------------------------------------

        # Subscribing to /whycode_node/markers, /throttle_pid, /pitch_pid, roll_pid
        self.create_subscription(MarkerArray, '/whycode_node/markers', self.whycode_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)
        self.heartbeat_timer = self.create_timer(0.1, self.publish_heartbeat)
        self.pid_timer = self.create_timer(self.sample_time, self.pid)
        self.disarm()

        # ------------------------Add other ROS Subscribers here-----------------------------------------------------

        self.publish_heartbeat()
        self.arm_timer = self.create_timer(0.5, self.arm_once)
        self.get_logger().info('Pico PID Controller Initialised')


    # Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)

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

    def arm_once(self):
        self.arm_timer.cancel()
        self.arm()

    def arm(self):
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

        self.command_pub.publish(self.cmd)  # Publishing /drone_command

        self.armed = True
        self.get_logger().info('Drone armed.')


    # Whycode callback function
    # The function gets executed each time when /whycode_node publishes /whycode_node/markers
    def whycode_callback(self, msg):
        if not msg.markers:
            self.marker_rec = False
            self.first_cycle = True

            self.error_sum = [0.0, 0.0, 0.0]
            self.filtered_der = [0.0, 0.0, 0.0]

            return

        pos = msg.markers[0].position.position
        self.current_state[0] = pos.x
        self.current_state[1] = pos.y
        self.current_state[2] = pos.z

        self.marker_rec = True
        self.last_marker_time = time.monotonic()

    # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
    def set_gains(self, axis, msg):
        if not self.use_gui:
            return

        self.Kp[axis] = (msg.kp * self.gui_scale[axis][0])
        self.Ki[axis] = (msg.ki * self.gui_scale[axis][1])
        self.Kd[axis] = (msg.kd * self.gui_scale[axis][2])
    # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /throttle_pid
    # This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
    def altitude_set_pid(self, alt):
        # self.Kp[2] = alt.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
        # self.Ki[2] = alt.ki * 0.008
        # self.Kd[2] = alt.kd * 0.6
        self.set_gains(2, alt)

    # ----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
    def pitch_set_pid(self, pitch):
        self.set_gains(1, pitch)

    def roll_set_pid(self, roll):
        self.set_gains(0, roll)
    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        #   1. Compute error in each axis. eg: error[0] = self.current_state[0] - self.desired_state[0] ,where error[0] corresponds to error in x...
        #   2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        #   3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        #   4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        #   5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        #   6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
        #                                                                                          self.cmd.rcPitch = self.max_values[1]
        #   7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        #   8. Add error_sum

        # ------------------------------------------------------------------------------------------------------------------------
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
        # calculate throttle error, pitch error and roll error, then publish it accordingly
        if self.last_pid_time is None:
            dt = self.sample_time
        else:
            dt = now - self.last_pid_time

        if dt <= 0.0 or dt > 0.15:
            dt = self.sample_time
            self.first_cycle = True

        self.last_pid_time = now

        for i in range(3):
            self.error[i] = (self.desired_state[i] - self.current_state[i])

        if self.first_cycle:
            self.prev_error = self.error.copy()
            self.filtered_der = [0.0, 0.0, 0.0]

            self.first_cycle = False

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

def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()

    try:
        rclpy.spin(swift_pico)
    except KeyboardInterrupt:
        swift_pico.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        swift_pico.disarm()
        swift_pico.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
