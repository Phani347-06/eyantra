#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, Scale
import threading
import rclpy
from rclpy.node import Node
from controller_msg.msg import PIDTune

class SliderNode(Node):
    def __init__(self):
        super().__init__('unified_pid_tuner')
        # Publish to both naming schemes so the controller captures them regardless
        self.pubs = {
            'throttle': [
                self.create_publisher(PIDTune, '/pid_tuning_altitude', 10),
                self.create_publisher(PIDTune, '/throttle_pid', 10)
            ],
            'pitch': [
                self.create_publisher(PIDTune, '/pid_tuning_pitch', 10),
                self.create_publisher(PIDTune, '/pitch_pid', 10)
            ],
            'roll': [
                self.create_publisher(PIDTune, '/pid_tuning_roll', 10),
                self.create_publisher(PIDTune, '/roll_pid', 10)
            ],
            'yaw': [
                self.create_publisher(PIDTune, '/pid_tuning_yaw', 10),
                self.create_publisher(PIDTune, '/yaw_pid', 10)
            ]
        }

    def publish_gains(self, axis, kp, ki, kd):
        msg = PIDTune()
        msg.kp = float(kp)
        msg.ki = float(ki)
        msg.kd = float(kd)
        for pub in self.pubs[axis]:
            pub.publish(msg)
        self.get_logger().info(f'[{axis.upper()}] Sent -> Kp={kp}, Ki={ki}, Kd={kd}')

class UnifiedTuningUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("Swift Pico Unified PID Tuner")
        self.root.geometry("450x380")
        self.root.attributes("-topmost", True)

        notebook = ttk.Notebook(root)
        notebook.pack(fill='both', expand=True, padx=8, pady=8)

        self.sliders = {}

        # Default values matching working hover dynamics
        defaults = {
            'throttle': (1700, 5, 750),
            'pitch': (100, 2, 50),
            'roll': (100, 2, 50),
            'yaw': (0, 0, 0)
        }

        for axis in ['throttle', 'pitch', 'roll', 'yaw']:
            frame = ttk.Frame(notebook)
            notebook.add(frame, text=axis.capitalize())
            self.create_axis_ui(frame, axis, defaults[axis])

    def create_axis_ui(self, parent, axis, init_vals):
        kp_init, ki_init, kd_init = init_vals

        s_kp = Scale(parent, orient="horizontal", from_=0, to=4000,
                     label="Kp", length=380, troughcolor="#e74c3c",
                     command=lambda e: self.on_slider(axis))
        s_kp.set(kp_init)
        s_kp.pack(pady=4)

        s_ki = Scale(parent, orient="horizontal", from_=0, to=200,
                     label="Ki", length=380, troughcolor="#2ecc71",
                     command=lambda e: self.on_slider(axis))
        s_ki.set(ki_init)
        s_ki.pack(pady=4)

        s_kd = Scale(parent, orient="horizontal", from_=0, to=2500,
                     label="Kd", length=380, troughcolor="#3498db",
                     command=lambda e: self.on_slider(axis))
        s_kd.set(kd_init)
        s_kd.pack(pady=4)

        self.sliders[axis] = (s_kp, s_ki, s_kd)

        btn = ttk.Button(parent, text=f"Publish {axis.capitalize()} Now",
                         command=lambda: self.on_slider(axis))
        btn.pack(pady=6)

    def on_slider(self, axis):
        s_kp, s_ki, s_kd = self.sliders[axis]
        self.node.publish_gains(axis, s_kp.get(), s_ki.get(), s_kd.get())

def main():
    rclpy.init()
    node = SliderNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = UnifiedTuningUI(root, node)
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
