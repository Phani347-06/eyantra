#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from error_msg.msg import Error
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time

class ErrorPlotter(Node):
    def __init__(self):
        super().__init__('error_plotter')
        self.sub = self.create_subscription(Error, '/pos_error', self.error_cb, 10)

        self.history_len = 120
        self.t_data = deque(maxlen=self.history_len)
        self.roll_err = deque(maxlen=self.history_len)
        self.pitch_err = deque(maxlen=self.history_len)
        self.alt_err = deque(maxlen=self.history_len)
        self.yaw_err = deque(maxlen=self.history_len)

        self.start_time = time.monotonic()

    def error_cb(self, msg: Error):
        t = time.monotonic() - self.start_time
        self.t_data.append(t)
        self.roll_err.append(getattr(msg, 'roll_error', 0.0))
        self.pitch_err.append(getattr(msg, 'pitch_error', 0.0))
        self.alt_err.append(getattr(msg, 'throttle_error', 0.0))
        self.yaw_err.append(getattr(msg, 'yaw_error', 0.0))

def main():
    rclpy.init()
    node = ErrorPlotter()

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(10, 6))
    fig.canvas.manager.set_window_title('Swift Pico 4-Axis Live Error Monitor')

    axes = [ax1, ax2, ax3, ax4]
    titles = ['Roll Error (Y)', 'Pitch Error (X)', 'Altitude Error (Z)', 'Yaw Error']
    colors = ['#e74c3c', '#2ecc71', '#3498db', '#f39c12']

    lines = []
    for ax, title, color in zip(axes, titles, colors):
        ax.set_title(title, fontsize=10, fontweight='bold')
        ax.set_xlabel('Time (s)', fontsize=8)
        ax.set_ylabel('Error', fontsize=8)
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.axhline(0, color='gray', linestyle=':', alpha=0.7)
        line, = ax.plot([], [], color=color, lw=1.8)
        lines.append(line)

    plt.tight_layout()

    def update(frame):
        rclpy.spin_once(node, timeout_sec=0.005)
        if len(node.t_data) > 1:
            t = list(node.t_data)
            data_series = [list(node.roll_err), list(node.pitch_err), list(node.alt_err), list(node.yaw_err)]

            for ax, line, d in zip(axes, lines, data_series):
                line.set_data(t, d)
                ax.set_xlim(t[0], t[-1] + 0.1)
                min_v, max_v = min(d), max(d)
                pad = max(abs(max_v - min_v) * 0.25, 0.2)
                ax.set_ylim(min_v - pad, max_v + pad)

        return lines

    ani = FuncAnimation(fig, update, interval=40, blit=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
