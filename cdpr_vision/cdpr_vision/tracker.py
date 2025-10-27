import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Empty
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from cdpr_msgs.srv import Waypoint, ActivateTrajWaypoints, Move, MoveTraj

import numpy as np
import matplotlib.pyplot as plt
import serial
import threading
import time
from collections import deque
from cdpr_vision.cdpr_serial import serial_read_thread, serial_write


class Tracker(Node):

    def __init__(self):
        super().__init__("Tracker")
        qos = QoSProfile(depth=10,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.timer = self.create_timer(1/100, self.timer_callback)

        self.move_service = self.create_service(Move, "move", self.move_callback)
        self.move_traj_service = self.create_service(MoveTraj, "move_traj", self.move_traj_callback)
        self.load_square_service = self.create_service(Waypoint, "load_square", self.load_square_callback)
        self.load_diamond_service = self.create_service(Waypoint, "load_diamond", self.load_diamond_callback)
        self.track_service = self.create_service(Empty, "track", self.track_callback)
        self.track_traj_service = self.create_service(ActivateTrajWaypoints, "track_traj", self.track_traj_callback)
        self.plot_service = self.create_service(Empty, "plot", self.plot_callback)

        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self, qos)

        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.01)
        self.queue = deque(maxlen=500)
        self.tracking_flag = [False]
        self.thread = threading.Thread(target=serial_read_thread,
                                       args=(self.ser, self.queue, self.tracking_flag),
                                       daemon=True)
        self.thread.start()

        # Wait until the required frames exist
        self.get_logger().info("Waiting for 'frame_tag' -> 'ee_tag' transform to appear...")
        while rclpy.ok():
            try:
                # Timeout of 0 seconds just checks if it’s available right now
                if self.buffer.can_transform('frame_tag', 'ee_tag', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1)):
                    self.get_logger().info("'frame_tag' -> 'ee_tag' transform found!")
                    break
            except Exception:
                pass
            time.sleep(0.05)  # avoid busy-wait

        frame2center = TransformStamped()
        frame2center.header.stamp = self.get_clock().now().to_msg()
        frame2center.header.frame_id = 'frame_tag'
        frame2center.child_frame_id = 'center'
        frame2center.transform.translation.y = -.53965
        frame2center.transform.translation.z = -.1052592
        self.static_tf_broadcaster.sendTransform(frame2center)
        self.get_logger().info("Static transform published from frame_tag -> center")

        self.ee_pos_array = np.empty((0, 2))
        self.apriltag_pos_array = np.empty((0, 2))
        self.waypoints = np.empty((0, 2))
        self.use_traj = False

    def timer_callback(self):
        if not self.tracking_flag[0]:
            return

        while not self.queue.empty():
            line = self.queue.get_nowait()
            if line == "done":
                self.stop_tracking()
            else:
                try:
                    x_str, y_str = line.split(",")
                    x, y = float(x_str), float(y_str)
                    self.ee_pos_array = np.vstack([self.ee_pos_array, np.array([x, y])])
                except Exception:
                    self.get_logger().warn(f"Invalid serial line: {line}")
        
        try:
            transform = self.buffer.lookup_transform("center", "ee_tag", rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            self.apriltag_pos_array = np.vstack([self.apriltag_pos_array, np.array([x, y])])
        except Exception:
            pass

    def move_callback(self, request, response):
        x = request.x
        y = request.y
        cmd = f"move {x:.3f} {y:.3f}\n"
        serial_write(self.ser, cmd)
        return response

    def move_traj_callback(self, request, response):
        x = request.x
        y = request.y
        speed = request.speed
        cmd = f"tmove {x:.3f} {y:.3f} {speed:.3f}\n"
        serial_write(self.ser, cmd)
        return response

    def load_square_callback(self, request, response):
        side_len = request.side_len
        x = request.x
        y = request.y

        half = side_len / 2
        self.waypoints = np.array([
            [x + half, y + half],
            [x - half, y + half],
            [x - half, y - half],
            [x + half, y - half],
            [x + half, y + half]
        ])

        cmd = f"loads {side_len:.3f} {x:.3f} {y:.3f}\n"
        serial_write(self.ser, cmd)
        return response

    def load_diamond_callback(self, request, response):
        side_len = request.side_len
        x = request.x
        y = request.y

        center2corner = side_len * np.sqrt(2) / 2
        self.waypoints = np.array([
            [x, y + center2corner],
            [x - center2corner, y],
            [x, y - center2corner],
            [x + center2corner, y],
            [x, y + center2corner]  # back to start
        ])

        cmd = f"loadd {side_len:.3f} {x:.3f} {y:.3f}\n"
        serial_write(self.ser, cmd)
        return response

    def track_callback(self, request, response):
        self.use_traj = False
        cmd = "waypoints\n"
        serial_write(self.ser, cmd)
        self.start_tracking()
        return response

    def track_traj_callback(self, request, response):
        self.use_traj = True
        speed = request.speed
        cmd = f"twaypoints {speed:.3f}\n"
        serial_write(self.ser, cmd)
        self.start_tracking()
        return response
    
    def plot_callback(self):
        plt.plot(self.waypoints[:, 0], self.waypoints[:, 1], '-ok')
        plt.plot(self.ee_pos_array[:, 0], self.ee_pos_array[:, 1], '-r')
        plt.plot(self.apriltag_pos_array[:, 0], self.apriltag_pos_array[:, 1], '-b')
        plt.legend(["Ideal Path", "Estimated EE Pos", "Actual EE Pos"])
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        if self.use_traj:
            plt.title("EE Movement Performance - Trajectory Control")
        else:
            plt.title("EE Movement Performance - Position Control")


    def start_tracking(self):
        self.queue.clear()
        self.ee_pos_array = np.empty((0, 2))
        self.apriltag_pos_array = np.empty((0, 2))
        self.tracking_flag[0] = True
        self.get_logger().info("Tracking started")

    def stop_tracking(self):
        self.tracking_flag[0] = False
        self.get_logger().info("Tracking stopped")


def main(args=None):
    """Entrypoint for the tracker ROS 2 node."""
    rclpy.init(args=args)
    node = Tracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)