import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Empty
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from cdpr_msgs.srv import Waypoint, ActivateTrajWaypoints, Move, TrajMove

import numpy as np
import matplotlib.pyplot as plt
import serial
import threading
from collections import deque
from cdpr_vision.cdpr_serial import serial_read_thread, serial_write, find_teensy_port


class Tracker(Node):

    def __init__(self):
        super().__init__("tracker")
        qos = QoSProfile(depth=10,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.timer = self.create_timer(1/300, self.timer_callback)

        self.home_service = self.create_service(Empty, "home", self.home_callback)
        self.move_service = self.create_service(Move, "move", self.move_callback)
        self.move_traj_service = self.create_service(TrajMove, "move_traj", self.move_traj_callback)
        self.load_square_service = self.create_service(Waypoint, "load_square", self.load_square_callback)
        self.load_diamond_service = self.create_service(Waypoint, "load_diamond", self.load_diamond_callback)
        self.track_service = self.create_service(Empty, "track", self.track_callback)
        self.track_traj_service = self.create_service(ActivateTrajWaypoints, "track_traj", self.track_traj_callback)
        self.plot_service = self.create_service(Empty, "plot", self.plot_callback)
        self.log_service = self.create_service(Empty, "log", self.log_callback)

        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self, qos)

        teensy_port = find_teensy_port()
        self.ser = serial.Serial(teensy_port, 115200, timeout=0.01)
        self.queue = deque(maxlen=500)
        self.tracking_flag = [False]
        self.thread = threading.Thread(target=serial_read_thread,
                                       args=(self.ser, self.queue, self.tracking_flag),
                                       daemon=True)
        self.thread.start()

        self.ee_pos_array = np.empty((0, 2))
        self.apriltag_pos_array = np.empty((0, 2))
        self.waypoints = np.empty((0, 2))
        self.use_traj = False
        self.traj_speed = 0.0
        self.center_frame_defined = False

    def timer_callback(self):
        if not self.center_frame_defined:
            self.get_logger().error("Waiting for center frame to be defined")
            try:
                t = self.buffer.lookup_transform(
                    'frame_tag',
                    'ee_tag',
                    rclpy.time.Time()
                )
                self.get_logger().info("Transform found! Publishing static transform...")

                frame2center = TransformStamped()
                frame2center.header.stamp = self.get_clock().now().to_msg()
                frame2center.header.frame_id = 'frame_tag'
                frame2center.child_frame_id = 'center'
                frame2center.transform.translation.x = 0.0
                frame2center.transform.translation.y = -0.53965
                frame2center.transform.translation.z = -0.1052592
                frame2center.transform.rotation.w = 1.0
                self.static_tf_broadcaster.sendTransform(frame2center)

                self.center_frame_defined = True

            except TransformException:
                pass  # still waiting
        else:
            if not self.tracking_flag[0]:
                return

            while len(self.queue) > 0:
                line = self.queue.popleft()
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
                transform = self.buffer.lookup_transform("center", "ee_tag", rclpy.time.Time(),
                                                         timeout=rclpy.time.Duration(seconds=0.01))
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                self.apriltag_pos_array = np.vstack([self.apriltag_pos_array, np.array([x, y])])
            except Exception:
                pass

    def home_callback(self, request, response):
        cmd = f"move 0 0\n"
        serial_write(self.ser, cmd)
        return response

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
        self.traj_speed = speed
        cmd = f"twaypoints {speed:.3f}\n"
        serial_write(self.ser, cmd)
        self.start_tracking()
        return response
    
    def plot_callback(self, request, response):
        plt.plot(self.waypoints[:, 0], self.waypoints[:, 1], '-ok')
        plt.plot(self.ee_pos_array[:, 0], self.ee_pos_array[:, 1], '-r')
        # plt.plot(self.apriltag_pos_array[:, 0], self.apriltag_pos_array[:, 1], '-b')
        plt.legend(["Ideal Path", "Estimated EE Pos"])
        # plt.legend(["Ideal Path", "Estimated EE Pos", "Actual EE Pos"])
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        if self.use_traj:
            plt.title(f"EE Movement Performance (FK Estimate) - Trajectory Control @ {self.traj_speed} m/s")
        else:
            plt.title("EE Movement Performance (FK Estimate) - Setpoint Control")
        plt.show()
        return response

    def log_callback(self, request, response):
        received_xy_pos = False
        while not received_xy_pos:
            try:
                transform = self.buffer.lookup_transform("center", "ee_tag", rclpy.time.Time(),
                                                            timeout=rclpy.time.Duration(seconds=0.01))
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                received_xy_pos = True
            except Exception:
                self.get_logger().info("Looking up transform betwen \"center\" and \"ee_tag\" frames")
        cmd = f"log {x:.4f} {y:.4f}\n"
        serial_write(self.ser, cmd)
        self.start_tracking()
        return response

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