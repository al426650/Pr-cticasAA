#!/usr/bin/env python3

import math
import socket
import struct
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


# ----------------------------
# Math utilities (fast path)
# ----------------------------

_SQRT_HALF = math.sqrt(0.5)

def quat_mul_wxyz(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )

def quat_norm_wxyz(q):
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    inv = 1.0 / n
    return (w*inv, x*inv, y*inv, z*inv)

def enu_to_ned(vx, vy, vz):
    # ENU -> NED: (xE, yN, zU) -> (xN, yE, zD)
    return (vy, vx, -vz)

def flu_to_frd(vx, vy, vz):
    # FLU -> FRD
    return (vx, -vy, -vz)

def quat_ros_enu_flu_to_ap_ned_frd(qx, qy, qz, qw):
    # ROS quaternion given as (x,y,z,w) in ENU/FLU -> ArduPilot NED/FRD (w,x,y,z)
    q_ros = (qw, qx, qy, qz)

    # Fixed rotations:
    # ENU -> NED (90deg about X then 180 about Z equivalently encoded as below)
    q_ned_enu = (0.0, _SQRT_HALF, _SQRT_HALF, 0.0)

    # FLU -> FRD (180deg about X): (w=0, x=1, y=0, z=0)
    q_frd_flu = (0.0, 1.0, 0.0, 0.0)

    q_ap = quat_mul_wxyz(quat_mul_wxyz(q_ned_enu, q_ros), q_frd_flu)
    return quat_norm_wxyz(q_ap)


# Compact JSON template (no spaces) + newline, expected by ArduPilot JSON backend
_JSON_TPL = (
    '{"timestamp":%.6f,'
    '"imu":{"gyro":[%.6f,%.6f,%.6f],"accel_body":[%.6f,%.6f,%.6f]},'
    '"position":[%.6f,%.6f,%.6f],'
    '"velocity":[%.6f,%.6f,%.6f],'
    '"quaternion":[%.6f,%.6f,%.6f,%.6f]}\n'
)

def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class IsaacJsonBackend(Node):
    def __init__(self):
        super().__init__("isaac_json_backend")

        # ----------------------------
        # Parameters / config
        # ----------------------------
        self.MAX_THRUST_N = 15.0

        # BestEffort + KeepLast(1) for high-rate streams
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        self.create_subscription(Imu, "/imu", self.imu_cb, qos_sensor)
        self.create_subscription(Odometry, "/drone/odom", self.odom_cb, qos_sensor)
        self.motor_pub = self.create_publisher(Float64MultiArray, "/motor_forces", qos_pub)

        # ----------------------------
        # UDP socket (RX+TX)
        # ----------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Reuse address to avoid "Address already in use" during quick restarts
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", 9002))

        self.last_peer: Optional[Tuple[str, int]] = None

        # Latest ROS messages
        self.last_imu: Optional[Imu] = None
        self.last_odom: Optional[Odometry] = None

        # ENU origin for position
        self.origin_enu = None  # tuple(x,y,z)

        # Avoid sending duplicates
        self._last_sent_imu_stamp = None  # (sec, nanosec)

        # Thread control
        self._lock = threading.Lock()
        self.running = True

        # RX thread: block on recvfrom (efficient)
        self.rx_thread = threading.Thread(target=self._udp_receiver_loop, daemon=True)
        self.rx_thread.start()

    # ----------------------------
    # ROS callbacks
    # ----------------------------
    def odom_cb(self, msg: Odometry):
        with self._lock:
            self.last_odom = msg
            if self.origin_enu is None:
                p = msg.pose.pose.position
                self.origin_enu = (p.x, p.y, p.z)
                self.get_logger().info(f"Origin fixed (ENU): {self.origin_enu}")

    def imu_cb(self, msg: Imu):
        # Event-driven TX: send immediately when new IMU arrives (best for control)
        with self._lock:
            self.last_imu = msg

            if self.last_peer is None or self.last_odom is None or self.origin_enu is None:
                return

            st = msg.header.stamp
            stamp_key = (st.sec, st.nanosec)
            if stamp_key == self._last_sent_imu_stamp:
                return  # prevent duplicate sends if callbacks re-enter
            self._last_sent_imu_stamp = stamp_key

            payload = self._build_json_frame_locked()
            peer = self.last_peer

        # Send outside lock (avoid blocking other callbacks)
        if payload is not None:
            try:
                self.sock.sendto(payload, peer)
            except Exception:
                pass

    # ----------------------------
    # UDP RX thread
    # ----------------------------
    def _udp_receiver_loop(self):
        while self.running and rclpy.ok():
            try:
                data, peer = self.sock.recvfrom(2048)  # blocking
                with self._lock:
                    self.last_peer = peer
                self._process_servo_packet(data)
            except OSError:
                # Socket closed during shutdown
                break
            except Exception:
                # Ignore transient RX errors
                continue

    def _process_servo_packet(self, data: bytes):
        # Packet header: magic(uint16), frame_rate(uint16), frame_count(uint32)
        if len(data) < 8:
            return

        magic, _, _ = struct.unpack_from("<HHI", data, 0)
        if magic == 18458:
            n = 16
        elif magic == 29569:
            n = 32
        else:
            return

        expected = 8 + 2 * n
        if len(data) < expected:
            return

        # Only need first 4 channels for quad
        # pwms[0..3] in [1000..2000]
        p0, p1, p2, p3 = struct.unpack_from("<HHHH", data, 8)

        # Normalize to [0..1]
        u0 = (p0 - 1000.0) * 0.001
        u1 = (p1 - 1000.0) * 0.001
        u2 = (p2 - 1000.0) * 0.001
        u3 = (p3 - 1000.0) * 0.001

        # Clamp
        if u0 < 0.0: u0 = 0.0
        elif u0 > 1.0: u0 = 1.0
        if u1 < 0.0: u1 = 0.0
        elif u1 > 1.0: u1 = 1.0
        if u2 < 0.0: u2 = 0.0
        elif u2 > 1.0: u2 = 1.0
        if u3 < 0.0: u3 = 0.0
        elif u3 > 1.0: u3 = 1.0

        # Motor mapping (keep your current mapping)
        # forces[0]=FL, [1]=FR, [2]=RR, [3]=RL
        f0 = u2 * self.MAX_THRUST_N  # FL
        f1 = u0 * self.MAX_THRUST_N  # FR
        f2 = u3 * self.MAX_THRUST_N  # RR
        f3 = u1 * self.MAX_THRUST_N  # RL

        msg = Float64MultiArray()
        msg.data = [f0, f1, f2, f3]
        self.motor_pub.publish(msg)

    # ----------------------------
    # JSON build (called under lock)
    # ----------------------------
    def _build_json_frame_locked(self) -> Optional[bytes]:
        imu = self.last_imu
        odom = self.last_odom
        if imu is None or odom is None or self.origin_enu is None:
            return None

        t = _stamp_to_float(imu.header.stamp)

        # IMU (ROS FLU) -> AP FRD
        gx, gy, gz = flu_to_frd(
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
        )
        ax, ay, az = flu_to_frd(
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        )

        # Position: subtract origin in ENU then convert to NED
        p = odom.pose.pose.position
        x = p.x - self.origin_enu[0]
        y = p.y - self.origin_enu[1]
        z = p.z - self.origin_enu[2]
        pn, pe, pd = enu_to_ned(x, y, z)

        # Velocity: ENU -> NED
        v = odom.twist.twist.linear
        vn, ve, vd = enu_to_ned(v.x, v.y, v.z)

        # Orientation: ENU/FLU -> NED/FRD
        q = odom.pose.pose.orientation
        q1, q2, q3, q4 = quat_ros_enu_flu_to_ap_ned_frd(q.x, q.y, q.z, q.w)

        s = _JSON_TPL % (
            t,
            gx, gy, gz,
            ax, ay, az,
            pn, pe, pd,
            vn, ve, vd,
            q1, q2, q3, q4,
        )
        return s.encode("utf-8")

    # ----------------------------
    # Shutdown
    # ----------------------------
    def shutdown(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = IsaacJsonBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
