#!/usr/bin/env python3
import json
import math
import socket
import struct
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


# --- FUNCIONES MATEMÁTICAS (IGUAL QUE ANTES) ---
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
    if n < 1e-12: return (1.0, 0.0, 0.0, 0.0)
    return (w/n, x/n, y/n, z/n)

def enu_to_ned(vx, vy, vz):
    return (vy, vx, -vz)

def flu_to_frd(vx, vy, vz):
    return (vx, -vy, -vz)

def quat_ros_enu_flu_to_ap_ned_frd(qx, qy, qz, qw):
    q_ros = (qw, qx, qy, qz)
    q_ned_enu = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
    q_frd_flu = (0.0, 1.0, 0.0, 0.0)
    q_ap = quat_mul_wxyz(quat_mul_wxyz(q_ned_enu, q_ros), q_frd_flu)
    return quat_norm_wxyz(q_ap)


class IsaacJsonBackend(Node):
    def __init__(self):
        super().__init__("isaac_json_backend")

        # --- CONFIGURACIÓN ---
        self.MAX_THRUST_N = 15  # Ajustado a tu valor óptimo
        self.TARGET_FREQ = 200.0  # Hz objetivo

        # Suscripciones ROS
        # QoS 10 es suficiente, no necesitamos historial grande
        self.create_subscription(Imu, "/imu", self.imu_cb, 10)
        self.create_subscription(Odometry, "/drone/odom", self.odom_cb, 10)
        self.motor_pub = self.create_publisher(Float64MultiArray, "/motor_forces", 10)

        # SOCKET UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 9002))
        
        # IMPORTANTE: Quitamos setblocking(False). 
        # Ahora usamos un hilo que se bloquea esperando datos eficientemente.
        self.sock.settimeout(1.0) 

        self.last_peer: Optional[Tuple[str, int]] = None
        self.last_imu: Optional[Imu] = None
        self.last_odom: Optional[Odometry] = None
        self.origin_enu = None 

        self.running = True

        # --- HILO 1: RECEPTOR UDP (Escucha Motores) ---
        # Este hilo corre paralelo y NO frena el envío de JSON
        self.rx_thread = threading.Thread(target=self.udp_receiver_loop)
        self.rx_thread.daemon = True
        self.rx_thread.start()

        # --- HILO 2 (Principal): ENVÍO JSON (Cronometrado) ---
        self.timer = self.create_timer(1.0 / self.TARGET_FREQ, self.send_json_loop)

        self.get_logger().info(f"Backend Optimizado (Threaded) iniciado a {self.TARGET_FREQ}Hz")

    def imu_cb(self, msg: Imu):
        self.last_imu = msg

    def odom_cb(self, msg: Odometry):
        self.last_odom = msg
        if self.origin_enu is None:
            p = msg.pose.pose.position
            self.origin_enu = (p.x, p.y, p.z)
            self.get_logger().info(f"Origen fijado: {self.origin_enu}")

    # --- LÓGICA DEL HILO RECEPTOR (RX) ---
    def udp_receiver_loop(self):
        """Este bucle solo se dedica a recibir datos de ArduPilot y publicarlos en ROS."""
        while self.running and rclpy.ok():
            try:
                # recvfrom ahora espera hasta que llegue algo (eficiente para la CPU)
                data, peer = self.sock.recvfrom(2048)
                self.last_peer = peer # Actualizamos a quién contestar
                self._process_servo_packet(data)
            except socket.timeout:
                continue # Si no llega nada en 1 seg, seguimos
            except Exception as e:
                self.get_logger().error(f"Error RX: {e}")

    def _process_servo_packet(self, data: bytes):
        if len(data) < 8: return

        magic, _, _ = struct.unpack_from("<HHI", data, 0)
        if magic == 18458: n = 16
        elif magic == 29569: n = 32
        else: return

        expected = 8 + 2*n
        if len(data) < expected: return

        pwms = struct.unpack_from("<" + "H"*n, data, 8)

        # Normalizar PWM
        u = []
        for i in range(4):
            val = (pwms[i] - 1000.0) / 1000.0
            u.append(max(0.0, min(1.0, val)))

        # Mapeo Motores
        forces = [0.0] * 4
        forces[0] = u[2] * self.MAX_THRUST_N  # FL
        forces[1] = u[0] * self.MAX_THRUST_N  # FR
        forces[2] = u[3] * self.MAX_THRUST_N  # RR
        forces[3] = u[1] * self.MAX_THRUST_N  # RL

        msg = Float64MultiArray()
        msg.data = forces
        self.motor_pub.publish(msg) # Thread-safe en rclpy

    # --- LÓGICA DEL HILO PRINCIPAL (TX) ---
    def send_json_loop(self):
        """Este bucle se encarga de enviar el estado a ArduPilot a 200Hz."""
        if self.last_peer is None or self.last_imu is None or self.last_odom is None or self.origin_enu is None:
            return
        
        payload = self._build_json_frame()
        if payload:
            try:
                self.sock.sendto(payload, self.last_peer)
            except Exception as e:
                pass # Ignorar errores puntuales de red

    def _build_json_frame(self) -> Optional[bytes]:
        imu = self.last_imu
        odom = self.last_odom

        t = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9

        # Convertir datos
        gx, gy, gz = flu_to_frd(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z)
        ax, ay, az = flu_to_frd(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z)

        p = odom.pose.pose.position
        x, y, z = (p.x - self.origin_enu[0], p.y - self.origin_enu[1], p.z - self.origin_enu[2])
        pn, pe, pd = enu_to_ned(x, y, z)

        v = odom.twist.twist.linear
        vn, ve, vd = enu_to_ned(v.x, v.y, v.z)

        q = odom.pose.pose.orientation
        q1, q2, q3, q4 = quat_ros_enu_flu_to_ap_ned_frd(q.x, q.y, q.z, q.w)

        frame = {
            "timestamp": t,
            "imu": {"gyro": [gx, gy, gz], "accel_body": [ax, ay, az]},
            "position": [pn, pe, pd],
            "velocity": [vn, ve, vd],
            "quaternion": [q1, q2, q3, q4],
        }
        return (json.dumps(frame) + "\n").encode("utf-8")

def main():
    rclpy.init()
    node = IsaacJsonBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
