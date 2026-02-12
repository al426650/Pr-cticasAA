import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from pymavlink import mavutil
import math

class IsaacArduBridge(Node):
    def __init__(self):
        super().__init__('isaac_ardu_bridge')

        # 1. Conexión MAVLink a ArduPilot SITL (Puerto 14551 para Física)
        self.get_logger().info("Abriendo conexión MAVLink en UDP:14551...")
        self.mav = mavutil.mavlink_connection('udp:127.0.0.1:14551')
        
        # No usamos wait_heartbeat() para evitar bloqueos en el arranque
        self.get_logger().info("Bridge iniciado. Esperando datos de Isaac Sim...")

        # Estado almacenado
        self.last_odom = None

        # 2. Suscriptores (Isaac Sim -> ROS 2)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/drone/odom', self.odom_callback, 10)

        # 3. Publicador (Bridge -> Isaac Sim)
        self.force_pub = self.create_publisher(Float64MultiArray, '/motor_forces', 10)

        # 4. Timers para MAVLink
        self.create_timer(0.002, self.read_motors_from_ardu)  # 500 Hz (Actuadores)
        self.create_timer(0.1, self.send_hil_gps)            # 10 Hz (GPS)
        self.create_timer(0.02, self.send_hil_state)         # 50 Hz (Estado/Horizonte)

        # Configuración
        self.base_lat, self.base_lon, self.base_alt = 39.986, -0.049, 0.0
        self.thrust_scalar = 0.3  # Aumentado para asegurar el despegue inicial

    def get_sim_time_us(self):
        """Obtiene el tiempo de la simulación en microsegundos."""
        return self.get_clock().now().nanoseconds // 1000

    def odom_callback(self, msg):
        self.last_odom = msg

    def imu_callback(self, msg):
        """Transforma IMU de ENU (Isaac) a NED (ArduPilot) e inyecta Barómetro."""
        # Mapeo ENU -> NED
        aN, aE, aD = msg.linear_acceleration.y, msg.linear_acceleration.x, -msg.linear_acceleration.z
        gN, gE, gD = msg.angular_velocity.y, msg.angular_velocity.x, -msg.angular_velocity.z

        # Inyección de Barómetro (Necesario para el EKF de ArduPilot)
        alt_z = self.last_odom.pose.pose.position.z if self.last_odom else 0.0
        press_abs = 1013.25 * (1 - 2.25577e-5 * alt_z)**5.25588 # Presión estática estimada

        # Campos actualizados: Accel(0x01) | Gyro(0x02) | Abs_Press(0x08)
        fields_updated = 0x01 | 0x02 | 0x08

        self.mav.mav.hil_sensor_send(
            self.get_sim_time_us(),
            aN, aE, aD, gN, gE, gD,
            0.0, 0.0, 0.0,      # Mag (no usado)
            press_abs, 0.0,     # Presión absoluta y diferencial
            alt_z, 0.0,         # Altitud barométrica y Temperatura
            fields_updated
        )

    def read_motors_from_ardu(self):
        """Recibe comandos de motor de ArduPilot y los manda a Isaac Sim."""
        msg = self.mav.recv_match(type='HIL_ACTUATOR_CONTROLS', blocking=False)
        if msg:
            forces = [float(f) * self.thrust_scalar for f in msg.controls[:4]]
            out_msg = Float64MultiArray(data=forces)
            self.force_pub.publish(out_msg)

    def send_hil_gps(self):
        """Simula GPS basado en la odometría de Isaac Sim."""
        if not self.last_odom: return
        o = self.last_odom
        
        # Posición y velocidad simplificada
        lat = self.base_lat + (o.pose.pose.position.y / 111111.0)
        lon = self.base_lon + (o.pose.pose.position.x / (111111.0 * math.cos(math.radians(self.base_lat))))
        
        vn, ve, vd = o.twist.twist.linear.y, o.twist.twist.linear.x, -o.twist.twist.linear.z
        speed = math.sqrt(vn**2 + ve**2 + vd**2)

        self.mav.mav.hil_gps_send(
            self.get_sim_time_us(), 3, int(lat * 1e7), int(lon * 1e7),
            int(o.pose.pose.position.z * 1000), 50, 50, 
            int(speed * 100), int(vn * 100), int(ve * 100), int(vd * 100), 0, 10
        )

    def send_hil_state(self):
        """Corrige el horizonte y soluciona el error de los 16 argumentos."""
        if not self.last_odom: return
        o = self.last_odom
        
        # 1. Corrección del Quaternion (ENU -> NED)
        # ROS [x, y, z, w] -> MAVLink [w, x, y, z] con transformación de ejes
        qx, qy, qz, qw = o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w
        q_ned = [qw, qy, qx, -qz] 

        # 2. Velocidades NED
        vN, vE, vD = o.twist.twist.linear.y, o.twist.twist.linear.x, -o.twist.twist.linear.z
        wN, wE, wD = o.twist.twist.angular.y, o.twist.twist.angular.x, -o.twist.twist.angular.z

        # 3. Envío con los 16 argumentos requeridos por Pymavlink
        self.mav.mav.hil_state_quaternion_send(
            self.get_sim_time_us(), q_ned, wN, wE, wD,
            0, 0, 0,                # Lat, Lon, Alt (usamos HIL_GPS)
            int(vN * 100), int(vE * 100), int(vD * 100), # Velocidades en cm/s
            0, 0,                   # Airspeed
            0, 0, 0                 # xacc, yacc, zacc (Ceros para evitar TypeError)
        )

def main():
    rclpy.init()
    node = IsaacArduBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()