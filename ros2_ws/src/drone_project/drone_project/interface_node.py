import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
import time

class InterfaceNode(Node):
    def __init__(self):
        super().__init__('interface_node')
        self.get_logger().info("Conectando con ArduPilot (UDP:14550)...")
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550') 
        self.connection.wait_heartbeat()
        self.get_logger().info("✅ CONECTADO A SITL.")

        # Publishers
        self.gps_publisher_ = self.create_publisher(NavSatFix, '/drone/gps', 10)
        self.battery_publisher_ = self.create_publisher(Float64, '/drone/battery', 10)

        # Servicios
        self.arm_service = self.create_service(SetBool, '/drone/arm', self.arm_callback)
        self.takeoff_service = self.create_service(SetBool, '/drone/takeoff', self.takeoff_callback)

        # Suscriptor
        self.cmd_sub = self.create_subscription(PoseStamped, '/drone/cmd_pose', self.go_to_callback, 10)

        # Timer rápido (evita bloqueos)
        self.timer = self.create_timer(0.05, self.update_sensors)

    def update_sensors(self):
        # Leemos mensajes sin bloquear
        msg = self.connection.recv_match(type=['GLOBAL_POSITION_INT', 'BATTERY_STATUS'], blocking=False)
        if not msg: return

        if msg.get_type() == 'GLOBAL_POSITION_INT':
            ros_msg = NavSatFix()
            ros_msg.header.stamp = self.get_clock().now().to_msg()
            ros_msg.header.frame_id = "map"
            ros_msg.latitude = msg.lat / 1e7
            ros_msg.longitude = msg.lon / 1e7
            ros_msg.altitude = msg.relative_alt / 1000.0 
            self.gps_publisher_.publish(ros_msg)

        elif msg.get_type() == 'BATTERY_STATUS':
            msg_bat = Float64()
            msg_bat.data = float(msg.battery_remaining)
            self.battery_publisher_.publish(msg_bat)

    def arm_callback(self, request, response):
        # 1. Forzamos modo GUIDED antes de armar (Mejor práctica)
        self.connection.set_mode('GUIDED')
        
        cmd = 1 if request.data else 0
        self.get_logger().info(f"Enviando comando de ARMADO: {cmd}")
        
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, cmd, 0, 0, 0, 0, 0, 0
        )
        
        # Esperamos confirmación (con timeout para no colgar ROS)
        if cmd:
            self.connection.motors_armed_wait()
            self.get_logger().info("✅ Dron ARMADO y listo.")
            response.message = "Armado OK"
        else:
            self.get_logger().info("⚠️ Dron DESARMADO")
            response.message = "Desarmado OK"
            
        response.success = True
        return response

    def takeoff_callback(self, request, response):
        if request.data:
            # Altura segura y estándar
            altitude = 10 
            
            self.get_logger().info(f"🛫 Iniciando DESPEGUE a {altitude}m...")
            
            # Comando TAKEOFF directo
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, 
                altitude
            )
            response.message = "Despegue enviado"
        else:
            # Comando LAND
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            self.get_logger().info("🛬 Iniciando ATERRIZAJE...")
            response.message = "Aterrizaje enviado"
            
        response.success = True
        return response

    def go_to_callback(self, msg):
        lat = msg.pose.position.x
        lon = msg.pose.position.y
        alt = msg.pose.position.z
        
        self.connection.mav.set_position_target_global_int_send(
            0,
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            0b110111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        # Log menos ruidoso
        # self.get_logger().info(f"WP: {lat:.6f}, {lon:.6f}, {alt}m")

def main(args=None):
    rclpy.init(args=args)
    node = InterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
