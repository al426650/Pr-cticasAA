import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped, Polygon
from sensor_msgs.msg import NavSatFix
from shapely.geometry import Polygon as ShapelyPolygon, LineString
import numpy as np
import time
import math

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        
        # CONFIGURACIÓN
        self.scan_width = 2.0         # Distancia entre pasadas
        self.flight_altitude = 10.0   # Altura de vuelo (usar float)
        self.EARTH_RADIUS = 6378137.0

        self.current_gps = None
        self.origin_lat = None
        self.origin_lon = None
        self.pending_waypoints = [] 

        # CLIENTES SERVICIOS
        self.cliente_armado = self.create_client(SetBool, '/drone/arm')
        self.cliente_despegue = self.create_client(SetBool, '/drone/takeoff')
        
        # PUBS/SUBS
        self.send_waypoint_ = self.create_publisher(PoseStamped, '/drone/cmd_pose', 10)
        self.sub_poly = self.create_subscription(Polygon, '/ui/mission_polygon', self.polygon_callback, 10)
        self.sub_gps = self.create_subscription(NavSatFix, '/drone/gps', self.gps_callback, 10)

        self.get_logger().info('🚀 Nodo de Control iniciado. Esperando GPS para fijar origen...')

    def gps_callback(self, msg):
        self.current_gps = msg
        if self.origin_lat is None and abs(msg.latitude) > 0.001:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f"✅ ORIGEN GPS FIJADO: {self.origin_lat:.6f}, {self.origin_lon:.6f}")

    def polygon_callback(self, msg):
        if self.origin_lat is None:
            self.get_logger().warn("⚠️ ¡Aún no tengo GPS! No puedo calcular ruta.")
            return

        self.get_logger().info(f"Polígono recibido: {len(msg.points)} puntos.")
        coords = [(p.x, p.y) for p in msg.points]
        poly_shape = ShapelyPolygon(coords)
        waypoints = self.generate_lawnmower_path(poly_shape)
        
        if waypoints:
            self.get_logger().info(f"Ruta calculada: {len(waypoints)} WPs. Iniciando en 3s...")
            time.sleep(3)
            self.pending_waypoints = waypoints

    def generate_lawnmower_path(self, polygon):
        minx, miny, maxx, maxy = polygon.bounds
        waypoints = []
        x_coords = np.arange(minx, maxx, self.scan_width)
        direction = 1 
        for x in x_coords:
            vertical_line = LineString([(x, miny), (x, maxy)])
            intersection = polygon.intersection(vertical_line)
            if intersection.is_empty: continue
            if intersection.geom_type == 'LineString': 
                pts = list(intersection.coords)
            elif intersection.geom_type == 'MultiLineString':
                all_lines = list(intersection.geoms)
                longest = max(all_lines, key=lambda l: l.length)
                pts = list(longest.coords)
            else: continue
            
            if direction == 1: pts.sort(key=lambda p: p[1])
            else: pts.sort(key=lambda p: p[1], reverse=True)
            
            waypoints.extend(pts)
            direction *= -1 
        return waypoints 

    def meters_to_gps(self, x, y):
        if self.origin_lat is None: return 0.0, 0.0
        origin_lat_rad = math.radians(self.origin_lat)
        d_lat = y / self.EARTH_RADIUS
        d_lon = x / (self.EARTH_RADIUS * math.cos(origin_lat_rad))
        lat = self.origin_lat + math.degrees(d_lat)
        lon = self.origin_lon + math.degrees(d_lon)
        return lat, lon

    def gps_to_meters(self, lat, lon):
        if self.origin_lat is None: return 0.0, 0.0
        d_lat = math.radians(lat - self.origin_lat)
        d_lon = math.radians(lon - self.origin_lon)
        origin_lat_rad = math.radians(self.origin_lat)
        x = self.EARTH_RADIUS * d_lon * math.cos(origin_lat_rad)
        y = self.EARTH_RADIUS * d_lat
        return x, y
    
    def llamar_servicio(self, cliente, valor_bool):
        if not cliente.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Servicio {cliente.srv_name} no disponible.")
            return False
        req = SetBool.Request()
        req.data = valor_bool
        future = cliente.call_async(req)
        
        start = time.time()
        while rclpy.ok():
            if future.done():
                return future.result().success
            if time.time() - start > 5.0: return False
            rclpy.spin_once(self, timeout_sec=0.1)

    def esperar_llegada(self, target_x, target_y):
        timeout = 40.0 
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info(f" > Navegando a ({target_x:.1f}, {target_y:.1f})m ...")
        
        # Sincronización inicial
        time.sleep(1.0) 

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_gps is None: continue

            curr_x, curr_y = self.gps_to_meters(self.current_gps.latitude, self.current_gps.longitude)
            distancia = math.sqrt((target_x - curr_x)**2 + (target_y - curr_y)**2)

            if distancia < 0.8:
                self.get_logger().info("✅ [LLEGADO] Estabilizando 2 segundos...")
                time.sleep(2.0) # Dwell Time vital para evitar vuelcos
                break

            now = self.get_clock().now().seconds_nanoseconds()[0]
            if (now - start_time) > timeout:
                self.get_logger().warn("⚠️ [TIMEOUT] Saltando WP...")
                break

    def run_mission(self):
        waypoints = self.pending_waypoints
        self.pending_waypoints = [] 

        self.get_logger().info("--- INICIO DE MISIÓN ---")
        
        # 1. ARMAR
        if not self.llamar_servicio(self.cliente_armado, True):
            self.get_logger().error("¡Fallo al armar!")
            return

        time.sleep(3) 

        # 2. DESPEGUE
        self.get_logger().info("Solicitando DESPEGUE...")
        if not self.llamar_servicio(self.cliente_despegue, True):
            return

        # Esperar altura de seguridad
        start_takeoff = time.time()
        while time.time() - start_takeoff < 20.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_gps and self.current_gps.altitude >= 9.0: 
                self.get_logger().info(f"✅ Altura alcanzada: {self.current_gps.altitude:.2f}m.")
                break
            time.sleep(0.5)

        # 3. EJECUCIÓN DE RUTA
        for i, (x, y) in enumerate(waypoints):
            # FILTRO: Si el WP es el origen (0,0) y ya estamos ahí, lo saltamos
            if i == 0:
                curr_x, curr_y = self.gps_to_meters(self.current_gps.latitude, self.current_gps.longitude)
                if math.sqrt((x - curr_x)**2 + (y - curr_y)**2) < 1.0:
                    self.get_logger().info(f" -> WP {i+1} omitido (es el punto de despegue).")
                    continue

            lat, lon = self.meters_to_gps(x, y)
            self.get_logger().info(f" -> WP {i+1}: Metros({x:.1f}, {y:.1f})")
            
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(lat)
            msg.pose.position.y = float(lon)
            msg.pose.position.z = float(self.flight_altitude) 
            
            self.send_waypoint_.publish(msg)
            self.esperar_llegada(x, y)

        self.get_logger().info("Misión Completada. Aterrizando...")
        self.llamar_servicio(self.cliente_despegue, False)

def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pending_waypoints:
                node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
