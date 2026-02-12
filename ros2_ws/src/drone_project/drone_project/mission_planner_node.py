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
        self.scan_width = 2.0         # Más ancho para menos giros bruscos
        self.flight_altitude = 2.5    # Altura segura
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

        self.get_logger().info('Esperando GPS para fijar origen...')

    def gps_callback(self, msg):
        self.current_gps = msg
        # PROTECCIÓN: Solo fijamos origen si tenemos satélites válidos (lat != 0)
        if self.origin_lat is None and abs(msg.latitude) > 0.001:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f"✅ ORIGEN GPS FIJADO: {self.origin_lat:.6f}, {self.origin_lon:.6f}")
            self.get_logger().info("Esperando polígono...")

    def polygon_callback(self, msg):
        if self.origin_lat is None:
            self.get_logger().warn("¡Aún no tengo GPS! No puedo calcular ruta.")
            return

        self.get_logger().info(f"Polígono recibido: {len(msg.points)} puntos.")
        coords = [(p.x, p.y) for p in msg.points]
        poly_shape = ShapelyPolygon(coords)
        waypoints = self.generate_lawnmower_path(poly_shape)
        
        if waypoints:
            self.get_logger().info(f"Ruta calculada: {len(waypoints)} WPs. Iniciando en 3s...")
            time.sleep(3) # Dar tiempo a leer el log
            self.pending_waypoints = waypoints

    # --- TUS MATEMÁTICAS (Necesarias para tu InterfaceNode) ---
    def generate_lawnmower_path(self, polygon):
        minx, miny, maxx, maxy = polygon.bounds
        waypoints = []
        x_coords = np.arange(minx, maxx, self.scan_width)
        direction = 1 
        for x in x_coords:
            vertical_line = LineString([(x, miny), (x, maxy)])
            intersection = polygon.intersection(vertical_line)
            if intersection.is_empty: continue
            if intersection.geom_type == 'LineString': pts = list(intersection.coords)
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
        # Conversión simple ENU -> GPS
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
    
    # --- EJECUCIÓN ---

    def llamar_servicio(self, cliente, valor_bool):
        if not cliente.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Servicio {cliente.srv_name} no disponible.")
            return False
        req = SetBool.Request()
        req.data = valor_bool
        future = cliente.call_async(req)
        
        # Espera activa (sin bloquear ROS)
        start = time.time()
        while rclpy.ok():
            if future.done():
                return future.result().success
            if time.time() - start > 5.0: return False
            rclpy.spin_once(self, timeout_sec=0.1)

    def esperar_llegada(self, target_x, target_y):
        self.get_logger().info(f" > Navegando a ({target_x:.1f}, {target_y:.1f})m ...")
        
        # Timeout de seguridad: Si no llega en 30s, pasa al siguiente
        start_wait = time.time()
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_gps is None or self.origin_lat is None: continue
            
            # Comparamos posición actual GPS convertida a metros vs Objetivo en metros
            curr_x, curr_y = self.gps_to_meters(self.current_gps.latitude, self.current_gps.longitude)
            dist = math.sqrt((target_x - curr_x)**2 + (target_y - curr_y)**2)
            
            # Margen de 1.5 metros (Suficiente para Isaac)
            if dist < 1.5: 
                self.get_logger().info("   [LLEGADO]")
                break
            
            if time.time() - start_wait > 30.0:
                self.get_logger().warn("   [TIMEOUT] Saltando WP...")
                break

    def run_mission(self):
        waypoints = self.pending_waypoints
        self.pending_waypoints = [] 

        self.get_logger().info("--- INICIO DE MISIÓN ---")
        
        # 1. ARMAR (Con reintentos)
        armado_ok = False
        for i in range(3):
            self.get_logger().info(f"Intento de ARMADO {i+1}/3...")
            if self.llamar_servicio(self.cliente_armado, True):
                armado_ok = True
                break
            time.sleep(2)
        
        if not armado_ok:
            self.get_logger().error("¡Fallo crítico al armar! Abortando misión.")
            return

        self.get_logger().info("Armado OK. Esperando 3s para estabilizar motores...")
        time.sleep(3) 

        # 2. DESPEGUE INTELIGENTE (Bucle de comprobación de altura)
        # Importante: Asegúrate de que tu InterfaceNode tenga una altura razonable (ej: 3m)
        self.get_logger().info("Solicitando DESPEGUE...")
        if not self.llamar_servicio(self.cliente_despegue, True):
            self.get_logger().error("Fallo al solicitar despegue.")
            return

        # Bucle de espera activa: No pasamos a la ruta hasta estar en el aire
        start_takeoff = time.time()
        taken_off = False
        
        self.get_logger().info("Esperando ascenso real (>1.5m)...")
        while time.time() - start_takeoff < 20.0: # Damos 20s máximo
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Leemos la altura actual del GPS (si está disponible)
            current_alt = 0.0
            if self.current_gps:
                current_alt = self.current_gps.altitude
            
            # Chequeo de éxito: Si estamos por encima de 9
            if current_alt >= 9: 
                self.get_logger().info(f"✅ Altura confirmada: {current_alt:.2f}m. Iniciando ruta.")
                taken_off = True
                break
            
            # Reintento si se queda pegado al suelo tras 5 segundos
            if (time.time() - start_takeoff > 5.0) and (current_alt < 0.3):
                self.get_logger().warn(f"El dron no sube (Alt: {current_alt:.2f}m). Reenviando TAKEOFF...")
                self.llamar_servicio(self.cliente_despegue, True)
                time.sleep(2) # Espera para no saturar

        if not taken_off:
            self.get_logger().error("TIMEOUT: El dron no ha logrado despegar tras 20s. Abortando.")
            # Intentamos desarmar o aterrizar por seguridad
            self.llamar_servicio(self.cliente_despegue, False) 
            return

        # 3. EJECUCIÓN DE RUTA (Solo llegamos aquí si el dron vuela)
        self.get_logger().info(f"Iniciando recorrido de {len(waypoints)} puntos.")
        
        for i, (x, y) in enumerate(waypoints):
            # Convertimos a GPS para tu InterfaceNode
            lat, lon = self.meters_to_gps(x, y)
            
            self.get_logger().info(f" -> WP {i+1}: Metros({x:.1f}, {y:.1f})")
            
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = lat  # Tu puente lee esto como Latitud
            msg.pose.position.y = lon  # Tu puente lee esto como Longitud
            msg.pose.position.z = self.flight_altitude 
            
            self.send_waypoint_.publish(msg)
            
            # Esperamos llegar antes de enviar el siguiente
            self.esperar_llegada(x, y)

        self.get_logger().info("Misión Completada. Aterrizando (RTL/Land)...")
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
