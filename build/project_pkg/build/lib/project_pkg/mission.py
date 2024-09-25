import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from mavros_msgs.msg import Waypoint, State
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time


class MissionModeNode(Node):
    def __init__(self):
        super().__init__('mission_mode_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.get_logger().info("Nodo missione avviato!")

        # Timer per chiamare la funzione dopo 20 secondi
        self.timer = self.create_timer(20, self.timer_callback)

        # Timer per pubblicare i setpoint di posizione a 10 Hz
        self.setpoint_timer = self.create_timer(
            0.1, self.publish_position_setpoint)

        # Publisher per setpoint di posizione sul topic /mavros/setpoint_position/local
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos_profile)

        # Subscriber per ottenere la posizione attuale del drone
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_pos_callback, qos_profile_sensor_data)

        # Subscriber per lo stato del drone
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)

        # Client per caricare i waypoint
        self.wp_push_client = self.create_client(
            WaypointPush, '/mavros/mission/push')
        while not self.wp_push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servizio /mavros/mission/push non disponibile, in attesa...')

        # Client per impostare la modalità del drone
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servizio /mavros/set_mode non disponibile, in attesa...')

        # Client per il servizio di arming
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')

        # Coordinate GPS della missione
        self.lat = 47.3986402  
        self.lon = 8.5462568 
        self.alt = 10.0 #in NED le altezze sono al contrario, c'è un segno -

        self.count_mode = 0
        self.current_state = State()
        self.setpoint = PoseStamped()
        self.current_position = None
        self.reference_position = None

        self.send_mission()
        self.service_check()
        self.arm_drone()

    def send_mission(self):
        # Creazione di un waypoint per la missione
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = 16  # NAV_WAYPOINT
        wp.is_current = True
        wp.autocontinue = True
        wp.z_alt = self.alt
        wp.x_lat = self.lat
        wp.y_long = self.lon

        # Invia il waypoint al servizio di MAVROS
        wp_push_req = WaypointPush.Request()
        wp_push_req.start_index = 0
        wp_push_req.waypoints.append(wp)

        future = self.wp_push_client.call_async(wp_push_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Missione caricata con successo!')
            # Imposta la modalità AUTO.MISSION dopo aver caricato la missione
            self.set_auto_mission_mode()
        else:
            self.get_logger().error('Errore nel caricamento della missione.')

    def service_check(self):
        # Funzione per controllare l'abilitazione corretta dei servizi
        while not self.arming_client.wait_for_service(timeout_sec=1.0) or not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Attendo i servizi...')
        self.get_logger().info("Servizi disponibili")

    def state_callback(self, msg):
        # Salva lo stato corrente e stampa un log quando cambia la modalità
        if msg.mode != self.current_state.mode:
            self.current_state = msg
            self.get_logger().info(f"Stato attuale: {self.current_state.mode}")

    def timer_callback(self):
        self.get_logger().info("Sono passati 20 secondi, imposto modalità OFFBOARD")

        # Salvo pos drone
        self.reference_position = self.current_position

        self.change_mode("AUTO.LOITER")
        time.sleep(2)
        self.change_mode("OFFBOARD")
        self.timer.cancel()  # Cancello il timer dopo la prima chiamata

    def arm_drone(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Drone armato con successo')
            else:
                self.get_logger().info('Impossibile armare il drone')
        else:
            self.get_logger().error('Errore durante la chiamata del servizio di arming')

    def set_auto_mission_mode(self):
        # Imposta la modalità del drone su AUTO.MISSION
        set_mode_req = SetMode.Request()
        set_mode_req.custom_mode = "AUTO.MISSION"

        future = self.set_mode_client.call_async(set_mode_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info('Modalità AUTO.MISSION impostata con successo!')
        else:
            self.get_logger().error('Errore nell\'impostazione della modalità AUTO.MISSION.')

    def local_pos_callback(self, msg):
        self.current_position = msg.pose.position

    def publish_position_setpoint(self):
        if self.current_state is None or self.current_position is None or self.reference_position is None:
            return

        # Spostamento di 5 metri rispetto alla posizione attuale del drone
        self.setpoint.pose.position.x = self.reference_position.x + 5.0 
        self.setpoint.pose.position.y = self.reference_position.y  
        self.setpoint.pose.position.z = self.reference_position.z

        # Invio il setpoint
        self.setpoint.header = Header()
        self.setpoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.setpoint)

    def change_mode(self, mode):
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            req = SetMode.Request()
            req.custom_mode = mode
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(self.mode_change_callback)

    def mode_change_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f"Modalità impostata con successo!")
            else:
                self.get_logger().warn(f"Cambio modalità fallito!")
        except Exception as e:
            self.get_logger().error(f"Errore nel cambio modalità: {e}")


def main(args=None):
    rclpy.init(args=args)
    mission_mode_node = MissionModeNode()
    rclpy.spin(mission_mode_node)
    mission_mode_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
