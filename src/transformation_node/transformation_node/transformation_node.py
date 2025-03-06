import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32MultiArray, String
from hexapod_interfaces.srv import Activar, SiguientePosicion
from hexapod_interfaces.action import Calcular
from std_srvs.srv import SetBool
from hexapod_interfaces.action import Posicionar

class TransformationNode(Node):
    def __init__(self):
        super().__init__('transformation_node')

        self.flag = False
        self.n = 0
        self.action_in_progress = False

        self.cop_client = self.create_client(SetBool, 'verificar_posiciones') 

        self.cli_action = ActionClient(self, Posicionar, 'posicionar_motores')
        self.srv = self.create_service(SetBool, 'activar', self.service_callback)
        self.cli = self.create_client(SiguientePosicion, 'siguiente_posicion')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio siguiente_posicion...')

        self.cli_action = ActionClient(self, Calcular, 'calcular_trayectoria')
        if not self.cli_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("El servidor de acciones 'calcular_trayectoria' no está disponible.")
            rclpy.shutdown()
            return

        self.sim = False
        self.publisher = self.create_publisher(String, 'joint_values_rad', 10)
        self.request_timer = self.create_timer(0.1, self.check_and_send_request)

    def send_req_action(self):
        """Envía solicitud de acción para calcular trayectoria."""
        goal_msg = Calcular.Goal()
        goal_msg.indicacion = True

        send_goal_future = self.cli_action.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Error al solicitar trayectoria.')
            return False

        self.get_logger().info('Calculando...')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=10.0)

        result = get_result_future.result()
        return result.result.flag if result else False

    def service_callback(self, request, response):
        self.flag = True
        self.sim = not request.data  
        response.success = self.send_req_action()
        self.get_logger().info(f'Nodo {"activado" if self.flag else "desactivado"}')
        return response

    def check_and_send_request(self):
        if not self.action_in_progress and self.flag:
            self.action_in_progress = True
            self.get_logger().info("Solicitando siguiente posición...")
            req = SiguientePosicion.Request()
            req.index = self.n
            future = self.cli.call_async(req)
            future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.radianes = [round((pos * 3.141592) / 180, 2) for pos in response.positions]

            self.get_logger().info(f"Publicando en joint_values_rad: {self.radianes}")
            msg = String()
            msg.data = ', '.join(map(str, self.radianes))
            self.publisher.publish(msg)

            if not self.sim:
                self.send_goal_to_motors(response.positions)
            else:
                self.wait_for_coppelia_confirmation()

        except Exception as e:
            self.get_logger().error(f"Error en la respuesta del servicio: {str(e)}")
            self.action_in_progress = False

    def send_goal_to_motors(self, positions):
        """Envía una meta a los motores y espera la confirmación antes de continuar."""
        goal_msg = Posicionar.Goal()
        goal_msg.positions = [max(0, min(4095, int(round(2045.0 + ((1028.0 * pos) / 90), 0)))) for pos in positions]

        future_goal = self.cli_action.send_goal_async(goal_msg)
        future_goal.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future_goal):
        goal_handle = future_goal.result()
        if not goal_handle.accepted:
            self.get_logger().error("La acción fue rechazada.")
            self.action_in_progress = False
            return

        self.get_logger().info("Acción aceptada. Esperando resultado...")
        future_result = goal_handle.get_result_async()
        future_result.add_done_callback(self.result_callback)

    def result_callback(self, future_result):
        try:
            result = future_result.result().result
            if result.flag:
                self.get_logger().info(f'Acción ejecutada correctamente. n={self.n}')
                self.wait_for_coppelia_confirmation()
            else:
                self.get_logger().error('La acción no se ejecutó con éxito.')
                self.action_in_progress = False
        except Exception as e:
            self.get_logger().error(f"Error en la ejecución de la acción: {str(e)}")
            self.action_in_progress = False

    def wait_for_coppelia_confirmation(self):
        """Llama al servicio de CoppeliaSim y espera hasta recibir confirmación."""
        req = SetBool.Request()
        req.data = True
        self.get_logger().info("Esperando confirmación de CoppeliaSim...")
        future_service = self.cop_client.call_async(req)
        future_service.add_done_callback(self.coppelia_response_callback)

    def coppelia_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("CoppeliaSim confirmó la posición. Avanzando a la siguiente posición.")
                self.n += 1
                self.action_in_progress = False  
            else:
                self.get_logger().info("CoppeliaSim no ha alcanzado la posición. Reintentando en 0.5s...")
                self.create_timer(0.5, self.wait_for_coppelia_confirmation)  # Reintento
        except Exception as e:
            self.get_logger().error(f"Error en la respuesta de CoppeliaSim: {str(e)}")
            self.action_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = TransformationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
