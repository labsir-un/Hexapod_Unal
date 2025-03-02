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

        self.flag = False  # Bandera para activar/desactivar el nodo
        self.n = 0  # Contador para el índice de posiciones
        self.action_in_progress = False  # Bandera para controlar el estado de la acción

        self.timer = 0.0001

        self.cli_action = ActionClient(self, Posicionar, 'posicionar_motores')
        # self.srv = self.create_service(Activar, 'activar', self.service_callback)
        self.srv = self.create_service(SetBool, 'activar', self.service_callback)
        self.cli = self.create_client(SiguientePosicion, 'siguiente_posicion')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = SiguientePosicion.Request()

        self.cli_action = ActionClient(self, Calcular, 'calcular_trayectoria')
        if not self.cli_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("El servidor de acciones 'calcular_trayectoria' no está disponible.")
            rclpy.shutdown()
            return

        self.limit = {'Hombro': [-90.0, 90.0], 'Codo': [-90.0, 90.0], 'Muneca': [-90.0, 90.0]}
        self.keys = ['Hombro', 'Codo', 'Muneca']
        
        self.sim = False
        self.publisher = self.create_publisher(String, 'joint_values_rad', 10)
        self.received_data = []
        self.radianes = []

        self.request_timer = self.create_timer(0.0001, self.check_and_send_request)

    # def service_callback(self, request, response):
    #     self.flag = request.indicacion
    #     response.flag = self.flag
    #     self.get_logger().info(f'Nodo {"activado" if self.flag else "desactivado"}')
    #     return response

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
        if not request.data:
            self.sim = True
        else:
            self.sim = False
        response.success = self.send_req_action()
        self.get_logger().info(f'Nodo {"activado" if self.flag else "desactivado"}')
        return response

    def check_and_send_request(self):
        if not self.action_in_progress:  # Solo intenta si no hay otra acción en progreso
            if self.flag:  # Solo inicia si está activado
                self.action_in_progress = True
                self.get_logger().info("Enviando solicitud al servicio siguiente_posicion...")
                self.req.index = self.n
                future = self.cli.call_async(self.req)
                future.add_done_callback(self.response_callback)
            else:
                self.get_logger().info("Nodo desactivado. No se enviarán más solicitudes.")

    def response_callback(self, future):
        try:
            response = future.result()
            data_aux = [
                max(0, min(4095, int(round(2045.0 + ((1028.0 * pos) / 90), 0))))
                for pos in response.positions
            ]
            self.radianes = [round((pos * 3.141592) / 180, 2) for pos in response.positions]
            
            self.get_logger().info(f"Publicando en joint_values_rad: {self.radianes}")
            msg = String()
            msg.data = ', '.join(map(str, self.radianes))
            self.publisher.publish(msg)
            
            
            if not self.sim and not self.cli_action.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("El servidor de acción 'posicionar_motores' no está disponible.")
                self.action_in_progress = False
                return
            else:
                self.get_logger().info("Modo simulacion")

            if not self.sim:
                goal_msg = Posicionar.Goal()
                goal_msg.positions = data_aux
                future_goal = self.cli_action.send_goal_async(goal_msg)
                future_goal.add_done_callback(self.goal_response_callback)
                self.get_logger().warn('Aca')
            else:
                self.n += 1
                self.get_logger().warn('Avance')
                self.action_in_progress = False

        except Exception as e:
            self.get_logger().error(f"Error al llamar al servicio: {str(e)}")
            self.action_in_progress = False

    def goal_response_callback(self, future_goal):
        goal_handle = future_goal.result()
        if not goal_handle.accepted:
            self.get_logger().error("La acción fue rechazada.")
            self.action_in_progress = False
            return

        self.get_logger().warning("Acción aceptada")
        future_result = goal_handle.get_result_async()
        future_result.add_done_callback(self.result_callback)

    def result_callback(self, future_result):
        try:
            result = future_result.result().result
            if result.flag:
                self.get_logger().warning(f'Acción ejecutada correctamente. n={self.n}')
                self.n += 1
            else:
                self.get_logger().error('La acción no se ejecutó con éxito.')
        except Exception as e:
            self.get_logger().error(f"Error en la ejecución de la acción: {str(e)}")
        finally:
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
