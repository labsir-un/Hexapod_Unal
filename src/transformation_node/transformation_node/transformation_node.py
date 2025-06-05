import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from hexapod_interfaces.srv import Activar, SiguientePosicion
from hexapod_interfaces.action import Posicionar


class TransformationNode(Node):
    def __init__(self):
        super().__init__('transformation_node')

        self.flag = False
        self.n = 0

        self.cli_action = ActionClient(self, Posicionar, 'posicionar_motores')
        self.cli = self.create_client(SiguientePosicion, 'siguiente_posicion')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio siguiente_posicion...')

        self.req = SiguientePosicion.Request()
        self.publisher = self.create_publisher(String, 'joint_values_rad', 10)
        self.srv = self.create_service(Activar, 'activar', self.service_callback)

    def service_callback(self, request, response):
        self.flag = request.indicacion
        response.flag = self.flag
        if self.flag:
            self.get_logger().info('Nodo activado. Iniciando trayectoria.')
            self.send_request()
        else:
            self.get_logger().info('Nodo desactivado.')
        return response

    def send_request(self):
        if not self.flag:
            return

        self.req.index = self.n
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            data_aux = [
                max(0, min(4095, int(round(2045.0 + ((1028.0 * pos) / 90), 0))))
                for pos in response.positions
            ]
            radianes = [round((pos * 3.141592) / 180, 2) for pos in response.positions]
            msg = String()
            msg.data = ', '.join(map(str, radianes))
            self.publisher.publish(msg)

            if not self.cli_action.wait_for_server(timeout_sec=0.5):
                self.get_logger().error("Acción 'posicionar_motores' no disponible.")
                return

            goal_msg = Posicionar.Goal()
            goal_msg.positions = data_aux
            future_goal = self.cli_action.send_goal_async(goal_msg)
            future_goal.add_done_callback(self.goal_response_callback)
        except Exception as e:
            self.get_logger().error(f"Error al llamar al servicio: {e}")

    def goal_response_callback(self, future_goal):
        goal_handle = future_goal.result()
        if not goal_handle.accepted:
            self.get_logger().error("Acción rechazada.")
            return
        self.get_logger().info("Acción aceptada.")
        future_result = goal_handle.get_result_async()
        future_result.add_done_callback(self.result_callback)

    def result_callback(self, future_result):
        try:
            result = future_result.result().result
            if result.flag:
                self.get_logger().info(f"Acción completada. n={self.n}")
                self.n += 1
                self.send_request()
            else:
                self.get_logger().error("La acción falló.")
        except Exception as e:
            self.get_logger().error(f"Error en la acción: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TransformationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
