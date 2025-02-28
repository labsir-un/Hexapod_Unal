import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hexapod_interfaces.srv import Activar, SiguientePosicion

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')

        self.flag = False  # Bandera para activar/desactivar el nodo
        self.n = 0  # Contador para el índice de posiciones
        
        self.srv = self.create_service(Activar, 'activar', self.service_callback)
        self.cli = self.create_client(SiguientePosicion, 'siguiente_posicion')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = SiguientePosicion.Request()

        self.publisher = self.create_publisher(String, 'joint_values_rad', 10)
        self.request_timer = self.create_timer(0.05, self.check_and_send_request)

    def service_callback(self, request, response):
        self.flag = request.indicacion
        response.flag = self.flag
        self.get_logger().info(f'Nodo {"activado" if self.flag else "desactivado"}')
        return response

    def check_and_send_request(self):
        if self.flag:
            self.get_logger().info("Enviando solicitud al servicio siguiente_posicion...")
            self.req.index = self.n
            future = self.cli.call_async(self.req)
            future.add_done_callback(self.response_callback)
        else:
            self.get_logger().info("Nodo desactivado. No se enviarán más solicitudes.")

    def response_callback(self, future):
        try:
            response = future.result()
            self.radianes = [round((pos * 3.141592) / 180, 2) for pos in response.positions]
            
            self.get_logger().info(f"Publicando en joint_values_rad: {self.radianes}")
            msg = String()
            msg.data = ', '.join(map(str, self.radianes))
            self.publisher.publish(msg)
            
            self.get_logger().warning(f'Acción simulada ejecutada correctamente. n={self.n}')
            self.n += 1
        except Exception as e:
            self.get_logger().error(f"Error al llamar al servicio: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = SimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()