import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String
from hexapod_interfaces.srv import Activar, SiguientePosicion
from hexapod_interfaces.action import Posicionar

class TransformationNode(Node):
    def __init__(self):
        super().__init__('transformation_node')

        # Suscriptor para recibir valores de las juntas
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joint_values',
            self.listener_callback,
            10
        )

        self.flag = False
        self.n = 0
        self.action_in_progress = False  # Bandera para controlar el estado de la acción

        # Cliente de acción "posicionar_motores"
        self.cli_action = ActionClient(self, Posicionar, 'posicionar_motores')

        # Servicio para activar o desactivar el nodo
        self.srv = self.create_service(Activar, 'activar', self.service_callback)

        # Cliente del servicio "siguiente_posicion"
        self.cli = self.create_client(SiguientePosicion, 'siguiente_posicion')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = SiguientePosicion.Request()

        self.limit = {'Hombro': [-90.0, 90.0], 'Codo': [-90.0, 90.0], 'Muneca': [-90.0, 90.0]}
        self.keys = ['Hombro', 'Codo', 'Muneca']

        # Publicadores
        self.publisher = self.create_publisher(Int32MultiArray, 'motors_values', 10)
        self.publisher2 = self.create_publisher(String, 'joint_values_rad', 10)

        self.received_data = []
        self.radianes = []

        # Timers
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.request_timer = self.create_timer(0.01, self.check_and_send_request)  # Timer para enviar solicitudes

    def service_callback(self, request, response):
        """ Servicio para activar/desactivar el nodo. """
        self.flag = request.indicacion
        response.flag = self.flag
        self.get_logger().info(f'Nodo {"activado" if self.flag else "desactivado"}')
        if self.flag: 
            self.get_logger().info("Finalizando nodo de transformacion...")
            self.destroy_node()
        return response

    def listener_callback(self, msg):
        """ Callback del suscriptor para recibir datos de las juntas. """
        self.received_data = msg.data
        self.radianes = []
        self.convertir()

    def convertir(self):
        """ Convierte los valores de grados a valores para los motores. """
        i_aux = 0
        for i in range(len(self.received_data)):
            if i_aux >= 3:
                i_aux = 0
            if self.received_data[i] > self.limit[self.keys[i_aux]][1] or self.received_data[i] < self.limit[self.keys[i_aux]][0]:
                self.received_data[i] = -10.0
                self.radianes.append(0)
            else:
                self.radianes.append(round((self.received_data[i] * 3.141592) / 180, 2))
                self.received_data[i] = round(2045.0 + ((1028.0 * self.received_data[i]) / 90), 0)
            i_aux += 1

    def timer_callback(self):
        """ Publica los valores convertidos a los motores y en radianes. """
        if self.received_data:
            msg = Int32MultiArray()
            msg.data = [int(round(value)) for value in self.received_data]
            self.publisher.publish(msg)

            msg2 = String()
            msg2.data = ', '.join(map(str, self.radianes))
            self.publisher2.publish(msg2)

    def check_and_send_request(self):
        """ Envía la solicitud al servicio si `self.flag` es `True` y no hay otra acción en curso. """
        if self.flag and not self.action_in_progress:
            self.action_in_progress = True  # Marcar que la acción está en progreso
            self.get_logger().info("Enviando solicitud al servicio siguiente_posicion...")
            self.req.index = self.n
            future = self.cli.call_async(self.req)
            future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """ Callback que maneja la respuesta del servicio. """
        try:
            response = future.result()
            data_aux = [int(round(2045.0 + ((1028.0 * pos) / 90), 0)) for pos in response.positions]
            self.get_logger().info(f"Respuesta del servicio: {data_aux}")

            # Esperar a que el servidor de acción esté disponible
            if not self.cli_action.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("El servidor de acción 'posicionar_motores' no está disponible.")
                self.action_in_progress = False
                return  # Detener ejecución

            # Configurar la meta de la acción
            goal_msg = Posicionar.Goal()
            goal_msg.positions = data_aux

            # Enviar la meta y esperar a que se acepte
            future_goal = self.cli_action.send_goal_async(goal_msg)
            future_goal.add_done_callback(self.goal_response_callback)

        except Exception as e:
            self.get_logger().error(f"Error al llamar al servicio: {str(e)}")
            self.action_in_progress = False

    def goal_response_callback(self, future_goal):
        goal_handle = future_goal.result()
        if not goal_handle.accepted:
            self.get_logger().error("La acción fue rechazada.")
            self.action_in_progress = False
            return  # Detener ejecución si la acción no fue aceptada
        
        self.get_logger().warning("Acción aceptada")

        # Esperar a que la acción se complete
        future_result = goal_handle.get_result_async()
        future_result.add_done_callback(self.result_callback)

    def result_callback(self, future_result):
        try:
            result = future_result.result().result
            if result.flag:
                self.get_logger().warning(f'Acción ejecutada correctamente. n={self.n}')
                self.n += 1  # Ajustar el índice según la lógica
            else:
                self.get_logger().error('La acción no se ejecutó con éxito.')
        except Exception as e:
            self.get_logger().error(f"Error en la ejecución de la acción: {str(e)}")
        finally:
            self.action_in_progress = False  # Liberar la bandera para permitir nuevas acciones


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
