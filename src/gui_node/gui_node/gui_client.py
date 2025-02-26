import rclpy  # Importa la librería de ROS 2 para Python
from rclpy.node import Node  # Importa la clase base para nodos en ROS 2
from rclpy.action import ActionClient  # Importa la clase para manejar acciones en ROS 2
from hexapod_interfaces.srv import Activar  # Importa el servicio personalizado
from hexapod_interfaces.action import Calcular  # Importa la accion personalizada
from std_srvs.srv import SetBool

class GuiClient(Node):
    def __init__(self):
        super().__init__('gui_client')  # Inicializa el nodo con el nombre 'gui_client'

        # Crea un cliente de accion para enviar solicitudes de cálculo de trayectoria
        self.act_cli = ActionClient(self, Calcular, 'calcular_trayectoria')

        self.cli1 = self.create_client(SetBool, 'finalizar_cinematica') 
        self.req1 = SetBool.Request()

        # Crea un cliente de servicio para activar o desactivar el funcionamiento del nodo trayectoria
        self.cli = self.create_client(Activar, 'activar')  
        while not self.cli.wait_for_service(timeout_sec=1.0):  # Espera a que el servicio esté disponible
            self.get_logger().info('Waiting for service...')
        self.req = Activar.Request()  # Crea una solicitud para el servicio
    
    def send_request(self, data):
        """Envía una solicitud al servicio 'activar'."""
        self.req.indicacion = data  # Asigna el valor de la solicitud (True o False)
        future = self.cli.call_async(self.req)  # Llama al servicio de forma asíncrona
        rclpy.spin_until_future_complete(self, future)  # Espera la respuesta del servicio
        return future.result()  # Retorna el resultado de la solicitud

    def send_req_action(self, data):
        """Envía una solicitud de accion al servidor de acciones 'calcular_trayectoria'."""
        goal_msg = Calcular.Goal()  # Crea un mensaje de meta para la accion
        goal_msg.indicacion = True  # Establece el parámetro de indicación en True

        self.act_cli.wait_for_server()  # Espera a que el servidor de acciones esté disponible
        send_goal_future = self.act_cli.send_goal_async(goal_msg)  # Envía la solicitud de accion
        rclpy.spin_until_future_complete(self, send_goal_future)  # Espera a que la solicitud sea procesada
        
        goal_handle = send_goal_future.result()  # Obtiene el resultado del envío de la meta
        if not goal_handle.accepted:  # Verifica si la meta fue rechazada
            self.get_logger().info('Error al solicitar trayectoria')
            return False

        self.get_logger().info('Calculando...')  # Mensaje de confirmación de aceptación de la meta

        get_result_future = goal_handle.get_result_async()  # Obtiene el resultado de la accion
        rclpy.spin_until_future_complete(self, get_result_future)  # Espera a que el resultado esté disponible

        result = get_result_future.result().result  # Extrae el resultado
        return result.flag  # Retorna el flag obtenido de la accion

    def send_req_service(self):
        self.req1.data = True
        future = self.cli1.call(self.req1)

def main(args=None):
    """Función principal que ejecuta el nodo y maneja la interaccion con el usuario."""
    rclpy.init(args=args)  # Inicializa ROS 2
    gui_client = GuiClient()  # Crea una instancia del cliente GUI
    
    while True:
        user_input = input("Ingrese 1 para Empezar o 0 para Finalizar")  # Solicita entrada al usuario
        
        if user_input=='1':
            action_response = gui_client.send_req_action(True)  # Envía la solicitud de accion
            gui_client.get_logger().info('Calculado' if action_response else 'Error')  # Imprime el resultado de la accion
            if action_response: gui_client.send_request(True)  # Envía la solicitud al servicio según la entrada
            continue
        elif user_input=='0' :
            gui_client.send_request(False)  # Envía la solicitud al servicio según la entrada
            gui_client.send_req_service()
            break
        else:
            print("Entrada invalida. Ingrese 1 o 0.")  # Mensaje de error si la entrada no es válida
            continue  # Reintenta pedir entrada

    rclpy.shutdown()  # Apaga ROS 2 al finalizar

if __name__ == '__main__':
    main()  # Ejecuta la función principal
