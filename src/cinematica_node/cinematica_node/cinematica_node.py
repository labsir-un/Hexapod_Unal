import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_srvs.srv import SetBool
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
from hexapod_interfaces.srv import SiguientePosicion
from hexapod_interfaces.action import Calcular

class CinematicaNode(Node):
    def __init__(self):
        super().__init__('cinematica_node')  # Inicializa el nodo con el nombre 'cinematica_node'
        
        self.get_logger().info('Calculate service ready.')  # Mensaje de inicio del nodo

        # Crea un servicio que responde con la siguiente posición del hexápodo
        self.srv1 = self.create_service(SiguientePosicion, 'siguiente_posicion', self.send_position)

        # Crea un servidor de acción para calcular la trayectoria del hexápodo
        self.act = ActionServer(self, Calcular, 'calcular_trayectoria', self.action_callback)
        
        self.send_true = False  # Bandera de control
        self.index = 0  # Índice para recorrer la trayectoria calculada
        self.max = 0  # Longitud máxima de la trayectoria calculada
        
    def action_callback(self, goal_handle):
        """Callback de la acción 'calcular_trayectoria'. Calcula la trayectoria y envía retroalimentación."""
        self.get_logger().info("Calculando trayectoria...")
        feedback = Calcular.Feedback()
        result = Calcular.Result()
        
        if goal_handle.request.indicacion:  # Verifica si la acción fue solicitada
            for i in range(1):  # Envía un valor de retroalimentación
                feedback.position = i
                goal_handle.publish_feedback(feedback)
                time.sleep(1)
            
            self.q = self.calcular_trayectoria()  # Calcula la trayectoria del hexápodo
            self.get_logger().info("Trayectoria calculada.")
            self.get_logger().info(str(len(self.q[1])))
            self.max = len(self.q[1])  # Guarda la longitud de la trayectoria calculada
            result.flag = True  # Indica que la acción se completó con éxito
        else:
            result.flag = False  # Indica que la acción falló
        
        goal_handle.succeed()  # Marca la acción como completada
        return result

    def send_position(self, request, response):
        """Servicio para enviar una posición específica de la trayectoria calculada."""
        index = request.index
        if index >= self.max:
            index -= (index // self.max)*self.max  # Asegura que el índice esté dentro del rango
        
        self.get_logger().info(f'Received request for Position {index}')
        response.positions = self.q[:, index].flatten().tolist()  # Envía la posición solicitada
        return response
    
    def calcular_trayectoria(self):
        """Calcula la trayectoria del hexápodo usando cinemática inversa."""
        L1 = 5  # Longitud del primer eslabón
        L2 = 6  # Longitud del segundo eslabón
        L3 = 13  # Longitud del tercer eslabón
        T_total = 5  # Tiempo total del ciclo
        N_ciclos = 1  # Número de ciclos de la trayectoria
        T = T_total / N_ciclos  # Tiempo por ciclo
        N_puntos = 20  # Número de puntos por ciclo
    
        t, x, y, z = self.trayectoria(7.5, 3, T, N_puntos, N_ciclos)  # Calcula la trayectoria deseada
    
        t_total = np.linspace(0, T_total, 4 * N_puntos * N_ciclos)
        L4xy = np.sqrt(x**2 + y**2) - L1  # Distancia en el plano XY
    
        q = np.zeros((18, 4 * N_puntos * N_ciclos), float)  # Matriz para almacenar los ángulos de las articulaciones
        for foot_number in range(1, 7):  # Itera sobre las 6 patas del hexápodo
            q1 = np.degrees(np.arctan2(y, x))
            q2 = np.degrees(np.arccos(((L4xy ** 2 + z ** 2) + L2 ** 2 - L3 ** 2) / (2 * L2 * np.sqrt(L4xy ** 2 + z ** 2))) + np.arctan2(z, L4xy)) - 3
            q2 = -q2
            q3 = np.degrees(np.arccos((L2 ** 2 + L3 ** 2 - (L4xy ** 2 + z ** 2)) / (2 * L2 * L3))) - 35
            
            # Ajusta la trayectoria según la paridad del número de pata
            if foot_number % 2 == 0:
                if foot_number == 4:
                    q1 = -q1
                    q2 = np.flip(q2)
                    q3 = np.flip(q3)
            else:
                q1 = -q1
                if foot_number in [1, 5]:
                    q1 = -q1
                    q2 = np.flip(q2)
                    q3 = np.flip(q3)
    
            # Asigna los ángulos a la matriz de trayectoria
            q[3 * foot_number - 3, :] = q1
            q[3 * foot_number - 2, :] = q2
            q[3 * foot_number - 1, :] = q3
    
        return q  # Retorna la trayectoria calculada

    def trayectoria(self, l_total, altura, T, N_puntos, N_ciclos):
        """Genera la trayectoria deseada en el espacio cartesiano."""
        x = np.zeros(N_puntos * 4) + 12  # Posición constante en X
        
        # Genera los tiempos
        t1 = np.linspace(0, T / 2, 2 * N_puntos)
        t2 = np.linspace(T / 2, T, 2 * N_puntos)
    
        # Movimiento en Y y Z
        y1 = (l_total / 2) * np.sin((2 * np.pi / T) * t1 - (np.pi / 2))
        y2 = np.linspace(l_total / 2, -l_total / 2, 2 * N_puntos)
    
        z1 = (altura / 2) * np.sin((4 * np.pi / T) * t1 - (np.pi / 2)) - 10
        z2 = np.zeros(N_puntos * 2) - 11.5
    
        # Une los valores de la trayectoria
        t = np.concatenate((t1, t2))
        y = np.concatenate((y1, y2))
        z = np.concatenate((z1, z2))
    
        # Repite la trayectoria para cada ciclo
        x = np.tile(x, N_ciclos)
        y = np.tile(y, N_ciclos)
        z = np.tile(z, N_ciclos)
    
        return t, x, y, z

# Función principal que inicializa el nodo
def main(args=None):
    rclpy.init(args=args)
    node = CinematicaNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
