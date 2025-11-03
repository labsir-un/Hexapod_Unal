import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from hexapod_interfaces.srv import SiguientePosicion
from hexapod_interfaces.action import Calcular
import numpy as np

class CinematicaNode(Node):
    def __init__(self):
        super().__init__('cinematica_node')
        
        self.get_logger().info('Nodo Cinemática iniciado.')

        # Servidor de servicio 'siguiente_posicion'
        self.srv1 = self.create_service(SiguientePosicion, 'siguiente_posicion', self.send_position)
        
        # Servidor de acción 'calcular_trayectoria'
        self.act = ActionServer(self, Calcular, 'calcular_trayectoria', self.action_callback)
        
        self.q = None  # Inicializa la trayectoria como None
        self.max = 0   # Tamaño de la trayectoria

    def action_callback(self, goal_handle):
        """Calcula la trayectoria y devuelve el resultado."""
        self.get_logger().info("Calculando trayectoria...")
        
        feedback = Calcular.Feedback()
        result = Calcular.Result()
        mode = goal_handle.request.indicacion
        
        if  mode >= 0:
            self.q = self.calcular_trayectoria()
            self.max = len(self.q[1]) if self.q is not None else 0
            
            if self.max > 0:
                self.get_logger().info(f"Trayectoria calculada con {self.max} posiciones.")
                result.flag = True
            else:
                self.get_logger().error("Error en el cálculo de trayectoria.")
                result.flag = False
        else:
            self.get_logger().warning("Solicitud de trayectoria inválida.")
            result.flag = False

        match mode:
            case 1:
                self.get_logger().warn("Trayectoria recta")
                pass
            case 2:
                self.get_logger().warn("Trayectoria giro izquierda")
                for i in range(1, 6, 2):
                    self.q[3 * i - 3, :] = -self.q[3 * i - 3, :]
            case 3:
                self.get_logger().warn("Trayectoria giro derecha")
                for i in range(2, 7, 2):
                    self.q[3 * i - 3, :] = -self.q[3 * i - 3, :]
            case 4:
                self.get_logger().warn("Trayectoria reversa")
                for i in range(1, 7):
                    self.q[3 * i - 3, :] = -self.q[3 * i - 3, :]
            case 0:
                self.get_logger().warn("Pausa")
                self.q = np.zeros(np.shape(self.q))


        goal_handle.succeed()
        return result

    def send_position(self, request, response):
        """Retorna una posición específica de la trayectoria."""
        if self.q is None:
            self.get_logger().error("No hay trayectoria calculada.")
            response.positions = []
            return response

        index = request.index % self.max  # Asegura que el índice está en el rango
        
        self.get_logger().info(f'Enviando posición {index}')
        response.positions = self.q[:, index].flatten().tolist()
        return response
    
    def calcular_trayectoria(self):
        """Calcula la trayectoria del hexápodo usando cinemática inversa."""
        L1, L2, L3 = 5, 6, 13
        T_total, N_ciclos, N_puntos = 5, 1, 10
        T = T_total / N_ciclos
    
        t, x, y, z = self.trayectoria(7.5, 3.5, T, N_puntos, N_ciclos)

        q = np.zeros((18, 4 * N_puntos * N_ciclos), float)
        for foot_number in range(1, 7):
            q1 = np.degrees(np.arctan2(y, x))
            q2 = -np.degrees(
                np.arccos(((np.sqrt(x**2 + y**2) - L1) ** 2 + z ** 2 + L2 ** 2 - L3 ** 2) / 
                (2 * L2 * np.sqrt((np.sqrt(x**2 + y**2) - L1) ** 2 + z ** 2))
            ) + np.arctan2(z, np.sqrt(x**2 + y**2) - L1)) - 3
            q3 = np.degrees(np.arccos((L2 ** 2 + L3 ** 2 - ((np.sqrt(x**2 + y**2) - L1) ** 2 + z ** 2)) / (2 * L2 * L3))) - 90
            q3=-q3

            if foot_number % 2 == 0:
                if foot_number == 4:
                    q1, q2, q3 = -q1, np.flip(q2), np.flip(q3)
            else:
                q1 = -q1
                if foot_number in [1, 5]:
                    q1, q2, q3 = -q1, np.flip(q2), np.flip(q3)
    
            q[3 * foot_number - 3, :] = q1
            q[3 * foot_number - 2, :] = q2
            q[3 * foot_number - 1, :] = q3

        return q

    def trayectoria(self, l_total, altura, T, N_puntos, N_ciclos):
        """Genera la trayectoria deseada en el espacio cartesiano."""
        x = np.full(N_puntos * 4, 12)
        t1 = np.linspace(0, T / 2, 2 * N_puntos)
        t2 = np.linspace(T / 2, T, 2 * N_puntos)
    
        y1 = (l_total / 2) * np.sin((2 * np.pi / T) * t1 - (np.pi / 2))
        y2 = np.linspace(l_total / 2, -l_total / 2, 2 * N_puntos)
    
        z1 = (altura / 2) * np.sin((4 * np.pi / T) * t1 - (np.pi / 2)) - 10
        z2 = np.full(N_puntos * 2, -11.5)
    
        t = np.concatenate((t1, t2))
        y = np.concatenate((y1, y2))
        z = np.concatenate((z1, z2))
    
        x = np.tile(x, N_ciclos)
        y = np.tile(y, N_ciclos)
        z = np.tile(z, N_ciclos)
    
        return t, x, y, z

def main(args=None):
    rclpy.init(args=args)
    node = CinematicaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()