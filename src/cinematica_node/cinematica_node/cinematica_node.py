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

        super().__init__('cinematica_node')
        
        self.get_logger().info('Calculate service ready.')

        self.srv1 = self.create_service(SiguientePosicion,'siguiente_posicion',self.send_position)

        self.act = ActionServer(self, Calcular, 'calcular_trayectoria', self.action_callback)
        
        self.srv = self.create_service(SetBool, 'finalizar_cinematica', self.finalizar)

        self.send_true = False
        self.index = 0  # Índice para recorrer los datos
        self.max = 0

    def finalizar(self, request, response):
        """
        Servicio que finaliza el nodo si recibe True.
        """
        if request.data:
            self.get_logger().info("Finalizando nodo de cinemática...")
            self.destroy_node()
            response.success = True
            response.message = "Nodo de cinemática finalizado."
        else:
            response.success = False
            response.message = "El nodo sigue activo."
        return response
    def action_callback(self, goal_handle):
        self.get_logger().info("Calculando trayectoria...")
        feedback = Calcular.Feedback()
        result = Calcular.Result()
        if goal_handle.request.indicacion:
            for i in range(1):
                feedback.position = i
                goal_handle.publish_feedback(feedback)
                time.sleep(1)
            self.q = self.calcular_trayectoria()
            self.get_logger().info("Trayectoria calculada.")
            self.get_logger().info(str(len(self.q[1])))
            self.max = len(self.q[1])
            result.flag = True
        else:
            result.flag = False
        goal_handle.succeed()
        return result

    def send_position(self,request,response):
        index = request.index
        if index >= self.max:
            index -= self.max
        self.get_logger().info(f'Received request for Position {index}')
        response.positions = self.q[:,index].flatten().tolist()
        return response
    
    def calcular_trayectoria(self):
        L1=5
        L2=6
        L3=13
        T_total=5
        N_ciclos=1
        T=T_total/N_ciclos
        N_puntos=20
    
        t, x, y, z = self.trayectoria(7.5,3,T,N_puntos,N_ciclos)
    
        t_total=np.linspace(0,T_total,4*N_puntos*N_ciclos)
        L4xy=np.sqrt(x**2+y**2)-L1
    
        q=np.zeros((18,4*N_puntos*N_ciclos),float)
        for foot_number in range(1,7):
            q1 = np.degrees(np.arctan2(y, x))
            q2 = np.degrees(np.arccos(((L4xy ** 2 + z ** 2) + L2 ** 2 - L3 ** 2) / (2 * L2 * np.sqrt(L4xy ** 2 + z ** 2))) + np.arctan2(z,L4xy)) - 3
            q2 = -q2
            q3 = np.degrees(np.arccos((L2 ** 2 + L3 ** 2 - (L4xy ** 2 + z ** 2)) / (2 * L2 * L3))) - 35
            if foot_number%2==0:
                if foot_number==4:
                    q1=-q1
                    q2=np.flip(q2)
                    q3=np.flip(q3)
            else:
                q1=-q1
                if foot_number==1 or foot_number==5:
                    q1=-q1
                    q2 = np.flip(q2)
                    q3 = np.flip(q3)
    
            q[3*foot_number-3, :] = q1
            q[3*foot_number-2, :] = q2
            q[3*foot_number-1, :] = q3
    
        return q

    def trayectoria(self, l_total, altura, T, N_puntos, N_ciclos):
        x = np.zeros(N_puntos * 4) + 12
        t1=np.linspace(0, T / 2, 2 * N_puntos)
        t2=np.linspace(T / 2, T, 2 * N_puntos)
    
        y1=(l_total/2)*np.sin((2*np.pi/T)*t1-(np.pi/2))
        y2=np.linspace(l_total/2,-l_total/2,2*N_puntos)
    
        z1=(altura/2)*np.sin((4*np.pi/T)*t1-(np.pi/2))-10
        z2=np.zeros(N_puntos*2)-11.5
    
        t=np.concatenate((t1,t2))
        y=np.concatenate((y1,y2))
        z=np.concatenate((z1,z2))
    
        x=np.tile(x, N_ciclos)
        y=np.tile(y,N_ciclos)
        z=np.tile(z,N_ciclos)
    
        return t,x,y,z

def main(args=None):
    rclpy.init(args=args)
    node = CinematicaNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
