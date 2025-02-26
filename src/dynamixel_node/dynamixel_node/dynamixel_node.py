import time
from dynamixel_sdk import *
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_srvs.srv import Trigger
from std_msgs.msg import String, Int32MultiArray
from hexapod_interfaces.action import Posicionar

# Control table address
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_HARDWARE_ERROR     = 70

# Protocol version
PROTOCOL_VERSION        = 2.0

# Default setting
DXM_ID                  = 1
BAUDRATE                = 1000000
DEVICENAME              = '/dev/ttyUSB0'

TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0
DXM_MOVING_STATUS_THRESHOLD = 10

MAX_RETRIES = 5  # Número máximo de intentos para abrir el puerto
RETRY_DELAY = 10  # Segundos de espera entre intentos

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

class DynamixelNode(Node):
    def __init__(self):
        super().__init__('dynamixel_node')

        #self.publisher = self.create_publisher(String, 'status_motor', 10)
        #self.timer = self.create_timer(1.0, self.status)
        #self.subscription = self.create_subscription(Int32MultiArray, 'motors_values', self.listener_callback, 10)
        #self.service = self.create_service(Trigger, 'status_motors', self.handle_Trigger)
        #self.position_service = self.create_service(Trigger, 'get_motor_positions', self.handle_get_motor_positions)
        self.action = ActionServer(self, Posicionar, 'posicionar_motores', self.action_callback)

        # Intentar abrir el puerto con reintentos y manejo de excepciones
        for attempt in range(MAX_RETRIES):
            try:
                if portHandler.openPort():
                    self.get_logger().info(f'Puerto abierto en {DEVICENAME}')
                    break
                else:
                    raise Exception(f'Error al abrir el puerto (Intento {attempt+1}/{MAX_RETRIES})')
            except Exception as e:
                self.get_logger().error(str(e))
                self.get_logger().info(f'Reintentando en {RETRY_DELAY} segundos...')
                time.sleep(RETRY_DELAY)
        else:
            self.get_logger().error("No se pudo abrir el puerto después de varios intentos. Saliendo...")
            exit(1)

        # Intentar configurar la tasa de baudios
        try:
            if portHandler.setBaudRate(BAUDRATE):
                self.get_logger().info("Baudrate configurado correctamente")
            else:
                raise Exception("Error al configurar el baudrate")
        except Exception as e:
            self.get_logger().error(str(e))

        # Habilitar el torque en todos los motores
        for i in range(1, 19):
            dxm_comm_result, dxm_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxm_comm_result != COMM_SUCCESS or dxm_error != 0:
                self.get_logger().error(f"Error al habilitar torque de motor [{i}]")
            else:
                self.get_logger().info(f"Torque habilitado en motor [{i}]")


    def action_callback(self, goal_handle):
        self.get_logger().info('Posicionando motores')
        feedback = Posicionar.Feedback()
        for i in range(1,len(goal_handle.request.positions)+1,1):
            posicion = goal_handle.request.positions[i-1]
            feedback.status = f'Posicionando motor {i} a {posicion}'
            self.set_motor_position(i,posicion)
            goal_handle.publish_feedback(feedback)
        result = Posicionar.Result()
        result.flag = True
        goal_handle.succeed()
        return result

    def handle_Trigger(self, request, response):
        errores = []
        for i in range(1, 19):
            error, dxm_comm_result, dxm_error = packetHandler.read1ByteTxRx(portHandler, i, ADDR_HARDWARE_ERROR)
            if dxm_comm_result != COMM_SUCCESS or dxm_error != 0:
                self.get_logger().error(f"Error al leer datos de motor [{i}]")
                errores.append('NA')
            elif error == 0:
                self.get_logger().info(f"Sin errores en motor [{i}]")
                errores.append('OK')
            else:
                errores.append('ER')
                if error & 0b00000001:
                    self.get_logger().warning(f"Voltaje incorrecto en motor [{i}]")
                if error & 0b00000100:
                    self.get_logger().warning(f"Sobrecalentamiento en motor [{i}]")
                if error & 0b00001000:
                    self.get_logger().warning(f"Error en encoder en motor [{i}]")
                if error & 0b00010000:
                    self.get_logger().warning(f"Choque eléctrico en motor [{i}]")
                if error & 0b00100000:
                    self.get_logger().warning(f"Carga excedida en motor [{i}]")

        response.success = True
        response.message = ", ".join(map(str, errores))
        return response

    def get_motor_positions(self):
        posiciones = []
        for i in range(1,19,1):
            dxm_comm_result, dxm_error, position = packetHandler.read4ByteTxRx(portHandler, k, ADDR_PRESENT_POSITION)
            if dxm_comm_result != COMM_SUCCESS or dxm_error != 0:
                self.get_logger().error(f"Error al leer la posición de motor [{k}]")
                posiciones.append(-100)
            else:
                self.get_logger().info(f"Posición de motor [{k}] es {position}")
                posiciones.append(position)
        return posiciones

    def set_motor_position(self, k, value):
        dxm_comm_result, dxm_error = packetHandler.write4ByteTxRx(portHandler, k, ADDR_GOAL_POSITION, value)
        if dxm_comm_result != COMM_SUCCESS or dxm_error != 0:
            self.get_logger().error(f'Error al mover motor [{k}]')
        else:
            self.get_logger().info(f'Motor [{k}] posicionado')

    def status(self):
        mensaje = String()
        mensaje.data = 'Hola'
        self.publisher.publish(mensaje)

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción detectada (Ctrl+C). Limpiando recursos...')
    finally:
        # Deshabilitar torque en todos los motores antes de salir
        for i in range(1, 19):
            dxm_comm_result, dxm_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxm_comm_result != COMM_SUCCESS or dxm_error != 0:
                node.get_logger().error(f"Error al deshabilitar torque de motor [{i}]")
            else:
                node.get_logger().info(f"Torque deshabilitado en motor [{i}]")

        portHandler.closePort()
        node.destroy_node()
        node.get_logger().info("Puerto cerrado y nodo destruido. Finalizando...")

if __name__ == '__main__':
    main()

