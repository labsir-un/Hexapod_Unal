import time
from dynamixel_sdk import *
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from hexapod_interfaces.action import Posicionar

# Control table address
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_HARDWARE_ERROR = 70

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXM_MOVING_STATUS_THRESHOLD = 10

MAX_RETRIES = 5
RETRY_DELAY = 10

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
sync_write = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4)

class DynamixelNode(Node):
    def __init__(self):
        super().__init__('dynamixel_node')
        self.action = ActionServer(self, Posicionar, 'posicionar_motores', self.action_callback)

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

        try:
            if portHandler.setBaudRate(BAUDRATE):
                self.get_logger().info("Baudrate configurado correctamente")
            else:
                raise Exception("Error al configurar el baudrate")
        except Exception as e:
            self.get_logger().error(str(e))

        for i in range(1, 19):
            dxm_comm_result, dxm_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxm_comm_result != COMM_SUCCESS or dxm_error != 0:
                self.get_logger().error(f"Error al habilitar torque de motor [{i}]")
            else:
                self.get_logger().info(f"Torque habilitado en motor [{i}]")

    def action_callback(self, goal_handle):
        self.get_logger().info('Posicionando motores con Sync Write')
        feedback = Posicionar.Feedback()

        sync_write.clearParam()
        
        for i, position in enumerate(goal_handle.request.positions, start=1):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(position)),
                                   DXL_HIBYTE(DXL_LOWORD(position)),
                                   DXL_LOBYTE(DXL_HIWORD(position)),
                                   DXL_HIBYTE(DXL_HIWORD(position))]
            if not sync_write.addParam(i, param_goal_position):
                self.get_logger().error(f'Error al agregar motor {i} a Sync Write')
                goal_handle.abort()
                return Posicionar.Result(flag=False)
            
            feedback.status = f'Preparando motor {i} a {position}'
            goal_handle.publish_feedback(feedback)
        
        dxm_comm_result = sync_write.txPacket()
        if dxm_comm_result != COMM_SUCCESS:
            self.get_logger().error('Error al enviar posiciones con Sync Write')
            goal_handle.abort()
            return Posicionar.Result(flag=False)
        
        self.get_logger().info('Todos los motores posicionados correctamente')
        goal_handle.succeed()
        return Posicionar.Result(flag=True)

    def close_motors(self):
        for i in range(1, 19):
            dxm_comm_result, dxm_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxm_comm_result != COMM_SUCCESS or dxm_error != 0:
                self.get_logger().error(f"Error al deshabilitar torque de motor [{i}]")
            else:
                self.get_logger().info(f"Torque deshabilitado en motor [{i}]")
        portHandler.closePort()
        self.get_logger().info("Motores apagados y puerto cerrado.")


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción detectada (Ctrl+C). Limpiando recursos...')
    finally:
        node.close_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()