import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from hexapod_interfaces.action import Posicionar
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

ADDR_GOAL_POSITION = 116
ADDR_HARDWARE_ERROR = 70
LEN_GOAL_POSITION = 4
PROTOCOL_VERSION = 2.0
DXL_ID = [i for i in range(1, 19)]
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'

class DynamixelNode(Node):
    def __init__(self):
        super().__init__('dynamixel_node')
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        if not self.port_handler.openPort():
            self.get_logger().error("Fallo al abrir el puerto.")
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Fallo al configurar el baudrate.")

        self.executor = ThreadPoolExecutor(max_workers=2)
        self.action_server = ActionServer(
            self,
            Posicionar,
            'posicionar_motores',
            self.handle_action_callback
        )

    def handle_action_callback(self, goal_handle):
        # Ejecutar en otro hilo para evitar bloqueo
        future = self.executor.submit(self.action_callback, goal_handle)
        return future

    def action_callback(self, goal_handle):
        self.get_logger().info("Acción recibida. Posicionando motores.")
        positions = goal_handle.request.positions

        for i, dxl_id in enumerate(DXL_ID):
            param_goal_position = [
                (positions[i] >> 0) & 0xFF,
                (positions[i] >> 8) & 0xFF,
                (positions[i] >> 16) & 0xFF,
                (positions[i] >> 24) & 0xFF
            ]
            success = self.sync_write.addParam(dxl_id, bytes(param_goal_position))
            if not success:
                self.get_logger().error(f"Fallo al agregar parámetros para motor {dxl_id}")
                goal_handle.abort()
                return Posicionar.Result(flag=False)

        dxl_comm_result = self.sync_write.txPacket()
        self.sync_write.clearParam()

        if dxl_comm_result != 0:
            self.get_logger().error(f"Error en transmisión: {dxl_comm_result}")
            goal_handle.abort()
            return Posicionar.Result(flag=False)

        # Verificar errores de hardware en cada motor
        for dxl_id in DXL_ID:
            dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, ADDR_HARDWARE_ERROR)[0]
            if dxl_error != 0:
                self.get_logger().error(f"Error de hardware en motor {dxl_id}: código {dxl_error}")
                goal_handle.abort()
                return Posicionar.Result(flag=False)

        self.get_logger().info("Todos los motores posicionados correctamente.")
        goal_handle.succeed()
        return Posicionar.Result(flag=True)

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
