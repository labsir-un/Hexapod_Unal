import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from hexapod_interfaces.action import Posicionar

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

ADDR_GOAL_POSITION = 116
ADDR_HARDWARE_ERROR = 70
LEN_GOAL_POSITION = 4
PROTOCOL_VERSION = 2.0
DXL_ID = [i for i in range(1, 19)]  # IDs del 1 al 18
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
        else:
            self.get_logger().info("Puerto abierto correctamente.")

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Fallo al configurar el baudrate.")
        else:
            self.get_logger().info("Baudrate configurado correctamente.")

        self.action_server = ActionServer(
            self,
            Posicionar,
            'posicionar_motores',
            self.handle_action_callback
        )

        ADDR_TORQUE_ENABLE = 64
        TORQUE_ENABLE = 1

        for dxl_id in DXL_ID:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
            )
            if dxl_comm_result != 0 or dxl_error != 0:
                self.get_logger().error(f"Error activando torque en motor {dxl_id} (result={dxl_comm_result}, error={dxl_error})")
            else:
                self.get_logger().info(f"Torque activado en motor {dxl_id}")

        ADDR_PROFILE_VELOCITY = 112
        profile_velocity = 1023  # Ajusta este valor entre 0–1023 o más según el modelo

        for dxl_id in DXL_ID:
            node.packet_handler.write4ByteTxRx(
                node.port_handler, dxl_id, ADDR_PROFILE_VELOCITY, profile_velocity
            )

    async def handle_action_callback(self, goal_handle):
        self.get_logger().info("Acción recibida. Posicionando motores.")
        positions = goal_handle.request.positions

        if len(positions) != len(DXL_ID):
            self.get_logger().error("Cantidad de posiciones no coincide con cantidad de motores.")
            goal_handle.abort()
            return Posicionar.Result(flag=False)

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

        # Verificar errores de hardware

        #for dxl_id in DXL_ID:
        #    _, dxl_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, ADDR_HARDWARE_ERROR)
        #    if dxl_result != 0 or dxl_error != 0:
        #        self.get_logger().error(f"Error de hardware en motor {dxl_id} (result={dxl_result}, error={dxl_error})")
        #        goal_handle.abort()
        #        return Posicionar.Result(flag=False)

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
        # Desactivar torque en todos los motores antes de apagar
        for dxl_id in DXL_ID:
            node.packet_handler.write1ByteTxRx(
                node.port_handler, dxl_id, 64, 0  # 64 = ADDR_TORQUE_ENABLE
            )
            node.get_logger().info(f"Torque desactivado en motor {dxl_id}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
