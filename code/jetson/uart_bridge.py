import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty  # <-- Dodano import serwisu
import serial
import struct
import os                       # <-- Dodano do obsługi komend systemowych

class UartBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # <-- NOWY KOD: Utworzenie serwisu do wyłączania
        self.shutdown_srv = self.create_service(Empty, '/shutdown_jetson', self.shutdown_callback)
        
        self.serial_port = serial.Serial('/dev/ttyTHS1', baudrate=115200, timeout=0.1)
        self.get_logger().info('Mostek UART uruchomiony. Czekam na /cmd_vel...')

    # <-- NOWY KOD: Funkcja wywoływana, gdy PC wyśle żądanie
    def shutdown_callback(self, request, response):
        self.get_logger().warn('!!! Otrzymano sygnał wyłączenia systemu. Zamykam Jetsona !!!')
        os.system('sudo shutdown now')
        return response

    def cmd_vel_callback(self, msg):
        v_lin = float(msg.linear.x)
        v_ang = float(msg.angular.z)
        
        frame = struct.pack('<BffB', 0xAA, v_lin, v_ang, 0x55)
        self.get_logger().info(f"Wysylam ramke binarna: v_lin={v_lin:.3f}, v_ang={v_ang:.3f}")
        self.serial_port.write(frame)


def main(args=None):
    rclpy.init(args=args)
    node = UartBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
