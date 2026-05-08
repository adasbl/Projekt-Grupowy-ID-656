import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class UartBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.serial_port = serial.Serial('/dev/ttyTHS1', baudrate=115200, timeout=0.1)
        self.get_logger().info('Mostek UART uruchomiony. Czekam na /cmd_vel...')

    def cmd_vel_callback(self, msg):
        # Pobieramy floaty (nie ma potrzeby zaokrąglania przy przesyłaniu binarnym)
        v_lin = float(msg.linear.x)
        v_ang = float(msg.angular.z)
        
        # Pakujemy dane do formatu binarnego:
        # '<' : Little-Endian (kolejność bajtów stosowana w STM32)
        # 'B' : 1 bajt bez znaku (unsigned char) dla 0xAA
        # 'f' : 4 bajty zmiennoprzecinkowe (float) dla prędkości liniowej
        # 'f' : 4 bajty zmiennoprzecinkowe (float) dla prędkości kątowej
        # 'B' : 1 bajt bez znaku (unsigned char) dla 0x55
        frame = f"{v_lin}, {v_ang}\n"
        
        self.get_logger().info(f"Wysylam ramke binarna: v_lin={v_lin:.3f}, v_ang={v_ang:.3f}")
        
        # Wysyłamy surowe bajty (nie używamy .encode('utf-8'))
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
