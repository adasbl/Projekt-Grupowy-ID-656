import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # velocities subscriber
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # line for checking velocities values in terminal
        self.get_logger().info(f'Linear={linear_x:.2f}, Angular={angular_z:.2f}')

        '''
        HERE PLACE THE CODE FOR TRANSFERING VELOCITIES FROM JETSON TO STM32 VIA UART !!!
        '''

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()