import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial

class ArduinoMagnetNode(Node):
    def __init__(self):
        super().__init__('arduino_magnet_node')
        self.subscription = self.create_subscription(
            Bool,
            'electromagnet_control',
            self.listener_callback,
            10)
        
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info('Connected to Arduino.')
        except serial.SerialException:
            self.get_logger().error('Could not connect to Arduino on /dev/ttyACM0.')
            exit(1)

    def listener_callback(self, msg):
        if msg.data:
            self.arduino.write(b'ON\n')
            self.get_logger().info('Magnet ON')
        else:
            self.arduino.write(b'OFF\n')
            self.get_logger().info('Magnet OFF')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoMagnetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

