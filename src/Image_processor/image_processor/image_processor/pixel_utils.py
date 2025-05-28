import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PIL import Image
import os
import json
import time

def publish_pixel_data():
    class PixelPublisher(Node):
        def __init__(self):
            super().__init__('pixel_publisher_gui')
            self.publisher_ = self.create_publisher(String, 'positionconv_package', 10)
            self.image_path = self.find_pixel_image()
            if self.image_path:
                self.publish_pixels()

        def find_pixel_image(self):
            picture_dir = os.path.expanduser("~/Pictures/Webcam")
            file_path = os.path.join(picture_dir, "pixelated_8x8.png")
            if os.path.exists(file_path):
                return file_path
            self.get_logger().error("pixelated_8x8.png not found.")
            return None

        def publish_pixels(self):
            try:
                img = Image.open(self.image_path).convert("RGB")
                pixels = img.load()
                for y in range(8):
                    for x in range(8):
                        label = f"{chr(65 + x)}{8 - y}"
                        color = pixels[x, y]
                        msg = String()
                        msg.data = json.dumps({
                            "label": label,
                            "rgb": {"r": color[0], "g": color[1], "b": color[2]}
                        })
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Published: {msg.data}")
                        time.sleep(0.1)
                self.get_logger().info("✅ Finished publishing all pixel data.")
            except Exception as e:
                self.get_logger().error(f"Error publishing pixels: {e}")

    rclpy.init()
    node = PixelPublisher()
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()
