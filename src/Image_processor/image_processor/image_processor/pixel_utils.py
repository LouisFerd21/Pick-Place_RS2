import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PIL import Image
import os
import time

# Strict color definitions with priority ordering
COLOR_DEFINITIONS = [
    ("Black",   lambda r, g, b: max(r, g, b) < 60),
    ("Red",     lambda r, g, b: r > max(g, b) + 50 and r > 150),
    ("Blue",    lambda r, g, b: b > max(r, g) + 50 and b > 150),
    ("Skin",    lambda r, g, b: (180 < r < 220 and 140 < g < 180 and 120 < b < 160)),
    ("White",   lambda r, g, b: min(r, g, b) > 200)  # Default catch-all
]

def get_color_name(rgb):
    r, g, b = rgb
    for name, condition in COLOR_DEFINITIONS:
        if condition(r, g, b):
            return name
    return "White"  # Final fallback

def publish_pixel_data():
    class PixelPublisher(Node):
        def __init__(self):
            super().__init__('pixel_publisher_gui')
            self.publisher_ = self.create_publisher(String, 'positionconv_package', 10)
            self.publish_chess_pixels()

        def publish_chess_pixels(self):
            try:
                img_path = os.path.expanduser("~/Pictures/Webcam/pixelated_8x8.png")
                if not os.path.exists(img_path):
                    self.get_logger().error("Pixelated image not found at: " + img_path)
                    return

                img = Image.open(img_path).convert('RGB')
                pixels = img.load()

                # Publish in chess board order (A8 to H1)
                for y in range(8):
                    for x in range(8):
                        # Convert to chess notation
                        col = chr(65 + x)  # A-H
                        row = str(8 - y)   # 1-8
                        position = f"{col}{row}"
                        
                        # Get color and force to one of our 5 colors
                        color = get_color_name(pixels[x, y])
                        
                        # Create and publish the strict format
                        msg = String()
                        msg.data = f"{position}, {color}"  # Exactly "A8, Red" format
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Published: {msg.data}")
                        time.sleep(0.05)

                self.get_logger().info("✅ Finished publishing in chess format")

            except Exception as e:
                self.get_logger().error(f"Publishing error: {str(e)}")

    try:
        rclpy.init()
        node = PixelPublisher()
        rclpy.spin_once(node, timeout_sec=1.0)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"ROS Error: {str(e)}")