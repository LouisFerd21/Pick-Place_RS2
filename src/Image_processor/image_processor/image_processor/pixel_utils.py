import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PIL import Image
import os
import time

# Enhanced color matching with better thresholds
COLOR_RANGES = {
    "Red":     lambda r, g, b: r > max(g, b) + 50 and r > 150,  # More dominant red
    "Blue":    lambda r, g, b: b > max(r, g) + 50 and b > 150,  # More dominant blue
    "Black":   lambda r, g, b: max(r, g, b) < 60,              # Dark colors
    "White":   lambda r, g, b: min(r, g, b) > 200,             # Bright colors
    "Skin":    lambda r, g, b: (180 < r < 220 and 140 < g < 180 and 120 < b < 160),
}

def get_named_color(rgb):
    r, g, b = rgb
    # Check colors in order of priority
    for color, condition in COLOR_RANGES.items():
        if condition(r, g, b):
            return color
    return "White"  # Default fallback

def publish_pixel_data():
    class PixelPublisher(Node):
        def __init__(self):
            super().__init__('pixel_publisher_gui')
            self.publisher_ = self.create_publisher(String, 'positionconv_package', 10)
            self.publish_pixels()

        def publish_pixels(self):
            try:
                # Direct path to pixelated image
                image_path = os.path.expanduser("~/Pictures/Webcam/pixelated_8x8.png")
                
                if not os.path.exists(image_path):
                    self.get_logger().error("Pixelated image not found!")
                    return

                img = Image.open(image_path).convert('RGB')
                pixels = img.load()
                
                # Publish all pixels in chess order (A8 to H1)
                for y in range(8):
                    for x in range(8):
                        # Chess notation conversion
                        chess_col = chr(65 + x)  # A-H
                        chess_row = str(8 - y)   # 1-8
                        position = f"{chess_col}{chess_row}"
                        
                        # Get and classify color
                        color = get_named_color(pixels[x, y])
                        
                        # Create and publish message
                        msg = String()
                        msg.data = f"{position}, {color}"
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Published: {msg.data}")
                        time.sleep(0.05)  # Small delay between messages
                
                self.get_logger().info("✅ Finished publishing all pixel data")

            except Exception as e:
                self.get_logger().error(f"Publishing failed: {str(e)}")

    try:
        rclpy.init()
        node = PixelPublisher()
        rclpy.spin_once(node, timeout_sec=1.0)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"ROS Error: {str(e)}")