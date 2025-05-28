import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PIL import Image
import os
import time

# Define RGB ranges for approximate color matching
COLOR_RANGES = {
    "Red":     lambda r, g, b: r > 150 and g < 100 and b < 100,
    "Black":   lambda r, g, b: r < 60 and g < 60 and b < 60,
    "White":   lambda r, g, b: r > 200 and g > 200 and b > 200,
    "Skin":    lambda r, g, b: 180 < r < 255 and 140 < g < 220 and 120 < b < 200,
    "Blue":    lambda r, g, b: b > 150 and r < 100 and g < 100,
}

def get_named_color(rgb):
    r, g, b = rgb
    for name, condition in COLOR_RANGES.items():
        if condition(r, g, b):
            return name
    return "Unknown"

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
                img = Image.open(self.image_path)
                img = img.convert('RGB')
                pixels = img.load()
                width, height = img.size
                
                # Prepare the complete message
                message_parts = []
                
                for y in range(height):
                    for x in range(width):
                        # Convert coordinates to chess notation (A-H, 1-8)
                        chess_x = chr(65 + x)  # A-H
                        chess_y = str(8 - y)   # 1-8
                        position = f"{chess_x}{chess_y}"
                        
                        color = pixels[x, y]
                        color_name = get_named_color(color)
                        
                        # Create simple "A8, Red" format message
                        simple_msg = f"{position}, {color_name}"
                        msg = String()
                        msg.data = simple_msg
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Published: {simple_msg}")
                        time.sleep(0.05)
                
                self.get_logger().info("✅ Finished publishing all pixel data.")
                
            except Exception as e:
                self.get_logger().error(f"Error publishing pixels: {e}")

    try:
        rclpy.init()
        node = PixelPublisher()
        rclpy.spin_once(node, timeout_sec=1)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"Error in ROS communication: {e}")