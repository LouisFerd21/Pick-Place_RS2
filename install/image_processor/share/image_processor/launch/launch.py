import rclpy
import time
from image_processor.pixel_publisher import PixelPublisher
from image_processor.capture_pixelate import main as capture_main

def main():
    print("STEP 1: Capturing and pixelating image...")
    capture_main()  # This should block until the image is saved
    print("STEP 2: Starting pixel data publisher after capture.")
    time.sleep(1)  # slight buffer if needed

    rclpy.init()
    node = PixelPublisher()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
