import subprocess
import time
import os
import cv2

def main():
    print("📸 Starting Cheese to capture photo...")

    # Launch Cheese
    cheese = subprocess.Popen(["cheese"])
    time.sleep(3)

    # Find Cheese window
    result = subprocess.run(["xdotool", "search", "--onlyvisible", "--class", "cheese"], capture_output=True, text=True)
    window_ids = result.stdout.strip().split("\n")

    if not window_ids or window_ids[0] == '':
        print("❌ Error: Cheese window not found!")
        return

    cheese_id = window_ids[0]
    subprocess.run(["xdotool", "windowactivate", cheese_id])
    time.sleep(1)
    subprocess.run(["xdotool", "key", "space"])  # Press space to take photo
    time.sleep(5)
    subprocess.run(["xdotool", "windowclose", cheese_id])
    time.sleep(2)

    # Find latest photo
    photo_dir = os.path.expanduser("~/Pictures/Webcam")
    os.makedirs(photo_dir, exist_ok=True)

    try:
        photos = sorted(
            [f for f in os.listdir(photo_dir) if f.endswith(('.png', '.jpg', '.jpeg'))],
            key=lambda f: os.path.getmtime(os.path.join(photo_dir, f)),
            reverse=True
        )

        if not photos:
            print("❌ No photo found in Webcam folder.")
            return

        latest_photo = os.path.join(photo_dir, photos[0])
        print(f"✅ Photo captured: {latest_photo}")

        # Pixelate
        image = cv2.imread(latest_photo)
        resized = cv2.resize(image, (8, 8), interpolation=cv2.INTER_AREA)
        pixelated_path = os.path.join(photo_dir, "pixelated_8x8.png")
        cv2.imwrite(pixelated_path, resized)
        print(f"✅ Saved pixelated image as: {pixelated_path}")

    except Exception as e:
        print(f"❌ Error processing photo: {e}")
