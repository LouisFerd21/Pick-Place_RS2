import tkinter as tk
from tkinter import Label
from PIL import Image, ImageTk
import subprocess
import os
import time
import cv2
import numpy as np
from tkinter import filedialog

# Define waypoints (joint angles for the robot to place each pixel)
waypoints = {
    'original_pose': {
        'shoulder_lift_joint': -1.8501, 'elbow_joint': -1.2195, 'wrist_1_joint': -1.5818,
        'wrist_2_joint': -4.6681, 'wrist_3_joint': 1.4268, 'shoulder_pan_joint': -1.6306
    },
    'waypoint_1': {
        'shoulder_lift_joint': -2.1929, 'elbow_joint': -0.8825, 'wrist_1_joint': -1.5872,
        'wrist_2_joint': -4.6686, 'wrist_3_joint': 1.4268, 'shoulder_pan_joint': -0.9825
    },
    'waypoint_2': {
        'shoulder_lift_joint': -2.2696, 'elbow_joint': -1.0022, 'wrist_1_joint': -1.3498,
        'wrist_2_joint': -4.6592, 'wrist_3_joint': 1.4268, 'shoulder_pan_joint': -1.3401
    },
    'waypoint_3': {
        'shoulder_lift_joint': -2.2698, 'elbow_joint': -1.0021, 'wrist_1_joint': -1.3498,
        'wrist_2_joint': -4.6591, 'wrist_3_joint': 1.4268, 'shoulder_pan_joint': -1.6718
    },
    'A1': {
        'shoulder_lift_joint': -2.9230, 'elbow_joint': -0.4769, 'wrist_1_joint': -1.2894,
        'wrist_2_joint': -4.6641, 'wrist_3_joint': 1.4278, 'shoulder_pan_joint': -1.1087
    },
    'C1': {
        'shoulder_lift_joint': -2.9126, 'elbow_joint': -0.4580, 'wrist_1_joint': -1.2753,
        'wrist_2_joint': -4.6591, 'wrist_3_joint': 1.4267, 'shoulder_pan_joint': -1.3868
    },
    'E1': {
        'shoulder_lift_joint': -2.9127, 'elbow_joint': -0.4579, 'wrist_1_joint': -1.2754,
        'wrist_2_joint': -4.6591, 'wrist_3_joint': 1.4268, 'shoulder_pan_joint': -1.6705
    }
}

# Function to convert pixel coordinates to waypoints (joint angles)
def pixel_to_waypoint(x, y, image_width, image_height):
    # Scale the pixel coordinates to the range of 0 to 7 (for 8x8 grid)
    scaled_x = int((x / image_width) * 8)
    scaled_y = int((y / image_height) * 8)
    
    # Map the pixel (scaled_x, scaled_y) to a corresponding waypoint (simplified)
    # For now, we just pick waypoints from the waypoints dictionary (mapping to an 8x8 grid)
    waypoint_keys = list(waypoints.keys())
    index = (scaled_x + scaled_y) % len(waypoint_keys)  # Map pixel to waypoint index
    waypoint_key = waypoint_keys[index]
    
    return waypoints[waypoint_key]

# Function to move robot to waypoint (For demonstration, printing the waypoint)
def move_robot_to_waypoint(waypoint):
    # Here you should add code to actually move the robot, for now it prints the waypoint
    print(f"Moving robot to: {waypoint}")


def capture_photo():
    # Open Cheese
    cheese_process = subprocess.Popen(["cheese"])
    time.sleep(3)  # Give Cheese time to fully load

    # Find Cheese's window ID
    window_id_result = subprocess.run(["xdotool", "search", "--onlyvisible", "--class", "cheese"], capture_output=True, text=True)
    window_ids = window_id_result.stdout.strip().split("\n")

    if not window_ids or window_ids[0] == '':
        print("Error: Cheese window not found!")
        return

    cheese_window_id = window_ids[0]

    # Bring Cheese to the foreground
    subprocess.run(["xdotool", "windowactivate", cheese_window_id])
    time.sleep(1)

    # Simulate pressing "Space" to take a photo
    subprocess.run(["xdotool", "key", "space"])

    time.sleep(5)  # Ensure photo is saved
    subprocess.run(["xdotool", "windowclose", cheese_window_id])
    time.sleep(2)

    photo_dir = os.path.expanduser("~/Pictures/Webcam")
    try:
        photos = sorted(
            [f for f in os.listdir(photo_dir) if f.endswith(('.png', '.jpg', '.jpeg'))],
            key=lambda x: os.path.getmtime(os.path.join(photo_dir, x)),
            reverse=True
        )
        if photos:
            latest_photo = os.path.join(photo_dir, photos[0])
            display_photo(latest_photo)
            pixelate_and_display(latest_photo, photo_dir)
        else:
            print("No photos found in", photo_dir)
    except FileNotFoundError:
        print(f"Error: The directory {photo_dir} does not exist.")

def display_photo(photo_path):
    try:
        img = Image.open(photo_path)
        img = img.resize((500, 300), Image.Resampling.LANCZOS)
        img_tk = ImageTk.PhotoImage(img)
        photo_label.config(image=img_tk)
        photo_label.image = img_tk
    except Exception as e:
        print(f"Error loading photo: {e}")

def pixelate_and_display(photo_path, photo_dir):
    try:
        image = cv2.imread(photo_path)
        resized_image = cv2.resize(image, (8, 8), interpolation=cv2.INTER_AREA)
        pixelated_image = cv2.resize(resized_image, (320, 320), interpolation=cv2.INTER_NEAREST)

        pixelated_image_path = os.path.join(photo_dir, "pixelated_8x8.png")
        cv2.imwrite(pixelated_image_path, resized_image)
        print(f"✅ Pixelated image saved as: {pixelated_image_path}")

        pixelated_image_pil = Image.fromarray(cv2.cvtColor(pixelated_image, cv2.COLOR_BGR2RGB))
        img_tk = ImageTk.PhotoImage(pixelated_image_pil)
        photo_label.config(image=img_tk)
        photo_label.image = img_tk
    except Exception as e:
        print(f"Error pixelating photo: {e}")

def text_to_pixel_image(text):


    if not text:
        print("No text entered.")
        return

    try:
        # Create a 16x16 white image
        img = np.ones((8, 8, 3), dtype=np.uint8) * 255
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.3  # Tiny font scale to fit in 16x16
        thickness = 1

        # Get text size
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        text_x = (img.shape[1] - text_size[0]) // 2
        text_y = (img.shape[0] + text_size[1]) // 2

        # Draw text in black
        cv2.putText(img, text, (text_x, text_y), font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)

        # Save actual 16x16 image if needed
        pixel_dir = os.path.expanduser("~/Pictures/Webcam")
        os.makedirs(pixel_dir, exist_ok=True)
        pixel_path = os.path.join(pixel_dir, "text_pixelated_8x8.png")
        cv2.imwrite(pixel_path, img)

        # Upscale for display
        pixelated = cv2.resize(img, (320, 320), interpolation=cv2.INTER_NEAREST)
        image_pil = Image.fromarray(cv2.cvtColor(pixelated, cv2.COLOR_BGR2RGB))
        img_tk = ImageTk.PhotoImage(image_pil)
        photo_label.config(image=img_tk)
        photo_label.image = img_tk

        print(f"✅ Text converted and saved at {pixel_path}")


    except Exception as e:


        print(f"Error creating pixelated text: {e}")

def upload_photo():
    file_path = filedialog.askopenfilename(
        title="Select an Image",
        filetypes=[("Image Files", ("*.png", "*.jpg", "*.jpeg"))]
    )
    if file_path:
        display_photo(file_path)
        photo_dir = os.path.dirname(file_path)
        pixelate_and_display(file_path, photo_dir)

# UI setup
root = tk.Tk()
root.title("What do you want UR3 to print?")
root.geometry("600x700")
root.config(bg="#F0F0F0")  # Light background color

# Buttons with square shape, aligned horizontally, and styled
button_style = {
    'width': 10,
    'height': 3,
    'font': ('Arial', 14),
    'bg': '#4CAF50',
    'fg': 'white',
    'relief': 'raised',
    'bd': 2,
}

# Horizontal frame for the first set of buttons (Capture, Insert, Upload)
button_frame = tk.Frame(root, bg="#F0F0F0")
button_frame.pack(pady=20)

tk.Button(button_frame, text="Capture Photo", command=capture_photo, **button_style).pack(side="left", padx=10)
tk.Button(button_frame, text="Insert Text", command=lambda: text_to_pixel_image(text_entry.get()), **button_style).pack(side="left", padx=10)
tk.Button(button_frame, text="Upload", command=upload_photo, **button_style).pack(side="left", padx=10)

# Text Entry Box
text_entry = tk.Entry(root, font=("Arial", 14))
text_entry.pack(pady=10)

# Label for "Type here to insert text"
text_label = tk.Label(root, text="Type here to insert text", font=('Arial', 12), bg="#F0F0F0")
text_label.pack(pady=5)

# Label to display photos
photo_label = Label(root)
photo_label.pack(pady=20)

# Clear Image button with original style at the bottom
clear_button = tk.Button(root, text="❌ Clear Image", command=lambda: photo_label.config(image=''), font=('Arial', 12), bg='#FF6347', fg='white', relief='raised', bd=2)
clear_button.pack(side="bottom", pady=20)

root.mainloop()
