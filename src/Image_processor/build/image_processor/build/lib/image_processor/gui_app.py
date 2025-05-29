import tkinter as tk
from tkinter import Label
from PIL import Image, ImageTk
import subprocess
import os
import time
import cv2
import numpy as np
from tkinter import filedialog
from image_processor.pixel_utils import publish_pixel_data


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

        publish_pixel_data()

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
        pixel_path = os.path.join(pixel_dir, "pixelated_8x8.png")
        cv2.imwrite(pixel_path, img)

        # Upscale for display
        pixelated = cv2.resize(img, (320, 320), interpolation=cv2.INTER_NEAREST)
        image_pil = Image.fromarray(cv2.cvtColor(pixelated, cv2.COLOR_BGR2RGB))
        img_tk = ImageTk.PhotoImage(image_pil)
        photo_label.config(image=img_tk)
        photo_label.image = img_tk

        print(f"✅ Text converted and saved at {pixel_path}")

        publish_pixel_data()


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
        
def main():
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

    button_frame = tk.Frame(root, bg="#F0F0F0")
    button_frame.pack(pady=20)

    tk.Button(button_frame, text="Capture Photo", command=capture_photo, **button_style).pack(side="left", padx=10)
    tk.Button(button_frame, text="Insert Text", command=lambda: text_to_pixel_image(text_entry.get()), **button_style).pack(side="left", padx=10)
    tk.Button(button_frame, text="Upload", command=upload_photo, **button_style).pack(side="left", padx=10)

    global text_entry
    text_entry = tk.Entry(root, font=("Arial", 14))
    text_entry.pack(pady=10)

    text_label = tk.Label(root, text="Type here to insert text", font=('Arial', 12), bg="#F0F0F0")
    text_label.pack(pady=5)

    global photo_label
    photo_label = Label(root)
    photo_label.pack(pady=20)

    clear_button = tk.Button(root, text="❌ Clear Image", command=lambda: photo_label.config(image=''), font=('Arial', 12), bg='#FF6347', fg='white', relief='raised', bd=2)
    clear_button.pack(side="bottom", pady=20)

    root.mainloop()
