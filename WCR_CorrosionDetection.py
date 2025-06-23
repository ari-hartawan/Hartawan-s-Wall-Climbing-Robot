import tkinter as tk
from tkinter import ttk, filedialog
import cv2
from PIL import Image, ImageTk
import os
import sys
from datetime import datetime
from ultralytics import YOLO
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import serial.tools.list_ports
import threading
import time
import numpy as np

def on_closing():
    root.quit()  # Stop the Tkinter event loop
    if cap.isOpened():  # Check if the camera is still open
        cap.release()  # Release the camera
    sys.exit()   # Exit the program

def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")

    return os.path.join(base_path, relative_path)

# Global variables
save_directory = ""
model = YOLO('../Cor-Seg.pt') # Load YOLO model
cap = None
video_writer = None
recording = False
ser = None  # Initialize serial as None

# Configure Baud rate
BAUD_RATE = 9600

# Data for graphing
data_x = []
data_rpmL = []
data_rpmR = []

# Function to read data from the serial port
def read_serial():
    while True:
        if ser and ser.is_open:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if "MotorL RPM:" in line and "MotorR RPM:" in line:
                try:
                    # Parse the RPM values
                    parts = line.split('|')
                    motorL_rpm = float(parts[2].split(':')[1].strip())
                    motorR_rpm = float(parts[3].split(':')[1].strip())
                    timestamp = time.time()

                    # Update data
                    data_x.append(timestamp)
                    data_rpmL.append(motorL_rpm)
                    data_rpmR.append(motorR_rpm)

                    # Limit the data to the last 100 points
                    if len(data_x) > 100:
                        data_x.pop(0)
                        data_rpmL.pop(0)
                        data_rpmR.pop(0)

                except (ValueError, IndexError):
                    pass

# Function to get available serial ports
def get_available_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]  # Extract the device names (e.g., 'COM3', 'COM4')


# Function to update the graph
def update_graph(i):
    # Clear the previous plots
    ax1.clear()
    ax2.clear()

    # Plot the left motor RPM on the first graph
    ax1.plot(data_x, data_rpmL, label="LEFT MOTOR", color="blue")
    ax1.set_title("LEFT MOTOR RPM")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("RPM")
    ax1.legend(loc="upper right")

    # Plot the right motor RPM on the second graph
    ax2.plot(data_x, data_rpmR, label="RIGHT MOTOR", color="red")
    ax2.set_title("RIGHT MOTOR RPM")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("RPM")
    ax2.legend(loc="upper right")


# Function to start the serial reading in a separate thread
def start_reading():
    global ser  # Access global 'ser'
    selected_com = serial_port_dropdown.get()  # Get selected COM port
    if selected_com:  # Check if a COM port is selected
        try:
            # Close any previous serial connection if open
            if ser and ser.is_open:
                ser.close()

            # Open the new serial connection
            ser = serial.Serial(selected_com, BAUD_RATE, timeout=1)
            print(f"Serial connection established on {selected_com}")
            # Start the serial reading in a separate thread
            threading.Thread(target=read_serial, daemon=True).start()
        except serial.SerialException:
            print(f"Error: Could not open serial port {selected_com}")
    else:
        print("Error: No COM port selected")

def browse_directory():
    global save_directory
    save_directory = filedialog.askdirectory()
    directory_entry.delete(0, tk.END)  # Clear current text in the entry
    directory_entry.insert(0, save_directory)  # Set new path in the entry

def get_available_cameras():
    available_cameras = []
    for i in range(3):  # Check the first 3 possible camera sources (adjust this if needed)
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(f"Camera {i}")
            cap.release()
    return available_cameras

def start_camera():
    global cap
    camera_source = camera_source_dropdown.get()
    camera_index = int(camera_source.split()[-1])  # Extract the index number from the string (e.g., "Camera 0" -> 0)

    if cap is not None:
        cap.release()

    cap = cv2.VideoCapture(camera_index)
    cap.set(3, 640)  # Set width
    cap.set(4, 480)  # Set height
    update_video_feed()

def capture_image():
    if save_directory and cap is not None:  # Check if a directory is selected
        success, frame = cap.read()  # Capture frame from the video feed
        if success:
            # Save the original frame before any annotation
            timestamp = datetime.now().strftime("%d-%m-%y_%H-%M-%S")  # Format: DD-MM-YY_HH-MM-SS
            original_img_name = f"{timestamp}_original.jpg"
            original_img_path = os.path.join(save_directory, original_img_name)
            cv2.imwrite(original_img_path, frame)  # Save the original frame
            print(f"Original image saved at: {original_img_path}")

            # YOLO Detection on the captured frame
            results = model(frame, conf=0.3, show=False)  # Run YOLO model on the frame
            annotated_frame = results[0].plot()  # Get the annotated frame

            # Initialize dictionaries to hold the total area for each class
            area_dict = {'Fair_Corrosion': 0, 'Poor_Corrosion': 0, 'Severe_Corrosion': 0}
            class_names = ['Fair_Corrosion', 'Poor_Corrosion', 'Severe_Corrosion']

            # Conversion factor: 1 cm² = 375 pixels² (25px x 15px box)
            pixel_to_cm2_conversion = 375

            height, width, _ = frame.shape  # Get the image size (height and width) for creating a blank mask

            # Iterate through all detected objects (boxes)
            for i, det in enumerate(results[0].boxes):
                class_id = int(det.cls)
                class_name = class_names[class_id]

                segmentation_mask = results[0].masks.xy[i]
                binary_mask = np.zeros((height, width), dtype=np.uint8)
                polygon_points = np.array(segmentation_mask, dtype=np.int32)
                cv2.fillPoly(binary_mask, [polygon_points], 255)

                corrosion_area_pixels = np.sum(binary_mask > 0)
                corrosion_area_cm2 = corrosion_area_pixels / pixel_to_cm2_conversion
                area_dict[class_name] += corrosion_area_cm2

            y_position = height - 60
            for class_name, area in area_dict.items():
                text = f"{class_name}: {area:.2f} cm^2"
                cv2.putText(annotated_frame, text, (5, y_position), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
                y_position += 25

            # Save the annotated frame
            annotated_img_name = f"{timestamp}_segmented.jpg"
            annotated_img_path = os.path.join(save_directory, annotated_img_name)
            cv2.imwrite(annotated_img_path, annotated_frame)
            print(f"Segmented image saved at: {annotated_img_path}")
        else:
            print("Failed to capture image")
    else:
        print("Please select a directory to save the image")


def start_record():
    global video_writer, recording
    if save_directory and cap is not None and not recording:
        # Generate timestamp for the video filename
        timestamp = datetime.now().strftime("%d-%m-%y_%H-%M-%S")  # Format: DD-MM-YY_HH-MM-SS
        video_name = f"{timestamp}.mp4"  # Name the video with timestamp
        video_path = os.path.join(save_directory, video_name)

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = 30  # Frames per second
        frame_width = int(cap.get(3))
        frame_height = int(cap.get(4))
        video_writer = cv2.VideoWriter(video_path, fourcc, fps, (frame_width, frame_height))
        recording = True
        print(f"Recording started: {video_path}")
    else:
        print("Please select a directory and ensure the camera is running")

def stop_record():
    global video_writer, recording
    if recording:
        recording = False
        video_writer.release()
        video_writer = None
        print("Recording stopped")

def update_video_feed():
    global video_writer, recording
    success, frame = cap.read()  # Capture a frame from the video feed
    if success:
        # YOLO Detection on the frame
        results = model(frame, conf=0.3, show=False)  # Run YOLO model on the frame
        annotated_frame = results[0].plot()  # Get the annotated frame

        # Initialize dictionaries to hold the total area for each class
        area_dict = {'Fair_Corrosion': 0, 'Poor_Corrosion': 0, 'Severe_Corrosion': 0}
        class_names = ['Fair_Corrosion', 'Poor_Corrosion', 'Severe_Corrosion']

        # Conversion factor: 1 cm² = 375 pixels² (25px x 15px box)
        pixel_to_cm2_conversion = 375

        height, width, _ = frame.shape  # Get the image size (height and width) for creating a blank mask

        # Iterate through all detected objects (boxes)
        for i, det in enumerate(results[0].boxes):  # Iterate through all detected objects
            class_id = int(det.cls)  # Get the class index
            class_name = class_names[class_id]  # Get the class name from the class index

            # Get the segmentation mask for this object
            segmentation_mask = results[0].masks.xy[i]  # Get the mask for this detection

            # Initialize a blank mask of the same size as the original image (black)
            binary_mask = np.zeros((height, width), dtype=np.uint8)

            # Fill the polygonal mask into the binary mask (white = 255, black = 0)
            polygon_points = np.array(segmentation_mask, dtype=np.int32)  # Convert the mask to an integer array
            cv2.fillPoly(binary_mask, [polygon_points], 255)  # Fill the polygon into the binary mask

            # Calculate the area of the corrosion region (in pixels)
            corrosion_area_pixels = np.sum(binary_mask > 0)  # Count the white pixels

            # Convert the area from pixels to cm²
            corrosion_area_cm2 = corrosion_area_pixels / pixel_to_cm2_conversion

            # Add the area to the corresponding corrosion class area in the dictionary
            area_dict[class_name] += corrosion_area_cm2

        # Display area measurements on the annotated frame
        y_position = height - 60  # Position it just above the bottom of the image

        for class_name, area in area_dict.items():
            text = f"{class_name}: {area:.2f} cm^2"
            cv2.putText(annotated_frame, text, (5, y_position), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
            y_position += 25  # Move down for the next text line

        # Write the frame to the video file if recording
        if recording and video_writer is not None:
            video_writer.write(annotated_frame)

        # Convert the frame to RGB (Pillow Image format)
        frame_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(frame_rgb)
        img_tk = ImageTk.PhotoImage(img_pil)

        # Update the label with the new frame
        video_label.img_tk = img_tk  # Keep a reference to avoid garbage collection
        video_label.config(image=img_tk)

    # Schedule the next frame update
    video_label.after(10, update_video_feed)  # Update every 10 ms


# Create the main window
root = tk.Tk()
root.title("Corr-Vision")
root.geometry("1280x1080")
root.configure(bg="#ffffff")

# Title
title_label = tk.Label(root, text="CAMERA FEED CORROSION DETECTION", font=("Arial", 20, "bold"), bg="#004aad", fg="#ffffff")
title_label.place(x=80, y=110, width=645, height=70)

# Camera Source Section
camera_source_label = tk.Label(root, text="CAMERA SOURCE", font=("Arial", 10, "bold"), bg="#004aad", fg="#ffffff")
camera_source_label.place(x=80, y=200, width=200, height=30)

available_cameras = get_available_cameras()
camera_source_dropdown = ttk.Combobox(root, values=available_cameras, state="readonly")
camera_source_dropdown.current(0)  # Default to the first available camera
camera_source_dropdown.place(x=285, y=200, width=440, height=30)

# Live Video Feed
video_frame = tk.Frame(root, bg="#004aad", width=645, height=480)
video_frame.place(x=80, y=250)

video_label = tk.Label(video_frame, text="CAMERA SOURCE: 640X480", bg="#004aad", fg="#000000")
video_label.pack(expand=True)

# Start Camera Button
start_camera_button = tk.Button(root, text="START CAMERA", font=("Arial", 10, "bold"), bg="#004aad", fg="#ffffff", command=start_camera)
start_camera_button.place(x=80, y=750, width=150, height=60)

# Capture Image Button
capture_button = tk.Button(root, text="CAPTURE IMAGE", font=("Arial", 10, "bold"), bg="#004aad", fg="#ffffff", command=capture_image)
capture_button.place(x=245, y=750, width=150, height=60)

# Start Record Button
record_button = tk.Button(root, text="RECORD VIDEO", font=("Arial", 10, "bold"), bg="#004aad", fg="#ffffff", command=start_record)
record_button.place(x=410, y=750, width=150, height=60)

# Stop Record Button
stop_record_button = tk.Button(root, text="STOP RECORD", font=("Arial", 10, "bold"), bg="#004aad", fg="#ffffff", command=stop_record)
stop_record_button.place(x=575, y=750, width=150, height=60)

# Directory Section
directory_label = tk.Label(root, text="IMAGE AND VIDEO DIRECTORY", font=("Arial", 10, "bold"), bg="#004aad", fg="#ffffff")
directory_label.place(x=80, y=835, width=645, height=30)

directory_frame = tk.Frame(root, bg="#004aad", width=520, height=30)
directory_frame.place(x=80, y=870)
directory_entry = tk.Entry(root, font=("Arial", 10), bg="#ffffff", fg="#000000")
directory_entry.place(x=81, y=871, width=518, height=28)

browse_button = tk.Button(root, text="BROWSE", font=("Arial", 10, "bold"), bg="#004aad", fg="#ffffff", command=browse_directory)
browse_button.place(x=615, y=870, width=110, height=30)

# COM Port Section
serial_port_label = tk.Label(root, text="SELECT COM PORT", font=("Arial", 10, "bold"), bg="#004aad", fg="#ffffff")
serial_port_label.place(x=1380, y=110, width=150, height=30)

# COM Port Dropdown
serial_ports = get_available_ports()
serial_port_dropdown = ttk.Combobox(root, values=serial_ports, state="readonly")
if serial_ports:
    serial_port_dropdown.current(0)  # Default to the first available port
else:
    serial_port_dropdown.set("No ports available")
serial_port_dropdown.place(x=1540, y=110, width=330, height=30)

# Start Monitoring Button
start_monitoring_button = tk.Button(root, text="START MONITORING", font=("Arial", 10, "bold"), bg="#004aad", fg="#ffffff", command=start_reading)
start_monitoring_button.place(x=770, y=110, width=170, height=30)

# Robot Speed Monitoring
speed_label = tk.Label(root, text="ROBOT SPEED", font=("Arial", 12, "bold"), bg="#004aad", fg="#ffffff")
speed_label.place(x=770, y=150, width=1100, height=30)

# Create a figure for two subplots (two graphs)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.63, 4), dpi=100)
fig.tight_layout(pad=4)

# Create a canvas to display the graphs
canvas = FigureCanvasTkAgg(fig, master=root)
canvas_widget = canvas.get_tk_widget()
canvas.get_tk_widget().place(x=770, y=180)

# Start the animation for the graphs
ani = FuncAnimation(fig, update_graph, interval=100, cache_frame_data=False)

# Bind the close event to stop operations
root.protocol("WM_DELETE_WINDOW", on_closing)

# Initialize the main loop
root.mainloop()