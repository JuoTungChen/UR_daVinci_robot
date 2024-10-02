import argparse
import dynamixel
import tkinter as tk
from tkinter import ttk
import numpy as np

# Function to update motor positions based on slider values
def update_motors():
    new_positions = [slider_vars[i].get() for i in range(len(slider_vars))]
    
    # Move the motors to the new positions
    for dev, q in zip(chain.devices, new_positions):
        dev.write_param_single('goal_position', q)

# Create GUI window
root = tk.Tk()
root.title("Motor Control GUI")

# Function to update motors when the "Apply" button is pressed
def apply_changes():
    update_motors()

# Create Dynamixel chain
parser = argparse.ArgumentParser()
parser.add_argument('port')
parser.add_argument('baudrate', type=int)
parser.add_argument('-d', '--device-ids', nargs='+', type=int, required=True, help='Device IDs')
args = parser.parse_args()
chain = dynamixel.Chain(args.port, args.baudrate, args.device_ids)

if not chain.devices:
    raise RuntimeError("No devices found")

if len(chain.devices) != len(args.device_ids):
    raise RuntimeError("Not all devices found")

# Read the current joint positions
current_positions = [dev.read_param_single('present_position') for dev in chain.devices]

# Create sliders for each motor with labels and angle values
slider_vars = [tk.DoubleVar(value=pos) for pos in current_positions]
slider_labels = ['Yaw1', 'Pitch', 'Yaw2', 'Roll']
slider_limits = [
    (current_positions[0] - np.pi/4, current_positions[0] + np.pi/4),  # Yaw1
    (current_positions[1] - np.pi/5, current_positions[1] + np.pi/5),  # Pitch
    (current_positions[2] - np.pi/4, current_positions[2] + np.pi/4),  # Yaw2
    (current_positions[3] - np.pi, current_positions[3] + np.pi)       # Roll
]
sliders = []

for i, (var, label, limits) in enumerate(zip(slider_vars, slider_labels, slider_limits)):
    slider_frame = ttk.Frame(root)
    slider_frame.pack(fill='x', padx=10, pady=5)
    
    # Label
    ttk.Label(slider_frame, text=label).pack(side='left')
    
    # Min, Max, and Current value labels
    min_label = ttk.Label(slider_frame, text=f'Min: {limits[0]:.2f}')
    min_label.pack(side='left', padx=5)
    
    max_label = ttk.Label(slider_frame, text=f'Max: {limits[1]:.2f}')
    max_label.pack(side='left', padx=5)
    
    current_label = ttk.Label(slider_frame, text=f'Current: {current_positions[i]:.2f}')
    current_label.pack(side='left', padx=5)
    
    # Slider
    slider = ttk.Scale(slider_frame, from_=limits[0], to=limits[1], variable=var, orient=tk.HORIZONTAL, length=200)
    slider.pack(side='left', padx=5)
    
    sliders.append(slider)

# Create "Apply" button
apply_button = tk.Button(root, text="Apply Changes", command=apply_changes)
apply_button.pack(pady=10)

# Run GUI loop
root.mainloop()
