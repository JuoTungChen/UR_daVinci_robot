"""
Bring Dynamixel servos with the given IDs to their center position.

Usage example:

    python3 center_servos.py /dev/ttyUSB0 2000000 --device-ids 20 21 22 23
"""

import argparse
import dynamixel
import numpy as np
import random
import time

# Function to randomly adjust motor positions
def random_adjustment(center, max_deviation):
    return center + random.uniform(-max_deviation, max_deviation)
def small_adjustment(center, deviation):
    return center + deviation
# def main():
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

# If limits both equal 2*pi (raw value 4095) the servo is in multi-turn mode
limits = [(dev.read_param_single('cw_angle_limit'), dev.read_param_single('ccw_angle_limit')) for dev in chain.devices]
# center = [lo + (hi - lo) / 2 if not np.allclose([lo, hi], 2*np.pi) else 0 for lo, hi in limits]
# Read the current joint positions
current_positions = [dev.read_param_single('present_position') for dev in chain.devices]

print("Limits: {}".format(np.degrees(limits)))
print("Moving to center: {}".format(current_positions))

# Specify the maximum deviation for random adjustments
max_deviation = np.radians(10)  # Adjust this value based on your requirements
print("max dev = ", max_deviation)
try:
    while True:
        # Randomly adjust the motors
        new_positions = [random_adjustment(q, max_deviation) for q in current_positions]
        # new_positions = [small_adjustment(q, max_deviation) for q in current_positions]
        print(new_positions)
        # Move the motors to the new positions
        for dev, q in zip(chain.devices, new_positions):
            dev.write_param_single('goal_position', q)

        # Wait for a short duration before the next adjustment
        time.sleep(1.0)  # Adjust this value based on your requirements

except KeyboardInterrupt:
    print("\nScript interrupted. Moving motors to the center before exiting.")


