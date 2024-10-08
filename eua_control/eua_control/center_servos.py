"""
Bring Dynamixel servos with the given IDs to their center position.

Usage example:

    python3 center_servos.py /dev/ttyUSB0 2000000 --device-ids 20 21 22 23
"""

import argparse
import dynamixel
import numpy as np


# def main():
parser = argparse.ArgumentParser()
parser.add_argument('port')
parser.add_argument('baudrate', type=int)
parser.add_argument('-d','--device-ids', nargs='+', type=int, required=True, help='Device IDs')
args = parser.parse_args()
chain = dynamixel.Chain(args.port, args.baudrate, args.device_ids)

if not chain.devices:
    raise RuntimeError("No devices found")

if len(chain.devices) != len(args.device_ids):
    raise RuntimeError("Not all devices found")

# If limits both equal 2*pi (raw value 4095) the servo is in multi-turn mode
limits = [(dev.read_param_single('cw_angle_limit'), dev.read_param_single('ccw_angle_limit')) for dev in chain.devices]
center = [lo + (hi - lo) / 2 if not np.allclose([lo, hi], 2*np.pi) else 0 for lo, hi in limits]

print("Limits: {}".format(np.degrees(limits)))
print("Moving to center: {}".format(center))

# for dev, q in zip(chain.devices, center):
    # dev.write_param_single('goal_position', q)
