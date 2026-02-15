#!/usr/bin/env python3
"""
Convert old camera data format to BundleSDF format.

Old format:
  - image_0.jpg, image_1.jpg, ... (RGB as JPG)
  - depth_0.png, depth_1.png, ... (depth as PNG)

New BundleSDF format:
  - rgb/0000.png, rgb/0001.png, ... (RGB as PNG)
  - depth/0000.png, depth/0001.png, ... (depth as uint16 PNG in mm)
  - cam_K.txt (3x3 intrinsic matrix)
"""

import os
import sys
import cv2
import numpy as np
from pathlib import Path
import re


def natural_sort_key(s):
    """Sort filenames numerically (e.g., image_2.jpg before image_10.jpg)"""
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split('([0-9]+)', str(s))]


def convert_data(input_dir, output_dir=None):
    """
    Convert camera data from old format to BundleSDF format.

    Args:
        input_dir: Directory containing old format data (image_*.jpg, depth_*.png)
        output_dir: Output directory (defaults to input_dir + '_bundlesdf')
    """
    input_path = Path(input_dir)

    if not input_path.exists():
        print(f"Error: Input directory '{input_dir}' does not exist")
        return

    # Set output directory
    if output_dir is None:
        output_path = input_path.parent / (input_path.name + '_bundlesdf')
    else:
        output_path = Path(output_dir)

    # Create output directories
    rgb_dir = output_path / 'rgb'
    depth_dir = output_path / 'depth'
    rgb_dir.mkdir(parents=True, exist_ok=True)
    depth_dir.mkdir(parents=True, exist_ok=True)

    print(f"Converting data from: {input_path}")
    print(f"Output directory: {output_path}")
    print(f"Created directories: rgb/ and depth/")

    # Find all image and depth files
    image_files = sorted(input_path.glob('image_*.jpg'), key=natural_sort_key)
    depth_files = sorted(input_path.glob('depth_*.png'), key=natural_sort_key)

    if len(image_files) == 0:
        print("Warning: No image_*.jpg files found")
    if len(depth_files) == 0:
        print("Warning: No depth_*.png files found")

    print(f"\nFound {len(image_files)} RGB images and {len(depth_files)} depth images")

    # Convert RGB images
    print("\nConverting RGB images...")
    for idx, img_file in enumerate(image_files):
        # Read JPG image (BGR format)
        img = cv2.imread(str(img_file))
        if img is None:
            print(f"  Warning: Could not read {img_file.name}")
            continue

        # Save as PNG with zero-padded filename (keep BGR format)
        output_file = rgb_dir / f'{idx:04d}.png'
        cv2.imwrite(str(output_file), img)

        if idx % 50 == 0 or idx == len(image_files) - 1:
            print(f"  Processed {idx + 1}/{len(image_files)} images")

    # Convert depth images
    print("\nConverting depth images...")
    for idx, depth_file in enumerate(depth_files):
        # Read depth image
        depth = cv2.imread(str(depth_file), cv2.IMREAD_UNCHANGED)
        if depth is None:
            print(f"  Warning: Could not read {depth_file.name}")
            continue

        # Verify it's uint16
        if depth.dtype != np.uint16:
            print(f"  Warning: {depth_file.name} is {depth.dtype}, expected uint16")
            # Try to convert if it's float
            if depth.dtype == np.float32 or depth.dtype == np.float64:
                depth = (depth * 1000.0).astype(np.uint16)

        # Save with zero-padded filename
        output_file = depth_dir / f'{idx:04d}.png'
        cv2.imwrite(str(output_file), depth)

        if idx % 50 == 0 or idx == len(depth_files) - 1:
            print(f"  Processed {idx + 1}/{len(depth_files)} depth images")

    print(f"\n✓ Conversion complete!")
    print(f"✓ RGB images saved to: {rgb_dir}")
    print(f"✓ Depth images saved to: {depth_dir}")
    print(f"\nNote: You still need to create cam_K.txt with your camera's 3x3 intrinsic matrix")
    print(f"      Save it to: {output_path / 'cam_K.txt'}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python convert_to_bundlesdf.py <input_directory> [output_directory]")
        print("\nExample:")
        print("  python convert_to_bundlesdf.py /path/to/old/data")
        print("  python convert_to_bundlesdf.py /path/to/old/data /path/to/new/data")
        sys.exit(1)

    input_dir = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None

    convert_data(input_dir, output_dir)
