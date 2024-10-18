import numpy as np
from PIL import Image

# Define depth image dimensions
depth_width = 640
depth_height = 576
image_id = 1
# Read the raw depth data
filename = 'depth_image_' + image_id.String() 
with open('captures' + filename + '.raw', 'rb') as f:
    depth_data = f.read()

# Calculate the expected size
expected_size = depth_width * depth_height * 2  # 2 bytes per pixel for uint16
if len(depth_data) != expected_size:
    print(f"Warning: Expected data size {expected_size}, but got {len(depth_data)}.")
    # You can choose to handle this discrepancy as needed

# Convert to NumPy array
depth_array = np.frombuffer(depth_data, dtype=np.uint16)
try:
    depth_array = depth_array.reshape((depth_height, depth_width))
except ValueError as e:
    print(f"Error reshaping array: {e}")
    exit(1)

# Convert depth values to meters if needed
depth_in_meters = depth_array

# Normalize depth values to the 0-255 range
# First, define the valid depth range (in meters)
min_depth = np.min(depth_in_meters[depth_in_meters > 0])  # Exclude zero values
max_depth = np.max(depth_in_meters)

# Handle cases where all depth values are zero
if np.isnan(min_depth) or np.isnan(max_depth):
    print("Depth data contains only zero values.")
    exit(1)

# Normalize depth values
# normalized_depth = (depth_in_meters - min_depth) / (max_depth - min_depth)  # Normalize to 0.0 - 1.0
normalized_depth = depth_in_meters / 1000
normalized_depth[normalized_depth > 1] = 1

normalized_depth = (normalized_depth * 255).astype(np.uint8)  # Scale to 0 - 255 and convert to uint8

# Create an RGB image where each channel is the normalized depth
rgb_image = np.stack((normalized_depth,)*3, axis=-1)  # Shape: (height, width, 3)

# Convert to PIL Image and save
image = Image.fromarray(rgb_image, 'RGB')
image.save('captures' + filename + '.png')

print("Depth image saved successfully as 'depth_image.png'.")
