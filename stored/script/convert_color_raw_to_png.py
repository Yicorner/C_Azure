import numpy as np
from PIL import Image

# Define image dimensions
width = 1920
height = 1080
image_id = 1
filename = 'color_image_' + image_id.String() ;
# Read the raw RGBA data
with open('captures' + filename + '.raw', 'rb') as f:
    raw_data = f.read()

# Calculate the expected size
expected_size = width * height * 4  # 4 bytes per pixel for RGBA

# Check if the file size matches the expected size
if len(raw_data) != expected_size:
    print(f"Warning: Expected data size {expected_size}, but got {len(raw_data)}.")
    # You can choose to handle this discrepancy as needed

# Convert the raw data to a NumPy array
image_array = np.frombuffer(raw_data, dtype=np.uint8)

# Reshape the array to match the image dimensions and channels
try:
    image_array = image_array.reshape((height, width, 4))
except ValueError as e:
    print(f"Error reshaping array: {e}")
    # Handle the error as needed
    exit(1)

# Optionally, flip the image vertically if it's upside-down
# image_array = np.flipud(image_array)

# Create a PIL Image from the NumPy array
image = Image.fromarray(image_array, 'RGBA')

# Save the image as a PNG file
image.save('captures/' + filename + '.png')

print("Image saved successfully as 'color_image.png'.")
