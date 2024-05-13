from PIL import Image
import os

# Function to split image into left and right halves
def split_image(image_path, output_dir):
    # Open the image
    image = Image.open(image_path)

    # Get the width and height of the image
    width, height = image.size

    # Calculate the width for splitting
    half_width = width // 2

    # Split the image into left and right halves
    left_half = image.crop((0, 0, half_width, height))
    right_half = image.crop((half_width, 0, width, height))

    # Create output directories if they don't exist
    left_output_dir = os.path.join(output_dir, 'depth_left_halfs')
    right_output_dir = os.path.join(output_dir, 'depth_right_halfs')
    os.makedirs(left_output_dir, exist_ok=True)
    os.makedirs(right_output_dir, exist_ok=True)

    # Save left and right halves
    image_name = os.path.basename(image_path)
    left_half.save(os.path.join(left_output_dir, image_name))
    right_half.save(os.path.join(right_output_dir, image_name))

# Path to the folder containing images
images_folder = 'rgb_dataset/depth'
dir_contents = os.listdir(images_folder)
input_files = [item for item in dir_contents if item.endswith(".png")]

# Output folder
output_folder = 'rgb_dataset'

# Iterate over each image in the images folder
for image_file in input_files:
    # Get the full path to the image
    image_path = os.path.join(images_folder, image_file)
    
    # Split the image and save left and right halves
    split_image(image_path, output_folder)

print("Images split successfully!")