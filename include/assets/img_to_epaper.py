from PIL import Image
import os




# Iterate over each folder and process the images
def get_image_paths():
    # Define the base path and the folders
    base_path = '/home/jake/Arduino/AccelStepperDimitri/include/assets'
    folders = [f'num_{i}' for i in range(10)]
    image_paths = []
    for folder in folders:
        folder_path = os.path.join(base_path, folder, 'down')
        image_path = os.path.join(folder_path, "img__.png")
        image_paths.append(image_path)

    return image_paths



# Load and resize the image
def resize_image(image_path, height):
    image = Image.open(image_path)
    width = int(image.width * height / image.height)
    # Resize the image with the new width and height
    image = image.resize((width, height), Image.ANTIALIAS).rotate(180)

    # Convert image to RGBA to handle transparency
    image = image.convert('RGBA')

    # Create a new image for binary data (black and whit
    binary_image = Image.new('1', image.size)

    # Define the alpha threshold
    alpha_threshold = 10

    # Iterate over each pixel to determine if it should be black or white
    for y in range(image.height):
        for x in range(image.width):
            r, g, b, a = image.getpixel((x, y))
            if a > alpha_threshold:  # Pixel is considered "visible"
                binary_image.putpixel((x, y), 0)  # Black pixel
            else:
                binary_image.putpixel((x, y), 1)  # White pixel

    return binary_image

def convert_image_pair_to_c_array(image_path1, image_path2, height):
    binary_image2 = resize_image(image_path1, height)
    binary_image1 = resize_image(image_path2, height)

    # Combine the two images into one
    combined_image = Image.new('1', (binary_image1.width + binary_image2.width, height), 1)  # White pixel
    combined_image.paste(binary_image1, (0, 0))
    combined_image.paste(binary_image2, (binary_image1.width, 0))
    binary_image = combined_image



        # Check if binary image is smaller than 200x200
    if binary_image.width < 200 or binary_image.height < 200:
        # Create a new image with size 200x200 and fill it with white pixels
        new_image = Image.new('1', (200, 200), 1)  # White pixel

        # Calculate the position to center the binary image
        x_offset = (200 - binary_image.width) // 2
        y_offset = (200 - binary_image.height) // 2

        # Paste the binary image onto the new image at the calculated position
        new_image.paste(binary_image, (x_offset, y_offset))

        # Use the new image for further processing
        binary_image = new_image

    # Convert binary image to a C array
    binary_data = binary_image.tobytes()
    hex_data = []
    for byte in binary_data:
        hex_data.append(f'0X{byte:02X}')

    # Format the hex data as a C array
    hex_data_lines = [','.join(hex_data[i:i+16]) for i in range(0, len(hex_data), 16)]

    # Add commas to the end of each line except the last one
    for i in range(len(hex_data_lines) - 1):
        hex_data_lines[i] += ','

    formatted_hex_data = '\n'.join(hex_data_lines)
    return formatted_hex_data

def convert_image_to_c_array(image_path, height):
    binary_image = resize_image(image_path, height)

    # Check if binary image is smaller than 200x200
    if binary_image.width < 200 or binary_image.height < 200:
        # Create a new image with size 200x200 and fill it with white pixels
        new_image = Image.new('1', (200, 200), 1)  # White pixel

        # Calculate the position to center the binary image
        x_offset = (200 - binary_image.width) // 2
        y_offset = (200 - binary_image.height) // 2

        # Paste the binary image onto the new image at the calculated position
        new_image.paste(binary_image, (x_offset, y_offset))

        # Use the new image for further processing
        binary_image = new_image

    # Convert binary image to a C array
    binary_data = binary_image.tobytes()
    hex_data = []
    for byte in binary_data:
        hex_data.append(f'0X{byte:02X}')

    # Format the hex data as a C array
    hex_data_lines = [','.join(hex_data[i:i+16]) for i in range(0, len(hex_data), 16)]

    # Add commas to the end of each line except the last one
    for i in range(len(hex_data_lines) - 1):
        hex_data_lines[i] += ','

    formatted_hex_data = '\n'.join(hex_data_lines)
    return formatted_hex_data


def compose_text_file(formatted_hex_data:list):
    c_array_str = f"""
    #include "imagedata.h"
    #include <avr/pgmspace.h>
    """
    i = 0
    for data in formatted_hex_data:
        c_array_str += f"""

        const unsigned char IMAGE_DATA_{i}[] PROGMEM = {{
        {data}
        }};
        """
        i+=1
    return c_array_str

def main():
    # Export the data to a text file
    #image_path = '/home/jake/Arduino/AccelStepperDimitri/inlcude/assets/num_7/down/img__.png'
    image_paths = get_image_paths()

    # Convert the images to a string list
    height = 150
    formatted_hex_data = []
    for image_path in image_paths:
        formatted_hex_data.append(convert_image_to_c_array(image_path, height))

    for i in range(3):
        image_path1 = f'/home/jake/Arduino/AccelStepperDimitri/include/assets/num_1/down/img__.png'
        image_path2 = f'/home/jake/Arduino/AccelStepperDimitri/include/assets/num_{i}/up/img__.png'
        formatted_hex_data.append(convert_image_pair_to_c_array(image_path1, image_path2, height))



    text_str = compose_text_file(formatted_hex_data)

    output_file_path = '/home/jake/Arduino/AccelStepperDimitri/include/epd1in54_V2/epd1in54_V2/imagedata.cpp'
    with open(output_file_path, 'w') as file:
        file.write(text_str)
    print(f"C array data has been written to {output_file_path}")


if __name__ == "__main__":
    main()