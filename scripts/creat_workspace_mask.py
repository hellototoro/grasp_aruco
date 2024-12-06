from PIL import Image
import numpy as np


def create_workspace_mask(width, height, rect_start, rect_end):
    # Create an empty image with all zeros (False)
    mask_array = np.zeros((height, width), dtype=np.uint8)
    # Set the specified rectangular region to 1 (True)
    mask_array[rect_start[1]:rect_end[1], rect_start[0]:rect_end[0]] = 255
    bool_image = Image.fromarray(mask_array, mode='L')
    bool_image = bool_image.convert('1')
    return bool_image


if __name__ == "__main__":
    # Example usage
    width = 1280
    height = 720
    rect_start = (width//2, height//2 + 60)
    rect_end = (width//2 + 150, height//2 + 160)

    
    bool_image = create_workspace_mask(width, height, rect_start, rect_end)
    bool_image.save('./data2/workspace_mask.png')

