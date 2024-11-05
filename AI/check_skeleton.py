import numpy as np


def get_grid_position(bounding_box, x, y):

    x1, y1, x2, y2 = bounding_box
    
    grid_width = (x2 - x1) / 3
    grid_height = (y2 - y1) / 3

    grid_x = int((x - x1) // grid_width)
    grid_y = int((y - y1) // grid_height)

    return grid_x, grid_y


def incline_judgment(a, b):

    upper_x, upper_y = a
    lower_x, lower_y = b

    incline_list = [(0, 0, 1, 2), (0, 0, 2, 2), (0, 0, 2, 1),
                    (0, 2, 1, 0), (0, 2, 2, 0), (0, 2, 2, 1),
                    (2, 0, 0, 1), (2, 0, 0, 2), (2, 0, 1, 2),
                    (2, 2, 0, 0), (2, 2, 0, 1), (2, 2, 1, 1)]
    
    if (upper_x, upper_y, lower_x, lower_y) in incline_list:
        return True

    else:
        return False


