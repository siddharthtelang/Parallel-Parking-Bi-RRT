import numpy as np
import cv2
from Utils.MathUtils import *
from Obstacle import *


def updateMap(space_map, node, color):
    
    if node.getParent() is not None:
        parent_state = node.getParent().getState()
        current_state = node.getState()
        parent_state_transformed = transformPoint(parent_state, space_map)
        current_state_transformed = transformPoint(current_state, space_map)

        space_map = cv2.line(space_map, (parent_state_transformed[1], parent_state_transformed[0]), (current_state_transformed[1], current_state_transformed[0]), color, 1)
    else:
        current_state = node.getState()
        current_state_transformed = transformPoint(current_state, space_map)
        space_map[current_state_transformed[0], current_state_transformed[1], :] = color
    return space_map

def updateMapViz(space_map, state, color):
    X, Y, _ = space_map.shape
    transformed_y = state[0]
    transformed_x = X - state[1] -1
    space_map[transformed_x, transformed_y, :] = color 
    return space_map

def addObstacles2Map(space_map):

    #circle
    for i in range(circle_offset_x - circle_radius, circle_offset_x + circle_radius):
        for j in range(circle_offset_y - circle_radius, circle_offset_y + circle_radius):
            if (i - circle_offset_x) **2 + (j - circle_offset_y)**2 <= circle_radius**2:
                updateMapViz(space_map, [i, j], [255, 0, 0])

    for i in range(circle_offset_x - circle_radius, circle_offset_x + circle_radius):
        for j in range(circle_offset_y - circle_radius, circle_offset_y + circle_radius):
            if (i - circle_offset_x) **2 + (j - circle_offset_y)**2 <= (circle_radius - total_clearance)**2:
                updateMapViz(space_map, [i, j], [255, 255, 0])

    #ellipse
    for i in range(ellipse_offset_x - ellipse_radius_x, ellipse_offset_x + ellipse_radius_x):
        for j in range(ellipse_offset_y - ellipse_radius_y, ellipse_offset_y + ellipse_radius_y):
            if ((i - ellipse_offset_x)/ellipse_radius_x) **2 + ((j - ellipse_offset_y)/ellipse_radius_y)**2 <= 1:
                updateMapViz(space_map, [i, j], [255, 0, 0])

    for i in range(ellipse_offset_x - ellipse_radius_x, ellipse_offset_x + ellipse_radius_x):
        for j in range(ellipse_offset_y - ellipse_radius_y, ellipse_offset_y + ellipse_radius_y):
            if ((i - ellipse_offset_x)/(ellipse_radius_x - total_clearance)) **2 + ((j - ellipse_offset_y)/(ellipse_radius_y - total_clearance))**2 <= 1:
                updateMapViz(space_map, [i, j], [255, 255, 0])


    #C shape
    for i in range(c_min_x, c_max_x):
        for j in range(c_min_y, c_max_y):
            if (i <= c_corner4_x):
                updateMapViz(space_map, [i, j], [255, 0, 0])
            if (j >= c_corner5_y) or (j <= c_corner3_y):
                updateMapViz(space_map, [i, j], [255, 0, 0])

    for i in range(c_offset_x, c_offset_x + c_length_x):
        for j in range(c_offset_y, c_offset_y + c_length_y):
            if (i <= (c_offset_x + c_width)):
                updateMapViz(space_map, [i, j], [255, 255, 0])
            if (j >= c_offset_y + c_height + c_width) or (j <= c_offset_y + c_width):
                updateMapViz(space_map, [i, j], [255, 255, 0])



    # # rectangle
    for i in range(rect_x_min, rect_x_max):
        for j in range(rect_y_min, rect_y_max):
            # if (j >= (np.tan(rect_angle) * (i - rect_corner1_x)  + rect_corner1_y)) and (j <= (np.tan(rect_angle) * (i -rect_corner4_x)  + rect_corner4_y)):
            #     if (j >= (-np.tan(np.pi/2 - rect_angle) * (i -rect_corner4_x)  + rect_corner4_y)) and (j <= (-np.tan(np.pi/2 - rect_angle) * (i -rect_corner3_x)  + rect_corner3_y)):
            #         updateMapViz(space_map, [i, j], [255, 0, 0])

            #condition for rectangle
            d1 = abs((j - 0.7002*i - 74.39) / (1 + (0.7002)**2)**(0.5))
            d2 = abs((j - 0.7002*i - 98.8) / (1 + (0.7002)**2)**(0.5))
            d3 = abs((j + 1.428*i - 176.55) / (1 + (1.428)**2)**(0.5))
            d4 = abs((j + 1.428*i - 439.44) / (1 + (1.428)**2)**(0.5))
            if (d1+d2 <= rect_width and d3+d4 <= rect_length):
                updateMapViz(space_map, [i, j], [255, 0, 0])

            # #condition for rectangle
            # d1 = abs((j - 0.7002*i - 74.39) / (1 + (0.7002)**2)**(0.5))
            # d2 = abs((j - 0.7002*i - 98.8) / (1 + (0.7002)**2)**(0.5))
            # d3 = abs((j + 1.428*i - 176.55) / (1 + (1.428)**2)**(0.5))
            # d4 = abs((j + 1.428*i - 439.44) / (1 + (1.428)**2)**(0.5))
            # if (d1+d2 <= rect_width - (2 * total_clearance) and d3+d4 <= rect_length - (2 * total_clearance)):
            #     updateMapViz(space_map, [i, j], [255, 255, 0])


    c4_x = int(rect_offset_x - (rect_width - 2 * total_clearance) * np.sin(rect_angle))
    c4_y = int(rect_offset_y + (rect_width - 2 * total_clearance) * np.cos(rect_angle))

    c2_x = int(rect_offset_x + (rect_length - 2 * total_clearance) * np.cos(rect_angle))
    c2_y = int(rect_offset_y + (rect_length - 2 * total_clearance) * np.sin(rect_angle))

    c3_x = int(c2_x - (rect_width - 2 * total_clearance) * np.sin(rect_angle))
    c3_y = int(c2_y + (rect_width - 2 * total_clearance) * np.cos(rect_angle))

    for i in range(rect_x_min, rect_x_max):
        for j in range(rect_y_min, rect_y_max):
            if (j >= (np.tan(rect_angle) * (i - rect_offset_x)  + rect_offset_y)) and (j <= (np.tan(rect_angle) * (i -c4_x)  + c4_y)):
                if (j >= (-np.tan(np.pi/2 - rect_angle) * (i -c4_x)  + c4_y)) and (j <= (-np.tan(np.pi/2 - rect_angle) * (i -c3_x)  + c3_y)):
                    updateMapViz(space_map, [i, j], [255, 255, 0])
    return space_map

    # for i in range(rect_x_min, rect_x_max):
    #     for j in range(rect_y_min, rect_y_max):
    #         if (j >= (np.tan(rect_angle) * (i - rect_offset_x)  + rect_offset_y)) and (j <= (np.tan(rect_angle) * (i -c4_x)  + c4_y)):
    #             if (j >= (-np.tan(np.pi/2 - rect_angle) * (i -c4_x)  + c4_y)) and (j <= (-np.tan(np.pi/2 - rect_angle) * (i -c3_x)  + c3_y)):
    #                 updateMapViz(space_map, [i, j], [255, 255, 0])
    
    



   
    # c4_x = int(rect_offset_x - (rect_width - total_clearance) * np.sin(rect_angle))
    # c4_y = int(rect_offset_y + (rect_width - total_clearance) * np.cos(rect_angle))

    # c2_x = int(rect_offset_x + (rect_length - total_clearance) * np.cos(rect_angle))
    # c2_y = int(rect_offset_y + (rect_length - total_clearance) * np.sin(rect_angle))

    # c3_x = int(c2_x - (rect_width - total_clearance) * np.sin(rect_angle))
    # c3_y = int(c2_y + (rect_width - total_clearance) * np.cos(rect_angle))
1
    # for i in range(rect_x_min, rect_x_max):
    #     for j in range(rect_y_min, rect_y_max):
    #         if (j >= (np.tan(rect_angle) * (i - rect_offset_x)  + rect_offset_y)) and (j <= (np.tan(rect_angle) * (i -c4_x)  + c4_y)):
    #             if (j >= (-np.tan(np.pi/2 - rect_angle) * (i -c4_x)  + c4_y)) and (j <= (-np.tan(np.pi/2 - rect_angle) * (i -c3_x)  + c3_y)):
    #                 updateMapViz(space_map, [i, j], [255, 255, 0])

    # return space_map