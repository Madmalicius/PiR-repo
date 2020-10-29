'''
Any questions ask Mikkel
'''
import os
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry.polygon import LinearRing
from scipy.spatial.distance import cdist
import math
from utm import utmconv
import pandas as pd

#FIELD OF VIEW OF CAMERA IN METERS
FIELD_OF_VIEW = 2
#OFFSET OF DRONE PATH FROM FENCE
FENCE_OFFSET = 2


def get_fence_position(filename):
    basePath = os.path.dirname(os.path.abspath(__file__))
    file_read = pd.read_csv(basePath + filename,)
    file_np = file_read.to_numpy()

    x, y = file_np.T

    path_fence = zip(x,y)

    #for row in file_read
    #    file_read[i][0]

    #FOR TESTING
    # path_fence = ([[9,7],[8,9],[8,11],[9,12],[11,12],[11,11],[12,11],[12,10],[13,9],[13,8],[12,7],[9,7]])
    #
    return path_fence

def calculate_flight_path(path_fence):
    poly_line = LinearRing(path_fence)
    poly_line_offset_left = poly_line.parallel_offset(FENCE_OFFSET, side="left", resolution=16, join_style=2)
    poly_line_offset_right = poly_line.parallel_offset(FENCE_OFFSET, side="right", resolution=16, join_style=2)

    if (poly_line_offset_left.length > poly_line_offset_right.length):
        poly_line_offset = poly_line_offset_right
    else:
        poly_line_offset = poly_line_offset_left

    flight_path_x, flight_path_y = poly_line_offset.xy

    return flight_path_x, flight_path_y

def calculate_photo_positions(path_x, path_y): 
    new_path_x = []
    new_path_y = []
    for i in range(len(path_x)-1):
        path_length = math.sqrt(((path_x[i]-path_x[i+1])**2)+((path_y[i]-path_y[i+1])**2)) 
        N = int(path_length/(FIELD_OF_VIEW))+2
        delta_path_x = np.linspace(path_x[i], path_x[i+1], N) # endpoit false for testing only
        delta_path_y = np.linspace(path_y[i], path_y[i+1], N)
        new_path_x = np.concatenate((new_path_x, delta_path_x))
        new_path_y = np.concatenate((new_path_y, delta_path_y))

    return new_path_x, new_path_y

def calculate_photo_orientation(path_x, path_y): 
    angles = []
    for i in range(len(path_x)):

        #get angle at each point (line seqments)
        if(i == len(path_x)-1):
            angle = math.atan2(path_y[i]-path_y[i-1], path_x[i]-path_x[i-1]) * 180 / 3.1415
        else:
            if(path_x[i] == path_x[i+1] and path_y[i] == path_y[i+1]):
                angle = math.atan2(path_y[i]-path_y[i-1], path_x[i]-path_x[i-1]) * 180 / 3.1415 
            else:    
                angle = math.atan2(path_y[i+1]-path_y[i], path_x[i+1]-path_x[i]) * 180 / 3.1415

        #map range from -180;180 to 0;360 deg where 0 deg is along x axis
        angle = (angle + 360)%360

        #calculating caputre orientation perendicular to the fence/drone path:
        angle -= 90
        if(angle < 0):
            angle = 360-abs(angle)

        angle = round(angle,2)
        angles.append(angle)
  
        #print(str(i) + ": " + str(path_x[i]) + " at: " + str(angles[i]))
    return angles

def save_csv(fence_x, fence_y, ori):

    array = np.column_stack((fence_x, fence_y))
    array = np.column_stack((array, ori))
    basePath = os.path.dirname(os.path.abspath(__file__))
    np.savetxt(basePath + "/output_coordiantas.csv", array, delimiter=",", fmt='%.16f')

    return


def fix_start_point(path_fence, flight_path_x, flight_path_y):

    node = path_fence[0]
    nodes = zip(flight_path_x,flight_path_y)

    shift = cdist([node], nodes).argmin()

    fixed_x = np.roll(flight_path_x,-shift)
    fixed_y = np.roll(flight_path_y,-shift)

    fixed_x = np.append(fixed_x, fixed_x[0])
    fixed_y = np.append(fixed_y, fixed_y[0])

    return fixed_x, fixed_y

if __name__ == "__main__":
    path_fence = get_fence_position("/hca_fence_complex_coordinates.csv")
    flight_path_x, flight_path_y = calculate_flight_path(path_fence)
    flight_path_x, flight_path_y = fix_start_point(path_fence,flight_path_x, flight_path_y)
    photo_pos_x, photo_pos_y = calculate_photo_positions(flight_path_x, flight_path_y)
    photo_orientation = calculate_photo_orientation(photo_pos_x, photo_pos_y)

    fence_x,fence_y = zip(*path_fence)

    save_csv(list(photo_pos_x), list(photo_pos_y), list(photo_orientation))

    plt.plot(fence_x,fence_y,'o-',color='black')
    plt.plot(photo_pos_x,photo_pos_y,'o-',color='red')
    for i, txt in enumerate(photo_orientation):
        plt.annotate(txt, (photo_pos_x[i], photo_pos_y[i]))
    plt.axis('equal')
    plt.show()