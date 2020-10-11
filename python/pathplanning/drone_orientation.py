'''
Any questions ask Mikkel
'''
import os
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry.polygon import LinearRing
import math
from utm import utmconv
import pandas as pd

#FIELD OF VIEW OF CAMERA IN METERS
FIELD_OF_VIEW = 30
#OFFSET OF DRONE PATH FROM FENCE
FENCE_OFFSET = 2


def get_fence_position(filename):
    basePath = os.path.dirname(os.path.abspath(__file__))
    file_read = pd.read_json(basePath + filename, orient='columns')
    fileLength = len(file_read['mission']['items'])

    path_fence = []
    uc = utmconv() 

    for i in range(2,fileLength): # Starts from item[1] because item[0] would be null
            lon = file_read['mission']['items'][i]['params'][4]
            lat = file_read['mission']['items'][i]['params'][5]
            alt = file_read['mission']['items'][i]['params'][6]

            if(alt == 0):
                break

            hemisphere, zone, letter, e, n = uc.geodetic_to_utm (lat,lon)

            path_fence.append([e,n])

    #FOR TESTING
    # path_fence = ([[9,7],[8,9],[8,11],[9,12],[11,12],[11,11],[12,11],[12,10],[13,9],[13,8],[12,7],[9,7]])
    #
    return hemisphere, zone, letter, path_fence

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

if __name__ == "__main__":
    hemisphere, zone, letter, path_fence = get_fence_position("/fence_location.plan")
    flight_path_x, flight_path_y = calculate_flight_path(path_fence)
    photo_pos_x, photo_pos_y = calculate_photo_positions(flight_path_x, flight_path_y)
    photo_orientation = calculate_photo_orientation(photo_pos_x, photo_pos_y)

    fence_x,fence_y = zip(*path_fence)
    plt.plot(fence_x,fence_y,'o-',color='black')
    plt.plot(photo_pos_x,photo_pos_y,'o-',color='red')
    for i, txt in enumerate(photo_orientation):
        plt.annotate(txt, (photo_pos_x[i], photo_pos_y[i]))
    plt.axis('equal')
    plt.show()