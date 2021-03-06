'''
Any questions ask Mikkel
'''
import os
import math
import csv
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry.polygon import LinearRing
from scipy.spatial.distance import cdist
from utm import utmconv
import pandas as pd


class Pathplanclass():
    def __init__(self):
        
        #plotting data
        self.plotData =  True

        self.plan_file = "/outside_fence_wed.plan"
        self.csv_filename = '/fence_2m.csv'

        self.uc = utmconv()
        #FIELD OF VIEW OF CAMERA IN METERS
        self.FIELD_OF_VIEW = 8
        #OFFSET OF DRONE PATH FROM FENCE
        self.FENCE_OFFSET = 2

        self.basePath = os.path.dirname(os.path.abspath(__file__))

        self.dirrec = ""

        ##### RUNNING THE SCIPTS #####
        self.run_main(self.plan_file)



    def get_fence_position(self,filename):
        file_read = pd.read_json(self.basePath + filename, orient='columns')
        fileLength = len(file_read['mission']['items'])

        path_fence = []
        uc = utmconv() 

        for i in range(1,fileLength): # Starts from item[1] because item[0] would be null
                lon = file_read['mission']['items'][i]['params'][4]
                lat = file_read['mission']['items'][i]['params'][5]
                alt = file_read['mission']['items'][i]['params'][6]

                if(alt == 0):
                    break

                #print(lon, lat)
                hemisphere, zone, letter, e, n = uc.geodetic_to_utm (lat,lon)

                

                path_fence.append([e,n])

        #FOR TESTING
        # path_fence = ([[9,7],[8,9],[8,11],[9,12],[11,12],[11,11],[12,11],[12,10],[13,9],[13,8],[12,7],[9,7]])
        #
        return hemisphere, zone, letter, path_fence

    def calculate_flight_path(self,path_fence):
        poly_line = LinearRing(path_fence)
        poly_line_offset_left = poly_line.parallel_offset(self.FENCE_OFFSET, side="left", resolution=16, join_style=2)
        poly_line_offset_right = poly_line.parallel_offset(self.FENCE_OFFSET, side="right", resolution=16, join_style=2)

        if (poly_line_offset_left.length > poly_line_offset_right.length):
            poly_line_offset = poly_line_offset_right
            self.dirrec = "CCW"
            print("CCW")
        else:
            poly_line_offset = poly_line_offset_left
            self.dirrec = "CW"
            print("CW")

        flight_path_x, flight_path_y = poly_line_offset.xy

        return flight_path_x, flight_path_y

    def calculate_photo_positions(self,path_x, path_y): 
        new_path_x = []
        new_path_y = []
        for i in range(len(path_x)-1):
            path_length = math.sqrt(((path_x[i]-path_x[i+1])**2)+((path_y[i]-path_y[i+1])**2)) 
            N = math.ceil(path_length/(self.FIELD_OF_VIEW))+1
            delta_path_x = np.linspace(path_x[i], path_x[i+1], N) # endpoit false for testing only
            delta_path_y = np.linspace(path_y[i], path_y[i+1], N)
            new_path_x = np.concatenate((new_path_x, delta_path_x))
            new_path_y = np.concatenate((new_path_y, delta_path_y))

        return new_path_x, new_path_y

    def calculate_photo_orientation(self,path_x, path_y): 
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

            angle *= -1

            if self.dirrec == "CW":
                angle += 180
            

            #calculating caputre orientation perendicular to the fence/drone path:
            #if (self.dirrec == "CW"):
            #    angle +=  180

            if angle > 360:
                angle = angle%360
            
            if(angle < 0):
                angle = 360-abs(angle)

            angle = round(angle,2)
            angles.append(angle)

            
    
            #print(str(i) + ": " + str(path_x[i]) + " at: " + str(angles[i]))
        return angles

    def convert_utm_to_lon_lat(self,hemisphere,zone,east,north):

        lat = []
        lon = []

        for x in range(len(north)):

            latx,lony = self.uc.utm_to_geodetic(hemisphere,zone,east[x],north[x])
            
            lat.append(latx)
            lon.append(lony)

        return lat,lon

    def collect_xy_oriantation(self,lat_x,lon_y,ori):

        N= 3 # params
        M = len(ori) #n
        data = np.zeros([M,N])#,dtype='float64') #creating a set of data for each point

        for x in range(M):
            data[x][0] = lat_x[x] # This should be lon or lat?
            data[x][1] = lon_y[x] # This should be lon or lat?
            data[x][2] = ori[x]

        return data

    def write_csv(self,data,fileName):
        with open( self.basePath + fileName,'w') as file:
            writeData = csv.writer(file)#,quoting=csv.QUOTE_ALL)

            for x in range(len(data)):
                writeData.writerow([data[x][1],data[x][0],data[x][2]])

    def fix_start_point(self, path_fence, flight_path_x, flight_path_y):

        node = path_fence[0]
        #print(path_fence[0])
        #print(flight_path_x)
        nodes = zip(flight_path_x,flight_path_y)

        shift = cdist([node], nodes).argmin()

        fixed_x = np.roll(flight_path_x,-shift)
        fixed_y = np.roll(flight_path_y,-shift)

        return fixed_x, fixed_y

    def fix_dir(self, flight_path_x, flight_path_y):
        if self.dirrec == "CCW":
            x = np.flipud(flight_path_x) 
            y = np.flipud(flight_path_y)
        else:
            x = flight_path_x
            y = flight_path_y
        return x, y


    def run_main(self,plan_file):
        hemisphere, zone, letter, path_fence = self.get_fence_position(plan_file) 
        #print(path_fence[0])
        flight_path_x, flight_path_y = self.calculate_flight_path(path_fence)
        flight_path_x, flight_path_y = self.fix_dir(flight_path_x, flight_path_y)
        photo_pos_x, photo_pos_y = self.calculate_photo_positions(flight_path_x, flight_path_y)
        photo_pos_x, photo_pos_y = self.fix_start_point(path_fence, photo_pos_x, photo_pos_y)
        
        photo_orientation = self.calculate_photo_orientation(photo_pos_x, photo_pos_y) # roation around z-axis

        lat,lon = self.convert_utm_to_lon_lat(hemisphere,zone,photo_pos_x,photo_pos_y) 

        pos = self.collect_xy_oriantation(lat,lon,photo_orientation)
        
        self.write_csv(pos,self.csv_filename)

        #print(len(pos))



        ## -------- Plotting data ------- ##
        if self.plotData is True:
            fence_x,fence_y = zip(*path_fence)
            plt.plot(fence_x,fence_y,'o-',color='black')
            plt.plot(photo_pos_x,photo_pos_y,'o-',color='red')
            for i, txt in enumerate(photo_orientation):
                plt.annotate(txt, (photo_pos_x[i], photo_pos_y[i]))
            plt.axis('equal')
            plt.show()




if __name__ == "__main__":
    #try:
        Pathplanclass()
    #except rospy.ROSInterruptException as e:
    #    print(e)

