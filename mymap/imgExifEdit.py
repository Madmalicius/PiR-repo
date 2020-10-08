from GPSPhoto import gpsphoto
from PIL import Image
import os

def ImgGPSCombiner(imgNr,lat,lon):
    imgPath = "Err"+str(imgNr)+".jpg"
    imgPathTagged = "Err"+str(imgNr)+"Tag.jpg"

    photo = gpsphoto.GPSPhoto(imgPath)

    info = gpsphoto.GPSInfo((lat, lon))

    # Modify GPS Data
    photo.modGPSData(info, imgPathTagged)
    os.remove(imgPath)

ImgGPSCombiner(1,55.474201, 10.323968)