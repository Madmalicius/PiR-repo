# PiR-repo
Repository for Project in Robotics 

##Run simulation environment
First ensure you have a working ROS and PX4 instalation by following the guide in 'Project in Robotics - appendix D' (PDF on blackboard)
Secound get the Expert-in-Teams(EIT) Playground world by following the guide in 'Project in Robotics - appendix G'

When your PX4 playground works with the EIT playground your are ready to run the simulation by doing the following: 

###Step 1
Replace sdu_drone_mono_cam.sdf in eit_ws/src/eit_playground/models/sdu_drone_mono_cam with modified version (GIT ROOT folder) to get the correct camera angle.

###Step 2
Download the gazebo world file [here](https://drive.google.com/file/d/1b8MNieB_JLFnwehBVa12ll2kzuR6cK6_/view?usp=sharing)

###Step 3
Run a terminal with the following command to start the PX4 environment
```cmd
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

###Step 4
Launch the gazebo simulation
```cmd
roslaunch eit_playground posix.launch vehicle:=sdu_drone_mono_cam env:=hca_airport_fence_complex
```

###Step 5
Start the simulation by running our control program
```cmd
beit-ws
cd PiR-repo/catkin_ws
source devel/setup.bash
rosrun hole_detector offb_control.py
```

## Needed python packages
To install the needed python packages run the following command
```cmd
pip install os math csv numpy matplotlib shapely scipy pandas opencv-python imutils
```


## Run Google Maps Interface
To run the Google Maps interface, a local server needs to be run. One quick way to do this is to use npm.

### Step 1
Download and install node.js
A download can be found [here](https://nodejs.org/en/download/)

### Step 2
open CMD in the folder containing the `index.html` file and run the following commands:
```cmd
npm install http-server -g
http-server
```

### Step 4
Access the server from your browser by going to `localhost:8080`
If the address does not work, a list of available addresses can be found in the cmd window after the http server is started.
