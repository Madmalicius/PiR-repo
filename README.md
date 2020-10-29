# PiR-repo
Repository for Project in Robotics 

## Needed python packages
To install the needed python packages run the following command
```
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
