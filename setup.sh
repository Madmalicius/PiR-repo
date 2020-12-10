#!/bin/bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

sudo apt update

sudo apt install ros-melodic-desktop-full

apt search ros-melodic

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y

sudo apt install python-rosdep -y
sudo rosdep init
rosdep update

sudo apt-get install astyle build-essential ccache clang clang-tidy cmake cppcheck doxygen file g++ gcc gdb git lcov make ninja-build python3 python3-dev python3-pip python3-setuptools python3-wheel rsync shellcheck unzip xsltproc zip libeigen3-dev libopencv-dev libroscpp-dev protobuf-compiler python-pip python3-pip ninja-build gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev xvfb catkin python-catkin-tools libgeographic-dev geographiclib-tools ros-melodic-mavros ros-melodic-mavros-extras -y
pip install --user argparse cerberus empy jinja2 numpy packaging pandas psutil pygments pyros-genmsg pyserial pyulog pyyaml setuptools six toml wheel rosdep uncertainties
pip3 install --user --upgrade empy jinja2 numpy packaging pyros-genmsg toml pyyaml pymavlink

curl https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | sudo bash

if [ -d ~/Firmware ]; then
    echo "PX4 Firmware is installed"
else
    pushd ~
        git clone https://github.com/PX4/Firmware.git
    popd
    pushd ~/Firmware
        git submodule update --init --recursive
    popd
    pushd ~/Firmware
        DONT_RUN=1 make px4_sitl_default gazebo
    popd
fi

sudo apt install ros-melodic-mavros ros-melodic-mavros-extras -y

pushd ./catkin_ws
    chmod +x ./src/hole_detector/src/offb_control.py
    catkin build || catkin_make
popd

pwd

pushd ~
    curl https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | sudo bash install_geographiclib_datasets.sh 
popd

if [ -d ~/eit_ws/src ]; then
    echo "eit_ws already exists"
else
    source /opt/ros/melodic/setup.bash
    mkdir -p ~/eit_ws/src
    pwd
    pushd ~/eit_ws/src/
        catkin_init_workspace
    popd
    pushd ~/eit_ws/
        catkin build
        sudo apt-get install unzip
    popd
    pushd ~/eit_ws/src
        wget https://nextcloud.sdu.dk/index.php/s/oycQ3pgQPXASHEY/download
        unzip download
        rm download
    popd
fi

cp -rf ./sdu_drone_mono_cam.sdf ~/eit_ws/src/eit_playground/models/sdu_drone_mono_cam

if [ -f ~/eit_ws/src/eit_playground/worlds/hca_airport_fence_complex.world ]; then
    echo "World file already exists"
else
    pushd ~/eit_ws/src/eit_playground/worlds/
        wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=1b8MNieB_JLFnwehBVa12ll2kzuR6cK6_' -O hca_airport_fence_complex.world
    popd
fi

if [ -d ~/eit_ws/src/eit_playground/models/fence_double_post ]; then
    echo "Fence models already exist"
else
    pushd ~/eit_ws/src/eit_playground/models/
        wget --no-check-certificate 'https://doc-9s-9c-drive-data-export.googleusercontent.com/download/hjgi1l3jtk6d2cks6tn71lf27hineubs/d22sebv8vkn6ofncgu14khau5hilu9js/1607589000000/46da7b9d-56fb-4e2d-be2b-4290e03a89f9/101012487977639686937/ADt3v-P4pqCOKHOi3N3IaSzjS2cQoHM3hGyDoIOzeYTVpn0CLphImcNLXwkSL8UjH_Q1UXZQ3WNnfx1wNwouKMjPErzV6b1Vctqrjkfq36w7ugWzESjNOqlSy3XTJAeEojDtDnO450D0kIDYV9RrkKScwxEWuuJtQukdk7ng4To7gXy19rkxvOLb9yRsmrLBhwsVuQkfwfNS1rh8In0e-3Rn8PdGIf1TaC19OrvBU_UFNMzEXBAtUsYjK45MmkXvb8G95j3WHU4PuJOFZ7SzFrxErkFlqf_3OuIZdRRH8qw7Z6CmtkMPhZln2cFkkufu7SD_9AyT36xs?authuser=0&nonce=53n3j2k1gojig&user=101012487977639686937&hash=7l7kfjo3up4mqth4o8h4nqiilvqa1i02' -O fence_double_post.zip
        wget --no-check-certificate 'https://doc-bo-2o-drive-data-export.googleusercontent.com/download/hjgi1l3jtk6d2cks6tn71lf27hineubs/vnos1q66u714kldd6bbqjngoocqmpmfl/1607589000000/ccc84262-e22b-4995-a6b2-07b9d067af7c/101012487977639686937/ADt3v-McILUyKwgMR3_hnh9xHr0KMlB_eQZpLR-59H4VOPwFFqy-KxlyI0tIVHefjqPSOS3vThrGGZ1BGZpt4oClw9g1OJJeEyUF6WXg-wD5ONcBkHXDMJNU4ICsoMW5eckGJ9_NEKhbPM0qU66htUewAsP8Q4Oq8qkOSGKQwQ441WAOMN6DLPC4rVa3pUScD69GmqkYuhNCnfs8tO-xqMVYyiobA5r1aUt3Mh9zh82xLN0veUwkfOBoMxAUBVV3MJmTrxcKsbn1prakc9TtYPKMCtEyOZP7J6fM0mOMe1V6GoVaodQzVJ6QStajMzQIkHDFoHzW4H9u?authuser=0&nonce=kj3o953unk5bk&user=101012487977639686937&hash=6tu87h1949fsdjmcpgo462v3pi3fbarh' -O fence_hole1.zip
        wget --no-check-certificate 'https://doc-a8-48-drive-data-export.googleusercontent.com/download/hjgi1l3jtk6d2cks6tn71lf27hineubs/g8fmvku98ab9prf4mk7thubvi4pp4hqa/1607589000000/3851d01f-7bba-480c-b1cd-8e821022034e/101012487977639686937/ADt3v-O8zbLyYcbh_UwGN7NMj9kHf0M2REqL5jkyUbZhsXvHZSaGPHszUG1dkBjDa-3gcRFbry5UAYRR0b8wpTPRYUruCseYoqOHoiHkYz7kqQVWTNgQ2icCcUoNyKr6WKE3y-LKFP23n5mN9zsAHIj0ZLy_S8syD5mfF-1zk94QAtl3cbzPXqcwqrUHCv2UpTa4-BRHcq9gzDKUJTrFwwr1Z25zJhH9s-s-r6lzN1ZjvuOjtVQNAcmQfmMdo0ieQNkyizgBzEHbAEaUoBZmwarAPBsa3ajjH0zwtJ7vX3i80R9U8sSJUkL2M4Dl8Ib4ECRRukdCJfA3?authuser=0&nonce=hn3rdgm5f4ans&user=101012487977639686937&hash=p1pjt9ri4tt0nnpcu51mqc8vf3otaqmq' -O fence_single.zip
        
        unzip fence_double_post
        unzip fence_hole1
        unzip fence_single

        rm fence_double_post.zip fence_hole1.zip fence_single.zip
    popd
fi

ln -s /home/$USER/eit_ws/src/eit_playground/init.d-posix/* /home/$USER/Firmware/ROMFS/px4fmu_common/init.d-posix/airframes
ln -s /home/$USER/eit_ws/src/eit_playground/mixers/* /home/$USER/Firmware/ROMFS/px4fmu_common/mixers/

echo $USER

bash start.sh