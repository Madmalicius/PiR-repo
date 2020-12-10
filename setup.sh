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
        wget --header 'Host: doc-00-bc-docs.googleusercontent.com' --user-agent 'Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:82.0) Gecko/20100101 Firefox/82.0' --header 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8' --header 'Accept-Language: en-US,en;q=0.5' --referer 'https://drive.google.com/drive/folders/17AFU4p_Sdx8LkFYFdu0QYXDO4p8DHIy8' --header 'Cookie: AUTH_08aoi9suu919kpbuterlgti9e89eemb0_nonce=keukqbemhj7gg' --header 'Upgrade-Insecure-Requests: 1' 'https://doc-00-bc-docs.googleusercontent.com/docs/securesc/8s0pc1bogt13qskbtsrb1bn0mtoe763v/ii3hhrmvl1p85vhrto4fe1t42b9dd17b/1607595975000/15606492013124874069/16707066989796886058/1b8MNieB_JLFnwehBVa12ll2kzuR6cK6_?e=download&authuser=0&nonce=keukqbemhj7gg&user=16707066989796886058&hash=198ctf9p49muua5kclqoqni47atn2438' --output-document 'hca_airport_fence_complex.world'
    popd
fi

if [ -d ~/eit_ws/src/eit_playground/models/fence_double_post ]; then
    echo "Fence models already exist"
else
    pushd ~/eit_ws/src/eit_playground/models/
        wget --header 'Host: doc-4k-ao-drive-data-export.googleusercontent.com' --user-agent 'Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:82.0) Gecko/20100101 Firefox/82.0' --header 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8' --header 'Accept-Language: en-US,en;q=0.5' --referer 'https://drive.google.com/drive/folders/1GXQd8hLl4PwjNRe5Nt09vRRVvILiGE0y' --header 'Cookie: AUTH_p8oalvncdji24ur7qqlm8344ada3s0b0_nonce=a1hgt2i2d7mdu' --header 'Upgrade-Insecure-Requests: 1' 'https://doc-4k-ao-drive-data-export.googleusercontent.com/download/hjgi1l3jtk6d2cks6tn71lf27hineubs/o8l0925n22g1ig3hoos5002u4540sna3/1607593500000/fc2f2fa1-04dd-4bb4-9bb2-eaf74ce196b5/101012487977639686937/ADt3v-NiAdo2XC059SeHWmfMv6gzzVmJcWO3-TD3OoP5YPW6pqhJ76FegCiOcZL-NLIfOUMaZxvBca7DQNcOomiBOP9gFdYOD9CrCOLgDNee-3MZ731gPqHk3zuKHML9yVm3A-4eqHrWhWQJrocpO4axYy6V0R4T8Pghqg32LcQAZjyJFL7kkdFvrgrLVrL3gg9n9D0AXeSoR-GWU6I96U6pXpaukbT2DrxmNVvjDLyJMbcsznIfFlkwRXpFc9LTBEv74hsCHhyaamJiq69wg27wqlZz2orng-Rd-ND8zgaHDSMVSyRjOQ_k1lqWr1pKse-Wit6t77Vd?authuser=0&nonce=a1hgt2i2d7mdu&user=101012487977639686937&hash=040t9ijj4pct2ffv98qdikf92lcl80cg' --output-document 'fence_double_post.zip'
        wget --header 'Host: doc-58-c4-drive-data-export.googleusercontent.com' --user-agent 'Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:82.0) Gecko/20100101 Firefox/82.0' --header 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8' --header 'Accept-Language: en-US,en;q=0.5' --referer 'https://drive.google.com/drive/folders/1GXQd8hLl4PwjNRe5Nt09vRRVvILiGE0y' --header 'Cookie: AUTH_p8oalvncdji24ur7qqlm8344ada3s0b0_nonce=kn76l6h0bo35o' --header 'Upgrade-Insecure-Requests: 1' 'https://doc-58-c4-drive-data-export.googleusercontent.com/download/hjgi1l3jtk6d2cks6tn71lf27hineubs/2dge70gdfkm2lbm0ljegqe3vvah3k11o/1607593500000/294f4d05-f84e-4189-a171-ccae189e5a2c/101012487977639686937/ADt3v-O3OAGs0dY8oUFwWCeJGk4t4W9srojffX4ZdwjDx3iM6FSpxRBfbdnWWjJIZ1ZcaA5a23EY3CEIx5J5E509bTKsZVGkIOTZ8to5m7jRIUnc-Pg6NppcNTLUKmS4IKQpvAlPUYyz8paGcpYR37c8K8W7DMSknS4VCwff225xFNPXXLorTGm69n-h9K0HpPO6iHMfs3NYGeygEHJs2MD5e369UnzZW5GO17cBbPOWjHQ9DwDLS3OfqKyRzzFGRRgFVf_nUKMZZOMd1FfUDCMES0jusLEWDHMh9cweHPEsmYo_UpCUitZdKz9N1EBnT4vfbrQ1l3WD?authuser=0&nonce=kn76l6h0bo35o&user=101012487977639686937&hash=v63idgb4l0pfjmd8q63d881ilo0c1cd9' --output-document 'fence_hole1.zip'
        wget --header 'Host: doc-54-58-drive-data-export.googleusercontent.com' --user-agent 'Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:82.0) Gecko/20100101 Firefox/82.0' --header 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8' --header 'Accept-Language: en-US,en;q=0.5' --referer 'https://drive.google.com/drive/folders/1GXQd8hLl4PwjNRe5Nt09vRRVvILiGE0y' --header 'Cookie: AUTH_p8oalvncdji24ur7qqlm8344ada3s0b0_nonce=dihmouh8sk194' --header 'Upgrade-Insecure-Requests: 1' 'https://doc-54-58-drive-data-export.googleusercontent.com/download/hjgi1l3jtk6d2cks6tn71lf27hineubs/64i8ves3f2qhqgq6keheuu9saf3dogig/1607593500000/04d9a462-71b7-4f3b-b08e-0abf91de19b6/101012487977639686937/ADt3v-N2PMoDmcg4zulMFJvhjPkqzzQdwhh_05HzghFEOh_Rhx0HPhYhfgSMcg7C2KhEfD-pHJB3iUp1S3NMjnZDpmMnoluPlSKiPlEswX4XDPoaGPapiNXCWmBrIfrcgMARqfHVjf0OZIj9KovhGUv4CPGrHqHIi7H8362SoKPyYNPb7dDPHcexqIR0qWbBRZ-DBTG4pCB-FOBjWGnJpifCsyGMpe8CCSPFJohXB3PeOnhjxkqZt9NEjz2-aiDhmqDFQ62u3j54r7izWnZrD8gL8Fj7GoyOEhwwC4Yj4uEYdJQUg-16du3aSi9T6jDWl5rlqG2gbf3l?authuser=0&nonce=dihmouh8sk194&user=101012487977639686937&hash=kti5chnhq7aqs0s21rdasfn981lvjjun' --output-document 'fence_single.zip'

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