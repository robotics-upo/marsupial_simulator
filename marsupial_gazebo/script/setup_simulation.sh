#!/bin/sh

read -p "Installation PACKAGES for Marsupial Simulator. Press a ENTER to contine. (CTRL+C) to cancel"


# For drone simulation
echo "\n Installing rotors_simulator \n\n"
git clone -b https://github.com/ethz-asl/rotors_simulator.git
echo "\n Installing mav_comm \n\n"
git clone https://github.com/ethz-asl/mav_comm.git

# For localization stuff
echo "\n Installing DLL \n\n"
git clone -noetic https://github.com/robotics-upo/dll.git
echo "\n Installing odom to tf \n\n"
git clone https://github.com/robotics-upo/odom_to_tf.git

# For Tracking
echo "\n Installing matrice_traj_tracker \n\n"
git clone https://github.com/robotics-upo/matrice_traj_tracker.git
echo "\n Installing Onboard-SDK-ROS \n\n"
git clone -b 3.8 https://github.com/dji-sdk/Onboard-SDK-ROS.git