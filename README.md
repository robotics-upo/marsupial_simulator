# marsupial_simulator
Initial simulation Version for Tethered Marsupial System.

## Dependencies

The UAV simulator use quadrotor_ign_utils and mav_comm package. Please follow the install instructions of the package:

```
your_catkin_ws/src> git clone https://github.com/robotics-upo/quadrotor_ign_utils
your_catkin_ws/src> git clone https://github.com/ethz-asl/mav_comm
```

The UGV simulator uses the ARCO simulator from IdMind. In the source folder of your workspace:

```
your_catkin_ws/src> git clone https://github.com/robotics-upo/catkin_newarco
your_catkin_ws/src> cd catkin_newarco/src/
your_catkin_ws/src> git clone https://github.com/idmind-robotics/idmind_serial2
```

Then build the project, for example in the root folder of your workspace:

```
your_catkin_ws> catkin_make
```

## Resources Ignition

In case to use the simulator in Teatro World, is necessary to download ".stl" file from: https://www.dropbox.com/s/khx1v76lt3jpcdv/teatro.stl?dl=0 .

Save the file in: marsupial_simulator/models/teatro/mesh/ .


## Usage

To launch a basic simulation please use the provided launch file.

```
 > roslaunch marsupial_gazebo marsupial_simulation.launch
```
