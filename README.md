# RobEx03_FANUC-URDF

Third exercise for Robotics course @UniSa. Academic Year 2020/2021.

1. Derive DH parameters for the Fanuc robot.
2. Translate DH parameters to URDF and visualize the Fanuc robot in RViz with urdf_tutorial display.launch.
3. Create a fanuc_moveit_config package and visualize the robot in RViz.
4. Visualize TFs in RViz and write one or more ROS nodes to compute and print the TF of the end-effector in all the reference frames of all joints. From the TF, compute the translational vector, the rotation matrix, the Euler angles and the axis-angle representation.
5. Verify the results by comparing with manual/MATLAB calculations.

In particular, the robot model will be FANUC M-20iA: please refer to the following documents for further information:
* [FANUC M-10iA / M-20iA Operating Space](https://github.com/Robotics2020/RobEx03_FANUC-URDF/blob/master/fanuc_description/doc/FANUC_M-10iA_M-20iA_drawing.pdf)
* [FANUC ARC Mate 120iC / M-20iA Manual](https://github.com/Robotics2020/RobEx03_FANUC-URDF/blob/master/fanuc_description/doc/FANUC_M-20iA_manual.pdf)

## Point 1

> Derive DH parameters for the Fanuc robot.

The DH parameters are available at [this pdf](https://github.com/Robotics2020/RobEx03_FANUC-URDF/blob/master/fanuc_description/doc/FANUC_M-20iA_dh.pdf).

## Point 2

> Translate DH parameters to URDF and visualize the Fanuc robot in RViz with urdf_tutorial display.launch.

The URDF file is available [here](https://github.com/Robotics2020/RobEx03_FANUC-URDF/blob/master/fanuc_description/robot/FANUC_M-20iA.urdf).

## Point 3

> Create a fanuc_moveit_config package and visualize the robot in RViz.

## Point 4

> Visualize TFs in RViz and write one or more ROS nodes to compute and print the TF of the end-effector in all the reference frames of all joints. From the TF, compute the translational vector, the rotation matrix, the Euler angles and the axis-angle representation.

## Point 5

> Verify the results by comparing with manual/MATLAB calculations.

## Usage

To visualize the FANUC M-20iA robot model, just run `roslaunch fanuc_description fanuc.launch`.
