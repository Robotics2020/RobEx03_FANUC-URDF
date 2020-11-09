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

The URDF file is available [here](https://github.com/Robotics2020/RobEx03_FANUC-URDF/blob/master/fanuc_description/robot/fanuc_m20iA.xacro).

## Point 3

> Create a fanuc_moveit_config package and visualize the robot in RViz.

The moveit_config package is available [here](https://github.com/Robotics2020/RobEx03_FANUC-URDF/blob/master/fanuc_moveit_config).

## Point 4

> Visualize TFs in RViz and write one or more ROS nodes to compute and print the TF of the end-effector in all the reference frames of all joints. From the TF, compute the translational vector, the rotation matrix, the Euler angles and the axis-angle representation.

The only ROS node is developed in the [fanuc_listener](https://github.com/Robotics2020/RobEx03_FANUC-URDF/blob/master/fanuc_listener_pkg) package.

## Point 5

> Verify the results by comparing with manual/MATLAB calculations.

The results given by the node are always the same given by the node [tf_echo](http://docs.ros.org/en/melodic/api/tf/html/c++/tf__echo_8cpp_source.html). Anyway, sometimes the same quaternion provided by tf_echo and the developed node gives a different tuple of RPY-angles when coded in matlab.

Here is an example:

```matlab
t = quat2tform([0.699679 0.111959 -0.697511 -0.106738
rpy = tform2eul(t, "XYZ")
```

gives as output

```matlab
t =

    0.0042   -0.0068   -1.0000         0
   -0.3055    0.9521   -0.0078         0
    0.9522    0.3056    0.0019         0
         0         0         0    1.0000

rpy =

    1.3325   -1.5628    1.0219
```

while the information given by tf_echo are

```text
- Rotation: in Quaternion [0.112, -0.698, -0.107, 0.700]
        in RPY (radian) [1.565, -1.260, -1.557]
        in RPY (degree) [89.646, -72.207, -89.218]
```

and the information provided by the developed node are

```text
------- Quaternion -------
[0.111959, -0.697511, -0.106738, 0.699679]

------- Axis/angle -------
Axis = [0.156705, -0.976281, -0.149398]
Angle = 1.5917

------- Rotation matrix -------
[0.0042, -0.0068, -1]
[-0.3056, 0.9521, -0.0078]
[0.9522, 0.3056, 0.0019]

------- Euler angles (RPY) -------
[1.56458, -1.26036, -1.55705]
```

As shown, the rotation matrices are the same, but Euler angles are different (please remember that matlab codes quaternions in the format `w-x-y-z`, while the last code blocks use the format `x-y-z-w` as per [tf2](https://docs.ros.org/en/melodic/api/tf2/html/namespacetf2.html) library)

## Usage

To visualize the FANUC M-20iA robot model, run `roslaunch fanuc_description fanuc.launch`.

To get the TFs for all joints, run `roslaunch fanuc_listener_pkg fanuc fanuc_listener.launch`.
