- Build the new package:
$ colcon build

- Source the setup files:
$ source install/setup.bash

- Launch gazebo to see the blue ball:
$ ros2 launch iiwa_bringup iiwa.launch.py

- Launch the robot within the world containing the Aruco marker with velocity interface:
$ ros2 launch iiwa_bringup iiwa.launch_aruco.py

- Launch the robot with effort controller:
$ ros2 launch iiwa_bringup iiwa.launch_effort.py

N.B.: As soon as the gazebo opens, the simulation must be started within 5 seconds pushing the play button, otherwise it won't load the controllers.

- Connect to the container from another terminal:
$ ./docker_connect.sh 

N.B.: Before using any other terminal, source the setup files.

Once the terminal is connected is possible :

- Launch the Aruco Detection node to detect specific ArUco markers in a video feed.This node calculates the pose (position and orientation) of the detected ArUco marker with respect to the camera and publishes the information on the related topics.
$ ros2 run aruco_ros single --ros-args -r /image:=/camera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.2 -p reference_frame:=camera_link -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link

-  Launch the rqt_image_view node that allows to view the feed from the camera in real time.It is used to check if the ArUco marker is visible and if the video feed is correctly configured:
$ ros2 run rqt_image_view rqt_image_view
choice the topic /aruco_single/result

-  Executes the positioning task:
$ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p task:=positioning

- Executes the look at point task:
$ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p task:=look-at-point

N.B.: Both the tasks must be excecuted with the velocity interface, in case there is an error connection, try to relaunch the nodes

- Start ros2_kdl_vision_dynamic, in order to observe the trajectories in the Cartesian Space, (with effort interface):

- launch the node into another terminal:
$ ros2 run ros2_kdl_package ros2_kdl_vision_dynamic









