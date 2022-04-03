CALIBRATE

roslaunch hand_eye_grasp kinova_realsense_calibrate_eye_on_hand.launch


launch 修改

    <arg name="robot_type" value="j2n6s300"/>

    <arg name="marker_size" default="0.11" doc="Size of the ArUco marker used, in meters" />
    <arg name="marker_id"   default="23" doc="The ID of the ArUco marker used" />


robot_type : arm model
marker_size: arcuo size ,length,
marker_id: arcuo_id


publish tf
roslaunch hand_eye_grasp kinova_realsense_eye_on_hand_publish.launch
