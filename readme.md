本功能包实现KInova机械臂与Realsense D435联合进行的手眼标定、抓取。

所抓物体为北京维尔码科技有限公司提供的三色（红色、蓝色、橙色）圆柱体木块；圆柱体木块树直放置与桌面上，即圆柱体上下平面与桌面平行。抓取时，机械臂手抓末端与桌面平行。

抓到的物体放置于aruco码的上方，aruco码在本例程中最多可选择为四个，id号分别为10、15、25、45。可以放置多个或一个（id号是10、15、25、45其中的一个或多个）


第一步、手眼标定：
    roslaunch hand_eye_grasp kinova_realsense_calibrate_eye_on_hand.launch
    完成后Ctrl+C杀掉该进程

第二步、启动机械臂、摄像头，以及手眼标定关系
    roslaunch hand_eye_grasp bringup_eye_on_hand.launch 

第三步、寻找Marker点坐标，（需保持机械臂底座与Marker点的相对位置不变）
    roslaunch hand_eye_grasp select_marker.launch
    根据提示完成后Ctrl+C杀掉该进程，Marker点相对机械臂底座的坐标保存在文件home/..../hand_eye_grasp/marker.yaml中

第四步、抓取木块：
    roslaunch hand_eye_grasp pick_place_demo.launch
    根据提示完成木块选择、marker选择，并完成抓取动作；


备注1（非常重要）：需要根据机械臂型号对程序进行修改

备注2：部分参数需要在程序中进行优化，主要在pick_place_on_markers_demo.cpp / pick_placecpp 及各个launch文件中进行

