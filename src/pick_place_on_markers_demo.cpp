#include <pick_place.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <tf_conversions/tf_eigen.h>

tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2)
{
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, ty, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(tz2, 0.0, 0.0);
    rot *= rot_temp;
    rot.getRotation(q);
    return q;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_pick", ros::init_options::NoSigintHandler);

    ros::AsyncSpinner spinner(5);
    spinner.start();

    ros::NodeHandle node("~");
  
    ros::WallDuration(1.0).sleep();

    Pick_Place_Demo::pick_place hand_eye_pick_place(node);

    ros::Rate loop_rate(0.2);
    while (ros::ok()) 
	{
        bool finished_grasp = false;

        hand_eye_pick_place.clear_workscene();//清空workscene
        ros::WallDuration(1.0).sleep(); 

        if(hand_eye_pick_place.start_or_restart == false && hand_eye_pick_place.get_pick_pose == true)//得到目标位置
        {
            try
            {

                int try_times = 0;
                bool find_plan_ = false;
                float target_distance = 0.0;
                float target_roll_distance = 0.00;
                float target_yaw_distance = 0.00;
                float target_pitch_distance = 0.0;

                hand_eye_pick_place.pick_from_target_position(hand_eye_pick_place. target_pick_position);

                //计算抓取时的欧拉角
                float oula_grasp = hand_eye_pick_place.calibrate_grasp_euler();
              //  std::cout << "grasp oula : "<<oula_grasp << std::endl;

                tf::Quaternion q_pick;
                tf::Quaternion q_place;

                //以当前姿态进行规划，若能规划出路径，证明当前姿态进行抓取可行；若规划不出路径，证明路径不可行，对姿态进行调整
                q_pick = EulerZYZ_to_Quaternion(oula_grasp, 1.5708, -1.5708);//初始姿态

                geometry_msgs::PoseStamped pick_place_position;
                geometry_msgs::PoseStamped marker_place_position;

                pick_place_position = hand_eye_pick_place. target_pick_position;

                pick_place_position.pose.orientation.x = q_pick.x();
                pick_place_position.pose.orientation.y = q_pick.y();
                pick_place_position.pose.orientation.z = q_pick.z();
                pick_place_position.pose.orientation.w = q_pick.w();

                pick_place_position.pose.position.z = 0.03;//抓取时的手抓距离机器人底座的高度，可根据实际情况进行修改
               
                hand_eye_pick_place.setPositionTolerance(0.02);//目标点位置精度，0.02米，需要根据情况调整；值越小，规划出的路径越精确，但是找不到路径的概率越大
                hand_eye_pick_place.setOrientationTolerance(0.03);//目标点方位精度,0.03弧度，需要根据情况调整；值越小，规划出的路径越精确，但是找不到路径的概率越大

                do
                {

                    find_plan_ = hand_eye_pick_place.find_pose_plan(pick_place_position);//规划路径
                    if(!find_plan_)
                    {
                        oula_grasp += 0.6;//调整角度，幅度可进行调整
                        q_pick = EulerZYZ_to_Quaternion(oula_grasp , 1.5708, -1.5708);
                        pick_place_position.pose.orientation.x = q_pick.x();
                        pick_place_position.pose.orientation.y = q_pick.y();
                        pick_place_position.pose.orientation.z = q_pick.z();
                        pick_place_position.pose.orientation.w = q_pick.w();
                        try_times++;
                    }
                  
                }while(try_times < 10 && !find_plan_); //寻找10次，可根据实际，调整寻找次数
                

                if(try_times >= 10 && !find_plan_)//尝试了各种角度去抓取，都没有找到路径，表明物体不可被抓取，无法避免碰撞
                {
                    std::cout << "///////////////////////////////////////////////////////////////////////////////////////"  << std::endl; 

                    std::cout << "We found no way to grasp the object you select, maybe you need reselect one"  << std::endl; 

                    std::cout << "///////////////////////////////////////////////////////////////////////////////////////"  << std::endl; 
                   
                    hand_eye_pick_place.start_or_restart = true;
                    hand_eye_pick_place.get_pick_pose = false;
                    hand_eye_pick_place.need_refind_marker = false;

                }
                else//有可用路径，证明目标是可被抓取的，
                {

                    //通过欧拉角计算四元数
                    q_pick = EulerZYZ_to_Quaternion(oula_grasp, 3.1416, 0);

                    //机械臂从当前位置规划到marker上方，确认码在；若没有找到对应的码。可以更换码
                    geometry_msgs::PoseStamped maybe_place_position;
                    if(hand_eye_pick_place.maybe_place_maker_id == 10)//选择的码的id为10
                    {                  
                        maybe_place_position.header.frame_id = hand_eye_pick_place. target_pick_position.header.frame_id;
                        maybe_place_position.header.stamp = ros::Time::now();
                        maybe_place_position.pose.position.x = hand_eye_pick_place.marker_10_x;
                        maybe_place_position.pose.position.y = hand_eye_pick_place.marker_10_y;
                        maybe_place_position.pose.position.z = hand_eye_pick_place.marker_10_z;

                    }
                    else if(hand_eye_pick_place.maybe_place_maker_id == 15)
                    {
                        maybe_place_position.header.frame_id = hand_eye_pick_place. target_pick_position.header.frame_id;
                        maybe_place_position.header.stamp = ros::Time::now();
                        maybe_place_position.pose.position.x = hand_eye_pick_place.marker_15_x;
                        maybe_place_position.pose.position.y = hand_eye_pick_place.marker_15_y;
                        maybe_place_position.pose.position.z = hand_eye_pick_place.marker_15_z;
                    }
                    else if(hand_eye_pick_place.maybe_place_maker_id == 25)
                    {
                        maybe_place_position.header.frame_id = hand_eye_pick_place. target_pick_position.header.frame_id;
                        maybe_place_position.header.stamp = ros::Time::now();
                        maybe_place_position.pose.position.x = hand_eye_pick_place.marker_25_x;
                        maybe_place_position.pose.position.y = hand_eye_pick_place.marker_25_y;
                        maybe_place_position.pose.position.z = hand_eye_pick_place.marker_25_z;                  

                    } 
                    else if(hand_eye_pick_place.maybe_place_maker_id == 45)
                    {
                        maybe_place_position.header.frame_id = hand_eye_pick_place. target_pick_position.header.frame_id;
                        maybe_place_position.header.stamp = ros::Time::now();
                        maybe_place_position.pose.position.x = hand_eye_pick_place.marker_45_x;
                        maybe_place_position.pose.position.y = hand_eye_pick_place.marker_45_y;
                        maybe_place_position.pose.position.z = hand_eye_pick_place.marker_45_z;                 

                    }
                    maybe_place_position.pose.position.z += 0.3;//垂直往下寻找码时机械臂末端距离机械臂底座的高度，可根据情况调整

                    maybe_place_position.pose.orientation.x = q_pick.x();
                    maybe_place_position.pose.orientation.y = q_pick.y();
                    maybe_place_position.pose.orientation.z = q_pick.z();
                    maybe_place_position.pose.orientation.w = q_pick.w();

                    hand_eye_pick_place.setPositionTolerance(0.1);//目标点位置精度，0.05米
                    hand_eye_pick_place.setOrientationTolerance(0.05);//目标点方位精度,0.1弧度
          
                    do
                    {

                        find_plan_ = hand_eye_pick_place.find_pose_plan(maybe_place_position);

                        if(find_plan_)
                        {                    
                            hand_eye_pick_place.evaluate_plan();
                            ros::WallDuration(5.0).sleep();                     
                        }

                        tf::Quaternion RQ2;  
                        double roll,pitch,yaw;
                        double target_pitch;

                        tf::quaternionMsgToTF(hand_eye_pick_place.current_pose.pose.orientation, RQ2);  
                        tf::Matrix3x3(RQ2).getRPY(roll, pitch, yaw);  

                        tf::quaternionMsgToTF(maybe_place_position.pose.orientation, RQ2);  
                        tf::Matrix3x3(RQ2).getRPY(roll, target_pitch, yaw);  

                        target_distance = sqrt((maybe_place_position.pose.position.x - hand_eye_pick_place.current_pose.pose.position.x) * (maybe_place_position.pose.position.x - hand_eye_pick_place.current_pose.pose.position.x) + (maybe_place_position.pose.position.y - hand_eye_pick_place.current_pose.pose.position.y) * (maybe_place_position.pose.position.y - hand_eye_pick_place.current_pose.pose.position.y));
                        target_roll_distance = fabs(target_pitch - pitch);//机械臂末端与地方垂直情况的偏差量；值越小越垂直

                    }while(target_distance > 0.1 || target_roll_distance > 0.1);//可根据实际情况调整偏差量

                    hand_eye_pick_place.need_refind_marker = true;//开始寻找精确marker的坐标

                    bool need_rotation_arm = false;
                    try_times = 0;
                    while(!hand_eye_pick_place.get_palce_marker_position && try_times < 10)//10次内都没有找到想要的码，可以根据实际情况调整寻找次数
                    {
                        if(hand_eye_pick_place.find_any_marker)//找到了其他的码
                        {

                            std::cout << "We didn't find the marker you selected, we find other markers,  would you like to select a new? "  << std::endl; 
                            std::cout << "If you want to reselect a new place marker , please enter Y or y "  << std::endl; 
                       
                            std::string pause_;
                            std::cin >> pause_;

                            ros::WallDuration(0.5).sleep();

                            if (pause_ == "Y" || pause_ == "y" )//更换新的码
                            {
                                hand_eye_pick_place.select_new_marker = true;
                                need_rotation_arm = false;                            
                            }
                            else //不更换新的码
                            {
                                hand_eye_pick_place.select_new_marker = false;
                                need_rotation_arm = true;
                            }
                        }
                        else//没有发现任何码，手臂需要旋转
                        {
                            need_rotation_arm = true;
                        }

                        if(need_rotation_arm)//需要旋转手臂
                        {
                            sensor_msgs::JointState target_joint;

                            target_joint.position.resize(6);
                            target_joint.name.resize(6);
                            target_joint.velocity.resize(6);
                            target_joint.effort.resize(6);

                            int joint_number = hand_eye_pick_place.current_state_.name.size();
                            if(joint_number >= 6)//获取当前关节值
                           {
                                for(int i = 0 ;i < joint_number; i++)
                                {
                                    if(hand_eye_pick_place.current_state_.name[i] == "m1n6s200_joint_1") target_joint.position[0] = hand_eye_pick_place.current_state_.position[i];//关节名称，需根据实际进行调整
                                    if(hand_eye_pick_place.current_state_.name[i] == "m1n6s200_joint_2")target_joint.position[1] = hand_eye_pick_place.current_state_.position[i];
                                    if(hand_eye_pick_place.current_state_.name[i] == "m1n6s200_joint_3")target_joint.position[2] = hand_eye_pick_place.current_state_.position[i];
                                    if(hand_eye_pick_place.current_state_.name[i] == "m1n6s200_joint_4")target_joint.position[3] = hand_eye_pick_place.current_state_.position[i];
                                    if(hand_eye_pick_place.current_state_.name[i] == "m1n6s200_joint_5")target_joint.position[4] = hand_eye_pick_place.current_state_.position[i];
                                    if(hand_eye_pick_place.current_state_.name[i] == "m1n6s200_joint_6")target_joint.position[5] = hand_eye_pick_place.current_state_.position[i];
                                }
                            } 

                            target_joint.position[5] += 0.6;//末端关节旋转0.6弧度，可根据情况调整
                            find_plan_ = hand_eye_pick_place.find_joint_plan(target_joint);//规划路径
                            if(find_plan_)hand_eye_pick_place.evaluate_plan();//执行规划的路径
                            ros::WallDuration(2.0).sleep();

                            need_rotation_arm = false;
                            try_times++;
                        }
                    }

                    need_rotation_arm = false;
                    ros::WallDuration(1.0).sleep();

                    if(!hand_eye_pick_place.get_palce_marker_position && try_times >= 10 )//旋转了10次，依然没有发现想要的码
                    {
                        if(hand_eye_pick_place.find_any_marker)//发现有其他码
                        {

                            std::cout << "We really didn't find the marker you selected, we find other markers,  would you like to select a new? "  << std::endl; 
                            std::cout << "If you want to reselect a new place marker , please enter Y or y "  << std::endl; 
                       
                            std::string pause_;
                            std::cin >> pause_;

                            ros::WallDuration(0.5).sleep();

                            if (pause_ == "Y" || pause_ == "y" )//最后一次机会更新码
                            {
                                hand_eye_pick_place.select_new_marker = true;
                                finished_grasp = false;                            
                            }
                            else //不更新新的码
                            {
                                hand_eye_pick_place.select_new_marker = false;
                                finished_grasp = true;                            
                            }
                        }
                        else//没有发现任何码
                        {
                            finished_grasp = true;
                        }

                    }
                    if(finished_grasp)//结束本次抓取
                    {
                        /////////////////////////////////////////////////
                        ////////////////机械臂回到初始位置，结束本次抓取






                        ////////////////////////////////////////////////
                        hand_eye_pick_place.start_or_restart = true;
                        hand_eye_pick_place.get_pick_pose = false;
                        hand_eye_pick_place.need_refind_marker = false;
                    }

                    if(hand_eye_pick_place.get_palce_marker_position && finished_grasp == false)//寻找得到精确marker坐标
                    {

                        //通过寻找到码上的路径，来判断是否存在放置物体到码的路径                        
                        //以当前姿态进行规划，若能规划出路径，证明当前姿态进行抓取可行；若规划不出路径，证明路径不可行，对姿态进行调整
                        q_place = EulerZYZ_to_Quaternion(oula_grasp, 1.5708, -1.5708);//初始姿态

                        marker_place_position = hand_eye_pick_place. target_place_position;

                        marker_place_position.pose.orientation.x = q_place.x();
                        marker_place_position.pose.orientation.y = q_place.y();
                        marker_place_position.pose.orientation.z = q_place.z();
                        marker_place_position.pose.orientation.w = q_place.w();

                        marker_place_position.pose.position.z = 0.03;//放置木块时的手抓距离机器人底座的高度，可根据实际情况进行修改
               
                        hand_eye_pick_place.setPositionTolerance(0.02);//目标点位置精度，0.02米，需要根据情况调整；值越小，规划出的路径越精确，但是找不到路径的概率越大
                        hand_eye_pick_place.setOrientationTolerance(0.03);//目标点方位精度,0.03弧度，需要根据情况调整；值越小，规划出的路径越精确，但是找不到路径的概率越大

                        do
                        {

                            find_plan_ = hand_eye_pick_place.find_pose_plan(marker_place_position);//规划路径
                            if(!find_plan_)
                            {
                                oula_grasp += 0.6;//调整角度，幅度可进行调整
                                q_place = EulerZYZ_to_Quaternion(oula_grasp , 1.5708, -1.5708);
                                marker_place_position.pose.orientation.x = q_place.x();
                                marker_place_position.pose.orientation.y = q_place.y();
                                marker_place_position.pose.orientation.z = q_place.z();
                                marker_place_position.pose.orientation.w = q_place.w();
                                try_times++;
                            }
                  
                        }while(try_times < 10 && !find_plan_); 

                        if(try_times >= 10 && !find_plan_)//尝试了各种角度去放置，都没有找到路径，表明物体不可被放置到选取的码上，无法避免碰撞，需要重新开始
                        {
                            std::cout << "///////////////////////////////////////////////////////////////////////////////////////"  << std::endl; 

                            std::cout << "We found no way to place the object on the marker you select, maybe you need check and restart"  << std::endl; 

                            std::cout << "///////////////////////////////////////////////////////////////////////////////////////"  << std::endl; 
////////////////////////////////////机械臂回到初始位置







///////////////////////////////////////////////////                   
                            hand_eye_pick_place.start_or_restart = true;
                            hand_eye_pick_place.get_pick_pose = false;
                            hand_eye_pick_place.need_refind_marker = false;

                        }
                        else//可以找到放置物体到码上的路径
                        {

                            //机械臂开始抓取
                            target_distance = 0.00;
                            try_times = 0;
                            find_plan_ = false;

                            hand_eye_pick_place.setPositionTolerance(0.02);//目标点位置精度，根据世纪情况调整
                            hand_eye_pick_place.setOrientationTolerance(0.03);//目标点方位精度,0.1弧度，根据情况调整
                            do
                            {

                                find_plan_ = hand_eye_pick_place.find_pose_plan(pick_place_position);
                                std::cout << find_plan_ << std::endl;

                                if(find_plan_)
                                {     
                                   std::cout <<"get plan" << std::endl;               
                                    hand_eye_pick_place.evaluate_plan();
                                    ros::WallDuration(5.0).sleep();                     
                                }


                                tf::Quaternion RQ2;  
                                double roll,pitch,yaw;
                                double target_roll,target_pitch,target_yaw;;

                                tf::quaternionMsgToTF(hand_eye_pick_place.current_pose.pose.orientation, RQ2);  
                                tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);  

                                //std::cout << "current r: " << roll << " ,  p: "<< pitch << " , y :" << yaw << std::endl;

                                tf::quaternionMsgToTF(pick_place_position.pose.orientation, RQ2);  
                                tf::Matrix3x3(RQ2).getRPY(target_roll,target_pitch,target_yaw);  
                               // std::cout << "target r: " << target_roll << " ,  p: "<< target_pitch << " , y :" << target_yaw << std::endl;

                                target_roll_distance = fabs(target_roll - roll);
                                target_pitch_distance = fabs(target_pitch - pitch);
                                target_yaw_distance = fabs(target_yaw - yaw);

                                target_distance = sqrt((pick_place_position.pose.position.x - hand_eye_pick_place.current_pose.pose.position.x) * (pick_place_position.pose.position.x - hand_eye_pick_place.current_pose.pose.position.x) + (pick_place_position.pose.position.y - hand_eye_pick_place.current_pose.pose.position.y) * (pick_place_position.pose.position.y - hand_eye_pick_place.current_pose.pose.position.y));
                                //std::cout << "The distance between the current position and the target position is: " << target_distance << std::endl;

                                try_times++;

                            }while(target_distance > 0.05 || target_roll_distance > 0.1 || target_yaw_distance > 0.1 || target_pitch_distance > 0.1);//位置、角度精度不足时，不停进行调整；可根据实验情况调整精度信息

                            hand_eye_pick_place.gripper_action(0.75*hand_eye_pick_place.FINGER_MAX);//手抓闭合，抓取物体

                            hand_eye_pick_place.place_to_target_position();

                            pick_place_position.pose.position.z = 0.20;//抓到木块后；首先往上提起到离机器人底座0.2高度处，根据实际需要进行调整
                            try_times = 0;
                            do
                            {

                                find_plan_ = hand_eye_pick_place.find_pose_plan(pick_place_position);
                                std::cout << find_plan_ << std::endl;

                                if(find_plan_)
                                {     
                                    //std::cout <<"get plan" << std::endl;               
                                    hand_eye_pick_place.evaluate_plan();
                                    ros::WallDuration(5.0).sleep();         //等待执行时间，可进行调整            
                                }

                                target_distance = fabs(pick_place_position.pose.position.z - hand_eye_pick_place.current_pose.pose.position.z);//高度差

                                try_times++;

                            }while(target_distance > 0.05);//只考虑高度偏差；可根据实验情况调整精度信息

                           //运动到pre_place处，即码上方距离机械臂底座0。3处
                            geometry_msgs::PoseStamped pre_place_position =  marker_place_position;
                            pre_place_position.pose.position.z = 0.3;//机械臂运动到码上方0.3处
                            do
                            {

                                find_plan_ = hand_eye_pick_place.find_pose_plan(pre_place_position);

                                if(find_plan_)
                                {     
                                    hand_eye_pick_place.evaluate_plan();
                                    ros::WallDuration(5.0).sleep();                     
                                }

                                tf::Quaternion RQ2;  
                                double roll,pitch,yaw;
                                double target_roll,target_pitch,target_yaw;;

                                tf::quaternionMsgToTF(hand_eye_pick_place.current_pose.pose.orientation, RQ2);  
                                tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);  

                                //std::cout << "current r: " << roll << " ,  p: "<< pitch << " , y :" << yaw << std::endl;

                                tf::quaternionMsgToTF(pre_place_position.pose.orientation, RQ2);  
                                tf::Matrix3x3(RQ2).getRPY(target_roll,target_pitch,target_yaw);  
                               // std::cout << "target r: " << target_roll << " ,  p: "<< target_pitch << " , y :" << target_yaw << std::endl;

                                target_roll_distance = fabs(target_roll - roll);
                                target_pitch_distance = fabs(target_pitch - pitch);
                                target_yaw_distance = fabs(target_yaw - yaw);

                                target_distance = sqrt((pre_place_position.pose.position.x - hand_eye_pick_place.current_pose.pose.position.x) * (pre_place_position.pose.position.x - hand_eye_pick_place.current_pose.pose.position.x) + (pre_place_position.pose.position.y - hand_eye_pick_place.current_pose.pose.position.y) * (pre_place_position.pose.position.y - hand_eye_pick_place.current_pose.pose.position.y));
                                //std::cout << "The distance between the current position and the target position is: " << target_distance << std::endl;

                                try_times++;

                            }while(target_distance > 0.05 || target_roll_distance > 0.1 || target_yaw_distance > 0.1 || target_pitch_distance > 0.1);//位置、角度精度不足时，不停进行调整；可根据实验情况调整精度信息

                            //往下运动，放置木块
                            do
                            {

                                find_plan_ = hand_eye_pick_place.find_pose_plan(marker_place_position);
                                std::cout << find_plan_ << std::endl;

                                if(find_plan_)
                                {     
                                   std::cout <<"get plan" << std::endl;               
                                    hand_eye_pick_place.evaluate_plan();
                                    ros::WallDuration(5.0).sleep();                     
                                }

/*
                                tf::Quaternion RQ2;  
                                double roll,pitch,yaw;
                                double target_roll,target_pitch,target_yaw;;

                                tf::quaternionMsgToTF(hand_eye_pick_place.current_pose.pose.orientation, RQ2);  
                                tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);  

                                //std::cout << "current r: " << roll << " ,  p: "<< pitch << " , y :" << yaw << std::endl;

                                tf::quaternionMsgToTF(marker_place_position.pose.orientation, RQ2);  
                                tf::Matrix3x3(RQ2).getRPY(target_roll,target_pitch,target_yaw);  
                               // std::cout << "target r: " << target_roll << " ,  p: "<< target_pitch << " , y :" << target_yaw << std::endl;

                                target_roll_distance = fabs(target_roll - roll);
                                target_pitch_distance = fabs(target_pitch - pitch);
                                target_yaw_distance = fabs(target_yaw - yaw);

                                target_distance = sqrt((marker_place_position.pose.position.x - hand_eye_pick_place.current_pose.pose.position.x) * (marker_place_position.pose.position.x - hand_eye_pick_place.current_pose.pose.position.x) + (marker_place_position.pose.position.y - hand_eye_pick_place.current_pose.pose.position.y) * (marker_place_position.pose.position.y - hand_eye_pick_place.current_pose.pose.position.y));
                                //std::cout << "The distance between the current position and the target position is: " << target_distance << std::endl;

                                try_times++;
*/
                                target_distance = fabs(marker_place_position.pose.position.x - hand_eye_pick_place.current_pose.pose.position.x) ;

                            }while(target_distance > 0.05);//只考虑了z方向上的偏差；可根据实验情况调整精度信息;

                            hand_eye_pick_place.gripper_action(0); //手抓动作，开始张开

                            hand_eye_pick_place.pick_from_target_position(marker_place_position);

                            //手抓向上运动一段距离
                            do
                            {

                                find_plan_ = hand_eye_pick_place.find_pose_plan(pre_place_position);
                                std::cout << find_plan_ << std::endl;

                                if(find_plan_)
                                {     
                                   std::cout <<"get plan" << std::endl;               
                                    hand_eye_pick_place.evaluate_plan();
                                    ros::WallDuration(5.0).sleep();                     
                                }


                                target_distance = fabs(pre_place_position.pose.position.x - hand_eye_pick_place.current_pose.pose.position.x) ;

                            }while(target_distance > 0.05);//只考虑了z方向上的偏差；可根据实验情况调整精度信息;


                            //////////////////////////////////////////////////////////
                            /////回到初始的位置





                               /////////////////////////////////////////////
                            hand_eye_pick_place.start_or_restart = true;
                            hand_eye_pick_place.get_pick_pose = false;
                            hand_eye_pick_place.need_refind_marker = false;
                        }                     
                    }                    
                } 
            }
            catch(const std::exception& e)
            {
		        ROS_ERROR("//////////////ERROR/////////////.");
            }
     
        }
        
       ros::spinOnce();
	    loop_rate.sleep();
	}
    //ros::spin();
    return 0;
}
