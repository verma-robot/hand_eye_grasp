#include <pick_place.h>
#include <ros/console.h>

#include <tf_conversions/tf_eigen.h>


using namespace Pick_Place_Demo;

pick_place::pick_place(ros::NodeHandle nodehandle):nh_(nodehandle),robot_connected(true),inited_(false),position_tolerance(0.05),oritention_tolerance(0.1),start_or_restart(true),need_refind_marker(false),get_pick_pose(false),select_new_marker(false), tfListener(tfBuffer),find_any_marker(false)
{

    if(!nh_.getParam("find_marker_10", find_marker_10))
    {
        find_marker_10 = false;
    }
    if(!nh_.getParam("find_marker_15", find_marker_15))find_marker_15 = false;
    if(!nh_.getParam("find_marker_25", find_marker_25))find_marker_25 = false;
    if(!nh_.getParam("find_marker_45", find_marker_45))find_marker_45 = false;

    if(!nh_.getParam("marker_10_x", marker_10_x))marker_10_x = 0;
    if(!nh_.getParam("marker_10_y", marker_10_y))marker_10_y = 0;
    if(!nh_.getParam("marker_10_z", marker_10_z))marker_10_z = 0;

    if(!nh_.getParam("marker_15_x", marker_15_x))marker_15_x = 0;
    if(!nh_.getParam("marker_15_y", marker_15_y))marker_15_y = 0;
    if(!nh_.getParam("marker_15_z", marker_15_z))marker_15_z = 0;

    if(!nh_.getParam("marker_25_x", marker_25_x))marker_25_x = 0;
    if(!nh_.getParam("marker_25_y", marker_25_y))marker_25_y = 0;
    if(!nh_.getParam("marker_25_z", marker_25_z))marker_25_z = 0;

    if(!nh_.getParam("marker_45_x", marker_45_x))marker_45_x = 0;
    if(!nh_.getParam("marker_45_y", marker_45_y))marker_45_y = 0;
    if(!nh_.getParam("marker_45_z", marker_45_z))marker_45_z = 0;

    if(!nh_.getParam("cylinder_radius", cylinder_radius))cylinder_radius = 0.025;
    if(!nh_.getParam("cylinder_length", cylinder_length))cylinder_length = 0.3;

    if(!nh_.getParam("table_size_x", table_size_x))table_size_x = 2.0;
    if(!nh_.getParam("table_size_y", table_size_y))table_size_y = 2.0;
    if(!nh_.getParam("table_size_z", table_size_z))table_size_z = 0.01;

    if(!nh_.getParam("table_orignal_x", table_orignal_x))table_orignal_x = 0.0;
    if(!nh_.getParam("table_orignal_y", table_orignal_y))table_orignal_y = 0.0;
    if(!nh_.getParam("table_orignal_z", table_orignal_z))table_orignal_z = -0.05;

   if(!nh_.getParam("wall_size_x", wall_size_x))wall_size_x = 2.0;
    if(!nh_.getParam("wall_size_y", wall_size_y))wall_size_y = 0.05;
    if(!nh_.getParam("wall_size_z", wall_size_z))wall_size_z = 2.0;

    if(!nh_.getParam("wall_orignal_x", wall_orignal_x))wall_orignal_x = 0.0;
    if(!nh_.getParam("wall_orignal_y", wall_orignal_y))wall_orignal_y = 1.0;
    if(!nh_.getParam("wall_orignal_z", wall_orignal_z))wall_orignal_z = 0;

    if(!nh_.getParam("plan_max_tries", plan_max_tries))plan_max_tries = 5;

    if(!nh_.getParam("arm_group_name", arm_group_name))arm_group_name = "arm";
    if(!nh_.getParam("gripper_group_name", gripper_group_name))gripper_group_name = "gripper";
    if(!nh_.getParam("effector_name", effector_name))effector_name = "j2n6s300_end_effector";


    if(!nh_.getParam("sub_pose_topic_name", sub_pose_topic_name))sub_pose_topic_name = "/j2n6s300_driver/out/tool_pose";

    if(!nh_.getParam("sub_joints_topic_name", sub_joints_topic_name))sub_joints_topic_name = "/j2n6s300_driver/out/joint_state";
  //  if(!nh_.getParam("robot_type", robot_type))robot_type = "/j2n6s300";

    if(!nh_.getParam("finger_action_name", finger_action_name))finger_action_name = "/j2n6s300_driver/fingers_action/finger_positions";
    if(!nh_.getParam("robot_connected", robot_connected))robot_connected = true;

    if(!nh_.getParam("marker_topic_name", marker_topic_name))marker_topic_name = "all_markers_on_root";
    if(!nh_.getParam("camera_original_frame", camera_original_frame))camera_original_frame = "camera_base_link";
    ros::AsyncSpinner spinner(5);
    spinner.start();
    init();
}

void pick_place::init()
{


   if(robot_connected)
   {
        sub_current_robot_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>(sub_pose_topic_name, 1, &pick_place::get_current_pose, this);//获取机械比当前坐标
        sub_current_robot_joint_ = nh_.subscribe<sensor_msgs::JointState>(sub_joints_topic_name, 1, &pick_place::get_current_state, this);
   }

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();


    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>(finger_action_name, false);
/*
    while(robot_connected && !finger_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the finger action server to come up");
    }
*/
    group_ = new moveit::planning_interface::MoveGroupInterface(arm_group_name);
    gripper_group_ = new moveit::planning_interface::MoveGroupInterface(gripper_group_name);

    group_->setEndEffectorLink(effector_name);
    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 1);//添加障碍物
    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 1);//添加接触物体
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);   //向规划场景中添加物体
    
    sub_marker_pose = nh_.subscribe<visualization_msgs::MarkerArray>(marker_topic_name, 1, &pick_place::get_marker_pose, this);

    inited_ = true;
}


pick_place::~pick_place()
{
    clear_workscene();
    
    sub_current_robot_pose_.shutdown();
    sub_current_robot_joint_.shutdown();
    pub_co_.shutdown();
    pub_aco_.shutdown();
    pub_planning_scene_diff_.shutdown();

    delete finger_client_;
    delete group_;
    delete gripper_group_;

};

void pick_place::get_current_state(const sensor_msgs::JointStateConstPtr &joints)
{
    boost::mutex::scoped_lock lock(mutex_robot_state_);
    current_state_ = *joints;
    //std::cout << "joint" << std::endl;
}
void pick_place::get_current_pose(const geometry_msgs::PoseStampedConstPtr& pose)
{
    boost::mutex::scoped_lock lock(mutex_robot_pose_);
    current_pose = *pose;      
    //    std::cout << "state" << std::endl;
  
}


bool pick_place::gripper_action(double finger_turn)
{
    if (finger_turn < 0)
    {
        finger_turn = 0.0;
    }
    else
    {
        finger_turn = std::min(finger_turn, FINGER_MAX);
    }

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_turn;
    goal.fingers.finger2 = goal.fingers.finger1;
    //goal.fingers.finger3 = goal.fingers.finger1;
    finger_client_->sendGoal(goal);

    if (finger_client_->waitForResult(ros::Duration(5.0)))
    {
        finger_client_->getResult();
        return true;
    }
    else
    {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}

void pick_place::clear_workscene()
{
//remove table
    co_.id = "table";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    //planning_scene_msg_.world.collision_objects.push_back(co_);

//remove wall
    co_.id = "wall";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    //planning_scene_msg_.world.collision_objects.push_back(co_);

    co_.id = "target_cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    //planning_scene_msg_.world.collision_objects.push_back(co_);
//remove attached target
    aco_.object.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_aco_.publish(aco_);

    planning_scene_msg_.world.collision_objects.clear();
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);

}

void pick_place::build_workscene()
{
    co_.header.frame_id = target_pick_position.header.frame_id;
    co_.header.stamp = ros::Time::now();    
    co_.id = "table";
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = table_size_x;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = table_size_y;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = table_size_z;
    co_.primitive_poses[0].position.x = table_orignal_x;
    co_.primitive_poses[0].position.y = table_orignal_y;
    co_.primitive_poses[0].position.z = table_orignal_z;

    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    co_.id = "wall";
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = wall_size_x;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = wall_size_y;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = wall_size_z;
    co_.primitive_poses[0].position.x = wall_orignal_x;
    co_.primitive_poses[0].position.y = wall_orignal_y;
    co_.primitive_poses[0].position.z = wall_orignal_z;

    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;

    pub_planning_scene_diff_.publish(planning_scene_msg_);

    ros::WallDuration(0.1).sleep();
}

void pick_place::add_attached_obstacle()
{

    co_.id = "target_cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    aco_.object.operation = moveit_msgs::CollisionObject::ADD;
    aco_.link_name = "m1n6s200_end_effector";//根据实际情况修改
    aco_.touch_links.push_back( "m1n6s200_end_effector");//根据实际情况修改
    aco_.touch_links.push_back( "m1n6s200_link_finger_1");//根据实际情况修改
    aco_.touch_links.push_back("m1n6s200_link_finger_2");//根据实际情况修改
    //aco_.touch_links.push_back("j2n6s300_link_finger_3");//根据实际情况修改
    aco_.touch_links.push_back( "m1n6s200_link_finger_tip_1");//根据实际情况修改
    aco_.touch_links.push_back("m1n6s200_link_finger_tip_2");//根据实际情况修改
    //aco_.touch_links.push_back("j2n6s300_link_finger_tip_3");//根据实际情况修改
    pub_aco_.publish(aco_);
}

void pick_place::add_target()
{
    //remove target_cylinder
    co_.id = "target_cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    //add target_cylinder
    co_.id = "target_cylinder";
    co_.header.frame_id =target_pick_position.header.frame_id ;
    co_.primitives.resize(all_object_marker.markers.size() + 1);
    co_.primitive_poses.resize(all_object_marker.markers.size() + 1);

    co_.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = cylinder_length;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = cylinder_radius;
    
    co_.primitive_poses[0].position.x = target_pick_position.pose.position.x;
    co_.primitive_poses[0].position.y = target_pick_position.pose.position.y;
    co_.primitive_poses[0].position.z = target_pick_position.pose.position.z ;

    co_.primitive_poses[0].orientation.x = 0;
     co_.primitive_poses[0].orientation.y = 0;
    co_.primitive_poses[0].orientation.z = 0;
    co_.primitive_poses[0].orientation.w = 1;


    for(int i = 1; i < all_object_marker.markers.size() + 1; i++)
    {

        co_.primitives[i].type = shape_msgs::SolidPrimitive::CYLINDER;
        co_.primitives[i].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
        co_.operation = moveit_msgs::CollisionObject::ADD;

        co_.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.1;
        co_.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = cylinder_radius;
    
        co_.primitive_poses[i].position.x = all_object_marker.markers[i].pose.position.x;
        co_.primitive_poses[i].position.y = all_object_marker.markers[i].pose.position.y;
        co_.primitive_poses[i].position.z = all_object_marker.markers[i].pose.position.z;

        co_.primitive_poses[i].orientation.x = 0;
        co_.primitive_poses[i].orientation.y = 0;
        co_.primitive_poses[i].orientation.z = 0;
        co_.primitive_poses[i].orientation.w = 1;

    }  

    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    aco_.object = co_;
    ros::WallDuration(0.1).sleep();
}

void pick_place::setPositionTolerance(float tplerance)
{
    position_tolerance = tplerance;
}

void pick_place::setOrientationTolerance(float tolerance)
{
    oritention_tolerance = tolerance;
}

bool pick_place::find_pose_plan(const geometry_msgs::PoseStamped pose)
{


    bool result_ = false;

    group_->setGoalPositionTolerance(position_tolerance);//目标点位置精度
    group_->setGoalOrientationTolerance(oritention_tolerance);//目标点方位精度
    group_->setPoseTarget(pose);//设置目标
    group_->setNumPlanningAttempts(plan_max_tries);//设置规划求解次数，达到这个次数后返回最优解
    group_->setPlanningTime(10.0);//设置求解最长时间

    result_ = (group_->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);    
    return result_;
}
bool pick_place::find_joint_plan(const sensor_msgs::JointState joint)
{

    bool replan = true;
    int count = 0;
    bool result_ = false;
    group_->setGoalJointTolerance(0.1);//

    std::vector<double> robot_joint;
    robot_joint.resize(6);//6DOF

    robot_joint[0] = 0;
    robot_joint[1] = 0;
    robot_joint[2] = 0;
    robot_joint[3] = 0;
    robot_joint[4] = 0;
    robot_joint[5] = 0;
    int joint_number = joint.position.size();
    if(joint_number >= 6)//获取当前关节值
    {
        for(int i = 0 ;i < joint_number; i++)
        {
            robot_joint[i] = joint.position[i];
        }
    }    
    group_->setJointValueTarget(robot_joint);
    group_->setNumPlanningAttempts(plan_max_tries);//设置规划求解次数，达到这个次数后返回最优解
    group_->setPlanningTime(10.0);//设置求解最长时间

    result_ = (group_->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);    
    return result_;

}

void pick_place::evaluate_plan()
{
 
    group_->execute(my_plan);//执行plan

};

void pick_place::pick_from_target_position(const geometry_msgs::PoseStamped pose)
{

    target_pick_position = pose;
    
    clear_workscene();
    ros::WallDuration(0.1).sleep();

    build_workscene();
    ros::WallDuration(0.1).sleep();

    add_target();
    ros::WallDuration(0.1).sleep();    

}

void pick_place::place_to_target_position()
{

    //target_place_position = *pose;
 
    clear_workscene();
    ros::WallDuration(0.1).sleep();

    build_workscene();
    ros::WallDuration(0.1).sleep();

    //add_attached_obstacle();
    //ros::WallDuration(0.1).sleep();    

}

void pick_place::get_marker_pose(const visualization_msgs::MarkerArrayConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_find_marker_);

    visualization_msgs::MarkerArray marker;

    if(start_or_restart == true && get_pick_pose == false)all_object_marker.markers.clear();//没有确认需要抓取的木块，此时还可以更新；一旦确认了木块，就不可以再更新了
    marker_number = 0;

    visualization_msgs::MarkerArray red_object;
    visualization_msgs::MarkerArray blue_object;
    visualization_msgs::MarkerArray orange_object;
           
    int blue_object_number = 0;
    int all_object_number = 0;
    int red_object_number = 0;
    int orange_object_number = 0;
    if( msg->markers.size() > 0)
    {
        for(int i = 0; i < msg->markers.size(); i++)
        {
            if(msg->markers[i].ns == "blue")//发现蓝色的object
            {
                blue_object.markers.push_back(msg->markers[i]); 
                if(start_or_restart == true && get_pick_pose == false)all_object_marker.markers.push_back(msg->markers[i]); 

                blue_object_number += 1;
                all_object_number += 1;
            }
            else if(msg->markers[i].ns == "red")//发现红色的object
            {
                 red_object.markers.push_back(msg->markers[i]);
                 if(start_or_restart == true && get_pick_pose == false)all_object_marker.markers.push_back(msg->markers[i]);

                red_object_number += 1;
                all_object_number += 1;
            }
            else if(msg->markers[i].ns == "orange")//发现橙色的object
            {
                orange_object.markers.push_back(msg->markers[i]); 
                if(start_or_restart == true && get_pick_pose == false)all_object_marker.markers.push_back(msg->markers[i]);

                orange_object_number += 1;
                all_object_number += 1;
            }
            else if(msg->markers[i].ns == "target_id10")//发现id为10的marker
            {
                marker.markers.push_back(msg->markers[i]); 
                marker_number += 1;
            }
            else if(msg->markers[i].ns == "target_id15")//发现id为15的marker
            {
                marker.markers.push_back(msg->markers[i]); 
                marker_number += 1;
            }
            else if(msg->markers[i].ns == "target_id25")//发现id为25的marker
            {
                marker.markers.push_back(msg->markers[i]);
                marker_number += 1;
            }
            else if(msg->markers[i].ns == "target_id45")//发现id为45的marker
            {
                marker.markers.push_back(msg->markers[i]);
                marker_number += 1;
             }
        }
    }

    if(start_or_restart && get_pick_pose == false && all_object_number >0 )
    {
        std::cout << "Are you ready ?  Y : yes; N : no "<<std::endl;

        std::cin >> pause_;
        ros::WallDuration(0.5).sleep();
        if (pause_ == "Y"  || pause_ == "y")
        {   

            std::cout << "/////////////////////////////////////////////////////  "  << std::endl; 
            std::cout << "Find Total Object Number:  " <<all_object_number << std::endl; 

            std::cout << "Find Blue objects number:  " << blue_object_number << "  , their Id is lists as below :  "<< std::endl; 
            for(int i = 1; i <= blue_object_number; i++)std::cout <<blue_object.markers[i-1].id << "  ,   " << std::endl; 

            std::cout << "Find Red objects number:  "  << red_object_number << "  , their Id is lists as below :  " << std::endl; 
            for(int i = 1; i <= red_object_number; i++)std::cout <<red_object.markers[i-1].id << "  ,   " << std::endl; 

            std::cout << "Find Orange objects number:  " << orange_object_number << "  , their Id is lists as below :  " << std::endl; 
            for(int i = 1; i <= orange_object_number; i++)std::cout <<orange_object.markers[i-1].id << "  ,   " << std::endl; 
           
            std::cout << "/////////////////////////////////////////////////////  "  << std::endl; 
            std::cout << "/////////////////////////////////////////////////////  "  << std::endl; 

            std::cout << "What colar Object do you want to grasp ?  B : blue; R : Red; O: Orange "<<std::endl;
            std::cin >> pause_;
            ros::WallDuration(0.5).sleep();

            bool object_select = false;

            if ((pause_ == "B"  || pause_ == "b") && blue_object_number != 0)
            {
                std::string bbrroo;        
                std::cout << "Which Blue Object do you want to pick ?  "<<std::endl;     

                std::cout <<"Blue Objects Id is lists as below :  "<< std::endl; 
               for(int i = 1; i <= blue_object_number; i++)std::cout <<blue_object.markers[i-1].id << "  ,   " << std::endl; 

                std::cin >> pause_;
                ros::WallDuration(0.5).sleep();

                for(int i = 0; i < blue_object_number; i++)
                {
                    bbrroo=std::to_string(blue_object.markers[i].id);
                    if(pause_ == bbrroo)
                    {

                        target_pick_position.header.frame_id = blue_object.markers[i].header.frame_id;
                        target_pick_position.header.stamp = ros::Time::now();
                        target_pick_position.pose = blue_object.markers[i].pose;
                        object_select = true;
                    }                  
                }  
            }
            else if ((pause_ == "R"  || pause_ == "r") && red_object_number != 0)
            {
                std::string bbrroo;
                std::cout << "Which Red Object do you want to pick ?  "<<std::endl;     

                std::cout <<"  Red Objects  Id is lists as below :  " << std::endl; 
                for(int i = 1; i <= red_object_number; i++)std::cout <<red_object.markers[i-1].id << "  ,   " << std::endl; 

                std::cin >> pause_;
                ros::WallDuration(0.5).sleep();

                for(int i = 0; i < red_object_number; i++)
                {
                    bbrroo=std::to_string(red_object.markers[i].id);
                    if(pause_ == bbrroo)
                    {

                        target_pick_position.header.frame_id = red_object.markers[i].header.frame_id;
                        target_pick_position.header.stamp = ros::Time::now();
                        target_pick_position.pose = red_object.markers[i].pose;
                        object_select = true;
                    }                   
                } 
            }
            else if ((pause_ == "O"  || pause_ == "o") && orange_object_number != 0)
            {
                std::string bbrroo;

                std::cout << "Which Orange Object do you want to pick ?  "<<std::endl;     

                std::cout <<  "  Orange Objects Id is lists as below :  " << std::endl; 
                for(int i = 1; i <= orange_object_number; i++)std::cout <<orange_object.markers[i-1].id << "  ,   " << std::endl; 
           
                std::cin >> pause_;
                ros::WallDuration(0.5).sleep();

                for(int i = 0; i < orange_object_number; i++)
                {
                    bbrroo=std::to_string(orange_object.markers[i].id);
                    if(pause_ == bbrroo)
                    {

                        target_pick_position.header.frame_id = orange_object.markers[i].header.frame_id;
                        target_pick_position.header.stamp = ros::Time::now();
                        target_pick_position.pose = orange_object.markers[i].pose;
                        object_select = true;
                    }
                }                
            }
            else object_select = false;
            bool marker_select = false;

            if(object_select)
            {
                //std::cout <<"find marker : "<<(int16_t) find_marker_10 << std::endl;
             

                if(!find_marker_10 && !find_marker_15 && !find_marker_25 && !find_marker_45)//没有找到marker
                {
                    std::cout << " We din't find any marker................................"<<std::endl;
                }
                else
                {
                    std::cout << "/////////////////////////////////////////////////////  "  << std::endl; 
                    if(find_marker_10 )std::cout << "We find marker 10,  if you want to place on it, please press A or a " << std::endl;
                    if(find_marker_15)std::cout << "We find marker 15,  if you want to place on it, please press C or c"  << std::endl;
                    if(find_marker_25)std::cout << "We find marker 25,  if you want to place on it, please press D or d"  << std::endl;
                    if(find_marker_45)std::cout << "We find marker 45,  if you want to place on it, please press F or f"  << std::endl;
                    std::cout << "/////////////////////////////////////////////////////  "  << std::endl; 
                    std::cout << "/////////////////////////////////////////////////////  "  << std::endl; 

                    std::cin >> pause_;
                    ros::WallDuration(0.5).sleep();

                    if ((pause_ == "A" || pause_ == "a" ) && find_marker_10)
                    {
                        maybe_place_maker_id = 10;
                        marker_select = true;
                    }
                    else if ((pause_ == "C" || pause_ == "c") && find_marker_15 )
                   {
                        maybe_place_maker_id = 15;
                        marker_select = true;
                    }
                    else if ((pause_ == "D" || pause_ == "d" ) && find_marker_25)
                    {
                        maybe_place_maker_id = 25;
                        marker_select = true;
                    }
                    else if ((pause_ == "F" || pause_ == "f" ) && find_marker_45)
                    {
                        maybe_place_maker_id = 45;
                        marker_select = true;
                    }
                    else 
                    {
                        marker_select = false;
                        std::cout << "we didn't find the marker you select  "  << std::endl; 
                    }                
                    if(marker_select)std::cout << "Yes ,Thanks, You have select the marker : "<< maybe_place_maker_id << std::endl;             
                }
            }
            if(marker_select && object_select )
            {
                start_or_restart = false;
                get_pick_pose = true;
            }
            else
            {
                start_or_restart = true;
                get_pick_pose = false;
                marker_select = false;
                object_select = false;

            }
        }
    }
    else if(need_refind_marker && marker_number > 0 && get_pick_pose)
    {
        get_palce_marker_position = false;     
        find_any_marker = true;
        if(select_new_marker == false)
        {

            std::cout << "/////////////////////////////////////////////////////  "  << std::endl; 
            std::cout << "Find target marker number:  " << marker_number  << "  , their Id is lists as below :  " << std::endl; 
            for(int i = 1; i <= marker_number; i++)std::cout <<marker.markers[i-1].id << "  ,   " << std::endl; 
            std::cout << "/////////////////////////////////////////////////////  "  << std::endl; 
        }
        else
        {   
            std::cout << "/////////////////////////////////////////////////////  "  << std::endl; 
            std::cout << "Find target marker number:  " << marker_number  << "  , their Id is lists as below :  " << std::endl; 
            for(int i = 1; i <= marker_number; i++)std::cout <<marker.markers[i-1].id << "  ,   " << std::endl; 
            std::cout << "Which one do you want place:   "  << std::endl; 
            std::cout << "/////////////////////////////////////////////////////  "  << std::endl; 

            std::cin >> pause_;
            ros::WallDuration(0.5).sleep();

            if (pause_ == "A" || pause_ == "a" )
            {
                maybe_place_maker_id = 10;
            }
            else if (pause_ == "C" || pause_ == "c" )
            {
                maybe_place_maker_id = 15;
            }
            else if (pause_ == "D" || pause_ == "d" )
            {
                maybe_place_maker_id = 25;
            }
            else if (pause_ == "F" || pause_ == "f" )
            {
                maybe_place_maker_id = 45;
            }

                std::cout << "Yes ,Thanks, You have select the marker : "<< maybe_place_maker_id << std::endl;             
        }
        for(int i = 0; i < marker.markers.size(); i++)
        {
            if(marker.markers[i].id == maybe_place_maker_id)
            {
                target_place_position.header.frame_id = marker.markers[i].header.frame_id;
                target_place_position.header.stamp = ros::Time::now();
                target_place_position.pose = marker.markers[i].pose;
                get_palce_marker_position = true;
            }
            else  get_palce_marker_position = false;
        }
        
       if(get_palce_marker_position)
       {
           need_refind_marker = false;
           std::cout << "Find the marker position... "  << std::endl; 

       }
       else  
       {
            std::cout << "We didn't find the marker you selected.............................."  << std::endl; 
            need_refind_marker = true;
       }
    }
    else if(need_refind_marker && marker_number <= 0 && get_pick_pose)//需要寻找码时，没有找到任何一个码
    {

      //  std::cout << "We can't find  nay marker, are you sure you are ready?"<<std::endl;
        find_any_marker = false;
        need_refind_marker = true;
        get_palce_marker_position = false;

    }
}

float pick_place::calibrate_grasp_euler()
{

    geometry_msgs::TransformStamped transformStamped;
    float oula;

    geometry_msgs::PoseStamped cemara_zero_position, camera_zero_position_on_target_frame;
    cemara_zero_position.header.frame_id = camera_original_frame;
    cemara_zero_position.pose.position.x = 0;
    cemara_zero_position.pose.position.y = 0;
    cemara_zero_position.pose.position.z = 0;
    cemara_zero_position.pose.orientation.x = 0;
    cemara_zero_position.pose.orientation.y = 0;
    cemara_zero_position.pose.orientation.z = 0;
    cemara_zero_position.pose.orientation.w = 0;

    try
    {
        transformStamped = tfBuffer.lookupTransform(target_pick_position.header.frame_id, camera_original_frame,  ros::Time::now(),ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }

    try
    {
        tfBuffer.transform(cemara_zero_position,camera_zero_position_on_target_frame,target_pick_position.header.frame_id,ros::Duration(3.0));
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }  

    float delt_x =  - camera_zero_position_on_target_frame.pose.position.x + target_pick_position.pose.position.x;
    float delt_y =  - camera_zero_position_on_target_frame.pose.position.y + target_pick_position.pose.position.y;

    if(fabs(delt_x)  < 0.01 && fabs(delt_y) < 0.01)//摄像头在物体的正上方
    {
        return atan2( target_pick_position.pose.position.y, target_pick_position.pose.position.x);
    }
    else//摄像头不在物体的正上方
    {
        float arm_angle = atan2( target_pick_position.pose.position.y, target_pick_position.pose.position.x);
        float camera_angle = atan2(delt_y,delt_x);

        if(arm_angle >= 0 && arm_angle <= 1.508)
        {
            if(camera_angle >= 0 && camera_angle <= 1.5708)return camera_angle;
            else if(camera_angle >= -1.5708 && camera_angle <0)return camera_angle + 1.5708;
            else if(camera_angle <= 3.1416 && camera_angle > 1.5708)return camera_angle - 1.5708;
            else if(camera_angle >= -3.1416 && camera_angle < - 1.5708)return camera_angle + 3.1416;
        }
        else if(arm_angle <0 && camera_angle > -1.5708)
        {
            if(camera_angle >= 0 && camera_angle <= 1.5708)return camera_angle - 1.5708;
            else if(camera_angle >= -1.5708 && camera_angle <0)return camera_angle;
            else if(camera_angle <= 3.1416 && camera_angle > 1.5708)return camera_angle - 3.1416;
            else if(camera_angle >= -3.1416 && camera_angle < - 1.5708)return camera_angle + 1.5708;
        }
        else if(arm_angle >= -3.1416 && arm_angle <= -1.5708)
        {

            if(camera_angle >= 0 && camera_angle <= 1.5708)return camera_angle - 3.1416;
            else if(camera_angle >= -1.5708 && camera_angle <0)return camera_angle -1.5708;
            else if(camera_angle <= 3.1416 && camera_angle > 1.5708)return -camera_angle;
            else if(camera_angle >= -3.1416 && camera_angle < - 1.5708)return camera_angle;
        }
        else if(arm_angle <= 3.1416 && arm_angle > 1.5708)
        {

            if(camera_angle >= 0 && camera_angle <= 1.5708)return camera_angle +1.5708;
            else if(camera_angle >= -1.5708 && camera_angle <0)return camera_angle +3.1416;
            else if(camera_angle <= 3.1416 && camera_angle > 1.5708)return camera_angle;
            else if(camera_angle >= -3.1416 && camera_angle < - 1.5708)return -camera_angle;
        }      
    }
}








