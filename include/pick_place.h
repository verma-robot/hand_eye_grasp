#ifndef PICK_PLACE_H
#define PICK_PLACE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit/kinematic_constraints/utils.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
using namespace std;

namespace Pick_Place_Demo
{
    class pick_place
    {

        public:

            pick_place(ros::NodeHandle nodehandle);
            ~pick_place();

	        moveit::planning_interface::MoveGroupInterface::Plan my_plan; 
            moveit::planning_interface::MoveGroupInterface* group_ ;
            moveit::planning_interface::MoveGroupInterface* gripper_group_ ;

            const double FINGER_MAX = 6400;

            bool start_or_restart;
            bool need_refind_marker;
            bool get_palce_marker_position;
            bool get_pick_pose;
            bool select_new_marker;
            bool find_any_marker;
            std::string pause_;
            int maybe_place_maker_id;

            void init();
            void pick_from_target_position(const geometry_msgs::PoseStamped pose);
            void place_to_target_position();

            void setPositionTolerance(float tplerance);
            void setOrientationTolerance(float tolerance);

            void evaluate_plan();
            bool find_pose_plan(const geometry_msgs::PoseStamped pose);
            bool find_joint_plan(const sensor_msgs::JointState joint);

            bool gripper_action(double gripper_rad);

            geometry_msgs::PoseStamped get_robot_current_pose(){return current_pose;};
            sensor_msgs::JointState get_robot_current_joints_value(){return current_state_;};

            geometry_msgs::PoseStamped get_target_pick_pose(){return target_pick_position;};
            geometry_msgs::PoseStamped get_target_place_pose(){return target_place_position;};

            void add_target();
            void add_attached_obstacle();
            void build_workscene();       
            void clear_workscene();

            float calibrate_grasp_euler();

            geometry_msgs::PoseStamped target_pick_position;
            geometry_msgs::PoseStamped target_place_position;

            geometry_msgs::PoseStamped current_pose;

            sensor_msgs::JointState current_state_;

            bool find_marker_10;;
            float marker_10_x;
            float marker_10_y;
            float marker_10_z;

            bool find_marker_15;;
            float marker_15_x;
            float marker_15_y;
            float marker_15_z;

            bool find_marker_25;;
            float marker_25_x;
            float marker_25_y;
            float marker_25_z;

            bool find_marker_45;;
            float marker_45_x;
            float marker_45_y;
            float marker_45_z;


        private:

            ros::NodeHandle nh_;

            tf2_ros::Buffer tfBuffer;

            tf2_ros::TransformListener tfListener;
            
            std::string arm_group_name;
            std::string gripper_group_name;
            std::string effector_name;
            std::string sub_pose_topic_name;
            std::string sub_joints_topic_name;
            std::string finger_action_name;
            std::string marker_topic_name;
            bool robot_connected;
            std::string robot_type;

            std::string camera_original_frame;
            std::string target_frame;

            bool inited_;
            float cylinder_length;
            float cylinder_radius;

            float table_size_x;
            float table_size_y;
            float table_size_z;
            float table_orignal_x;
            float table_orignal_y;
            float table_orignal_z;

            float wall_size_x;
            float wall_size_y;
            float wall_size_z;
            float wall_orignal_x;
            float wall_orignal_y;
            float wall_orignal_z;

            int plan_max_tries;

            float position_tolerance;
            float oritention_tolerance;

            actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>* finger_client_;

            robot_model::RobotModelPtr robot_model_;

            ros::Subscriber sub_current_robot_pose_;
            ros::Subscriber sub_current_robot_joint_;
            ros::Subscriber sub_marker_pose;

            ros::Publisher pub_aco_;
            ros::Publisher pub_co_;       
            ros::Publisher pub_planning_scene_diff_;

            moveit_msgs::CollisionObject co_;
            moveit_msgs::AttachedCollisionObject aco_;       
            moveit_msgs::PlanningScene planning_scene_msg_;        

            planning_scene::PlanningScenePtr planning_scene_;
            planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

            //tf::Quaternion RQ2;  
            double roll,pitch,yaw;
       
            boost::mutex mutex_robot_state_;
            boost::mutex mutex_robot_pose_;
            boost::mutex mutex_find_marker_;

            boost::mutex mutex_object_marker_pose_;

            int marker_number = 0;

            visualization_msgs::MarkerArray all_object_marker;

            void get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg);//
            void get_current_state(const sensor_msgs::JointStateConstPtr &msg);//
           void get_marker_pose(const visualization_msgs::MarkerArrayConstPtr &msg);//
        
    };

}


#endif 
