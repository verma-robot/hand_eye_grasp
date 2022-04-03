//将识别到色块位置转换到目标坐标下上,并过滤掉不在范围内的色块
//target_frame：目标坐标系名称
//all_marker_on_target_frame：重新发布的主题名称
//all_marker_on_camera_frame：原坐标系下的主题名称
//tf_wait_max_time：可以容忍的两个坐标系变换的tf时间
//min_distance：色块在目标坐标系最近容许距离
//max_distance：色块在目标坐标系的最远容许距离
#ifndef TRANSLATE_TO_TARGET_FRAME_H
#define TRANSLATE_TO_TARGET_FRAME_H

#include<ros/ros.h> //ros标准库头文件  
#include<iostream> //C++标准输入输出库     
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


class transe_markers_to_target_frame
{
  private:
    std::string target_frame;
    std::string all_marker_on_target_frame;
    std::string all_marker_on_camera_frame;
    int tf_wait_max_time;
    float min_distance;
    float max_distance;
    tf2_ros::Buffer tfBuffer;

    tf2_ros::TransformListener tfListener;
    ros::NodeHandle node_;

    ros::Publisher pub;
    ros::Subscriber sub;
    bool init_;

  public:

    void init();
    void start_trans_();
    void cloud_cb(const visualization_msgs::MarkerArrayConstPtr& position);

  public:

    transe_markers_to_target_frame(ros::NodeHandle nodehandle);
    ~transe_markers_to_target_frame(){ };
};

transe_markers_to_target_frame::transe_markers_to_target_frame(ros::NodeHandle nodehandle)
  : node_(nodehandle), tfListener(tfBuffer),tf_wait_max_time(1.0),init_(false)
{

  if(!node_.getParam("target_frame", target_frame))target_frame = "root";
  if(!node_.getParam("all_marker_on_target_frame", all_marker_on_target_frame))all_marker_on_target_frame = "all_markers_objects_on_root";
  if(!node_.getParam("all_marker_on_camera_frame", all_marker_on_camera_frame))all_marker_on_camera_frame = "markers_object_on_camera_frame";
  if(!node_.getParam("tf_wait_max_time", tf_wait_max_time))tf_wait_max_time = 1.0;
  if(!node_.getParam("min_distance", min_distance))min_distance = 0.2;
  if(!node_.getParam("max_distance", max_distance))max_distance = 0.8;
  init();
}

void transe_markers_to_target_frame::init()
{

  init_ = true;

};

void transe_markers_to_target_frame::start_trans_()
{

  if(!init_)init();
  pub = node_.advertise<visualization_msgs::MarkerArray> (all_marker_on_target_frame, 1);

  sub = node_.subscribe<visualization_msgs::MarkerArray> (all_marker_on_camera_frame, 1, &transe_markers_to_target_frame::cloud_cb, this); 
}


 void transe_markers_to_target_frame::cloud_cb(const visualization_msgs::MarkerArrayConstPtr& position)
 {
   
   visualization_msgs::MarkerArray wait_to_pub;

   for(int i = 0; i < position->markers.size() ; i++)
   {
      geometry_msgs::PointStamped  position_on_camera_frame_temp, position_on_target_frame_temp;
      visualization_msgs::Marker marker;
      marker = position->markers[i];

      position_on_camera_frame_temp.header = position->markers[i].header;
      position_on_camera_frame_temp.point = position->markers[i].pose.position;

      geometry_msgs::TransformStamped transformStamped;

      try
      {
        transformStamped = tfBuffer.lookupTransform(target_frame, position->markers[i].header.frame_id,  ros::Time::now(),ros::Duration(tf_wait_max_time));
      }
      catch (tf2::TransformException &ex) 
      {
        ROS_WARN("%s",ex.what());
        continue;
      }
      try
      {
        tfBuffer.transform(position_on_camera_frame_temp,position_on_target_frame_temp,target_frame, ros::Duration(tf_wait_max_time));
        
        float distance = position_on_target_frame_temp.point.x * position_on_target_frame_temp.point.x + position_on_target_frame_temp.point.y * position_on_target_frame_temp.point.y + position_on_target_frame_temp.point.z + position_on_target_frame_temp.point.z;
        distance = sqrt(distance);
        //std::cout << distance << std::endl;
        if(distance < min_distance || distance > max_distance)continue;
        else
        {      
          marker.header.frame_id= position_on_target_frame_temp.header.frame_id;
          marker.header.stamp = ros::Time::now();
          marker.pose.position = position_on_target_frame_temp.point;
          wait_to_pub.markers.push_back(marker);
        }

      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
        continue;
      }   
   }
   
   pub.publish(wait_to_pub);

 };
 
int main (int argc, char** argv)
{
  ros::init (argc, argv, "translate_markers_to_target_frame",ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");
  ros::WallDuration(2.0).sleep();

  transe_markers_to_target_frame transe_all_markers_to_target_frame(nh); 
  transe_all_markers_to_target_frame.start_trans_();


  ros::spin();
  return 0;
}


#endif
