//相机原始点云图进行体素滤波，
//参数sub_cloud_topic_name：原始点云话题名称
//参数pub_cloud_topic_name，经体素滤波后重新发布的话题名称
//参数voxel_filtered，体素滤波的粒子尺寸

#ifndef FILTER_CLOUD_H
#define FILTER_CLOUD_H

#include<ros/ros.h> //ros标准库头文件  
#include<iostream> //C++标准输入输出库     

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class filter_cloud
{
  private:
    std::string sub_cloud_topic_name;
    std::string pub_cloud_topic_name;
    float voxel_filtered;

    bool init_;
    ros::NodeHandle node_;

    ros::Publisher pub;
    ros::Subscriber sub;

  public:

    void init();
    void start_filter();
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud);

  public:

    filter_cloud();
    ~filter_cloud(){ };
};

filter_cloud::filter_cloud()
  : node_("~"),voxel_filtered(0.05),init_(false)
{
  init();
}

void filter_cloud::init()
{

  if(!node_.getParam("sub_cloud_topic_name", sub_cloud_topic_name))sub_cloud_topic_name = "/camera/depth_registered/points";
  if(!node_.getParam("pub_cloud_topic_name", pub_cloud_topic_name))pub_cloud_topic_name = "camera_filtered_cloud";
  if(!node_.getParam("voxel_filtered", voxel_filtered))voxel_filtered = 0.05;
  init_ = true;

};

void filter_cloud::start_filter()
{
    if(!init_)init();

  pub = node_.advertise<sensor_msgs::PointCloud2> (pub_cloud_topic_name, 1);

  sub = node_.subscribe<sensor_msgs::PointCloud2> (sub_cloud_topic_name, 1, &filter_cloud::cloud_cb, this); 

}

 void filter_cloud::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud)
 {

    pcl::PCLPointCloud2 pcl_pc;
    pcl::PCLPointCloud2::Ptr cloud_orginal(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    pcl_conversions::toPCL(*cloud, *cloud_orginal);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_orginal);
    sor.setLeafSize(voxel_filtered,voxel_filtered, voxel_filtered);
    sor.filter(*cloud_filtered);

    sensor_msgs::PointCloud2 pub_data;

    pcl_conversions::moveFromPCL(*cloud_filtered , pub_data);
    pub_data.header.frame_id = cloud->header.frame_id;
    pub_data.header.stamp = ros::Time::now();

    pub.publish(pub_data);    

 };
 
int main (int argc, char** argv)
{
  ros::init (argc, argv, "filter_camera_cloud",ros::init_options::NoSigintHandler);
  //ros::NodeHandle nh;
  ros::WallDuration(2.0).sleep();
  filter_cloud filter_cloud;
  filter_cloud.start_filter();
  ros::spin();
 return 0;
}


#endif 