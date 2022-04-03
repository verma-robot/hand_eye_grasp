#ifndef FIND_OBJECT_H
#define FIND_OBJECT_H

//#include <Python.h>
//#include <numpy/arrayobject.h>

#include<ros/ros.h> //ros标准库头文件  
#include<iostream> //C++标准输入输出库      
#include<cv_bridge/cv_bridge.h>       
#include<sensor_msgs/image_encodings.h>      
#include<image_transport/image_transport.h> 
#include<sensor_msgs/Image.h>
#include<opencv2/core/core.hpp>  
#include<opencv2/highgui/highgui.hpp>  
#include<opencv2/imgproc/imgproc.hpp>  
#include <opencv2/opencv.hpp> 
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include"scholar_kinova_msgs/marker.h"
#include"scholar_kinova_msgs/markerarray.h"   

#include<sensor_msgs/PointCloud2.h>   
#include<sensor_msgs/CameraInfo.h>
#include<geometry_msgs/PointStamped.h>
#include<tf/transform_listener.h>
#include <std_msgs/Header.h>
#include <boost/thread.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vtkAutoInit.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>


using namespace std;
using namespace cv;
using namespace message_filters;
using namespace pcl;

namespace HAND_EYE_GRASP
{        
  

    struct camera_info
    {
        double camera_factor = 1000;
        double camera_cx = 321.798;
        double camera_cy = 239.607;
        double camera_fx = 615.899;
        double camera_fy = 616.468;

        int camera_height;
        int camera_weight;
    };

    struct contourcs_info
    {
       cv::Point2f centor;
       float min_x;
       float min_y;
       float max_x;
       float max_y;     

    };

    struct marker_on_axis
    {
        std::string frame_;
        int id_;

        //cv::Point2f centor;
        //float radius_;

        string color;

        std::vector<cv::Point> contourcs; 
        RotatedRect minRect;
    };

    struct axis_filter_para
    {
        string color_name;

        int iLowH;
        int iHighH;
  
        int iLowS;
        int iHighS;
  
        int iLowV;
        int iHighV ;
         
        int erode_size;
        int threash;
        std::vector<std::vector<cv::Point>> contourcs; 

    };


	class find_object
	{
            public:

                visualization_msgs::MarkerArray markers;
                camera_info camera_info_;
              
                std::vector<marker_on_axis>* marker_on_axis_;
                std::vector<marker_on_axis>* marker_on_axis_filtered_;


                std::string target_frame;
                std::string camera_frame;

                std::string topic_pub_name_;

                std::string colar_image_;
                std::string depth_image_;
                std::string camera_info_data_;

                std::string color_frame_id_;

                int min_point_number;
                float min_point_area;

                bool image_process(cv::Mat color) ;

                bool depth_process(cv::Mat depth_image) ;

               void get_camera_info(const sensor_msgs::CameraInfo::ConstPtr& camera_);

                void combineCallback(const sensor_msgs::ImageConstPtr& pDepth, const sensor_msgs::ImageConstPtr& pImg);  //回调中包含多个消息

            private:  

                ros::NodeHandle nh_; //定义ROS句柄  

                ros::Subscriber camera_info_sub;

                typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> find_point;
                message_filters::Subscriber<sensor_msgs::Image>* depth_sub_ ;             // topic1 输入
                message_filters::Subscriber<sensor_msgs::Image>* image_sub_;   // topic2 输入
                message_filters::Subscriber<sensor_msgs::CameraInfo>* camera_info_sub_;   // topic2 输入
                message_filters::Synchronizer<find_point>* sync_;

                ros::Publisher image_filter_pub_;
                ros::Publisher marker_pub_; //发布目标点像素坐标点               

                bool get_camera_info_;           

            public:
                find_object(const ros::NodeHandle &nodehandle) ;
                ~find_object();              


	};
    
}















#endif /* MOBILE_ROBOT_H */
