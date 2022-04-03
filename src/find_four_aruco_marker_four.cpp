#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace cv;
using namespace aruco;

cv::Mat inImage;
aruco::CameraParameters camParam;
bool useRectifiedImages, normalizeImageIllumination;
int dctComponentsToRemove;
MarkerDetector mDetector;
vector<Marker> markers;
ros::Subscriber cam_info_sub;
bool cam_info_received;
image_transport::Publisher image_pub;
image_transport::Publisher debug_pub;

ros::Publisher marker_pub;

std::string child_name10;
std::string parent_name;
std::string child_name15;
std::string child_name25;
std::string child_name45;
std::string marker_topic;

std::string reference_frame;

double marker_size;
int marker_id10;
int marker_id15;
int marker_id25;
int marker_id45;

bool rotate_marker_axis_;
tf::StampedTransform rightToLeft;
//t//f::TransformListener _tfListener;

visualization_msgs::MarkerArray marker_array;

visualization_msgs::Marker marker;

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  double ticksBefore = cv::getTickCount();
  static tf::TransformBroadcaster br;
  
  unsigned int marker_id10_number = 0;
  unsigned int marker_id15_number = 0;
  unsigned int marker_id25_number = 0;
  unsigned int marker_id45_number = 0;

  if(cam_info_received)
  {
    ros::Time curr_stamp(ros::Time::now());
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage = cv_ptr->image;

      if(normalizeImageIllumination)
      {
        ROS_WARN("normalizeImageIllumination is unimplemented!");      
      }

      //detection results will go into "markers"
      markers.clear();
      marker_array.markers.clear();
      //Ok, let's detect
      mDetector.detect(inImage, markers, camParam, marker_size, false);
      //for each marker, draw info and its boundaries in the image
      for(unsigned int i=0; i< markers.size(); ++i)
      {
        // only publishing the selected marker
        if ( markers[i].id == marker_id10 )//ID为10的Marker点
        {
          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i], rotate_marker_axis_);

          tf::StampedTransform cameraToReference;
          cameraToReference.setIdentity();
          tf::TransformListener _tfListener;
          if ( reference_frame != parent_name )
          {
                  
              try
              {
                      _tfListener.lookupTransform( reference_frame, parent_name,ros::Time(0), cameraToReference);
              }
              catch ( const tf::TransformException& e)
              {


              }

          }


          transform = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft)  * transform;

          br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name10));//发布坐标变换

          geometry_msgs::Pose poseMsg;

          tf::poseTFToMsg(transform, poseMsg);

          marker_id10_number = marker_id10_number + 1;

          marker.header.frame_id=parent_name;
          marker.ns = "target_id10";
          marker.id = markers[i].id;  
          marker.type = visualization_msgs::Marker::CUBE; 
          marker.action = visualization_msgs::Marker::ADD; 

          marker.header.stamp = ros::Time::now();
          marker.pose =    poseMsg;  

          marker.color.r = 0.0f;
          marker.color.g = 0.0f;
          marker.color.b = 255.0f;
          marker.color.a = 1.0f;

          marker.scale.x = marker_size;
          marker.scale.y = 0.1;
          marker.scale.z = marker_size;

          marker_array.markers.push_back(marker);

        }
        else if ( markers[i].id == marker_id15 )
        {


          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i], rotate_marker_axis_);

          tf::StampedTransform cameraToReference;
          cameraToReference.setIdentity();
          tf::TransformListener _tfListener;
          if ( reference_frame != parent_name )
          {
                  
              try
              {
                      _tfListener.lookupTransform( reference_frame, parent_name,ros::Time(0), cameraToReference);
              }
              catch ( const tf::TransformException& e)
              {


              }

          }


          transform = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft)  * transform;



          br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name15));
          geometry_msgs::Pose poseMsg;
          tf::poseTFToMsg(transform, poseMsg);

          marker_id15_number = marker_id15_number + 1;

          marker.header.frame_id=parent_name;
          marker.ns="target_id15";
          marker.id = markers[i].id;  
          marker.type = visualization_msgs::Marker::CUBE; 
          marker.action = visualization_msgs::Marker::ADD; 

          marker.header.stamp = ros::Time::now();
          marker.pose=    poseMsg;  
          marker.color.r = 0.0f;
          marker.color.g = 0.0f;
          marker.color.b = 255.0f;
          marker.color.a = 1.0f;
          marker.scale.x = marker_size;
          marker.scale.y = 0.1;
          marker.scale.z = marker_size;
          marker_array.markers.push_back(marker);

        }
        else if ( markers[i].id == marker_id25 )
        {
          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i], rotate_marker_axis_);

          tf::StampedTransform cameraToReference;
          cameraToReference.setIdentity();
          tf::TransformListener _tfListener;
          if ( reference_frame != parent_name )
          {
                  
              try
              {
                      _tfListener.lookupTransform( reference_frame, parent_name,ros::Time(0), cameraToReference);
              }
              catch ( const tf::TransformException& e)
              {


              }

          }


          transform = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft)  * transform;

          br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name25));
          geometry_msgs::Pose poseMsg;

          tf::poseTFToMsg(transform, poseMsg);

          marker_id25_number = marker_id25_number + 1;
         
          marker.header.frame_id=parent_name;
         marker.type = visualization_msgs::Marker::CUBE; 
          marker.action = visualization_msgs::Marker::ADD; 

          marker.ns="target_id25";
          marker.id = markers[i].id;  
          marker.header.stamp = ros::Time::now();
          marker.pose=    poseMsg;  
          marker.color.r = 0.0f;
          marker.color.g = 0.0f;
          marker.color.b = 255.0f;
          marker.color.a = 1.0f;
          marker.scale.x = marker_size;
          marker.scale.y = 0.1;
          marker.scale.z = marker_size;
          marker_array.markers.push_back(marker);

        }

        else if ( markers[i].id == marker_id45 )
        {
          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i], rotate_marker_axis_);

          tf::StampedTransform cameraToReference;
          cameraToReference.setIdentity();
          tf::TransformListener _tfListener;
          if ( reference_frame != parent_name )
          {
                  
              try
              {
                      _tfListener.lookupTransform( reference_frame, parent_name,ros::Time(0), cameraToReference);
              }
              catch ( const tf::TransformException& e)
              {


              }

          }


          transform = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft)  * transform;

          br.sendTransform(tf::StampedTransform(transform, curr_stamp,
                                                parent_name, child_name45));
          geometry_msgs::Pose poseMsg;
          tf::poseTFToMsg(transform, poseMsg);
                   marker_id45_number = marker_id45_number + 1;
          marker.header.frame_id=parent_name;
          marker.ns="target_id45";
          marker.type = visualization_msgs::Marker::CUBE; 
          marker.action = visualization_msgs::Marker::ADD; 

          marker.id = markers[i].id;  
          marker.header.stamp = ros::Time::now();
          marker.pose=    poseMsg;  
          marker.color.r = 0.0f;
          marker.color.g = 0.0f;
          marker.color.b = 255.0f;
          marker.color.a = 1.0f;
          marker.scale.x = marker_size;
          marker.scale.y = 0.1;
          marker.scale.z = marker_size;
          marker_array.markers.push_back(marker);

        }
        markers[i].draw(inImage,Scalar(0,0,255),2);

      }



        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());



/*

      if ( markers.size() == 2 )
      {
        float x[2], y[2], u[2], v[2];
        for (unsigned int i = 0; i < 2; ++i)
        {
          ROS_DEBUG_STREAM("Marker(" << i << ") at camera coordinates = ("
                           << markers[i].Tvec.at<float>(0,0) << ", "
                           << markers[i].Tvec.at<float>(1,0) << ", "
                           << markers[i].Tvec.at<float>(2,0));
          //normalized coordinates of the marker
          x[i] = markers[i].Tvec.at<float>(0,0)/markers[i].Tvec.at<float>(2,0);
          y[i] = markers[i].Tvec.at<float>(1,0)/markers[i].Tvec.at<float>(2,0);
          //undistorted pixel
          u[i] = x[i]*camParam.CameraMatrix.at<float>(0,0) + camParam.CameraMatrix.at<float>(0,2);
          v[i] = y[i]*camParam.CameraMatrix.at<float>(1,1) + camParam.CameraMatrix.at<float>(1,2);
        }

        ROS_DEBUG_STREAM("Mid point between the two markers in the image = ("
                         << (x[0]+x[1])/2 << ", " << (y[0]+y[1])/2 << ")");

        //              //paint a circle in the mid point of the normalized coordinates of both markers
        //              cv::circle(inImage,
        //                         cv::Point((u[0]+u[1])/2, (v[0]+v[1])/2),
        //                         3, cv::Scalar(0,0,255), CV_FILLED);


        //compute the midpoint in 3D:
        float midPoint3D[3]; //3D point
        for (unsigned int i = 0; i < 3; ++i )
          midPoint3D[i] = ( markers[0].Tvec.at<float>(i,0) + markers[1].Tvec.at<float>(i,0) ) / 2;
        //now project the 3D mid point to normalized coordinates
        float midPointNormalized[2];
        midPointNormalized[0] = midPoint3D[0]/midPoint3D[2]; //x
        midPointNormalized[1] = midPoint3D[1]/midPoint3D[2]; //y
        u[0] = midPointNormalized[0]*camParam.CameraMatrix.at<float>(0,0) +
            camParam.CameraMatrix.at<float>(0,2);
        v[0] = midPointNormalized[1]*camParam.CameraMatrix.at<float>(1,1) +
            camParam.CameraMatrix.at<float>(1,2);

        ROS_DEBUG_STREAM("3D Mid point between the two markers in undistorted pixel coordinates = ("
                         << u[0] << ", " << v[0] << ")");

        //paint a circle in the mid point of the normalized coordinates of both markers
        cv::circle(inImage,
                   cv::Point(u[0], v[0]),
                   3, cv::Scalar(0,0,255), CV_FILLED);

      }
*/
      marker_pub.publish(marker_array);


      ROS_DEBUG("runtime: %f ms", 1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}

// wait for one camerainfo, then shut down that subscriber
void cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
  //camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
  //cam_info_received = true;
 // cam_info_sub.shutdown();


  camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CamaraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(
        tf::Vector3(
            -msg.P[3]/msg.P[0],
            -msg.P[7]/msg.P[5],
            0.0));

    cam_info_received = true;
    cam_info_sub.shutdown();


}

/*
void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
{
  mDetector.setThresholdParams(config.param1,config.param2);
  normalizeImageIllumination = config.normalizeImage;
  dctComponentsToRemove      = config.dctComponentsToRemove;
}


*/

int main(int argc,char **argv)
{
  ros::init(argc, argv, "aruco_simple");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

//  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> server;
//  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig>::CallbackType f_;
//  f_ = boost::bind(&reconf_callback, _1, _2);
//  server.setCallback(f_);

  normalizeImageIllumination = false;

  nh.param<bool>("image_is_rectified", useRectifiedImages, true);
  ROS_INFO_STREAM("Image is rectified: " << useRectifiedImages);


  std::string refinementMethod;
  nh.param<std::string>("corner_refinement", refinementMethod, "LINES");
  if ( refinementMethod == "SUBPIX" )mDetector.setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);
  else if ( refinementMethod == "HARRIS" )mDetector.setCornerRefinementMethod(aruco::MarkerDetector::HARRIS);
  else if ( refinementMethod == "NONE" )mDetector.setCornerRefinementMethod(aruco::MarkerDetector::NONE); 
  else mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES); 


  nh.param<double>("marker_size", marker_size, 0.1);

  nh.param<int>("marker_id10", marker_id10, 10);
  nh.param<int>("marker_id15", marker_id15, 15);
  nh.param<int>("marker_id25", marker_id25, 25);
  nh.param<int>("marker_id45", marker_id45, 45);

  nh.param<bool>("rotate_marker_axis", rotate_marker_axis_, true);

  nh.param<std::string>("parent_name", parent_name, "");

  nh.param<std::string>("child_name10", child_name10, "");
  nh.param<std::string>("child_name15", child_name15, "");

  nh.param<std::string>("child_name25", child_name25, "");
  nh.param<std::string>("child_name45", child_name45, "");
  nh.param<std::string>("child_name45", child_name45, "");
  nh.param<std::string>("marker_topic", marker_topic, "/marker");

  nh.param<std::string>("reference_frame", reference_frame, "camera_link");

  image_transport::Subscriber image_sub = it.subscribe("/image", 1, &image_callback);
  cam_info_sub = nh.subscribe("/camera_info", 1, &cam_info_callback);

  cam_info_received = false;
  image_pub = it.advertise("result", 1);

  marker_pub=nh.advertise<visualization_msgs::MarkerArray>(marker_topic, 100);

  nh.param<bool>("normalizeImage", normalizeImageIllumination, true);
  nh.param<int>("dct_components_to_remove", dctComponentsToRemove, 2);
  if(dctComponentsToRemove == 0)
    normalizeImageIllumination = false;




  if(parent_name == "" || child_name10 == "" || child_name15 == ""|| child_name25 == ""|| child_name45 == "")
  {
    ROS_ERROR("parent_name and/or child_name was not set!");
    return -1;
  }
  
  ros::spin();
}
