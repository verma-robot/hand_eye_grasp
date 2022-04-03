//实现三种色块在相机坐标系下的识别，木块为圆柱体，直径5cm，高度10cm
//参数color_image：相机发布的RGB图的主题名称
//depth_image：相机发布的深度图主题名称
//camera_info_data：发布相机内参的主题名称
//topic_pub_name：发布识别出来的色块颜色和位置的主题名称
//min_point_number：
//min_point_area：滤波参数，
#include <find_object.h>
#include <vector>

namespace HAND_EYE_GRASP
{

    find_object::find_object(const ros::NodeHandle &nodehandle): nh_(nodehandle),get_camera_info_(false)
   { 
          nh_.getParam("color_image",colar_image_);
          nh_.getParam("depth_image",depth_image_);
          nh_.getParam("camera_info_data",camera_info_data_);
          nh_.getParam("topic_pub_name",topic_pub_name_);
          nh_.getParam("min_point_number",min_point_number);                   
          nh_.getParam("min_point_area",min_point_area);                   


           get_camera_info_ = false;

           marker_on_axis_ = new std::vector<marker_on_axis>();
           marker_on_axis_filtered_ = new std::vector<marker_on_axis>();

           depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_,depth_image_, 10);
           image_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_, colar_image_, 10);
   
          sync_ = new  message_filters::Synchronizer<find_point>(find_point(10), *depth_sub_, *image_sub_);
          sync_->registerCallback(boost::bind(&find_object::combineCallback,this, _1, _2));

           marker_pub_=nh_.advertise<visualization_msgs::MarkerArray>(topic_pub_name_,1);    
           image_filter_pub_ = nh_.advertise<sensor_msgs::Image>("/image_converter_filter", 10);
          camera_info_sub=nh_.subscribe (camera_info_data_, 1, &find_object::get_camera_info,this); 

   }

   find_object::~find_object()
   {
       if(marker_on_axis_) delete marker_on_axis_;
       if(marker_on_axis_filtered_)delete marker_on_axis_filtered_;
       if(depth_sub_)delete depth_sub_;
       if(image_sub_)delete image_sub_;
       if(sync_)delete sync_;
   }

    void find_object::get_camera_info(const sensor_msgs::CameraInfo::ConstPtr& camera_)
    {

        if(!get_camera_info_)
        {
            camera_info_.camera_factor = 1000;
            camera_info_.camera_cx = camera_ -> P[2];
            camera_info_.camera_cy = camera_ -> P[6];
            camera_info_.camera_fx = camera_ -> P[0];
            camera_info_.camera_fy = camera_ -> P[5];
            camera_info_.camera_height =camera_ ->height;
            camera_info_.camera_weight=camera_->width;

            get_camera_info_ = true;
            camera_info_sub.shutdown();
        }
    }   


    void find_object::combineCallback(const sensor_msgs::ImageConstPtr& pDepth, const sensor_msgs::ImageConstPtr& pImg)  //回调中包含多个消息
    {

        markers.markers.clear();
       
        marker_on_axis_->clear();
        marker_on_axis_filtered_->clear();
    
        if(get_camera_info_ == false)return;    

        bool deal_color_image = false;
        bool deal_depth_image  =  false;

        cv::Mat orginal_color_image_;
        cv::Mat orginal_depth_image_;

        cv_bridge::CvImagePtr cv_ptr; 
        cv_bridge::CvImagePtr depth_image_ptr;
      
        try  
        {  
            cv_ptr =  cv_bridge::toCvCopy(pImg, sensor_msgs::image_encodings::BGR8);
            depth_image_ptr = cv_bridge::toCvCopy(pDepth, sensor_msgs::image_encodings::TYPE_32FC1); 

            color_frame_id_ = (*pImg).header.frame_id;

            if(!(cv_ptr->image.empty()) && !(depth_image_ptr->image.empty()))
            {
                orginal_color_image_ = cv_ptr->image.clone();
                orginal_depth_image_ = depth_image_ptr->image.clone();

                deal_color_image = image_process(orginal_color_image_);
            
                if(deal_color_image)
                {
                    deal_depth_image = depth_process(orginal_depth_image_);

                    if(deal_depth_image)
                    {
                        if(marker_on_axis_filtered_->size() > 0)
                        {

                            for(std::vector<marker_on_axis>::iterator it = marker_on_axis_filtered_->begin() ; it != marker_on_axis_filtered_->end() ; it++)
                            {

                                std::string  put_number_on_img =   std::to_string(it->id_);
                                if(it->color =="blue")
                                {
                                    cv::circle( orginal_color_image_, cvPoint(it->minRect.center.x, it->minRect.center.y), 10, CV_RGB(0,0,255), 5 );
	                                cv::putText(orginal_color_image_, put_number_on_img, cv::Point(it->minRect.center.x, it->minRect.center.y), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,0,0),1,8,false);//写字
                                }
                                else if(it->color =="red")
                                {
                                    cv::circle( orginal_color_image_, cvPoint(it->minRect.center.x, it->minRect.center.y), 10, CV_RGB(255,0,0), 5 );
                                    cv::putText(orginal_color_image_,put_number_on_img, cv::Point(it->minRect.center.x, it->minRect.center.y), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,0,255),1,8,false);//写字
                                }
                                else if(it->color  =="orange")
                                {
                                    cv::circle( orginal_color_image_, cvPoint(it->minRect.center.x, it->minRect.center.y), 10, CV_RGB(255,255,0), 5 );
                                    cv::putText(orginal_color_image_, put_number_on_img, cv::Point(it->minRect.center.x, it->minRect.center.y), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,0),1,8,false);//写字
                                }
                            }
                        } 
                    }                
                }

                sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", orginal_color_image_).toImageMsg();
                msg_->header.frame_id = color_frame_id_;
                msg_->header.stamp = ros::Time::now();
                image_filter_pub_.publish(*msg_);
            }
            if(markers.markers.size() > 0 )marker_pub_.publish(markers);
        }  
        catch(cv_bridge::Exception& e)  
        {  
            ROS_ERROR("cv_bridge exception: %s", e.what());  
        }  
    
   }
 
   bool find_object::image_process(cv::Mat color_image)
   {
        axis_filter_para red_filter_;
        axis_filter_para blue_filter_;
        axis_filter_para orange_filter;

        blue_filter_.color_name = "blue";
        blue_filter_.iLowH = 100;
        blue_filter_.iHighH = 140;
  
        blue_filter_.iLowS = 90;
        blue_filter_.iHighS = 255;
  
        blue_filter_.iLowV = 0;
        blue_filter_.iHighV = 255;         

        blue_filter_.threash = 40;
        blue_filter_.erode_size = 20;

        std::vector<cv::Mat> hsvSplit; 

        std::vector<std::vector<Vec4i>> hiearchy; 
        cv::Mat img_out;

        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(blue_filter_.erode_size,blue_filter_.erode_size));

        cv::Mat blue_imgThresholded;

        cv::cvtColor(color_image, img_out, CV_BGR2HSV); 

        split(img_out, hsvSplit);           
        equalizeHist(hsvSplit[2],hsvSplit[2]);
        merge(hsvSplit,img_out);      

        cv::inRange(img_out, cv::Scalar(blue_filter_.iLowH, blue_filter_.iLowS, blue_filter_.iLowV), cv::Scalar(blue_filter_.iHighH, blue_filter_.iHighS, blue_filter_.iHighV), blue_imgThresholded);//二值化
 
        cv::erode(blue_imgThresholded, blue_imgThresholded, element);

        element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(blue_filter_.erode_size ,blue_filter_.erode_size ));
        cv::dilate(blue_imgThresholded, blue_imgThresholded, element);
     
        cv::Canny(blue_imgThresholded, blue_imgThresholded, blue_filter_.threash, blue_filter_.threash*2, 3);

        cv::findContours(blue_imgThresholded, blue_filter_.contourcs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());//寻找边界
      
        if(blue_filter_.contourcs.size()>0)
        {
            for(size_t i = 0; i < blue_filter_.contourcs.size(); i++)
            {

                RotatedRect minRect;
                marker_on_axis blue_object;
                float aread_blue = contourArea(cv::Mat(blue_filter_.contourcs[i]),true);
                if(fabs(aread_blue) < min_point_area)continue;

                blue_object.minRect = cv::minAreaRect(cv::Mat(blue_filter_.contourcs[i]));

                blue_object.id_ = i ;
                blue_object.color = blue_filter_.color_name;
                blue_object.frame_ = color_frame_id_;
                blue_object.contourcs = blue_filter_.contourcs[i];
                marker_on_axis_->push_back(blue_object);

             }
        }

        red_filter_.color_name = "red";
        red_filter_.iLowH = 156;//0
        red_filter_.iHighH = 180;//5
  
        red_filter_.iLowS = 40;
        red_filter_.iHighS = 255;
  
        red_filter_.iLowV = 0;
        red_filter_.iHighV = 255;

        red_filter_.threash = 20;
        red_filter_.erode_size = 20;//腐蚀参数

        cv::Mat red_imgThresholded;
        cv::Mat red_imgThresholded_two;

        cv::inRange(img_out, cv::Scalar(red_filter_.iLowH, red_filter_.iLowS, red_filter_.iLowV), cv::Scalar(red_filter_.iHighH, red_filter_.iHighS, red_filter_.iHighV), red_imgThresholded);
 

        red_filter_.iLowH = 0;//0
        red_filter_.iHighH = 1;//5  

        cv::inRange(img_out, cv::Scalar(red_filter_.iLowH, red_filter_.iLowS, red_filter_.iLowV), cv::Scalar(red_filter_.iHighH, red_filter_.iHighS, red_filter_.iHighV), red_imgThresholded_two);

       cv::Mat red_image_out;
        cv::addWeighted(red_imgThresholded, 0.5, red_imgThresholded_two , 0.5, 0.0, red_image_out);

        element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(red_filter_.erode_size,red_filter_.erode_size));

        cv::erode(red_image_out, red_image_out, element);
        element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(red_filter_.erode_size ,red_filter_.erode_size ));
        cv::dilate(red_image_out, red_image_out, element);//膨胀
  
        cv::Canny(red_image_out,red_image_out, red_filter_.threash, red_filter_.threash*2, 3);
        cv::findContours(red_image_out, red_filter_.contourcs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0,0));//xunzhao bianjie

        if(red_filter_.contourcs.size()>0)
        {
            for(size_t i=0;i<red_filter_.contourcs.size();i++)
            {

               // RotatedRect minRect;
                marker_on_axis red_object;

                float aread_red = contourArea(cv::Mat(red_filter_.contourcs[i]),true);
                if(fabs(aread_red) < min_point_area)continue;

                red_object.minRect = cv::minAreaRect(cv::Mat(red_filter_.contourcs[i]));

                red_object.id_ = i ;
                red_object.color = red_filter_.color_name;
                red_object.frame_ = color_frame_id_;
                red_object.contourcs = red_filter_.contourcs[i];
                marker_on_axis_->push_back(red_object);

            }
        }
        
/////////////////////orange/////////////////
        orange_filter.color_name = "orange";
        orange_filter.iLowH = 6;//0
        orange_filter.iHighH = 30;//5
  
        orange_filter.iLowS = 100;
        orange_filter.iHighS = 255;
  
        orange_filter.iLowV = 60;
        orange_filter.iHighV = 255;

        orange_filter.threash = 40;
        orange_filter.erode_size = 20;//腐蚀参数

        cv::Mat orange_imgThresholded;

        cv::inRange(img_out, cv::Scalar(orange_filter.iLowH, orange_filter.iLowS, orange_filter.iLowV), cv::Scalar(orange_filter.iHighH, orange_filter.iHighS, orange_filter.iHighV), orange_imgThresholded);
 
        element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(orange_filter.erode_size,orange_filter.erode_size));

        cv::morphologyEx(orange_imgThresholded,orange_imgThresholded,cv::MORPH_OPEN,element);

        cv::Canny(orange_imgThresholded, orange_imgThresholded, orange_filter.threash, orange_filter.threash*2, 3);
        cv::findContours(orange_imgThresholded, orange_filter.contourcs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0,0));//xunzhao bianjie

        if(orange_filter.contourcs.size()>0)
        {               

            for(size_t i = 0; i < orange_filter.contourcs.size(); i++)
            {
                marker_on_axis orange_object;

                float aread_orange = contourArea(cv::Mat(orange_filter.contourcs[i]),true);
                if(fabs(aread_orange) < min_point_area)continue;

                orange_object.minRect = cv::minAreaRect(cv::Mat(orange_filter.contourcs[i]));

                orange_object.id_ = i ;
                orange_object.color = orange_filter.color_name;
                orange_object.frame_ = color_frame_id_;
                orange_object.contourcs = orange_filter.contourcs[i];
                marker_on_axis_->push_back(orange_object);

            }
        } 
        if(marker_on_axis_->size() > 0)return true;
        else return false;
   }

   bool find_object::depth_process(cv::Mat depth_image)
   {
       if(marker_on_axis_->size() == 0)return false;
       else
       {
            marker_on_axis_filtered_->clear();
            markers.markers.clear();

            int red_number = 0;
            int blue_number = 0;
            int orange_number = 0;
            for(std::vector<marker_on_axis>::iterator it = marker_on_axis_->begin() ; it != marker_on_axis_->end() ; it++)
            {                
        
               CvPoint2D32f Corner[4];
               cvBoxPoints(it->minRect,Corner);
               float min_x,min_y,max_y,max_x;

                min_x = min(Corner[0].x, Corner[1].x);
                min_x = min(min_x, Corner[2].x);
                min_x = min(min_x, Corner[3].x);

                max_x = max(Corner[0].x, Corner[1].x);
                max_x = max(max_x,Corner[2].x);
                max_x = max(max_x,Corner[3].x);

                min_y = min(Corner[0].y, Corner[1].y);
                min_y = min(min_y ,Corner[2].y);
                min_y = min(min_y ,Corner[3].y);

                max_y = max(Corner[0].y,Corner[1].y);
                max_y= max( max_y,Corner[2].y);
                max_y = max( max_y,Corner[3].y);            

                int centor_x,centor_y,min_x_,min_y_,max_x_,max_y_;

                centor_x = floor(it->minRect.center.x);
                centor_y = floor(it->minRect.center.y);

                if(centor_x< 0)centor_x = 0;
                else if(centor_x>camera_info_.camera_weight)centor_x = camera_info_.camera_weight;

                if(centor_y<0)centor_y = 0;
                else if(centor_y > camera_info_.camera_height)centor_y = camera_info_.camera_height;

                min_x_ = floor(min_x);
                if(min_x_< 0)min_x_ = 0;
                else if(min_x_>camera_info_.camera_weight)min_x_ = camera_info_.camera_weight;
 
                max_x_ = floor(max_x);
                if(max_x_< 0)max_x_ = 0;
                else if(max_x_>camera_info_.camera_weight)max_x_ = camera_info_.camera_weight;               
 
                min_y_ = floor(min_y);
                if(min_y_< 0)min_y_ = 0;
                else if(min_y_>camera_info_.camera_height)min_y_ = camera_info_.camera_height;
               
                max_y_ = floor(max_y);
                if(max_y_< 0)max_y_ = 0;
                else if(max_y_>camera_info_.camera_height)max_y_ = camera_info_.camera_height;
               
               if(min_x_ >= max_x_ || min_y_ >= max_y_)continue;

                if(isnan(depth_image.ptr<float>(centor_y)[centor_x]) || isinf(depth_image.ptr<float>(centor_y)[centor_x])    || depth_image.ptr<float>(centor_y)[centor_x] < 50 || depth_image.ptr<float>(centor_y)[centor_x] > 2000)continue;//距离从
                float d = depth_image.ptr<float>(centor_y)[centor_x];//目标点的深度

                float filter_min_area = 2000  * min_point_area / d;
            
                float contource_area = contourArea(cv::Mat(it->contourcs),true);

                if(fabs(contource_area) < 0.3 * filter_min_area)continue;
                
                Point3d point;
                point.x = 0;
                point.y = 0;
                point.z = 0;

                int point_number = 0 ;

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);


                for(int i = min_x_; i < max_x_ ; i++)
                {
                    for(int j = min_y_; j < max_y_ ; j++)
                    {
                        double inot = 0;

                        inot = cv::pointPolygonTest(it->contourcs , cvPointTo32f(cv::Point(i, j)) , false);

                        if(inot > 0)
                        {

                            if( !isnan(depth_image.ptr<float>(j)[i]) && !isinf(depth_image.ptr<float>(j)[i]) && depth_image.ptr<float>(j)[i] > 50   && depth_image.ptr<float>(j)[i] < 2000 && fabs(depth_image.ptr<float>(j)[i] - d) <= 150)
                            {
                                float dll = depth_image.ptr<float>(j)[i];
                                pcl::PointXYZ cloud_point;

                                cv::Point3d temp_point_;
                            
                                temp_point_.z =   double(dll) / camera_info_.camera_factor;
                                temp_point_.x =  (i - camera_info_.camera_cx) * temp_point_.z / camera_info_.camera_fx;
                                temp_point_.y = (j - camera_info_.camera_cy) * temp_point_.z / camera_info_.camera_fy;
                               
                                cloud_point.z = temp_point_.z;
                                cloud_point.x = temp_point_.x;
                                cloud_point.y = temp_point_.y;                                
                                
                                cloud->push_back(cloud_point);                                                                
                            }                          
                        }                        
                    }
                }

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
                
                pcl::ModelCoefficients::Ptr coff(new pcl::ModelCoefficients);

                pcl::VoxelGrid<pcl::PointXYZ> sor;
                sor.setInputCloud(cloud);
                sor.setLeafSize(0.005,0.005,0.005);
                sor.filter(*cloud_filter);

                SACSegmentation<pcl::PointXYZ> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setMaxIterations(100);
                seg.setDistanceThreshold(0.005);
                seg.setInputCloud(cloud_filter);
                seg.segment(*inliers, *coff);

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter_on_plate(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter_out_plate(new pcl::PointCloud<pcl::PointXYZ>);

                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_filter);
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter(*cloud_filter_on_plate);

                if(cloud_filter_on_plate->size() < 1)continue;
                float max_distance = 0;

                for(int i = 0 ; i < cloud_filter_on_plate->size(); i++)
                {
                    point_number++;                                 
                    point.z = ( point.z * (point_number - 1 ) + cloud_filter_on_plate->points[i].z) / point_number;
                    point.x = ( point.x * (point_number - 1 ) + cloud_filter_on_plate->points[i].x) / point_number;
                    point.y = ( point.y * (point_number - 1 ) + cloud_filter_on_plate->points[i].y) / point_number;
                }

                for(int i = 0 ; i < cloud_filter_on_plate->size(); i++)
                {
                    float distance =   (cloud_filter_on_plate->points[i].z - point.z) * (cloud_filter_on_plate->points[i].z - point.z)    
                                                     +  (cloud_filter_on_plate->points[i].x - point.x) * (cloud_filter_on_plate->points[i].x - point.x) 
                                                      +  (cloud_filter_on_plate->points[i].y - point.y) * (cloud_filter_on_plate->points[i].y - point.y) ;
                  
                    distance = sqrt(distance);
                    if(distance > max_distance)
                    {
                        max_distance = distance;           
                    }                       
                }

                if(max_distance < 0.03)point.z += 0.05;
                else point.z += 0.025;
     
                if(point_number > min_point_number && point.z > 0.05 && point.z < 1.5 && fabs(point.x) < 1.5 && fabs(point.y) < 1.5)//点数太少，可过滤
                {
                    if(it->color == "blue")
                    {
                        it->id_ = blue_number;
                        blue_number++;
                    }
                    else if(it->color == "red")
                    {
                        it->id_ = red_number;
                        red_number++;
                    }
                    else if(it->color == "blue")
                    {
                        it->id_ = orange_number;
                        orange_number++;
                    }
                    marker_on_axis_filtered_->push_back(*it);                       

                    visualization_msgs::Marker marker;
                    marker.id = it->id_;
                    marker.header.frame_id = it->frame_;
                    marker.header.stamp = ros::Time::now();
                    marker.ns = it->color;

                    if(marker.ns=="blue")
                    {
                        marker.color.r = 237.0f;
                        marker.color.g = 145.0f;
                        marker.color.b = 33.0f;
                        marker.color.a = 1.0f;
                    }
                    else if(marker.ns == "red")
                    {
                        marker.color.r = 0.0f;
                        marker.color.g = 255.0f;
                        marker.color.b = 0.0f;
                        marker.color.a = 1.0f;

                    }
                    else if(marker.ns == "orange")
                    {
                        marker.color.r = 0.0f;
                        marker.color.g = 0.0f;
                        marker.color.b = 255.0f;
                        marker.color.a = 1.0f;

                    }

                    marker.lifetime = ros::Duration(0.1);
                    marker.frame_locked = false;
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.action = visualization_msgs::Marker::ADD;

                    marker.pose.position.z = point.z;
                    marker.pose.position.x = point.x;
                    marker.pose.position.y = point.y;
                    
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.05;
                    marker.scale.y = 0.05;
                    marker.scale.z = 0.05;
                    markers.markers.push_back(marker);
                    
                }                
            }   
            if(marker_on_axis_filtered_->size()> 0 )return true;
            else return false; 
       }
   }
;
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "find_object_on_axis");	
								
	ros::NodeHandle nh("~");
    

      HAND_EYE_GRASP::find_object find_object_on_axis(nh);
      ros::spin();  
      return 0;

}