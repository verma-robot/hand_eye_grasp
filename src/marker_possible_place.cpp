
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

 class maker_posible_place
 {

  private:
    std::string dir;
    std::string marker_sub;
  
    ros::NodeHandle node_;
    ros::Subscriber sub;
    bool init_;

  public:

    void init();
    void start_trans_();
    void cloud_cb(const visualization_msgs::MarkerArrayConstPtr& position);

  public:

    maker_posible_place(ros::NodeHandle nodehandle);
    ~maker_posible_place(){ };

 };

maker_posible_place::maker_posible_place(ros::NodeHandle nodehandle)
  : node_(nodehandle),init_(false)
{

  if(!node_.getParam("dir", dir))dir = "/home/tianjin-university/marker.yaml";
  if(!node_.getParam("marker_sub", marker_sub))marker_sub = "all_markers_objects_on_root";
 
  init();
}

void maker_posible_place::init()
{

  init_ = true;

};

void maker_posible_place::start_trans_()
{

  if(!init_)init();
  sub = node_.subscribe<visualization_msgs::MarkerArray> (marker_sub, 1, &maker_posible_place::cloud_cb, this); 

}

 void maker_posible_place::cloud_cb(const visualization_msgs::MarkerArrayConstPtr& position)
{

   std::string pause_;

   float marker_10_x = 0.00;
   float marker_10_y = 0.00;
   float marker_10_z = 0.00;

   float marker_15_x = 0.00;
   float marker_15_y = 0.00;
   float marker_15_z = 0.00;

   float marker_25_x = 0.00;
   float marker_25_y = 0.00;
   float marker_25_z = 0.00;

   float marker_45_x = 0.00;
   float marker_45_y = 0.00;
   float marker_45_z = 0.00;

   int marker_number = 0;

   bool find_10 = false;
   bool find_15 = false;
   bool find_25 = false;
   bool find_45 = false;


   if(position->markers.size() >0)
   {
      for(int i = 0 ; i < position->markers.size(); i++)
      {
         if(position->markers[i].ns=="target_id10")
         {
             marker_number += 1;
             find_10 = true;

             marker_10_x = position->markers[i].pose.position.x;
             marker_10_y = position->markers[i].pose.position.y;
             marker_10_z = position->markers[i].pose.position.z;

         }
         else if(position->markers[i].ns=="target_id15")
         {
            marker_number += 1;
            find_15 = true;

             marker_15_x = position->markers[i].pose.position.x;
             marker_15_y = position->markers[i].pose.position.y;
             marker_15_z = position->markers[i].pose.position.z;

         }
         else if(position->markers[i].ns=="target_id25")
         {
            marker_number += 1;
            find_25 = true;

             marker_25_x = position->markers[i].pose.position.x;
             marker_25_y = position->markers[i].pose.position.y;
             marker_25_z = position->markers[i].pose.position.z;

         }
         else if(position->markers[i].ns=="target_id45")
         {
            marker_number += 1;
            find_45 = true;

            marker_45_x = position->markers[i].pose.position.x;
            marker_45_y = position->markers[i].pose.position.y;
            marker_45_z = position->markers[i].pose.position.z;

         }
      }
   }

   if(marker_number >0)
   {
      if(find_10) std::cout << "find marker ID 10 " <<" , "<< marker_10_x<< " , " << marker_10_y << " , "<< marker_10_z << std::endl;
      if(find_15) std::cout << "find marker ID 15 " <<" , "<< marker_15_x<< " , " << marker_15_y << " , "<< marker_15_z << std::endl;
      if(find_25) std::cout << "find marker ID 25 " <<" , "<< marker_25_x<< " , " << marker_25_y << " , "<< marker_25_z << std::endl;
      if(find_45) std::cout << "find marker ID 45 " <<" , "<< marker_45_x<< " , " << marker_45_y << " , "<< marker_45_z << std::endl;

      std::cout << "are you sure , y(Y): YES "<<std::endl;
      std::cin >> pause_;


     sleep(0.5);

    if (pause_ == "Y" || pause_ == "y" )
    {

      //YAML::Node yamlConfig = YAML::LoadFile(dir);

       std::ifstream fin(dir); 
       YAML::Node yamlConfig = YAML::Load(fin);

      std::ofstream file(dir);

      file.flush();

      if(find_10)
      {

         yamlConfig["find_marker_10"] = true;
         yamlConfig["find_marker_10"] = yamlConfig["find_marker_10"].as<bool>();


         yamlConfig["marker_10_x"] = marker_10_x;
         yamlConfig["marker_10_y"] = marker_10_y;
         yamlConfig["marker_10_z"] = marker_10_z;  

      }
     else
      {

         yamlConfig["find_marker_10"] = false;
         yamlConfig["marker_10_x"] = 0;
         yamlConfig["marker_10_y"] = 0;
         yamlConfig["marker_10_z"] = 0;  

      }

     if(find_15)
      {
         yamlConfig["find_marker_15"] = true;
         yamlConfig["marker_15_x"] = marker_15_x;
         yamlConfig["marker_15_y"] = marker_15_y;
         yamlConfig["marker_15_z"] = marker_15_z;  

      }
      else
       {
         yamlConfig["find_marker_15"] = false;
         yamlConfig["marker_15_x"] = 0;
         yamlConfig["marker_15_y"] = 0;
         yamlConfig["marker_15_z"] = 0;  

      }

        if(find_25)
      {
         yamlConfig["find_marker_25"] = true;

         yamlConfig["marker_25_x"] = marker_25_x;
         yamlConfig["marker_25_y"] = marker_25_y;
         yamlConfig["marker_25_z"] = marker_25_z;  

      }
      else
      {
         yamlConfig["find_marker_25"] = false;

         yamlConfig["marker_25_x"] = 0;
         yamlConfig["marker_25_y"] = 0;
         yamlConfig["marker_25_z"] = 0;  

      }

        if(find_45)
      {

         yamlConfig["find_marker_45"] = true;

         yamlConfig["marker_45_x"] = marker_45_x;
         yamlConfig["marker_45_y"] = marker_45_y;
         yamlConfig["marker_45_z"] = marker_45_z;  

      }
      else 
      {

         yamlConfig["find_marker_45"] = false;
         yamlConfig["marker_45_x"] = marker_45_x;
         yamlConfig["marker_45_y"] = marker_45_y;
         yamlConfig["marker_45_z"] = marker_45_z;  

      }

      file << yamlConfig;
      file.close(); 
      ros::shutdown();

    }
   }


};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "marker_place",1);

    ros::NodeHandle nh("~");
    ros::WallDuration(2.0).sleep();

   maker_posible_place maker_posible_place(nh); 
   maker_posible_place.start_trans_();


    sleep(1);
 
    ros::spin(); 

    return 0;
}


