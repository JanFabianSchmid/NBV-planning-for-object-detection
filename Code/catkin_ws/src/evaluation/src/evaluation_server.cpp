#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>

#include <evaluation/Evaluate.h>
#include <evaluation/CompareGroundTruthsToProposals.h>
#include <octomap_msgs/MergeCandidates.h>
#include <candidate_locator/ArrayPointClouds.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

#include <boost/foreach.hpp>

namespace evaluation
{
  class EvaluationServer
	  {
		  private:

		  ros::NodeHandle nh_;
		  tf::TransformListener listener_;
		  ros::Publisher frontier_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("ground_truths",20);

		  public:

      bool evaluate(evaluation::Evaluate::Request  &req,
               evaluation::Evaluate::Response &res)
      {
        std::vector<std::string> objects;
        objects.push_back("ball");
        objects.push_back("banana");
        objects.push_back("bleach_cleanser");
        objects.push_back("bowl");
        objects.push_back("cracker_box");
        objects.push_back("hammer");
        objects.push_back("mug");
        objects.push_back("pitcher_base");
        objects.push_back("potted_meat_can");
        objects.push_back("power_drill");
        objects.push_back("sugar_box");
        objects.push_back("wood_block");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PCDReader reader;
        
        std::string objects_package_path(ros::package::getPath("objects"));
        std::string fileName;
        candidate_locator::ArrayPointClouds array_pc_msg; 
        
        /*
        ros::ServiceClient gms_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        gazebo_msgs::GetModelState getmodelstate;
        //ros::ServiceClient gms_client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
        //gazebo_msgs::GetLinkState getmodelstate;
        gms_client.waitForExistence();  
        
        
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "/map";
        transform.child_frame_id = "/map";
        */
        
        tf::StampedTransform tf_transform;
        
        geometry_msgs::TransformStamped transform;
        //transform.header.frame_id = //"/map";        
        
        // TEST - merge ground truth point clouds into octomap
        bool merge_ground_truths = false;
        uint merge_ground_truths_counter = 0;
        nh_.param("merge_ground_truths", merge_ground_truths, merge_ground_truths);
        ROS_INFO_STREAM("Value of merge_ground_truths is " << merge_ground_truths);
        
        BOOST_FOREACH(std::string object, objects){  
          fileName = objects_package_path + "/data/"+object+"/"+object+"_downsampled.pcd";    
          reader.read (fileName, *cloud);  
          //fileName = objects_package_path + "/data/"+object+"/"+object+".ply";  
          //pcl::io::loadPLYFile(fileName, *cloud);
          
          sensor_msgs::PointCloud2 pc_msg;

          // TEST - merge ground truth point clouds into octomap
          if (merge_ground_truths)
          {
            BOOST_FOREACH(pcl::PointXYZRGB& point, *cloud)
            {
              merge_ground_truths_counter ++;
              point.r = 0;
              point.g = merge_ground_truths_counter;
              point.b = 5;
            }
          }

          pcl::toROSMsg<pcl::PointXYZRGB>(*cloud,pc_msg); 
          std::string frame = "/" + object;
          try{
            listener_.lookupTransform("/map", frame,  
                                     ros::Time(0), tf_transform);
          }
          catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
          }         
          
          /*
          transform.child_frame_id = frame;
          transform.header.frame_id = "/map"; 
          transform.transform.translation.x = tf_transform.getOrigin().getX();
          transform.transform.translation.y = tf_transform.getOrigin().getY();
          transform.transform.translation.z = tf_transform.getOrigin().getZ();
          transform.transform.rotation.x = tf_transform.getRotation().getAxis().getX();
          transform.transform.rotation.y = tf_transform.getRotation().getAxis().getY();
          transform.transform.rotation.z = tf_transform.getRotation().getAxis().getZ();
          transform.transform.rotation.w = tf_transform.getRotation().getW();*/
         
          //tf2::doTransform (cloud_in, cloud_out, transform);
          //tf2::doTransform(pc_msg, pc_msg, transform);
          
          pcl_ros::transformPointCloud(
          "/map",
          tf_transform,
          pc_msg,
          pc_msg);
          
          array_pc_msg.data.push_back(pc_msg);
          //frontier_cloud_pub.publish(pc_msg);
        }

        // TEST - merge ground truth point clouds into octomap
        if (merge_ground_truths)
        {
          ROS_INFO("TEST: passing ground truths to be merged into octomap");
          ros::ServiceClient merge_client = nh_.serviceClient<octomap_msgs::MergeCandidates>("octomap_server/merge_candidates"); 
          octomap_msgs::MergeCandidates merge_srv;
          merge_srv.request.candidates = array_pc_msg;
          merge_client.call(merge_srv);
        }
        
        ros::ServiceClient comparison_client = nh_.serviceClient<evaluation::CompareGroundTruthsToProposals>("octomap_server/compare_ground_truths_to_proposals"); 
        comparison_client.waitForExistence();
        
        evaluation::CompareGroundTruthsToProposals comparison_srv;
        comparison_srv.request.ground_truths = array_pc_msg;
        
        ROS_INFO("Requesting to compare ground truths to proposals");
        if (comparison_client.call(comparison_srv))
        {
          ROS_INFO("Received statistics of comparison");
        }
        else
        {
          ROS_ERROR("Failed to compare ground truths to proposals");
        }
        
        ros::ServiceClient restart_client = nh_.serviceClient<std_srvs::Empty>("/restarter"); 
        std_srvs::Empty restart_srv;
        
        ROS_INFO("Requesting to restart the system for the next experiment");
        restart_client.call(restart_srv);

        return true;
      }
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluation_server");
  ros::NodeHandle nh;
  
  evaluation::EvaluationServer server;
  
  ros::ServiceServer service = nh.advertiseService("evaluate", &evaluation::EvaluationServer::evaluate, &server);
  
  ROS_INFO("Ready to evaluate");
  ros::spin();

  return 0;
}









