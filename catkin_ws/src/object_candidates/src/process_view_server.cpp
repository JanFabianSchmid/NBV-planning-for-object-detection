#include "ros/ros.h"
#include "object_candidates/Snapshot.h"
#include "candidate_locator/ArrayImages.h"
#include "object_candidates/Objectcandidates.h"
#include "candidate_locator/LocateCandidates.h"
#include "octomap_msgs/MergeCandidates.h"
#include "std_srvs/Empty.h"

#include <sensor_msgs/image_encodings.h>


bool process_view(std_srvs::Empty::Request  &req,
                  std_srvs::Empty::Response &res)
{
  ros::NodeHandle nh;
  
  ros::ServiceClient snapshot_client = nh.serviceClient<object_candidates::Snapshot>("get_snapshot");
  object_candidates::Snapshot snap_srv;
  
  ROS_INFO("Requesting snapshot");
  if (snapshot_client.call(snap_srv))
  {
    ROS_INFO_STREAM("Received snapshot, " << snap_srv.response.candidates.data.size() << " candidates");

    ros::ServiceClient locator_client = nh.serviceClient<candidate_locator::LocateCandidates>("locate_candidates");
    candidate_locator::LocateCandidates locator_srv;
    locator_srv.request.depth_image = snap_srv.response.depth_image;
    locator_srv.request.rgb_image = snap_srv.response.rgb_image;
    locator_srv.request.rgb_info = snap_srv.response.rgb_info;
    locator_srv.request.candidates = snap_srv.response.candidates;
    locator_srv.request.publish = true;

    ROS_INFO("Locating candidates");
    if (locator_client.call(locator_srv) && locator_srv.response.candidates.data.size() > 0)
    {
      ros::ServiceClient octomap_merge_client = nh.serviceClient<octomap_msgs::MergeCandidates>("octomap_server/merge_candidates");
      octomap_msgs::MergeCandidates octomap_merge_srv;
      octomap_merge_srv.request.candidates = locator_srv.response.candidates;

      ROS_INFO("Merging candidates");
      if (octomap_merge_client.call(octomap_merge_srv))
      {
        ROS_INFO("Candidates merged");
        //finishedTask(false);
        return true;
      }
      else
      {
        ROS_ERROR("Failed to call octomap candidate merging service");
        return false; 
      }
    }
    else
    {
      if (locator_srv.response.candidates.data.size() <= 0)
        ROS_ERROR("Candidate locator service returned no point clouds");
      else
        ROS_ERROR("Failed to call candidate locator service");
      return false;
    }
  }
  else
  {
    ROS_ERROR("Failed to call snapshot service");
    return false;
  }
  
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "process_view_server");
  ros::NodeHandle nh;
  
  ros::ServiceServer service = nh.advertiseService("process_view", process_view);
  
  ROS_INFO("Ready to process views");
  ros::spin();

  return 0;
}









