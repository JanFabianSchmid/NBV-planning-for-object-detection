#ifndef CANDIDATE_LOCATOR_H
#define CANDIDATE_LOCATOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

#include "object_candidates/ArrayImages.h"
#include "object_candidates/SnapshotMsg.h"
#include "candidate_locator/ArrayPointClouds.h"

class CandidateLocator
{
public:
	CandidateLocator();

private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_candidates_;
	image_transport::ImageTransport it_;
	image_transport::CameraSubscriber sub_depth_cam_info_;
	ros::Publisher pub_point_clouds_;

	//DEBUG
	ros::Publisher pub_debug_;

	image_geometry::PinholeCameraModel cam_model_;
	tf::TransformListener tf_listener_;
	tf::StampedTransform map_transform_;
	tf::StampedTransform camera_transform_;
	cv::Mat depth_image_;
	bool cam_model_assigned = false;

	void getTransforms(const ros::Time& stamp);
	void candidatesCallback(const object_candidates::SnapshotMsg& msg);
	void cameraInfoCallback(const sensor_msgs::ImageConstPtr& depth_img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
	void calculateObjectPoints(cv::Mat& I, pcl::PointCloud<pcl::PointXYZ>& msg);
};

#endif