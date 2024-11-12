#ifndef CONE_DETECTION_HPP
#define CONE_DETECTION_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

std::tuple<bool, float, float> detectCone(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

#endif // CONE_DETECTION_HPP
