#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <vector>
#include <Eigen/Core>
#include <string>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// PCL library
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/features/moment_of_inertia_estimation.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudXYZRGBNormal;
geometry_msgs::PoseStamped pose_3d;
ros::Publisher centroid_publisher, centroid_marker_publisher, bonding_box_publisher, cloudnum_publisher;

visualization_msgs::Marker add_centroid(PointCloudXYZRGB::Ptr cloud, Eigen::Vector4f centroid, int id){
  visualization_msgs::Marker cen_pose;
  cen_pose.header.frame_id = "map";
  cen_pose.ns = "basic_shapes";
  cen_pose.id = id;
  cen_pose.type = visualization_msgs::Marker::SPHERE;
  cen_pose.action = visualization_msgs::Marker::ADD;
  cen_pose.lifetime = ros::Duration(2);

  cen_pose.scale.x = 0.007;
  cen_pose.scale.y = 0.007;
  cen_pose.scale.z = 0.007;
  cen_pose.color.r = 1;
  cen_pose.color.g = 0;
  cen_pose.color.b = 0;
  cen_pose.color.a = 1.0;
  cen_pose.pose.position.x = centroid[0];
  cen_pose.pose.position.y = centroid[1];
  cen_pose.pose.position.z = centroid[2];
  cen_pose.pose.orientation.x = 0.0;
  cen_pose.pose.orientation.y = 0.0;
  cen_pose.pose.orientation.z = 0.0;
  cen_pose.pose.orientation.w = 1.0;

  return cen_pose;
}

visualization_msgs::Marker boxing(PointCloudXYZRGB::Ptr cloud, Eigen::Vector4f centroid, pcl::PointXYZRGB min, pcl::PointXYZRGB max, int id){
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "map";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "box_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  // line_list.lifetime = ros::Duration(1);

  line_list.id = id;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.002f;
  line_list.color.g = 1.0f;
  line_list.color.b = 1.0f;
  line_list.color.r = 1.0f;
  line_list.color.a = 1.0f;

  geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
  p1.x = p2.x = p5.x = p6.x = min.x;
  p3.x = p4.x = p7.x = p8.x = max.x;
  p1.y = p4.y = p5.y = p8.y = min.y;
  p2.y = p3.y = p6.y = p7.y = max.y;
  p1.z = p2.z = p3.z = p4.z = min.z;
  p5.z = p6.z = p7.z = p8.z = max.z;

  line_list.points.push_back(p1);
  line_list.points.push_back(p2);
  line_list.points.push_back(p1);
  line_list.points.push_back(p4);
  line_list.points.push_back(p1);
  line_list.points.push_back(p5);

  line_list.points.push_back(p3);
  line_list.points.push_back(p2);
  line_list.points.push_back(p3);
  line_list.points.push_back(p4);
  line_list.points.push_back(p3);
  line_list.points.push_back(p7);

  line_list.points.push_back(p6);
  line_list.points.push_back(p2);
  line_list.points.push_back(p6);
  line_list.points.push_back(p5);
  line_list.points.push_back(p6);
  line_list.points.push_back(p7);

  line_list.points.push_back(p8);
  line_list.points.push_back(p4);
  line_list.points.push_back(p8);
  line_list.points.push_back(p5);
  line_list.points.push_back(p8);
  line_list.points.push_back(p7);

  return line_list;
}

visualization_msgs::Marker boxing_2(PointCloudXYZRGB::Ptr cloud, pcl::PointXYZRGB min, pcl::PointXYZRGB max, int id){

  //實例化一個Momentof...
  pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute ();
//聲明一些必要的變量
  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;
  pcl::PointXYZRGB min_point_AABB;
  pcl::PointXYZRGB max_point_AABB;
  pcl::PointXYZRGB min_point_OBB;
  pcl::PointXYZRGB max_point_OBB;
  pcl::PointXYZRGB position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;
//計算描述符和其他的特徵
  feature_extractor.getMomentOfInertia(moment_of_inertia);
  feature_extractor.getEccentricity(eccentricity);
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues(major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter(mass_center);
  Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat(rotational_matrix_OBB);
  // cout << "--------------------------------------------------------------" << endl;
  // // cout << "centroid: \n" << centroid << endl;
  // cout << "position: \n" << position << endl;
  // // cout << "rotational: \n" << rotational_matrix_OBB << endl;
  // cout << "quat: \n" << quat.coeffs() << endl;
  pose_3d.pose.position.x = position[0]; pose_3d.pose.position.y = position[1]; pose_3d.pose.position.z = position[2];
  pose_3d.pose.orientation.x = quat.x(); pose_3d.pose.orientation.y = quat.y(); pose_3d.pose.orientation.z = quat.z(); pose_3d.pose.orientation.w = quat.w();
 
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "map";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "box_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.lifetime = ros::Duration(2);

  line_list.id = id;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.002f;
  line_list.color.g = 0.9f;
  line_list.color.b = 0.9f;
  line_list.color.r = 0.9f;
  line_list.color.a = 1.0f;

  Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

  p1 = rotational_matrix_OBB * p1 + position;
  p2 = rotational_matrix_OBB * p2 + position;
  p3 = rotational_matrix_OBB * p3 + position;
  p4 = rotational_matrix_OBB * p4 + position;
  p5 = rotational_matrix_OBB * p5 + position;
  p6 = rotational_matrix_OBB * p6 + position;
  p7 = rotational_matrix_OBB * p7 + position;
  p8 = rotational_matrix_OBB * p8 + position;

  geometry_msgs::Point pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8;
  pt1.x = p1[0]; pt1.y = p1[1]; pt1.z = p1[2]; 
  pt2.x = p2[0]; pt2.y = p2[1]; pt2.z = p2[2]; 
  pt3.x = p3[0]; pt3.y = p3[1]; pt3.z = p3[2]; 
  pt4.x = p4[0]; pt4.y = p4[1]; pt4.z = p4[2]; 
  pt5.x = p5[0]; pt5.y = p5[1]; pt5.z = p5[2]; 
  pt6.x = p6[0]; pt6.y = p6[1]; pt6.z = p6[2]; 
  pt7.x = p7[0]; pt7.y = p7[1]; pt7.z = p7[2]; 
  pt8.x = p8[0]; pt8.y = p8[1]; pt8.z = p8[2]; 

  line_list.points.push_back(pt1);
  line_list.points.push_back(pt2);
  line_list.points.push_back(pt1);
  line_list.points.push_back(pt4);
  line_list.points.push_back(pt1);
  line_list.points.push_back(pt5);

  line_list.points.push_back(pt3);
  line_list.points.push_back(pt2);
  line_list.points.push_back(pt3);
  line_list.points.push_back(pt4);
  line_list.points.push_back(pt3);
  line_list.points.push_back(pt7);

  line_list.points.push_back(pt6);
  line_list.points.push_back(pt2);
  line_list.points.push_back(pt6);
  line_list.points.push_back(pt5);
  line_list.points.push_back(pt6);
  line_list.points.push_back(pt7);

  line_list.points.push_back(pt8);
  line_list.points.push_back(pt4);
  line_list.points.push_back(pt8);
  line_list.points.push_back(pt5);
  line_list.points.push_back(pt8);
  line_list.points.push_back(pt7);

  return line_list;
}

visualization_msgs::Marker add_cloudnum(PointCloudXYZRGB::Ptr cloud, Eigen::Vector4f centroid, pcl::PointXYZRGB min, pcl::PointXYZRGB max, int id){
  visualization_msgs::Marker marker;
  marker.header.frame_id="map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.lifetime = ros::Duration(2);

  marker.scale.z = 0.02f;
  marker.color.b = 1.0f;
  marker.color.g = 1.0f;
  marker.color.r = 1.0f;
  marker.color.a = 1.0f;

  geometry_msgs::Pose pose;
  pose.position.x = centroid[0] - (max.x - min.x);
  pose.position.y = centroid[1] + (max.y - min.y);
  pose.position.z = centroid[2] + (max.z - min.z);
  string str= to_string(cloud->points.size());
  marker.text=str;
  marker.pose=pose;

  return marker;
}

void point_preprocess(PointCloudXYZRGB::Ptr cloud)
{
  vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  // =============Cluster=============
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud);

  vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (3200);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  visualization_msgs::MarkerArray cen_poses, bonding_boxes, cloudnum_texts;
  geometry_msgs::PoseArray cens;
  cens.header.stamp = ros::Time::now();
  cens.header.frame_id = "map";

  int currentClusterNum = 0;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator point = it->indices.begin(); point != it->indices.end(); point++)
        cluster->points.push_back(cloud->points[*point]);
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    if (cluster->points.size() <= 0)
      break;

    Eigen::Vector4f centroid; 
    pcl::compute3DCentroid(*cluster, centroid);
    pcl::PointXYZRGB min, max; 
    pcl::getMinMax3D(*cluster, min, max);  

    visualization_msgs::Marker cen_pose = add_centroid(cluster, centroid, currentClusterNum);
    cen_poses.markers.push_back(cen_pose);

    visualization_msgs::Marker bonding_box = boxing_2(cluster, min, max, currentClusterNum);
    bonding_boxes.markers.push_back(bonding_box);
    cens.poses.push_back(pose_3d.pose);
    
    visualization_msgs::Marker cloudnum = add_cloudnum(cluster, centroid, min, max, currentClusterNum);
    cloudnum_texts.markers.push_back(cloudnum);
    

    currentClusterNum++;
  }
  centroid_publisher.publish(cens);
  centroid_marker_publisher.publish(cen_poses);
  bonding_box_publisher.publish(bonding_boxes);
  cloudnum_publisher.publish(cloudnum_texts);

  vector<int> indices2;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices2);
  return;
}


void callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  fromROSMsg(*msg, *cloud);
  point_preprocess(cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp");
  ros::NodeHandle nh;
  ros::Subscriber filtedcloud_subscriber = nh.subscribe("/zedm01/zed/zed_node/pointcloud_filted", 1, callback);
  centroid_publisher = nh.advertise<geometry_msgs::PoseArray>("/centroid_cloud", 1);
  centroid_marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("/Marker_centroid", 1);
  bonding_box_publisher = nh.advertise<visualization_msgs::MarkerArray>("/Marker_bonding_box", 1);
  cloudnum_publisher = nh.advertise<visualization_msgs::MarkerArray>("/Marker_cloudnum", 1);

  ros::spin();
  return 0;
}
