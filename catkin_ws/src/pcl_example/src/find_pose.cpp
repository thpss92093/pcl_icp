#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <vector>
#include <Eigen/Core>
#include <string>
#include <geometry_msgs/Pose.h>
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudXYZRGBNormal;
using namespace std;
ros::Publisher centroid_publisher, centroid_marker_publisher, bonding_box_publisher, cloudnum_publisher;

void point_preprocess(PointCloudXYZRGB::Ptr cloud)
{
	/*Preprocess point before ICP
	Args:
		cloud: PointCloudXYZRGB
	*/
	//////////////Step1. Remove Nan////////////
  // cout << "Original point number: " << cloud->points.size() << endl;
  vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  // //////////////Step2. Downsample////////////
  // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  // sor.setInputCloud (cloud);
  // sor.setLeafSize (0.004f, 0.004f, 0.004f);
  // sor.filter (*cloud);
  // copyPointCloud(*cloud, *cloud);
  // printf("Downsampled point number: %d\n",cloud->points.size());

  //////////////Step3. Denoise//////////////
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
  if (cloud->points.size()>100){
    sor2.setInputCloud (cloud);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (0.5);
    sor2.filter (*cloud);
  }

  //////////////Step4. Cluster//////////////
  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  // tree->setInputCloud (cloud);

  // std::vector<pcl::PointIndices> cluster_indices;
  // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // ec.setClusterTolerance (0.01);                           //<<<<<<<<<<=============== 1
  // ec.setMinClusterSize (200);                              //<<<<<<<<<<=============== 10    30
  // ec.setMaxClusterSize (8000);                            //<<<<<<<<<<=============== 1500  500
  // ec.setSearchMethod (tree);
  // ec.setInputCloud (cloud);
  // ec.extract (cluster_indices);
  // std::cout << "Cluster size: " << cluster_indices.size() << std::endl;

  // int currentClusterNum = 1;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZRGB>);

  // for (std::vector<pcl::PointIndices>::const_iterator i = cluster_indices.begin(); i != cluster_indices.end(); ++i){
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
  //   for (std::vector<int>::const_iterator point = i->cluster_indices.begin(); point != i->cluster_indices.end(); point++)
  //       cluster->points.push_back(cloud->points[*point]);
  //   cluster->width = cluster->points.size();
  //   cluster->height = 1;
  //   cluster->is_dense = true;
  //   *cloud_clusters += *cluster;
  //   if (cluster->points.size() <= 0)
  //     break;
  //   currentClusterNum++;
  // }
  // *cloud = *cloud_clusters;

  vector<int> indices2;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices2);
  // cout << "Donoised point number: " << cloud->points.size() << endl;
  return;
}

visualization_msgs::Marker add_centroid(PointCloudXYZRGB::Ptr cloud){
  // cen->header.frame_id = "map";
  Eigen::Vector4f centroid; 
  pcl::compute3DCentroid(*cloud, centroid);
  visualization_msgs::Marker cen_pose;
  cen_pose.header.frame_id = "map";
  cen_pose.ns = "basic_shapes";
  cen_pose.id = 0;
  cen_pose.type = visualization_msgs::Marker::SPHERE;
  cen_pose.action = visualization_msgs::Marker::ADD;

  cen_pose.scale.x = 0.01;
  cen_pose.scale.y = 0.01;
  cen_pose.scale.z = 0.01;
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

visualization_msgs::Marker boxing(PointCloudXYZRGB::Ptr cloud){
  Eigen::Vector4f centroid; 
  pcl::compute3DCentroid(*cloud, centroid);
  pcl::PointXYZRGB min, max; 
  pcl::getMinMax3D(*cloud, min, max);  
  // cout << "\ncentroid: " << centroid << "\nmin: " << min << "\nmax: " << max;


  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "map";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "box_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
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

visualization_msgs::Marker add_cloudnum(PointCloudXYZRGB::Ptr cloud){
  // cen->header.frame_id = "map";
  Eigen::Vector4f centroid; 
  pcl::compute3DCentroid(*cloud, centroid);
  pcl::PointXYZRGB min, max; 
  pcl::getMinMax3D(*cloud, min, max);  

  visualization_msgs::Marker marker;
  marker.header.frame_id="map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id =0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

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


void callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  fromROSMsg(*msg, *cloud);
  point_preprocess(cloud);

  visualization_msgs::Marker cen_pose = add_centroid(cloud);
  geometry_msgs::Pose cen;
  cen.position = cen_pose.pose.position;
  // cout << "centroid position: "<< cen.position << endl;
  centroid_publisher.publish(cen);
  centroid_marker_publisher.publish(cen_pose);
  visualization_msgs::Marker bonding_box = boxing(cloud);
  bonding_box_publisher.publish(bonding_box);
  visualization_msgs::Marker cloudnum = add_cloudnum(cloud);
  cloudnum_publisher.publish(cloudnum);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp");
  ros::NodeHandle nh;
  ros::Subscriber filtedcloud_subscriber = nh.subscribe("/zedm01/zed/zed_node/pointcloud_filted", 1, callback);
  centroid_publisher = nh.advertise<geometry_msgs::Pose>("/cloud_centroid", 1);
  centroid_marker_publisher = nh.advertise<visualization_msgs::Marker>("/Marker_centroid", 1);
  bonding_box_publisher = nh.advertise<visualization_msgs::Marker>("/Marker_bonding_box", 1);
  cloudnum_publisher = nh.advertise<visualization_msgs::Marker>("/Marker_cloudnum", 1);

  ros::spin();
  return 0;
}
