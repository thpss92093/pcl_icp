#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp> 
#include <cv_bridge/cv_bridge.h>
// message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
// PCL library
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
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
// MSG
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>

using namespace std;

ros::Publisher cloud_publisher_1, cloud_publisher_2, cloud_filted_publisher_1, cloud_filted_publisher_2;

void filted_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filted_cloud)
{
  vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  /* // =============Downsample=============
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.004f, 0.004f, 0.004f);
  sor.filter (*cloud);
  copyPointCloud(*cloud, *cloud);
  printf("Downsampled point number: %d\n",cloud->points.size());

  // =============passthrough filter=============
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-25, 25);
  pass.filter (*cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-25, 25);
  pass.filter (*cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-3, 1); 
  pass.filter (*cloud);*/

  // =============Denoise=============
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
  if (cloud->points.size()>100){
    sor2.setInputCloud (cloud);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (0.5);
    sor2.filter (*cloud);
  }

  for (std::size_t i = 0; i < cloud->points.size (); ++i){
    if (cloud->points[i].r < 100 && cloud->points[i].b > 100 &&cloud->points[i].g < 100){
      filted_cloud->push_back(cloud->points[i]);
      // cout<<"r: "<<to_string(cloud->points[i].r)<<endl<<"g: "<<to_string(cloud->points[i].g)<<endl<<"b: "<<to_string(cloud->points[i].b)<<endl;
    }
  }

  return;
}

void callback_rosbag(const sensor_msgs::ImageConstPtr& color_image_1, const sensor_msgs::ImageConstPtr& depth_image_1, const sensor_msgs::CameraInfoConstPtr& cam_info_1)
{
  std::vector<double> intrinsic_1;
  intrinsic_1.resize(4);
  intrinsic_1[0] = cam_info_1->K[0]; // fx
  intrinsic_1[1] = cam_info_1->K[4]; // fy
  intrinsic_1[2] = cam_info_1->K[2]; // cx
  intrinsic_1[3] = cam_info_1->K[5]; // cy
  // std::vector<double> intrinsic_2;
  // intrinsic_2.resize(4);
  // intrinsic_2[0] = cam_info_2->K[0]; // fx
  // intrinsic_2[1] = cam_info_2->K[4]; // fy
  // intrinsic_2[2] = cam_info_2->K[2]; // cx
  // intrinsic_2[3] = cam_info_2->K[5]; // cy
  // cv_bridge::CvImagePtr color_img_ptr_1, depth_img_ptr_1, color_img_ptr_2, depth_img_ptr_2;
  cv_bridge::CvImagePtr color_img_ptr_1, depth_img_ptr_1;
  try{
    color_img_ptr_1 = cv_bridge::toCvCopy(color_image_1, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }  
  try{
    depth_img_ptr_1 = cv_bridge::toCvCopy(depth_image_1, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what()); return;
  }
 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_f(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(int x=300; x<=879; ++x){ // 1280  640
    for(int y=0; y<=719; ++y){ // 720  480
      // More readable
      auto depth = depth_img_ptr_1->image.at<float>(cv::Point(x, y));
      geometry_msgs::Point p;

      if(depth<0.80)
      {
        p.z = depth /* * 0.001f*/; // Represent in 1 mm
        p.x = (x-intrinsic_1[2])/intrinsic_1[0]*p.z; // x = (u-cx)/fx*z
        p.y = (y-intrinsic_1[3])/intrinsic_1[1]*p.z; // y = (v-cy)/fy*z
        if(1)
        {
          pcl::PointXYZRGB pc_p;
          pc_p.x = p.x; pc_p.y = p.y; pc_p.z = p.z;
          pc_p.r = color_img_ptr_1->image.at<cv::Vec3b>(cv::Point(x, y))[2];
          pc_p.g = color_img_ptr_1->image.at<cv::Vec3b>(cv::Point(x, y))[1];
          pc_p.b = color_img_ptr_1->image.at<cv::Vec3b>(cv::Point(x, y))[0];
          pc->push_back(pc_p);
        
          if (pc_p.r < 100 && pc_p.b > 100 && pc_p.g < 100){
            pc_f->push_back(pc_p);
          }
        } 
      }
      else{
        geometry_msgs::Point p;
        p.x = p.y = p.z = std::numeric_limits<double>::quiet_NaN();
      }
      pc->height = 1; pc->width = pc->points.size();
    }
  }
  //////////////Denoise//////////////
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
  if (pc_f->points.size()>100){
    sor2.setInputCloud (pc_f);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (0.5);
    sor2.filter (*pc_f);
  }
  sensor_msgs::PointCloud2 cloud, cloud_filted;
  pcl::toROSMsg(*pc, cloud);
  pcl::toROSMsg(*pc_f, cloud_filted);
  cloud.header.frame_id = "map";
  cloud_filted.header.frame_id = "map";
  cloud_publisher_1.publish(cloud);
  cloud_filted_publisher_1.publish(cloud_filted);
}

void callback_realtime(const sensor_msgs::PointCloud2ConstPtr& msg_1, const sensor_msgs::PointCloud2ConstPtr& msg_2)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filted_cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filted_cloud_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  sensor_msgs::PointCloud2 cloud_filted_1, cloud_filted_2;
  fromROSMsg(*msg_1, *cloud_1);
  fromROSMsg(*msg_2, *cloud_2);
  filted_color(cloud_1, filted_cloud_1);
  filted_color(cloud_2, filted_cloud_2);
  pcl::toROSMsg(*filted_cloud_1, cloud_filted_1);
  pcl::toROSMsg(*filted_cloud_2, cloud_filted_2);
  cloud_filted_publisher_1.publish(cloud_filted_1);
  cloud_filted_publisher_2.publish(cloud_filted_2);
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "single_camera_filter");
  ros::NodeHandle nh;

  cloud_publisher_1 = nh.advertise<sensor_msgs::PointCloud2> ("/zedm01/zed/zed_node/pointcloud", 1);
  cloud_filted_publisher_1 = nh.advertise<sensor_msgs::PointCloud2> ("/zedm01/zed/zed_node/pointcloud_filted", 1);

  message_filters::Subscriber<sensor_msgs::Image> color_image_sub_1(nh, "/zedm01/zed/zed_node/rgb/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_1(nh, "/zedm01/zed/zed_node/depth/depth_registered", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_1(nh, "/zedm01/zed/zed_node/rgb/camera_info", 1);
  
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color_image_sub_1, depth_image_sub_1, info_sub_1);
  sync.registerCallback(boost::bind(&callback_rosbag, _1, _2, _3));
  ros::spin();

  return (0);
}