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

void addNormal(PointCloudXYZRGB::Ptr cloud, PointCloudXYZRGBNormal::Ptr cloud_with_normals)
{
	/*Add normal to PointXYZRGB
		Args:
			cloud: PointCloudXYZRGB
			cloud_with_normals: PointXYZRGBNormal
	*/
  pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  searchTree->setInputCloud ( cloud );
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud ( cloud );
  normalEstimator.setSearchMethod ( searchTree );
  //normalEstimator.setKSearch ( 50 );
  normalEstimator.setRadiusSearch (0.01);
  normalEstimator.compute ( *normals );
  pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );
  vector<int> indices;
  pcl::removeNaNNormalsFromPointCloud(*cloud_with_normals, *cloud_with_normals, indices);
  return;
}


void point_preprocess(PointCloudXYZRGB::Ptr cloud)
{
	/*Preprocess point before ICP
	Args:
		cloud: PointCloudXYZRGB
	*/
	//////////////Step1. Remove Nan////////////
  cout << "Original point number: " << cloud->points.size() << endl;
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
  cout << "Donoised point number: " << cloud->points.size() << endl;
  return;
}


Eigen::Matrix4f initial_guess(PointCloudXYZRGB::Ptr cloud_src, PointCloudXYZRGB::Ptr cloud_target)
{
  Eigen::Vector4f src_centroid, target_centroid;
  pcl::compute3DCentroid (*cloud_src, src_centroid);
  pcl::compute3DCentroid (*cloud_target, target_centroid);
  Eigen::Matrix4f tf_tran = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tf_rot = Eigen::Matrix4f::Identity();
  //
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(*cloud_src, src_centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

  Eigen::Matrix3f covariance2;
  pcl::computeCovarianceMatrixNormalized(*cloud_target, target_centroid, covariance2);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver2(covariance2, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA2 = eigen_solver2.eigenvectors();

  // Eigen::Quaternion<float> rot_q = Eigen::Quaternion<float>::FromTwoVectors(eigenVectorsPCA.row(0),eigenVectorsPCA2.row(0));
  Eigen::Matrix3f R ;
  R = eigenVectorsPCA2 * eigenVectorsPCA.inverse();
  for (int i = 0;i<3;i++)
      for (int j = 0;j<3;j++)
          tf_rot(i,j) = R(i,j);


  tf_tran(0,3) = target_centroid[0] - src_centroid[0];
  tf_tran(1,3) = target_centroid[1] - src_centroid[1];
  tf_tran(2,3) = target_centroid[2] - src_centroid[2];
  Eigen::Matrix4f tf = tf_rot * tf_tran ;
  return tf;
}

Eigen::Matrix4f point_2_plane_icp(PointCloudXYZRGB::Ptr cloud_src, PointCloudXYZRGB::Ptr cloud_target, PointCloudXYZRGBNormal::Ptr trans_cloud)
{
  PointCloudXYZRGBNormal::Ptr cloud_source_normals ( new PointCloudXYZRGBNormal );
  PointCloudXYZRGBNormal::Ptr cloud_target_normals ( new PointCloudXYZRGBNormal );
  addNormal( cloud_src, cloud_source_normals );
  addNormal( cloud_target, cloud_target_normals );
  pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp ( new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> () );
  icp->setMaximumIterations ( 2000 );
  // icp.setMaxCorrespondenceDistance(1);
  icp->setTransformationEpsilon(1e-15);
  icp->setEuclideanFitnessEpsilon(1e-15);
  icp->setInputSource ( cloud_source_normals ); // not cloud_source, but cloud_source_trans!
  icp->setInputTarget ( cloud_target_normals );
  // registration
  icp->align ( *trans_cloud ); // use cloud with normals for ICP

  if ( icp->hasConverged() ){
  	cout <<"has conveged:" << icp->hasConverged() << endl;
    cout << "icp score: " << icp->getFitnessScore() << endl;
    cout << "icp Converge Criteria: " << icp->getConvergeCriteria()->getConvergenceState()  << endl;
  }
  else
  	cout << "Not converged." << endl;
	Eigen::Matrix4f inverse_transformation = icp->getFinalTransformation();
}

Eigen::Matrix4f point_2_point_icp(PointCloudXYZRGB::Ptr cloud_src, PointCloudXYZRGB::Ptr cloud_target, PointCloudXYZRGB::Ptr trans_cloud)
{
  // pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr icp( new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRG>() );  //建立ICP的例項類
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr icp ( new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> () );
  icp->setInputSource(cloud_src);
  icp->setInputTarget(cloud_target);
  // icp->setMaxCorrespondenceDistance(100);  
  icp->setTransformationEpsilon(1e-15); // 1e-15 //criterion state 2
  icp->setEuclideanFitnessEpsilon(1e-15); // 1e-15 //criterion state 3...4??
  icp->setMaximumIterations(2000);   //2000 //criterion state 1
  icp->align(*trans_cloud);

  if ( icp->hasConverged() ){
    cout <<"has conveged:" << icp->hasConverged() << endl;
    cout << "icp score: " << icp->getFitnessScore() << endl;
    cout << "icp Converge Criteria: " << icp->getConvergeCriteria()->getConvergenceState()  << endl;
  }
  else
    cout << "Not converged." << endl;
  Eigen::Matrix4f inverse_transformation = icp->getFinalTransformation();
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
  cen_pose.color.a = 0.7;
  cen_pose.pose.position.x = centroid[0];
  cen_pose.pose.position.y = centroid[1];
  cen_pose.pose.position.z = centroid[2];
  cen_pose.pose.orientation.x = 0.0;
  cen_pose.pose.orientation.y = 0.0;
  cen_pose.pose.orientation.z = 0.0;
  cen_pose.pose.orientation.w = 1.0;

  return cen_pose;

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp");
  ros::NodeHandle nh;
  ros::Publisher model_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/model", 1);
  ros::Publisher cloud_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/cloud", 1);
  ros::Publisher initial_guess_tf_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/ini_guess", 1);
  ros::Publisher registered_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/registered_cloud", 1);
  ros::Publisher centroid_publisher = nh.advertise<geometry_msgs::Pose>("/cloud_centroid", 1);
  ros::Publisher centroid_marker_publisher = nh.advertise<visualization_msgs::Marker>("/Marker_centroid", 1);

  PointCloudXYZRGB::Ptr model(new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr ini_guess_tf_cloud(new PointCloudXYZRGB);
  // PointCloudXYZRGBNormal::Ptr registered_cloud(new PointCloudXYZRGBNormal);
  PointCloudXYZRGB::Ptr registered_cloud(new PointCloudXYZRGB);
  string model_path;
  string cloud_path;
  nh.getParam("model_path", model_path);
  nh.getParam("cloud_path", cloud_path);

  printf("Load model\n");
  pcl::io::loadPLYFile<pcl::PointXYZRGB>(model_path, *model);
  printf("Finish Load model\n");
  pcl::io::loadPLYFile<pcl::PointXYZRGB>(cloud_path, *cloud);
  printf("Finish Load model\n--------------------------\n");
  model ->header.frame_id = "/map";
  cloud ->header.frame_id = "/map";

  point_preprocess(model);
  point_preprocess(cloud);
  printf("Initial guess\n--------------------------\n");
  // Eigen::Matrix4f tf1 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tf1 = initial_guess(cloud, model);

  pcl::transformPointCloud (*cloud, *ini_guess_tf_cloud, tf1);
  printf("ICP:\n");
  // Eigen::Matrix4f tf2 = point_2_plane_icp(ini_guess_tf_cloud, model, registered_cloud);
  Eigen::Matrix4f tf2 = point_2_point_icp(ini_guess_tf_cloud, model, registered_cloud);
  Eigen::Matrix4f final_tf = tf1 * tf2;
  cout << final_tf << endl;

  visualization_msgs::Marker cen_pose = add_centroid(registered_cloud);
  geometry_msgs::Pose cen;
  cen.position = cen_pose.pose.position;
  cout << "centroid position: "<< cen.position << endl;


  while (ros::ok()){
	model_publisher.publish(model);
	cloud_publisher.publish(cloud);
	initial_guess_tf_publisher.publish(ini_guess_tf_cloud);
	registered_cloud_publisher.publish(registered_cloud);
  centroid_publisher.publish(cen);
  centroid_marker_publisher.publish(cen_pose);
  }

  return 0;
}
