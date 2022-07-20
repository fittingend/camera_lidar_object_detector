#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>

#include "planar_segmentation.h"

int planar_segmentation(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, int MaxIterations, float DistanceThreshold)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
  
  //XYZI 타입인 cloud 를 XYZ 로 바꾸어서 cloud_seg 에 복사
  pcl::copyPointCloud(*cloud_in, *cloud_seg);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(MaxIterations);
  seg.setDistanceThreshold(DistanceThreshold);

  seg.setInputCloud(cloud_seg);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    std::cout << "Could not estimate a planar anymore." << std::endl;
  }
  else
  {
    // Filter, [Extracting indices from a PointCloud](https://pcl.readthedocs.io/projects/tutorials/en/latest/extract_indices.html#extract-indices)
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_seg);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*cloud_seg, min, max);
    double min_z = min.z;
    std::cout << "ground plane size: " << cloud_plane->points.size() << ", min_z:" << min_z << std::endl;
    //show_point_cloud(cloud_plane, "gound plane in point cloud");
    // filter planar
    extract.setNegative(true);
    extract.filter(*cloud_f);
    //show_point_cloud(cloud_f, "plane filtered point cloud");
//    *cloud_seg = *cloud_f;
    pcl::copyPointCloud(*cloud_f, *cloud_out);
  //    *cloud_seg = *cloud_plane;
  }
  return (0);
}