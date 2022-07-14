#ifndef _LIDAR_CLUSTERING_H_
#define _LIDAR_CLUSTERING_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <functional>
#include <string>


#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "lidar_data_types.h"
using namespace std;
extern float leafsize;
extern float clusterTolerance;
extern int minClusterSize;
extern int maxClusterSize; 

//uint16_t lidar_clustering(string name, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, pcl::PointCloud<PointXYZIR> &out_data);
uint16_t lidar_clustering(string name, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, pcl::PointCloud<PointXYZIC> &out_data);

int findClosest(std::vector<pcl::PointXYZ>, float, int);

#endif
