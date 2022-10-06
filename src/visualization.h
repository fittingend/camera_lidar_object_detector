#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "lidar_clustering.h"

using namespace std;

class Visualization
{
    private:
    pcl::visualization::CloudViewer *myViewer;
    
    public:
    Visualization(string name);
    ~Visualization();     
    void update(pcl::PointCloud<PointXYZIR> data);
    int test();
};

#endif
