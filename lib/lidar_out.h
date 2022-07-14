#ifndef _LIDAR_OUT_H_
#define _LIDAR_OUT_H_

#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include "lidar_data_types.h"

#ifdef LIDAR_VISUALIZATION_ON
#include "visualization.h"
#endif

using namespace std;

class LIDAR_OUT
{
    private:
    #ifdef LIDAR_VISUALIZATION_ON
    Visualization *_vs;
    #endif
    
    public:
    string name;
    LIDAR_OUT(string _name);
    ~LIDAR_OUT();
    void out(pcl::PointCloud<PointXYZIR> data, uint16_t obj_cnt);
};

#endif