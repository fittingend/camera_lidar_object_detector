
#include "lidar_out.h"
#define LIDAR_VISUALIZATION_ON

LIDAR_OUT::LIDAR_OUT(string _name)
{
    name = _name;
    #ifdef LIDAR_VISUALIZATION_ON
    _vs = new Visualization(name);
    #endif
}

void LIDAR_OUT::out(pcl::PointCloud<PointXYZIR> data, uint16_t obj_cnt)
{
    #ifdef LIDAR_VISUALIZATION_ON
    _vs->update(data);
    #endif
    cout<< name <<endl;
    cout<< " obj_cnt : " << obj_cnt << " , Point cnt : "<<data.size()<<endl; 
}



