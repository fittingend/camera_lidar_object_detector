#ifndef _LIDAR_DATA_TYPES_H_
#define _LIDAR_DATA_TYPES_H_
#include <vector>

using namespace std;

struct PointXYZIR
{
    float x;
    float y;
    float z;
    float intensity;
    uint16_t r;
};

struct PointXYZIC
{
    float x;
    float y;
    float z;
    float intensity;
    uint16_t c; //clustered object ID 
};

class LidarClusteringResult
{
    public:
    uint16_t n_obj;
    vector <PointXYZIR> points; 
};

#endif