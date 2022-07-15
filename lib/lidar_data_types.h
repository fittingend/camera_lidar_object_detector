#ifndef _LIDAR_DATA_TYPES_H_
#define _LIDAR_DATA_TYPES_H_
#include <vector>

using namespace std;

/*struct PointXYZI ///< user defined point type
{
  float x;
  float y;
  float z;
  uint8_t intensity;
};
*/
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

    PointXYZIC()
    {

    }

    PointXYZIC(float temp_x, float temp_y, float temp_z, int temp_clusterID)
    : x (temp_x), y(temp_y), z(temp_z), intensity(0), c(temp_clusterID)
    {

    }
};


/*
class PointXYZ_CameraXY
{
protected:
    float m_x;
    float m_y;
    float m_z;
    float m_camera_x;
    float m_camera_y;

public:
    PointXYZ_CameraXY (int x, int y, int z, int camera_x, int camera_y)
    {
        m_x = x;
        m_y = y;
        m_z = z;
        m_camera_x = camera_x;
        m_camera_y = camera_y;
    }
};
*/
typedef struct pointxyz_cameraxy
{
    float m_x;
    float m_y;
    float m_z;
    float m_camera_x;
    float m_camera_y;

    pointxyz_cameraxy (float x, float y, float z, float camera_x, float camera_y)
        : m_x(x), m_y(y), m_z(z), m_camera_x(camera_x), m_camera_y(camera_y)
    {        
    }
    
}PointXYZ_CameraXY;


class LidarClusteringResult
{
    public:
    uint16_t n_obj;
    vector <PointXYZIR> points; 
};

#endif