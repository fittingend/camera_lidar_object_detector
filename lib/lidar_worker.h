#ifndef _LIDAR_WORKER_H_
#define _LIDAR_WORKER_H_

#include <stdio.h>
//#include <netinet/in.h>
//#include <sys/socket.h>
// include <arpa/inet.h>
#include <error.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include "udp_worker.h"
#include "lidar_clustering.h"
#include "lidar_data_types.h"
#include "lidar_out.h"

#define LIDAR_WORKER_VLP16 1
#define LIDAR_WORKER_M1 2

struct MEAS_RANGE
{
    float x_low = 0;
    float x_high = 0;
    float y_low = 0;
    float y_high = 0;
    float z_low = 0;
    float z_high = 0;
    float yaw_low = 0;
    float yaw_high = 0;
    float pitch_low = 0;
    float pitch_high = 0;
};

struct LIDAR_INFO
{
    string sensor_name;
    uint8_t model;
    uint16_t port;
    MEAS_RANGE range;
};

class VLP16 : public UDP_WORKER
{
private:
    MEAS_RANGE _range;
    int x_range(float _x);
    int y_range(float _y);
    int z_range(float _z);
    int x_range_(float _x);
    int y_range_(float _y);
    int z_range_(float _z);

    int yaw_range(float _yaw);
    int yaw_range_(float _yaw);
    int yaw_range__(float _yaw);
    int pitch_range(float _pitch);
    int pitch_range_(float _pitch);
    int pitch_range__(float _pitch);

    void MsgParser(uint8_t *frame);

    int (VLP16::*x_func)(float);
    int (VLP16::*y_func)(float);
    int (VLP16::*z_func)(float);

    int (VLP16::*yaw_func)(float);
    int (VLP16::*pitch_func)(float);

public:
    pcl::PointCloud<pcl::PointXYZI> pData;
    LIDAR_OUT *output;
    virtual void worker(uint8_t *frame);
    void init_meas_range(MEAS_RANGE *_info);
    void init(LIDAR_INFO *_info);
};

class M1 : public UDP_WORKER
{
private:
    MEAS_RANGE _range;
    int x_range(float _x);
    int y_range(float _y);
    int z_range(float _z);
    int x_range_(float _x);
    int y_range_(float _y);
    int z_range_(float _z);

    int yaw_range(float _yaw);
    int yaw_range_(float _yaw);
    int yaw_range__(float _yaw);
    int pitch_range(float _pitch);
    int pitch_range_(float _pitch);
    int pitch_range__(float _pitch);

    void MsgParser(uint8_t *frame);

    int (M1::*x_func)(float);
    int (M1::*y_func)(float);
    int (M1::*z_func)(float);

    int (M1::*yaw_func)(float);
    int (M1::*pitch_func)(float);

public:
    pcl::PointCloud<pcl::PointXYZI> pData;
    LIDAR_OUT *output;
    virtual void worker(uint8_t *frame);
    void init_meas_range(MEAS_RANGE *_info);
    void init(LIDAR_INFO *_info);
};

class LIDAR_WORKER
{
private:
public:
    vector<VLP16 *> vlp16_list;
    void update_vlp_16(uint8_t *frame);

    vector<M1 *> M1_list;
    void update_M1(uint8_t *frame);
    
    LIDAR_WORKER(vector<LIDAR_INFO> &lidar_info);
    ~LIDAR_WORKER();
};

#endif