#include "visualization.h"
#include <unistd.h>


uint8_t rgb_patten[6][3]={
    {255,0,0},
    {0,255,0},
    {255,255,255},
    {255,255,0},
    {125,255,0},
    {125,125,0}
};

Visualization::Visualization(string name)
{
    myViewer = new pcl::visualization::CloudViewer(name);
    
    //pcl::visualization::CloudViewer aasd("QWERT");
    //pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
}
Visualization::~Visualization()
{

}
void Visualization::update(pcl::PointCloud<PointXYZIR> data)
{
    uint16_t data_len ,old_r=0;
    uint8_t r=0,g=0,b=0;
    uint8_t step;
    pcl::PointCloud<pcl::PointXYZRGB> vs_cloud;
    for(int i=0;i< data.size() ;i++)
    {
        pcl::PointXYZRGB pt;
        
        pt.x = data[i].x/100.0;
        pt.y = data[i].y/100.0;
        pt.z = data[i].z/100.0;

        pt.r = rgb_patten[data[i].r%6][0];
        pt.g = rgb_patten[data[i].r%6][1];
        pt.b = rgb_patten[data[i].r%6][2];
        vs_cloud.push_back(pt);
        //std::cout<<"x"<<pt.x<<std::endl;

        myViewer->showCloud(vs_cloud.makeShared());
    }
    
}
int Visualization::test() 
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 10000;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    for (size_t i = 0; i < cloud.points.size(); ++i) 
    { 
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    } 
    for (size_t i = 0; i < cloud.points.size(); ++i) std::cerr << " " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
    pcl::visualization::CloudViewer viewer("PCL Viewer");
    viewer.showCloud(cloud.makeShared());
    while (!viewer.wasStopped())
    {
        sleep(1);
        for (size_t i = 0; i < cloud.points.size(); ++i) 
        { 
            cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
            
        } 
        viewer.showCloud(cloud.makeShared());
    }
    return 0;

}
