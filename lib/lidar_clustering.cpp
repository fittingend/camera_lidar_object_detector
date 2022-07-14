#include "lidar_clustering.h"
#include <cmath>

using namespace std;
int closest_index = 0;
float y_abs_diff;

//uint16_t lidar_clustering(string name,const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud ,pcl::PointCloud<PointXYZIR> &ret_cloud)
//uint16_t lidar_clustering(string name,const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud ,pcl::PointCloud<pcl::PointXYZI> &ret_cloud)
uint16_t lidar_clustering(string name,const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud ,pcl::PointCloud<PointXYZIC> &ret_cloud)
{
    static uint16_t count = 0;
    static uint16_t runflag=0;
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn = *cloud;
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn2;
    
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_v2 (new pcl::PointCloud<pcl::PointXYZI>);
    vg.setInputCloud (laserCloudIn.makeShared());
    vg.setLeafSize (leafsize, leafsize, leafsize);	// original 0.5f; the larger it is the more downsampled it gets 
    vg.filter (*cloud_filtered_v2);			
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud_filtered_v2);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (clusterTolerance); 			
    ec.setMinClusterSize (minClusterSize);				//original:4
    ec.setMaxClusterSize (maxClusterSize);				// original value is 40; try to increase to get accurate data
    ec.setSearchMethod (tree);				
    ec.setInputCloud (cloud_filtered_v2);	
    ec.extract (cluster_indices);

    float y_min, y_max = 0;
    vector<pcl::PointXYZ> xyz_vector;

    uint16_t j = 0;
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            pcl::PointXYZI pt = cloud_filtered_v2->points[*pit];
            
            PointXYZIC pt2;
        
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = pt.intensity;
            pt2.c=(float)(j + 1);
            ret_cloud.push_back(pt2);
        //    ret_cloud.push_back(pt);
            xyz_vector.push_back(pcl::PointXYZ (pt.x, pt.y, pt.z));
//            cout <<pt.x << ", "<<pt.y << ", "<< pt.z << endl;

        }
        // cout << "No. of points of the obj is " << xyz_vector.size() <<endl;

        for (int i = 0; i < xyz_vector.size(); i++)
        {
            if (i == 0)
            {
                y_min = xyz_vector[i].y;
                y_max = xyz_vector[i].y;
            }
            else
            {
                if (xyz_vector[i].y < y_min) y_min = xyz_vector[i].y;
                if (xyz_vector[i].y > y_max) y_max = xyz_vector[i].y;

            }

        }
	float y_width = y_max - y_min;
	if (y_width > 1.5 && y_width < 2.5 )
	{
		cout << "\nFor object ID " << j+1 << ":" << endl;
		cout << "Min y value is " << y_min << endl;
		cout << "Max y value is " << y_max << endl;
		
		cout << "Width is " << y_width << endl;
		float y_mid_value = y_min + y_width/2;
		cout << "Mid value is "<< y_mid_value <<endl;
		findClosest(xyz_vector, y_mid_value, xyz_vector.size());
		cout << "Mid coordinate of the obejct is " << xyz_vector[closest_index].x << ", "<<xyz_vector[closest_index].y << ", " <<xyz_vector[closest_index].z<< " with difference of "<< y_abs_diff << endl;
		cout << "Distance to the identified object " << j << " is " << sqrt(pow(xyz_vector[closest_index].x, 2) + pow(xyz_vector[closest_index].y, 2)) << endl; 
		cout << "No. of points of the obj is " << xyz_vector.size() <<endl;
	}
	else
	{/*
        	cout << "\nFor object ID " << j+1 << ":" << endl;
//		cout << "!!!!!!!!!!Not a car!!!!!!!!!!!" << endl;
		cout << "Min y value is " << y_min << endl;
		cout << "Max y value is " << y_max << endl;
		
		cout << "Width is " << y_width << endl;
		float y_mid_value = y_min + y_width/2;
		cout << "Mid value is "<< y_mid_value <<endl;
		findClosest(xyz_vector, y_mid_value, xyz_vector.size());
		cout << "Mid coordinate of the obejct is " << xyz_vector[closest_index].x << ", "<<xyz_vector[closest_index].y << ", " <<xyz_vector[closest_index].z<< " with difference of "<< y_abs_diff << endl;
		cout << "Distance to the identified object " << j << " is " << sqrt(pow(xyz_vector[closest_index].x, 2) + pow(xyz_vector[closest_index].y, 2)) << endl; 
		cout << "No. of points of the obj is " << xyz_vector.size() <<endl;
//	*/}
        j++;
        xyz_vector.clear();
        vector<pcl::PointXYZ>().swap(xyz_vector);
    }
    return j;

}


// xyz_vector[] 의 인덱스 중 오브젝트의 중간점에 제일 가까운 인덱스를 리턴
int findClosest(std::vector<pcl::PointXYZ> xyz_vector, float y_mid_value, int vector_size)
{
    y_abs_diff = 100; // assign a random value large enough
    for (int i = 0; i < vector_size; i++)
    {
       if(abs(xyz_vector[i].x - y_mid_value) < y_abs_diff )
       {
           y_abs_diff =  abs(xyz_vector[i].x - y_mid_value);
//           cout << "y_abs test: " << y_abs_diff <<endl;
           closest_index = i;
       }

    }

    return closest_index;
}

