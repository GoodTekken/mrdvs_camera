#include "transfer.h"

void transfer_X_Y_Z_Basler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>& cloud_transter)
{
    for(std::size_t i=0;i<cloud->points.size();i++)
    {
        cloud_transter.points[i].y = -cloud->points[i].x/1000.0;
        cloud_transter.points[i].z = -cloud->points[i].y/1000.0;
        cloud_transter.points[i].x = cloud->points[i].z/1000.0;
    }
}
