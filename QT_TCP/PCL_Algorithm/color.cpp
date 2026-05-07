#include "color.h"

void colorPointSet(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, unsigned char red, unsigned char green, unsigned char blue)
{
    for(std::size_t i = 0;i<cloud->size();i++)
    {
        cloud->points.at(i).r=red;
        cloud->points.at(i).g=green;
        cloud->points.at(i).b=blue;
    }
}

