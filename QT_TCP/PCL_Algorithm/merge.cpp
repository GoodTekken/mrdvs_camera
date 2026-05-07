#include "merge.h"

void MergeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr sourcecloud, pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud,pcl::PointCloud<pcl::PointXYZ>&destinationcloud)
{
    pcl::copyPointCloud(*sourcecloud,destinationcloud); //#include<pcl/io/io.h>

    for(std::size_t i = 0; i< newcloud->size();i++)
    {
        destinationcloud.push_back(newcloud->at(i));
    }
}

void MergeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud,pcl::PointCloud<pcl::PointXYZ>&destinationcloud)
{
    for(std::size_t i = 0; i< newcloud->size();i++)
    {
        destinationcloud.push_back(newcloud->at(i));
    }
}

void MergeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr newcloud,pcl::PointCloud<pcl::PointXYZRGB>&destinationcloud)
{
    for(std::size_t i = 0; i< newcloud->size();i++)
    {
        destinationcloud.push_back(newcloud->at(i));
    }
}

//pcl::PointXYZ point;
//point.x = newcloud->at(i).x;
//point.y = newcloud->at(i).y;
//point.z = newcloud->at(i).z;
