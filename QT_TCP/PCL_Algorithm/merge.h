#ifndef MERGE_H
#define MERGE_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/io/io.h>

void MergeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr sourcecloud, pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud,pcl::PointCloud<pcl::PointXYZ>&destinationcloud);
void MergeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud,pcl::PointCloud<pcl::PointXYZ>&destinationcloud);
void MergeCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr newcloud,pcl::PointCloud<pcl::PointXYZRGB>&destinationcloud);

#endif // MERGE_H
