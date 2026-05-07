#ifndef TRANSFER_H
#define TRANSFER_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

void transfer_X_Y_Z_Basler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>& cloud_transter);

#endif // TRANSFER_H
