#ifndef FEATURES_H
#define FEATURES_H

#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>

void NormalEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr originCloud, pcl::PointCloud<pcl::Normal>& normalCloud,int k);

void BoundaryEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr originCloud, pcl::PointCloud<pcl::Normal>::Ptr normalCloud,int k,pcl::PointCloud<pcl::Boundary>& boundaries);

void BoundaryCloudPickup(pcl::PointCloud<pcl::PointXYZRGB>::Ptr originCloud,pcl::PointCloud<pcl::Boundary>& boundaries,pcl::PointCloud<pcl::PointXYZRGB>& cloud_boundary);
#endif // FEATURES_H
