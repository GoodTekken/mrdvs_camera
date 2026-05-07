#ifndef FILTERS_H
#define FILTERS_H

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

void PassFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>& cloud_filtered, float limit_min, float limit_max,std::string axis);
void PassFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloud_filtered, float limit_min, float limit_max,std::string axis);
void PassThrough_X_Y_Z_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>& cloud_filtered);

void BoxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud,
               pcl::PointCloud<pcl::PointXYZRGB>& filterBox_cloudRGB,
               Eigen::Vector4f min_pt,
               Eigen::Vector4f max_pt,
               bool IsOutside);

void VoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud,
                     pcl::PointCloud<pcl::PointXYZRGB>& voxelGridfilter_cloudRGB,
                     float leaf_x,
                     float leaf_y,
                     float leaf_z
                     );

void NormalsFilter(pcl::PointCloud<pcl::Normal>::Ptr normalCloud,
                   pcl::PointCloud<pcl::PointXYZRGB>& cloud_filtered);

void RadiusOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud,
                                double raduis,
                                int min_pts,
                                bool negative,
                                pcl::PointCloud<pcl::PointXYZRGB>& cloud_filtered);

#endif // FILTERS_H
