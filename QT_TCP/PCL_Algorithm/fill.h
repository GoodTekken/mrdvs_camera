#ifndef FILL_H
#define FILL_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>

void fillPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
               pcl::ModelCoefficients::Ptr plane_coefficients,
               pcl::PointXYZRGB minPt,
               pcl::PointXYZRGB maxPt,
               pcl::PointCloud<pcl::PointXYZRGB>& cloud_filled);

void fillPoint_lineScan(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
               pcl::ModelCoefficients::Ptr plane_coefficients,
               pcl::PointXYZRGB minPt,
               pcl::PointXYZRGB maxPt,
               pcl::PointCloud<pcl::PointXYZRGB>& cloud_filled);

void fillPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundcloud,
               pcl::ModelCoefficients::Ptr plane_coefficients,
               pcl::PointXYZRGB minPt,
               pcl::PointXYZRGB maxPt,
               pcl::PointCloud<pcl::PointXYZRGB>& cloud_filled);

void BoundaryDilate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud,
                    pcl::ModelCoefficients::Ptr plane_coefficients,
                    double dilate,
                    pcl::PointCloud<pcl::PointXYZRGB>& cloud_filled
                    );

#endif // FILL_H
