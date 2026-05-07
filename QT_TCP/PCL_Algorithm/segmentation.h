#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/filters/extract_indices.h>

void PlaneSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud,
                  pcl::PointIndices& plane_inliers,
                  pcl::ModelCoefficients& plane_coefficients);

void PointNearThePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud,
                       pcl::ModelCoefficients& plane_coefficients,
                       float radiusSearch,
                       pcl::PointCloud<pcl::PointXYZRGB>& cloudOutput);

void EuclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud,
                                double tolerance,
                                int max_cluster_size,
                                int min_cluster_size,
                                std::vector<pcl::PointIndices>& PointIndices);

#endif // SEGMENTATION_H
