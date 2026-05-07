#ifndef COLOR_H
#define COLOR_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

void colorPointSet(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, unsigned char red, unsigned char green, unsigned char blue);

#endif // COLOR_H
