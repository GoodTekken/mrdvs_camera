#ifndef SETTEMPLATECLOUD_H
#define SETTEMPLATECLOUD_H

#include <pcl/filters/passthrough.h>
#include "Pallet_Struct.h"

void SetTemplateCloud_Tian(pcl::PointCloud<pcl::PointXYZ>& cloudTemplate);
void SetTemplateCloud_Chuan(pcl::PointCloud<pcl::PointXYZ>& cloudTemplate);

void SetTemplateCloud_Zone(float min_x,float max_x,float min_y,float max_y,float min_z,float max_z,
                           int color_red,int color_green,int color_blue, float cloudDensity,
                           pcl::PointCloud<pcl::PointXYZRGB>& cloud_zone);

void SetTemplateCloud_Zone(zone_struct zone,
                           pcl::PointCloud<pcl::PointXYZRGB>& cloud_zone);

#endif // SETTEMPLATECLOUD_H
