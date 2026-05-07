#include "fill.h"
#include "filters.h"

void fillPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
               pcl::ModelCoefficients::Ptr plane_coefficients,
               pcl::PointXYZRGB minPt,
               pcl::PointXYZRGB maxPt,
               pcl::PointCloud<pcl::PointXYZRGB>& cloud_filled)
{
    pcl::PointXYZRGB searchPoint;
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = 0.006;  //0.006
    searchPoint.r = 0;
    searchPoint.g = 200;
    searchPoint.b = 100;
    cloud_filled.clear();
    for(float y=minPt.y;y<maxPt.y;y=y+0.005)
    {
        for(float z=minPt.z;z<maxPt.z;z=z+0.005)
        {
            searchPoint.y = y;
            searchPoint.z = z;

            searchPoint.x=(-plane_coefficients->values[3] - searchPoint.y * plane_coefficients->values[1] - searchPoint.z * plane_coefficients->values[2]) / plane_coefficients->values[0];
            if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0)
            {
                cloud_filled.push_back(searchPoint);
            }
        }
    }
}

void fillPoint_lineScan(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
               pcl::ModelCoefficients::Ptr plane_coefficients,
               pcl::PointXYZRGB minPt,
               pcl::PointXYZRGB maxPt,
               pcl::PointCloud<pcl::PointXYZRGB>& cloud_filled)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr middleColumn_boxFilter_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);                //声明cloud，用于保存输入点云
    pcl::PointXYZRGB searchPoint;
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    int begin_fill_index = 0;
    float radius = 0.006;  //0.006
    searchPoint.r = 0;
    searchPoint.g = 200;
    searchPoint.b = 100;
    cloud_filled.clear();

    for(float z = minPt.z; z<maxPt.z; z= z+0.005)
    {
        Eigen::Vector4f min_pt_temp(minPt.x,-0.3,z,1.0);
        Eigen::Vector4f max_pt_temp(maxPt.x,0.3, z+0.004,1.0);
        BoxFilter(cloud,*middleColumn_boxFilter_cloudRGB,min_pt_temp,max_pt_temp,false); // true: ouside   false:inside
        if(middleColumn_boxFilter_cloudRGB->size() > 6)  //(6 * 0.005 = 0.03  as 3 CM )
        {
            begin_fill_index ++;
            if(begin_fill_index>2)  // Avoid tilting
            {

                for (float y = minPt.y; y < maxPt.y; y = y + 0.005)
                {
                    searchPoint.y = y;
                    searchPoint.z = z;

                    float caculate_x = (-plane_coefficients->values[3] - searchPoint.y * plane_coefficients->values[1] - searchPoint.z * plane_coefficients->values[2]) / plane_coefficients->values[0];
                    searchPoint.x = caculate_x;
                    if(kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0)
                    {
                        cloud_filled.push_back(searchPoint);  //fill the point
                    }
                }
            }
        }
    }
}

//
void fillPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundcloud,
               pcl::ModelCoefficients::Ptr plane_coefficients,
               pcl::PointXYZRGB minPt,
               pcl::PointXYZRGB maxPt,
               pcl::PointCloud<pcl::PointXYZRGB>& cloud_filled)
{
    pcl::PointXYZRGB searchPoint;
    pcl::PointXYZRGB detectUpPoint;
    pcl::PointXYZRGB detectDownPoint;
    pcl::PointXYZRGB detectLeftPoint;
    pcl::PointXYZRGB detectRightPoint;
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud);

    pcl::KdTreeFLANN<pcl::PointXYZRGB> detectkdtree;
    detectkdtree.setInputCloud(boundcloud);
    float detectradius = 0.035; //0.025

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = 0.006; //0.006
    searchPoint.r = 0;
    searchPoint.g = 200;
    searchPoint.b = 0;
    cloud_filled.clear();
    float y=minPt.y;
    while(y<maxPt.y)
    {
        detectUpPoint.y = y;
        detectUpPoint.z = maxPt.z;
        detectUpPoint.x = (-plane_coefficients->values[3] - detectUpPoint.y * plane_coefficients->values[1] - detectUpPoint.z * plane_coefficients->values[2]) / plane_coefficients->values[0];

        detectDownPoint.y = y;
        detectDownPoint.z = minPt.z;
        detectDownPoint.x = (-plane_coefficients->values[3] - detectDownPoint.y * plane_coefficients->values[1] - detectDownPoint.z * plane_coefficients->values[2]) / plane_coefficients->values[0];

        if(detectkdtree.radiusSearch(detectUpPoint, detectradius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 &&
           detectkdtree.radiusSearch(detectDownPoint, detectradius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            float z=minPt.z;
            while(z<maxPt.z)
            {
                //not used
                detectLeftPoint.y = maxPt.y;
                detectLeftPoint.z = z;
                detectLeftPoint.x = (-plane_coefficients->values[3] - detectLeftPoint.y * plane_coefficients->values[1] - detectLeftPoint.z * plane_coefficients->values[2]) / plane_coefficients->values[0];
                //not used
                detectRightPoint.y = minPt.y;
                detectRightPoint.z = z;
                detectRightPoint.x = (-plane_coefficients->values[3] - detectLeftPoint.y * plane_coefficients->values[1] - detectLeftPoint.z * plane_coefficients->values[2]) / plane_coefficients->values[0];

                searchPoint.y = y;
                searchPoint.z = z;
                searchPoint.x=(-plane_coefficients->values[3] - searchPoint.y * plane_coefficients->values[1] - searchPoint.z * plane_coefficients->values[2]) / plane_coefficients->values[0];
                if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0)
                {
                    cloud_filled.push_back(searchPoint);
                }
                z=z+0.005;
            }
        }
        y=y+0.005;
    }
}


void BoundaryDilate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud,
                    pcl::ModelCoefficients::Ptr plane_coefficients,
                    double dilate,
                    pcl::PointCloud<pcl::PointXYZRGB>& cloud_filled
                    )
{
    cloud_filled.clear();
    pcl::PointXYZRGB searchPoint;
    for (std::size_t i = 0; i < origincloud->size(); i++)
    {
        searchPoint = origincloud->at(i);
        searchPoint.z = searchPoint.z+dilate;
        float caculate_x = (-plane_coefficients->values[3] - searchPoint.y * plane_coefficients->values[1] - searchPoint.z * plane_coefficients->values[2]) / plane_coefficients->values[0];
        searchPoint.x = caculate_x;
        cloud_filled.push_back(searchPoint);

        searchPoint = origincloud->at(i);
        searchPoint.z = searchPoint.z - dilate;
        caculate_x = (-plane_coefficients->values[3] - searchPoint.y * plane_coefficients->values[1] - searchPoint.z * plane_coefficients->values[2]) / plane_coefficients->values[0];
        searchPoint.x = caculate_x;
        cloud_filled.push_back(searchPoint);
    }
}
