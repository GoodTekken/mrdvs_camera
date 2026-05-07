#include "features.h"

void NormalEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr originCloud, pcl::PointCloud<pcl::Normal>& normalCloud,int k)
{

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;  //法线估计对象
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud(originCloud);
    ne.setInputCloud(originCloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(k);  //12:需要1300ms  5:需要1000ms，但过滤点会失败  default(15)30
    ne.compute(normalCloud);
}

void BoundaryEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr originCloud, pcl::PointCloud<pcl::Normal>::Ptr normalCloud,int k,pcl::PointCloud<pcl::Boundary>& boundaries)
{
    pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary> boundEst;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud(originCloud);
    boundEst.setInputCloud(originCloud);
    boundEst.setInputNormals(normalCloud);
    boundEst.setSearchMethod(tree);
    boundEst.setKSearch(k);
    boundEst.setAngleThreshold(M_PI / 2);
    boundEst.compute(boundaries);
}


void BoundaryCloudPickup(pcl::PointCloud<pcl::PointXYZRGB>::Ptr originCloud,pcl::PointCloud<pcl::Boundary>& boundaries,pcl::PointCloud<pcl::PointXYZRGB>& cloud_boundary)
{
    cloud_boundary.width = originCloud->points.size();
    cloud_boundary.height = 1;
    cloud_boundary.points.resize(cloud_boundary.width * cloud_boundary.height);
    int j = 0;
    for (std::size_t i = 0; i < originCloud->points.size(); i++)
    {
        if (boundaries.points[i].boundary_point != 0)
        {
            cloud_boundary.points[j].x = originCloud->points[i].x;
            cloud_boundary.points[j].y = originCloud->points[i].y;
            cloud_boundary.points[j].z = originCloud->points[i].z;
            cloud_boundary.points[j].r = 255;
            cloud_boundary.points[j].g = 0;
            cloud_boundary.points[j].b = 0;
            j++;
        }
        continue;
    }
    cloud_boundary.width = j;
    cloud_boundary.points.resize(cloud_boundary.width * cloud_boundary.height);
}


