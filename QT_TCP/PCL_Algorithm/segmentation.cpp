#include "segmentation.h"

void PlaneSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud, pcl::PointIndices& plane_inliers, pcl::ModelCoefficients& plane_coefficients)
{
    pcl::SACSegmentation<pcl::PointXYZRGB> plane_seg;
    plane_seg.setOptimizeCoefficients(true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);//SACMODEL_PLANE  SACMODEL_LINE
    plane_seg.setMethodType(pcl::SAC_RANSAC);//pcl::SAC_RANSAC
    plane_seg.setDistanceThreshold(0.01); //0.01
    plane_seg.setMaxIterations(30);   //设置最大的迭代次数，默认是30
    plane_seg.setProbability(0.99);  //至少一个样本不包含离群点的概率（默认值为0.99)
    plane_seg.setInputCloud(origincloud);            //输入托盘点云数据
    plane_seg.segment(plane_inliers, plane_coefficients);//得到托盘法向量
}

void PointNearThePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud, pcl::ModelCoefficients& plane_coefficients, float radiusSearch, pcl::PointCloud<pcl::PointXYZRGB>& cloudOutput)
{
    cloudOutput.clear();
    pcl::PointXYZRGB searchPoint;
    for (std::size_t i = 0; i < origincloud->size(); i++)
    {
       searchPoint = origincloud->at(i);
       float caculate_x = (-plane_coefficients.values[3] - searchPoint.y * plane_coefficients.values[1] - searchPoint.z * plane_coefficients.values[2]) / plane_coefficients.values[0];

       if (abs(caculate_x- searchPoint.x)< radiusSearch)  //0.02  0.035 0.050
       {
           searchPoint.x = caculate_x;
           cloudOutput.push_back(searchPoint);
       }
    }
}

void EuclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud,
                                double tolerance,
                                int max_cluster_size,
                                int min_cluster_size,
                                std::vector<pcl::PointIndices>& PointIndices)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_seg(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    tree_seg->setInputCloud(origincloud);
    ec.setInputCloud(origincloud);
    ec.setClusterTolerance(tolerance);   //0.01
    ec.setMaxClusterSize(max_cluster_size);     //1200
    ec.setMinClusterSize(min_cluster_size);       //50
    ec.setSearchMethod(tree_seg);
    ec.extract(PointIndices);
}
