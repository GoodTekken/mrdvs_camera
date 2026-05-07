#include "filters.h"

//直通滤波
void PassFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>& cloud_filtered,float limit_min,float limit_max,std::string axis)
{
    pcl::PassThrough<pcl::PointXYZ> pass; //创建滤波器 pass
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(limit_min, limit_max);          //过滤z轴方向上超出范围的数据
    //pass.setFilterLimitsNegative (true);
    pass.filter(cloud_filtered);
}

//直通滤波
void PassFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloud_filtered,float limit_min,float limit_max,std::string axis)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass; //创建滤波器 pass
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(limit_min, limit_max);          //过滤z轴方向上超出范围的数据
    //pass.setFilterLimitsNegative (true);
    pass.filter(cloud_filtered);
}

void PassThrough_X_Y_Z_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>& cloud_filtered)
{
    PassFilter(cloud, *cloud, 1, 3, "x");     //352x264=92928  =>  16910
    PassFilter(cloud, *cloud, -1, 1, "y");     //        16910  =>  16907
    PassFilter(cloud, cloud_filtered, -0.4, 0.1, "z");     //  -0.1 0.1      16907  =>  16907
}

void BoxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud,
               pcl::PointCloud<pcl::PointXYZRGB>& filterBox_cloudRGB,
               Eigen::Vector4f min_pt,
               Eigen::Vector4f max_pt,
               bool IsOutside)
{
    pcl::CropBox<pcl::PointXYZRGB> box_filter;
    box_filter.setMin(min_pt);
    box_filter.setMax(max_pt);
    box_filter.setInputCloud(origincloud);
    box_filter.setNegative(IsOutside); //true: ouside   false:inside
    box_filter.filter(filterBox_cloudRGB);
}

void VoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud,
                     pcl::PointCloud<pcl::PointXYZRGB>& voxelGridfilter_cloudRGB,
                     float leaf_x,
                     float leaf_y,
                     float leaf_z
                     )
{
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelgard;
    voxelgard.setInputCloud(origincloud);
    voxelgard.setLeafSize(leaf_x, leaf_y, leaf_z);//体素大小设置为5*5*5mm
    voxelgard.filter(voxelGridfilter_cloudRGB);                        //16907  =>  8997
}

void NormalsFilter(pcl::PointCloud<pcl::Normal>::Ptr normalCloud, pcl::PointCloud<pcl::PointXYZRGB>& cloud_filtered)
{
    for (std::size_t i = 0; i < normalCloud->size(); i++)
    {

       if (abs(normalCloud->at(i).normal_z) > 0.1)  //即法向量开口向上 0.2
       {
           cloud_filtered.at(i).x = 0;
       }
       else if (abs(normalCloud->at(i).normal_y) > 0.8)
       {
           cloud_filtered.at(i).x = 0;
       }
       //else if (abs(cloud_normals->at(i).normal_x) < 0.8)
       {
           //cloud_filtered->at(i).x = 0;
       }
    }
}

void RadiusOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr origincloud,
                                double raduis,
                                int min_pts,
                                bool negative,
                                pcl::PointCloud<pcl::PointXYZRGB>& cloud_filtered)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(origincloud);
    sor.setRadiusSearch(raduis);       //0.01
    sor.setMinNeighborsInRadius(min_pts);  //2
    sor.setNegative(negative);          //false
    sor.filter(cloud_filtered);
}

