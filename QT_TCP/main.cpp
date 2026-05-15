#include <QApplication>
#include <ros/ros.h>
#include <pcl/common/time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>


#include "main.h"
#include "serverwidget.h"
#include "clientwidget.h"
#include "clientPDS.h"
#include "Math/UserMatrix.h"

#include "setTemplateCloud.h"
#include <std_msgs/String.h>

#include "Basler.h"
#include "PCL_Algorithm/algo_head.h"

#define PI 3.1415926

using namespace std;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_coe(new pcl::PointCloud<pcl::PointXYZ>());

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZRGB PointRGBT;
typedef pcl::PointCloud<PointRGBT> PointRGBCloudT;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
pcl::StopWatch stopWatch;

// 创建点云渲染对象，导入待渲染文件
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemplate(new pcl::PointCloud<pcl::PointXYZ>);        //声明cloudTemplate，用于构造托盘的模型
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTemplateRGB(new pcl::PointCloud<pcl::PointXYZRGB>);        //声明cloudTemplate，用于构造托盘的模型
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);                //声明cloud，用于保存输入点云
pcl::PointCloud<pcl::PointXYZ>::Ptr firstcloud(new pcl::PointCloud<pcl::PointXYZ>);                //声明cloud，用于保存输入点云
pcl::PointCloud<pcl::PointXYZ>::Ptr secondcloud(new pcl::PointCloud<pcl::PointXYZ>);                //声明cloud，用于保存输入点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr secondcloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);                //声明cloud，用于保存输入点云

pcl::PointCloud<pcl::PointXYZRGB>::Ptr totalRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr safezonecloud(new pcl::PointCloud<pcl::PointXYZRGB>);        //声明safezonecloud，用于保存输入点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr palletzonecloud(new pcl::PointCloud<pcl::PointXYZRGB>);        //声明safezonecloud，用于保存输入点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundzonecloud(new pcl::PointCloud<pcl::PointXYZRGB>);        //声明groundzonecloud，用于保存输入点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotation_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterBoxOutside_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterBoxInside_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterCoreArea_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterDetectRoomTemp_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pallet_boxFilter_up_left_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pallet_boxFilter_up_right_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pallet_boxFilter_up_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pallet_boxFilter_down_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);                //声明cloud，用于保存输入点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pallet_boxFilter_reduce_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);                //声明cloud，用于保存输入点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pallet_voxelFilter_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);                //声明cloud，用于保存输入点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pallet_normalFilter_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);                //声明cloud，用于保存输入点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_output_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);                //声明cloud，用于保存输入点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr kdTreeCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);          //kdtree 寻找距离托盘模型最近的点
pcl::PointCloud<pcl::PointXYZRGB>::Ptr kdTreeVoxelCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);          //kdtree 寻找距离托盘模型最近的点
pcl::PointCloud<pcl::PointXYZRGB>::Ptr kdfillCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);          //kdtree 寻找距离托盘模型最近的点

pcl::PointCloud<pcl::PointXYZRGB>::Ptr kdTreeVoxelAfterDilateCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);      //

pcl::PointCloud<pcl::PointXYZRGB>::Ptr palletDetect_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);      //

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary_after_digate(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary_after_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundDilateCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterGroundBoxInside_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr fill_point_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);                //声明cloud，用于保存输入点云

bool groundExist;
float increaseCameraHeight;

QSharedPointer<AGVExternalPara> g_AGV_ptr = AGVExternalPara::getIntance();

pcl::PointCloud<pcl::PointXYZ>::Ptr origin(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>());

pcl::visualization::CloudViewer viewer1("QT_TCP");//直接创造一个显示窗口

Basler basler;
Eigen::Vector4f centroid;
Eigen::Vector4f rightcentroid;                  // 质心
Eigen::Vector4f middlecentroid;                   // 质心
Eigen::Vector4f leftcentroid;                   // 质心

Eigen::Vector4f right_up_centroid;                  // 质心
Eigen::Vector4f middle_up_centroid;                   // 质心
Eigen::Vector4f left_up_centroid;                   // 质心

Eigen::Vector4f right_down_centroid;                  // 质心
Eigen::Vector4f middle_down_centroid;                   // 质心
Eigen::Vector4f left_down_centroid;                   // 质心

float round(float a,int n)
{
    return a>0?(((int)(a*pow(10,n)+0.5))*pow(10,-n)):(((int)(a*pow(10,n)-0.5))*pow(10,-n));
}


int safeZoneMethod()
{
    //1.Zone Select
    Eigen::Vector4f min_pt(basler.safezone.min_x,basler.safezone.min_y,basler.safezone.min_z,1.0);
    Eigen::Vector4f max_pt(basler.safezone.max_x,basler.safezone.max_y,basler.safezone.max_z,1.0);
    BoxFilter(rotation_cloudRGB,*filterBoxOutside_cloudRGB,min_pt,max_pt,true); // true: ouside   false:inside
    BoxFilter(rotation_cloudRGB,*filterBoxInside_cloudRGB,min_pt,max_pt,false); // true: ouside   false:inside

    //2.Color Set
    //colorPointSet(filterBoxInside_cloudRGB, 255, 0, 0);

    //3.Show message
    g_existpoint = filterBoxInside_cloudRGB->size();
    //cout << "safezone filterBoxOutside_cloudRGB:" <<filterBoxOutside_cloudRGB->size()<< endl;
    //cout << "safezone filterBoxInside_cloudRGB:" <<filterBoxInside_cloudRGB->size()<< endl;

    //4.Initialization
    basler.safezoneExist = false;
    basler.safezone_max_left = 0;
    basler.safezone_max_right = 0;

    //5.Calculate the core area base on "pallet_length"
    float detect_density = 0.01;  //密度，5mm
    float core_right = basler.safezone.max_y-basler.pallet_length; //0.65-1.1=-0.45
    float core_left = basler.pallet_length - basler.safezone.max_y; //1.1 - 0.65= 0.45
    Eigen::Vector4f min_core_pt(basler.safezone.min_x,core_right,basler.safezone.min_z,1.0);
    Eigen::Vector4f max_core_pt(basler.safezone.max_x,core_left,basler.safezone.max_z,1.0);
    BoxFilter(filterBoxInside_cloudRGB,*filterCoreArea_cloudRGB,min_core_pt,max_core_pt,false); // true: ouside   false:inside

    //6,Process
    if(filterCoreArea_cloudRGB->size()>= basler.safezone_pointCountForObjectExist)
    {
        basler.pub_safezoneExist =false;
        basler.pub_safezone_max_right = 0;
        basler.pub_safezone_max_left = 0;
        g_safezoneExist = false;
        g_safezone_max_left = 0;
        g_safezone_max_right = 0;
    }
    else
    {
        for(float start_y = basler.safezone.min_y;start_y<=(basler.safezone.max_y-basler.pallet_length);start_y+=detect_density)  //(-0.65)-----(-0.45)
        {
            float detect_right_y = start_y;          //-0.65
            float detect_left_y = start_y + basler.pallet_length;  //-0.65+11=4.5
            Eigen::Vector4f min_pt(basler.safezone.min_x,detect_right_y,basler.safezone.min_z,1.0);
            Eigen::Vector4f max_pt(basler.safezone.max_x,detect_left_y,basler.safezone.max_z,1.0);
            BoxFilter(filterBoxInside_cloudRGB,*filterDetectRoomTemp_cloudRGB,min_pt,max_pt,false); // true: ouside   false:inside
            if(filterDetectRoomTemp_cloudRGB->size() < basler.safezone_pointCountForObjectExist)
            {
                if(basler.safezoneExist == false)
                {
                    basler.safezoneExist = true;
                    float delta = start_y + basler.pallet_length/2.0;
                    basler.safezone_max_right = delta;
                    basler.safezone_max_left = delta;

                    basler.pub_safezoneExist =true;
                    basler.pub_safezone_max_right = round(delta,3);
                    basler.pub_safezone_max_left = round(delta,3);
                    cout<<"pub_safezone_max_right"<<basler.pub_safezone_max_right<<endl;
                    cout<<"delta"<<delta<<endl;

                    g_safezoneExist = true;
                    g_safezone_max_left = basler.pub_safezone_max_left;
                    g_safezone_max_right = basler.pub_safezone_max_right;
                }
                else
                {
                    float delta = start_y + basler.pallet_length/2.0;
                    basler.safezone_max_left = delta;
                    basler.pub_safezone_max_left = round(delta,3);
                    g_safezone_max_left = basler.pub_safezone_max_left;
                }
            }

            if(start_y==(basler.safezone.max_y-basler.pallet_length))
            {
                if(basler.safezoneExist == false)
                {
                    basler.pub_safezoneExist =false;
                    basler.pub_safezone_max_right = 0;
                    basler.pub_safezone_max_left = 0;
                    g_safezoneExist = false;
                    g_safezone_max_left = 0;
                    g_safezone_max_right = 0;
                }
            }
        }

    }
    return 0;
}

int failCodeAndReset(int failcode,Eigen::Vector4f leftcentroid, Eigen::Vector4f middlecentroid, Eigen::Vector4f rightcentroid)
{
    g_detectFlag = true;
    g_errorCode = failcode;
    g_leftPoint = {leftcentroid.x(),leftcentroid.y(),leftcentroid.z()};
    g_certenPoint = {middlecentroid.x(),middlecentroid.y(),middlecentroid.z()};
    g_rightPoint = {rightcentroid.x(),rightcentroid.y(),rightcentroid.z()};
    return failcode;
}

int palletDetectMethod_Tian()
{
    //1.Zone Select
    Eigen::Vector4f min_ground_pt(basler.groundzone.min_x,basler.groundzone.min_y,basler.groundzone.min_z,1.0);
    Eigen::Vector4f max_ground_pt(basler.groundzone.max_x,basler.groundzone.max_y,basler.groundzone.max_z,1.0);
    BoxFilter(rotation_cloudRGB,*filterGroundBoxInside_cloudRGB,min_ground_pt,max_ground_pt,false); // true: ouside   false:inside
    //2.Color Set
    colorPointSet(filterGroundBoxInside_cloudRGB, 255, 0, 0);
    //3.Show message
    //cout << "filterGroundBoxInside_cloudRGB:" <<filterGroundBoxInside_cloudRGB->size()<< endl;
    if(filterGroundBoxInside_cloudRGB->size() < basler.ground_pointCountForGroundExist)
    {
        groundExist = false;
        increaseCameraHeight = 0;
    }
    else
    {
        groundExist = true;
        increaseCameraHeight = basler.ground_increaseCameraHeight;
    }


    //1.Zone Select   ===>   pallet_boxFilter_down_cloudRGB
    Eigen::Vector4f min_pt(basler.pallet_roi_min_x,basler.pallet_roi_min_y,basler.pallet_roi_min_z - increaseCameraHeight,1.0);
    Eigen::Vector4f max_pt(basler.pallet_roi_max_x,basler.pallet_roi_max_y,basler.pallet_roi_max_z - increaseCameraHeight,1.0);
    BoxFilter(rotation_cloudRGB,*pallet_boxFilter_down_cloudRGB,min_pt,max_pt,false); // true: ouside   false:inside

    //2. Small the zone for correct palne   ====>   pallet_boxFilter_reduce_cloudRGB
    Eigen::Vector4f min_pt2(basler.pallet_roi_min_x,basler.pallet_roi_min_y,basler.pallet_roi_min_z+0.03 -increaseCameraHeight,1.0);
    Eigen::Vector4f max_pt2(basler.pallet_roi_max_x,basler.pallet_roi_max_y,basler.pallet_roi_max_z-0.03 -increaseCameraHeight,1.0);
    BoxFilter(rotation_cloudRGB,*pallet_boxFilter_reduce_cloudRGB,min_pt2,max_pt2,false); // true: ouside   false:inside

    //3,VoxelGridFilter 体素大小设置为5*5*5mm
    float leaf_x = 0.005f; float leaf_y = 0.005f; float leaf_z = 0.005f;
    VoxelGridFilter(pallet_boxFilter_reduce_cloudRGB,*pallet_voxelFilter_cloudRGB,leaf_x,leaf_y,leaf_z);  //16907  =>  8997
    //3,Return FailCode
    if(pallet_voxelFilter_cloudRGB->size()<2000)
    {
        return failCodeAndReset(PDS_NOT_ENOUGH_PIXELS_ON_THE_PLANE,leftcentroid,middlecentroid,rightcentroid);
    }

    //4.法向量滤波
    pcl::copyPointCloud(*pallet_voxelFilter_cloudRGB,*pallet_normalFilter_cloudRGB);
    int search_k = 30;
    NormalEstimation(pallet_normalFilter_cloudRGB,*cloud_normals,search_k);

    //5.Filt the normal point
    NormalsFilter(cloud_normals,*pallet_normalFilter_cloudRGB);
    PassFilter(pallet_normalFilter_cloudRGB, *pallet_normalFilter_cloudRGB, 1, 3, "x");     //30000  =>  5000
    //5,Return FailCode
    if(pallet_normalFilter_cloudRGB->size() < 100)
    {
        return failCodeAndReset(PDS_NOT_ENOUGH_PIXELS_ON_THE_Normal_PLANE,leftcentroid,middlecentroid,rightcentroid);
    }

    //6,Calculate the plane
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    PlaneSegment(pallet_normalFilter_cloudRGB,*plane_inliers, *plane_coefficients);
    g_roll =  plane_coefficients->values[0];   //X
    g_pitch = plane_coefficients->values[1];  //Y
    g_yaw = plane_coefficients->values[2];    //Z
    std::cerr << "A:" << plane_coefficients->values[0] << std::endl;
    std::cerr << "B:" << plane_coefficients->values[1] << std::endl;
    std::cerr << "C:" << plane_coefficients->values[2] << std::endl;
    std::cerr << "D:" << plane_coefficients->values[3] << std::endl;

    //7,Search the point around the plane
    float radiusSearch = 0.050; //0.02  0.035 0.050
    PointNearThePlane(pallet_boxFilter_down_cloudRGB, *plane_coefficients, radiusSearch, *kdTreeCloudRGB);

    //8,VoxelGridFilter 体素大小设置为1*1*1mm
    leaf_x = 0.001f; leaf_y = 0.001f; leaf_z = 0.001f;
    VoxelGridFilter(kdTreeCloudRGB,*kdTreeVoxelCloudRGB,leaf_x,leaf_y,leaf_z);
    //MergeCloud(kdTreeVoxelCloudRGB,*totalRGB);

    //-----------------边缘检测起点-------------------------//
    //计算法线
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    NormalEstimation(kdTreeVoxelCloudRGB, *normals,20);

    //判断边缘点
    pcl::PointCloud<pcl::Boundary> boundaries;
    BoundaryEstimation(kdTreeVoxelCloudRGB, normals,20,boundaries);

    //提取边缘点重组点云
    BoundaryCloudPickup(kdTreeVoxelCloudRGB,boundaries,*cloud_boundary);
    cout << "Before dilate: cloud_boundary:" <<cloud_boundary->size()<< endl;
    //MergeCloud(cloud_b,*totalRGB);

    //过滤离散点
    RadiusOutlierRemovalFilter(cloud_boundary,0.01,2,false,*cloud_boundary_after_filter);
    cout << "Before dilate: cloud_boundary_after_filter:" <<cloud_boundary_after_filter->size()<< endl;
//    colorPointSet(cloud_boundary_after_filter,100,100,100);
//    MergeCloud(cloud_boundary_after_filter,*totalRGB);
    //-----------------边缘检测终点-------------------------//

    //=================add:dilate Begin=================//
    BoundaryDilate(cloud_boundary,plane_coefficients,0.005,*boundDilateCloudRGB);
    pcl::copyPointCloud(*kdTreeVoxelCloudRGB,*kdTreeVoxelAfterDilateCloudRGB);
    MergeCloud(boundDilateCloudRGB,*kdTreeVoxelAfterDilateCloudRGB);

    //again:
    //判断边缘点
    NormalEstimation(kdTreeVoxelAfterDilateCloudRGB, *normals,20);
    BoundaryEstimation(kdTreeVoxelAfterDilateCloudRGB, normals,20,boundaries);

    //提取边缘点重组点云
    BoundaryCloudPickup(kdTreeVoxelAfterDilateCloudRGB,boundaries,*cloud_boundary_after_digate);
    cout << "After dilate: cloud_boundary_after_digate:" <<cloud_boundary_after_digate->size()<< endl;

    //过滤离散点
    RadiusOutlierRemovalFilter(cloud_boundary_after_digate,0.01,2,false,*cloud_boundary_after_filter);
    cout << "After dilate: cloud_boundary_after_filter:" <<cloud_boundary_after_filter->size()<< endl;

    colorPointSet(cloud_boundary_after_filter,100,100,100);
    //MergeCloud(cloud_boundary_after_filter,*totalRGB);
    //=================add:dilate End=================//

//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_seg(new pcl::search::KdTree<pcl::PointXYZRGB>());
    std::vector<pcl::PointIndices> boundPoints_filter_seg_indices;
    EuclideanClusterExtraction(cloud_boundary_after_filter,0.01,1200,50,boundPoints_filter_seg_indices);

    if(normals != nullptr)
    {
        normals = nullptr;
    }
    int j = 0;
    int hole_count = 0;

    for(std::vector<pcl::PointIndices>::const_iterator it = boundPoints_filter_seg_indices.begin();it!=boundPoints_filter_seg_indices.end();it++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundPoints_filter_seg(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(std::vector<int>::const_iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
        {
            boundPoints_filter_seg->points.push_back(cloud_boundary_after_filter->points[*pit]);
        }
        boundPoints_filter_seg->width = boundPoints_filter_seg->points.size();
        boundPoints_filter_seg->height = 1;
        boundPoints_filter_seg->is_dense =true;


        pcl::PointXYZRGB minPt,maxPt;
        pcl::getMinMax3D(*boundPoints_filter_seg,minPt,maxPt);
        float y_delta = maxPt.y-minPt.y;
        float z_delta = maxPt.z-minPt.z;
        if((y_delta<0.4) && (y_delta>0.05) && (z_delta>0.05) &&(z_delta<0.15))
        {
            //fillPoint(kdTreeVoxelCloudRGB,plane_coefficients,minPt,maxPt,*kdfillCloudRGB);   //Method_1
            fillPoint(kdTreeVoxelCloudRGB,boundPoints_filter_seg,plane_coefficients,minPt,maxPt,*kdfillCloudRGB);//Method_2 Method_3

            cout << "kdfillCloudRGB:" <<kdfillCloudRGB->size()<< endl;
            pcl::compute3DCentroid(*kdfillCloudRGB, centroid);    // 齐次坐标，（c0,c1,c2,1）
            if(abs(centroid.y())<0.45)
            {
                if(kdfillCloudRGB->size()>600)
                {
                    if(centroid.y()>0)
                    {
                        leftcentroid = centroid;
                    }
                    if(centroid.y()<0)
                    {
                        rightcentroid = centroid;
                    }
                    //MergeCloud(kdfillCloudRGB,*totalRGB);
                }
                //MergeCloud(boundPoints_filter_seg,*totalRGB);
                hole_count++;
            }
        }
        j++;
        if(boundPoints_filter_seg != nullptr)
        {
            boundPoints_filter_seg = nullptr;
        }
    }

    if(plane_coefficients != nullptr)
    {
        plane_coefficients = nullptr;
    }
    if(plane_inliers != nullptr)
    {
        plane_inliers = nullptr;
    }
    cout << "########-----------j:" <<j<< endl;
    cout << "########--hole_count:" <<hole_count<< endl;
    std::cerr << "左质心结果:" << leftcentroid.x() << "  " << leftcentroid.y() << "  " << leftcentroid.z()<< std::endl;
    std::cerr << "右质心结果:" << rightcentroid.x() << "  " << rightcentroid.y()  << "  " << rightcentroid.z() << std::endl;

    double time4 = stopWatch.getTime();
    std::cerr<< "elapsed_time:" << time4 << std::endl;

    float center_x = (leftcentroid.x()+rightcentroid.x())/2.0;
    float center_y = (leftcentroid.y()+rightcentroid.y())/2.0;
    float center_z = (leftcentroid.z()+rightcentroid.z())/2.0;
    middlecentroid = Eigen::Vector4f(center_x,center_y,center_z,1);
    if((rightcentroid.x()>0)&&(leftcentroid.x()>0)&&abs(center_y)<0.3&& (hole_count==2))
    {
        return failCodeAndReset(PDS_NO_ERRORS,leftcentroid,middlecentroid,rightcentroid);
    }
    else
    {
        return failCodeAndReset(PDS_COMPOSED_CENTER_STRINGER_RATIO_ERROR,leftcentroid,middlecentroid,rightcentroid);
    }
}


//int palletDetectMethod2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGB>& outputCloud)
int palletDetectMethod_All()
{
    //1.Zone Select
    Eigen::Vector4f min_ground_pt(basler.groundzone.min_x,basler.groundzone.min_y,basler.groundzone.min_z,1.0);
    Eigen::Vector4f max_ground_pt(basler.groundzone.max_x,basler.groundzone.max_y,basler.groundzone.max_z,1.0);
    BoxFilter(rotation_cloudRGB,*filterGroundBoxInside_cloudRGB,min_ground_pt,max_ground_pt,false); // true: ouside   false:inside
    //2.Color Set
    colorPointSet(filterGroundBoxInside_cloudRGB, 255, 0, 0);
    //3.Show message
    //cout << "filterGroundBoxInside_cloudRGB:" <<filterGroundBoxInside_cloudRGB->size()<< endl;
    if(filterGroundBoxInside_cloudRGB->size() < basler.ground_pointCountForGroundExist)
    {
        groundExist = false;
        increaseCameraHeight = 0;
    }
    else
    {
        groundExist = true;
        increaseCameraHeight = basler.ground_increaseCameraHeight;
    }

    //1.Zone Select up  ===>   pallet_boxFilter_up_cloudRGB
    Eigen::Vector4f min_pt(basler.safezone.min_x,basler.safezone.min_y,basler.safezone.min_z - increaseCameraHeight,1.0);
    Eigen::Vector4f max_pt(basler.safezone.max_x,-0.35,basler.safezone.max_z - increaseCameraHeight,1.0);
    BoxFilter(rotation_cloudRGB,*pallet_boxFilter_up_right_cloudRGB,min_pt,max_pt,false); // true: ouside   false:inside
    MergeCloud(pallet_boxFilter_up_right_cloudRGB,*pallet_boxFilter_up_cloudRGB);
    Eigen::Vector4f min_pt_1(basler.safezone.min_x,0.35,basler.safezone.min_z - increaseCameraHeight,1.0);
    Eigen::Vector4f max_pt_1(basler.safezone.max_x,basler.safezone.max_y,basler.safezone.max_z - increaseCameraHeight,1.0);
    BoxFilter(rotation_cloudRGB,*pallet_boxFilter_up_left_cloudRGB,min_pt_1,max_pt_1,false); // true: ouside   false:inside
    MergeCloud(pallet_boxFilter_up_left_cloudRGB,*pallet_boxFilter_up_cloudRGB);

    //2,VoxelGridFilter 体素大小设置为5*5*5mm
    float leaf_x = 0.005f; float leaf_y = 0.005f; float leaf_z = 0.005f;
    VoxelGridFilter(pallet_boxFilter_up_cloudRGB,*pallet_voxelFilter_cloudRGB,leaf_x,leaf_y,leaf_z);  //16907  =>  8997
    //3,Return FailCode
    if(pallet_voxelFilter_cloudRGB->size()<1000)
    {
        return failCodeAndReset(PDS_NOT_ENOUGH_PIXELS_ON_THE_PLANE,leftcentroid,middlecentroid,rightcentroid);
    }
    //3.法向量滤波
    pcl::copyPointCloud(*pallet_voxelFilter_cloudRGB,*pallet_normalFilter_cloudRGB);
    int search_k = 30;
    NormalEstimation(pallet_normalFilter_cloudRGB,*cloud_normals,search_k);
    NormalsFilter(cloud_normals,*pallet_normalFilter_cloudRGB);
    PassFilter(pallet_normalFilter_cloudRGB, *pallet_normalFilter_cloudRGB, 0.5, 1.5, "x");     //30000  =>  5000

    //4
    pcl::PointIndices::Ptr inliers_up(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_up(new pcl::ModelCoefficients);
    PlaneSegment(pallet_normalFilter_cloudRGB,*inliers_up, *coefficients_up);
    g_roll =  coefficients_up->values[0];   //X
    g_pitch = coefficients_up->values[1];  //Y
    g_yaw = coefficients_up->values[2];    //Z

//    std::cerr << "A:" << coefficients_up->values[0] << std::endl;
//    std::cerr << "B:" << coefficients_up->values[1] << std::endl;
//    std::cerr << "C:" << coefficients_up->values[2] << std::endl;
//    std::cerr << "D:" << coefficients_up->values[3] << std::endl;
    //6,Search the point around the plane
    float radiusSearch = 0.020; //0.02  0.035 0.050
    PointNearThePlane(pallet_boxFilter_up_cloudRGB, *coefficients_up, radiusSearch, *kdTreeCloudRGB);
    colorPointSet(kdTreeCloudRGB, 0, 0, 255);
    MergeCloud(kdTreeCloudRGB,*totalRGB);

    //7,segmentation
    std::vector<pcl::PointIndices> boundPoints_filter_seg_indices;
    EuclideanClusterExtraction(kdTreeCloudRGB,0.01,6000,2000,boundPoints_filter_seg_indices); //4000

    int j = 0;
    int leg_up_count = 0;

    for(std::vector<pcl::PointIndices>::const_iterator it = boundPoints_filter_seg_indices.begin();it!=boundPoints_filter_seg_indices.end();it++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundPoints_filter_seg(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(std::vector<int>::const_iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
        {
            boundPoints_filter_seg->points.push_back(kdTreeCloudRGB->points[*pit]);
        }
        boundPoints_filter_seg->width = boundPoints_filter_seg->points.size();
        boundPoints_filter_seg->height = 1;
        boundPoints_filter_seg->is_dense =true;


        pcl::PointXYZRGB minPt,maxPt;
        pcl::getMinMax3D(*boundPoints_filter_seg,minPt,maxPt);
        float y_delta = maxPt.y-minPt.y;  //0.15
        float z_delta = maxPt.z-minPt.z;  //0.05
        if((y_delta<0.3) && (y_delta>0.1) && (z_delta>0.02) &&(z_delta<0.15))
        {
            pcl::compute3DCentroid(*boundPoints_filter_seg, centroid);    // 齐次坐标，（c0,c1,c2,1）
                if(boundPoints_filter_seg->size()>1000)
                {
                    if(centroid.y()>0)
                    {
                        left_up_centroid = centroid;
                    }
                    if(centroid.y()<0)
                    {
                        right_up_centroid = centroid;
                    }
                    //MergeCloud(boundPoints_filter_seg,*totalRGB);
                    leg_up_count++;
                }
//                MergeCloud(boundPoints_filter_seg,*totalRGB);
                 //cout << "########-----------boundPoints_filter_seg->size():" <<boundPoints_filter_seg->size() << "  y_delta:"<<y_delta << "  z_delta:"<< z_delta << endl;
        }
        j++;
//        cout << "########-----------boundPoints_filter_seg->size():" <<boundPoints_filter_seg->size() << "  y_delta:"<<y_delta << "  z_delta:"<< z_delta << endl;
//        ########-----------boundPoints_filter_seg->size():1553
//        ########-----------boundPoints_filter_seg->size():1532
//        ########-----------boundPoints_filter_seg->size():1441
        if(boundPoints_filter_seg != nullptr)
        {
            boundPoints_filter_seg = nullptr;
        }
    }

//    cout << "########--leg_up_count:" <<leg_up_count<< endl;
//    std::cerr << "左_up_质心结果:" << left_up_centroid.x() << "  " << left_up_centroid.y() << "  " << left_up_centroid.z()<< std::endl;
//    std::cerr << "右_up_质心结果:" << right_up_centroid.x() << "  " << right_up_centroid.y()  << "  " << right_up_centroid.z() << std::endl;

    //1. 1.Zone Select   ===>   pallet_boxFilter_up_cloudRGB
    Eigen::Vector4f min_pt2(basler.pallet_roi_min_x,basler.pallet_roi_min_y,basler.pallet_roi_min_z -increaseCameraHeight,1.0);
    Eigen::Vector4f max_pt2(basler.pallet_roi_max_x,basler.pallet_roi_max_y,basler.pallet_roi_max_z -increaseCameraHeight,1.0);
    BoxFilter(rotation_cloudRGB,*pallet_boxFilter_down_cloudRGB,min_pt2,max_pt2,false); // true: ouside   false:inside
//    //2
//    pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> ne_down;
//    pcl::PointCloud<pcl::Normal>::Ptr normals_down(new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_down(new pcl::search::KdTree<pcl::PointXYZRGB>());
//    ne_down.setSearchMethod(tree_down);
//    ne_down.setInputCloud(pallet_boxFilter_down_cloudRGB);
//    ne_down.setKSearch(50);
//    ne_down.compute(*normals_down);
//    //3
//    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB,pcl::Normal> seg_down;
//    pcl::PointIndices::Ptr inliers_down(new pcl::PointIndices);
//    pcl::ModelCoefficients::Ptr coefficients_down(new pcl::ModelCoefficients);
//    seg_down.setOptimizeCoefficients(true);
//    seg_down.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
//    seg_down.setMethodType(pcl::SAC_RANSAC);
//    seg_down.setDistanceThreshold(0.05);
//    seg_down.setInputCloud(pallet_boxFilter_down_cloudRGB);
//    seg_down.setInputNormals(normals_down);
//    //4
//    Eigen::Vector3f axis_down = Eigen::Vector3f(1.0,0,0);
//    seg_down.setAxis(axis_down);
//    seg_down.setEpsAngle(10.0f*(M_PI /180.0f));
//    seg_down.segment(*inliers_down, *coefficients_down);
//    if(inliers_down->indices.empty()){
//        PCL_ERROR("Could not estimate a plannar model for the given dataset.");
//        return -1;
//    }
//    //5
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud_down(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::ExtractIndices<pcl::PointXYZRGB> extract_down;
//    extract_down.setInputCloud(pallet_boxFilter_down_cloudRGB);
//    extract_down.setIndices(inliers_down);
//    extract_down.setNegative(false);
//    extract_down.filter(*plane_cloud_down);
//    std::cout << "Plane cloud down has "<<plane_cloud_down->size() << " points." << std::endl;
//    colorPointSet(plane_cloud_down, 0, 255, 0);
//    MergeCloud(plane_cloud_down,*totalRGB);

    //2,VoxelGridFilter 体素大小设置为5*5*5mm
    leaf_x = 0.005f; leaf_y = 0.005f; leaf_z = 0.005f;
    VoxelGridFilter(pallet_boxFilter_down_cloudRGB,*pallet_voxelFilter_cloudRGB,leaf_x,leaf_y,leaf_z);  //16907  =>  8997
    //3,Return FailCode
    if(pallet_voxelFilter_cloudRGB->size()<1000)
    {
        return failCodeAndReset(PDS_NOT_ENOUGH_PIXELS_ON_THE_PLANE,leftcentroid,middlecentroid,rightcentroid);
    }
    //3.法向量滤波
    pcl::copyPointCloud(*pallet_voxelFilter_cloudRGB,*pallet_normalFilter_cloudRGB);
    search_k = 30;
    NormalEstimation(pallet_normalFilter_cloudRGB,*cloud_normals,search_k);
    NormalsFilter(cloud_normals,*pallet_normalFilter_cloudRGB);
    PassFilter(pallet_normalFilter_cloudRGB, *pallet_normalFilter_cloudRGB, 0.5, 1.5, "x");     //30000  =>  5000

    //4
    pcl::PointIndices::Ptr inliers_down(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_down(new pcl::ModelCoefficients);
    PlaneSegment(pallet_normalFilter_cloudRGB,*inliers_down, *coefficients_down);
    g_roll =  coefficients_down->values[0];   //X
    g_pitch = coefficients_down->values[1];  //Y
    g_yaw = coefficients_down->values[2];    //Z
//    std::cerr << "A:" << coefficients_down->values[0] << std::endl;
//    std::cerr << "B:" << coefficients_down->values[1] << std::endl;
//    std::cerr << "C:" << coefficients_down->values[2] << std::endl;
//    std::cerr << "D:" << coefficients_down->values[3] << std::endl;
    //6,Search the point around the plane
    radiusSearch = 0.020; //0.02  0.035 0.050
    PointNearThePlane(pallet_boxFilter_down_cloudRGB, *coefficients_down, radiusSearch, *kdTreeCloudRGB);
    colorPointSet(kdTreeCloudRGB, 0, 255, 0);
    MergeCloud(kdTreeCloudRGB,*totalRGB);

    //7,segmentation
    EuclideanClusterExtraction(kdTreeCloudRGB,0.01,6000,2000,boundPoints_filter_seg_indices); //4000
    int leg_down_count = 0;

    for(std::vector<pcl::PointIndices>::const_iterator it = boundPoints_filter_seg_indices.begin();it!=boundPoints_filter_seg_indices.end();it++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundPoints_filter_seg(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(std::vector<int>::const_iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
        {
            boundPoints_filter_seg->points.push_back(kdTreeCloudRGB->points[*pit]);
        }
        boundPoints_filter_seg->width = boundPoints_filter_seg->points.size();
        boundPoints_filter_seg->height = 1;
        boundPoints_filter_seg->is_dense =true;


        pcl::PointXYZRGB minPt,maxPt;
        pcl::getMinMax3D(*boundPoints_filter_seg,minPt,maxPt);
        float y_delta = maxPt.y-minPt.y;  //0.09
        float z_delta = maxPt.z-minPt.z;  //0.1
        if((y_delta<0.2) && (y_delta>0.06) && (z_delta>0.02) &&(z_delta<0.15))
        {
            pcl::compute3DCentroid(*boundPoints_filter_seg, centroid);    // 齐次坐标，（c0,c1,c2,1）
                if(boundPoints_filter_seg->size()>1000)
                {
                    if(centroid.y()>0)
                    {
                        left_down_centroid = centroid;
                    }
                    if(centroid.y()<0)
                    {
                        right_down_centroid = centroid;
                    }
                    //MergeCloud(boundPoints_filter_seg,*totalRGB);
                    leg_down_count++;
                }
//                MergeCloud(boundPoints_filter_seg,*totalRGB);
                 //cout << "########-----------boundPoints_filter_seg->size():" <<boundPoints_filter_seg->size() << "  y_delta:"<<y_delta << "  z_delta:"<< z_delta << endl;
        }
        j++;
//        cout << "########-----------boundPoints_filter_seg->size():" <<boundPoints_filter_seg->size() << "  y_delta:"<<y_delta << "  z_delta:"<< z_delta << endl;
//        ########-----------boundPoints_filter_seg->size():1553
//        ########-----------boundPoints_filter_seg->size():1532
//        ########-----------boundPoints_filter_seg->size():1441
        if(boundPoints_filter_seg != nullptr)
        {
            boundPoints_filter_seg = nullptr;
        }
    }
//    cout << "########--leg_down_count:" <<leg_down_count<< endl;
//    std::cerr << "左_down_质心结果:" << left_down_centroid.x() << "  " << left_down_centroid.y() << "  " << left_down_centroid.z()<< std::endl;
//    std::cerr << "右_down_质心结果:" << right_down_centroid.x() << "  " << right_down_centroid.y()  << "  " << right_down_centroid.z() << std::endl;

    if(coefficients_up != nullptr)
    {
        coefficients_up = nullptr;
    }
    if(inliers_up != nullptr)
    {
        inliers_up = nullptr;
    }

    if(coefficients_down != nullptr)
    {
        coefficients_down = nullptr;
    }
    if(inliers_down != nullptr)
    {
        inliers_down = nullptr;
    }

    float center_x = (left_up_centroid.x()+right_up_centroid.x())/2.0;
    float center_y = (left_up_centroid.y()+right_up_centroid.y())/2.0;
    float center_z = (left_up_centroid.z()+right_up_centroid.z())/2.0;
    middle_up_centroid = Eigen::Vector4f(center_x,center_y,center_z,1);

    center_x = (left_down_centroid.x()+right_down_centroid.x())/2.0;
    center_y = (left_down_centroid.y()+right_down_centroid.y())/2.0;
    center_z = (left_down_centroid.z()+right_down_centroid.z())/2.0;
    middle_down_centroid = Eigen::Vector4f(center_x,center_y,center_z,1);

    center_x = middle_up_centroid.x()-middle_down_centroid.x();
    center_y = middle_up_centroid.y()-middle_down_centroid.y();
    center_z = middle_up_centroid.z()-middle_down_centroid.z();
    middlecentroid = Eigen::Vector4f(center_x,center_y,center_z,1);

    g_up_angle = -(TwoPointToAngle(left_up_centroid.y(), left_up_centroid.x(), right_up_centroid.y(), right_up_centroid.x()));
    g_down_angle = -(TwoPointToAngle(left_down_centroid.y(), left_down_centroid.x(), right_down_centroid.y(), right_down_centroid.x()));
    g_delta_angle = g_up_angle - g_down_angle;

    std::cerr << "质心结果: dx:" << middlecentroid.x() << "  dy:" << middlecentroid.y() << "  dz:" << middlecentroid.z() <<" g_delta_angle:"<<g_delta_angle<< std::endl;


    double time4 = stopWatch.getTime();
//    std::cerr<< "elapsed_time:" << time4 << std::endl;
    if(abs(center_y)<0.3 && (leg_up_count==2) && (leg_down_count==2))
    {
        return failCodeAndReset(PDS_NO_ERRORS,middle_up_centroid,middlecentroid,middle_down_centroid);
    }
    else
    {
        return failCodeAndReset(PDS_COMPOSED_CENTER_STRINGER_RATIO_ERROR,middle_up_centroid,middlecentroid,middle_down_centroid);
    }
}


void BlaserPointSafeZoneCallback(const sensor_msgs::PointCloud2ConstPtr &cloudPtr)
{
    stopWatch.reset();
    //Initialization
    pcl::fromROSMsg(*cloudPtr, *origin);
    //viewer->removeAllPointClouds();
    totalRGB->clear();
    pallet_boxFilter_up_cloudRGB->clear();
    pallet_boxFilter_down_cloudRGB->clear();

    //deal with the cloud point
    pcl::copyPointCloud(*origin,*firstcloud);         //original
    pcl::copyPointCloud(*firstcloud,*secondcloud);    //set cloud point size
    transfer_X_Y_Z_Basler(firstcloud,*secondcloud);    //just Basler Camera transform
    pcl::copyPointCloud(*secondcloud,*secondcloudRGB);
    for(std::size_t i = 0;i<secondcloudRGB->size();i++)
    {
        secondcloudRGB->points.at(i).r=100;
        secondcloudRGB->points.at(i).g=100;
        secondcloudRGB->points.at(i).b=0;
    }

    if(g_command_id == 5)
    {
        basler.safezone.min_x = g_volCheck_min_x;
        basler.safezone.max_x = g_volCheck_max_x;
        basler.safezone.min_y = g_volCheck_min_y;
        basler.safezone.max_y = g_volCheck_max_y;
        basler.safezone.min_z = g_volCheck_min_z;
        basler.safezone.max_z = g_volCheck_max_z;
    }
    else
    {
        basler.safezone.min_x = basler.safezone_min_x;
        basler.safezone.max_x = basler.safezone_max_x;
        basler.safezone.min_y = basler.safezone_min_y;
        basler.safezone.max_y = basler.safezone_max_y;
        basler.safezone.min_z = basler.safezone_min_z;
        basler.safezone.max_z = basler.safezone_max_z;
    }

    //1.Zone Select
    Eigen::Vector4f min_ground_pt(basler.groundzone.min_x,basler.groundzone.min_y,basler.groundzone.min_z,1.0);
    Eigen::Vector4f max_ground_pt(basler.groundzone.max_x,basler.groundzone.max_y,basler.groundzone.max_z,1.0);
    BoxFilter(rotation_cloudRGB,*filterGroundBoxInside_cloudRGB,min_ground_pt,max_ground_pt,false); // true: ouside   false:inside
    //2.Color Set
    colorPointSet(filterGroundBoxInside_cloudRGB, 255, 0, 0);
    //3.Show message
    //cout << "filterGroundBoxInside_cloudRGB:" <<filterGroundBoxInside_cloudRGB->size()<< endl;
    if(filterGroundBoxInside_cloudRGB->size() < basler.ground_pointCountForGroundExist)
    {
        groundExist = false;
        increaseCameraHeight = 0;
    }
    else
    {
        groundExist = true;
        increaseCameraHeight = basler.ground_increaseCameraHeight;
    }


    safezonecloud ->clear();
    basler.safezone.min_z = basler.safezone_min_z - increaseCameraHeight;
    basler.safezone.max_z = basler.safezone_max_z - increaseCameraHeight;
    SetTemplateCloud_Zone(basler.safezone,*safezonecloud);//safe zone

    groundzonecloud->clear();
    SetTemplateCloud_Zone(basler.groundzone,*groundzonecloud);//ground zone

    /*Start with safe zone detect*/
    //rotate    camera_rotate_x_angle
    //angle to radian
    //原理：弧度 = 角度 * 圆周率 / 180.0；
    //原理：角度 = 弧度 / 圆周率 * 180.0；
    float radian_x = basler.camera_rotate_x_angle*PI/180.0;
    float radian_y = basler.camera_rotate_y_angle*PI/180.0;
    float radian_z = (basler.camera_rotate_z_angle + g_angleOffset)*PI/180.0;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();//初始化变换矩阵为单位矩阵
    // 在 x,y,z 轴上平移.
    //transform.translation() << 0.0, 0.0, 0.0;
    transform.translation() << basler.camera_shift_x_distance,basler.camera_shift_y_distance,basler.camera_shift_z_distance;
    // 旋转; Z 轴上旋转0 弧度,Y轴上旋转0弧度
    transform.rotate(Eigen::AngleAxisf(radian_x, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(radian_y, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(radian_z, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*secondcloudRGB,*rotation_cloudRGB,transform);
    //cout<<"变换矩阵为：" << transform.matrix() << endl;

    //optimize function of safeZoneMethod
    safeZoneMethod();
    /*End with safe zone detect*/

    /****************************************/
    /*Start with pallet detect*/
    /*Original Point Cloud: secondcloudRGB*/
    /*Rotation Point Cloud: rotation_cloudRGB*/
    leftcentroid.setZero();
    middlecentroid.setZero();
    rightcentroid.setZero();
//    palletDetectMethod_Tian(); // only for "Tian" type
    palletDetectMethod_All();   // detect   "Tian" type and "Chuan" type

    palletzonecloud->clear();
    basler.palletzone.min_z = basler.pallet_roi_min_z - increaseCameraHeight;
    basler.palletzone.max_z = basler.pallet_roi_max_z - increaseCameraHeight;
    SetTemplateCloud_Zone(basler.palletzone,*palletzonecloud);//pallet zone

//    int result = palletDetectMethod2(rotation_cloudRGB,*palletDetect_cloudRGB); //ScanWay
    /*End with pallet detect*/


    if(basler.safezone_show == true)
    {
        //MergeCloud(filterBoxInside_cloudRGB,*totalRGB);
        MergeCloud(safezonecloud,*totalRGB);
    }
    if(basler.pallet_roi_show == true)
    {
        MergeCloud(palletzonecloud,*totalRGB);
    }
    if(basler.ground_roi_show == true)
    {
        MergeCloud(filterGroundBoxInside_cloudRGB,*totalRGB);
        MergeCloud(groundzonecloud,*totalRGB);
    }

    viewer1.showCloud(totalRGB);
//    viewer1.showCloud(pallet_boxFilter_reduce_cloudRGB);
//    viewer1.showCloud(cloud_b);
//    viewer1.showCloud(kdTreeVoxelCloudRGB);
//    viewer1.showCloud(cloud_b_after_digate);
//    viewer1.showCloud(kdTreeVoxelAfterDilateCloudRGB);
//    viewer1.showCloud(kdTreeCloud);
//    viewer1.showCloud(kdTreeCloudRGB);
//    viewer1.showCloud(boundFillupCloudRGB);


}

void *thfrunc(void* arg)
{
    int argc;
    char **argv;
    ros::init(argc, argv, "QT_TCP");
    basler.Show_config();
    if(basler.kindOfPalletTemplate ==1)
    {
        SetTemplateCloud_Tian(*cloudTemplate);
    }
    if(basler.kindOfPalletTemplate ==2)
    {
        SetTemplateCloud_Chuan(*cloudTemplate);
    }
//    SetTemplateCloud_Zone(basler.safezone,*safezonecloud);//safe zone
//    SetTemplateCloud_Zone(basler.palletzone,*palletzonecloud);//pallet zone

    ros::NodeHandle nh;
    //ros::Subscriber laser_sub = nh.subscribe("/basler/point_cloud", 1, BlaserPointSafeZoneCallback);   //pallet detect
    ros::Subscriber laser_sub = nh.subscribe("/lx_camera_node/LxCamera_Cloud", 1, BlaserPointSafeZoneCallback);   //pallet detect
    ros::Publisher pallet_pub = nh.advertise<std_msgs::String>("SafeZone",2);
    ros::Publisher new_pallet_pub = nh.advertise<std_msgs::String>("MySafeZone",2);
    ros::Rate loop_rate(1);  //HZ
    while(ros::ok())
    {
        //for safezone detect
        std_msgs::String msg;
        std::stringstream ss;
        ss  <<" safezoneExist:" << basler.pub_safezoneExist
            <<" safezone_max_left:" << basler.pub_safezone_max_left
            <<" safezone_max_right:" << basler.pub_safezone_max_right;
        msg.data = ss.str();
        ROS_INFO("%s",msg.data.c_str());
        pallet_pub.publish(msg);
        new_pallet_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    //ros
    pthread_t pid;
    int res;
    res = pthread_create(&pid,NULL,thfrunc,NULL);
    if(res)
    {
        std::cout <<"Failed to create thread!!!" <<  std::endl;
    }

    QApplication a(argc, argv);
    ServerWidget server_weight;
    server_weight.show();
//        ClientWidget client_weight;
//        client_weight.show();
    ClientPDS client_PDS;
    client_PDS.show();
    return a.exec();
}
