#ifndef BASLER_H
#define BASLER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include "Config.h"
#include "Pallet_Struct.h"

/*
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>
*/
//#include <pcl/point_types.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/registration/ndt.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/icp_nl.h>
//#include <pcl/features/boundary.h>

class Basler
{
public:
    Basler();
    ~Basler();
    void Show_config();

    bool safezoneExist;
    float safezone_max_left;  //unit:mm
    float safezone_max_right; //unit:mm
    bool pub_safezoneExist;
    float pub_safezone_max_left;  //unit:mm
    float pub_safezone_max_right; //unit:mm

    int port;
    std::string ipAddress;
    std::string username;
    std::string password;

    int ground_count = 0;
    int max_ground_count = 250;
    int success_count = 0;
    int receive_count = 0;
    bool debug = false;
    int kindOfPalletTemplate;  //1:Tian 2:Chuan
    //Camera Parameter
    float camera_shift_x_distance;
    float camera_shift_y_distance;
    float camera_shift_z_distance;
    float camera_rotate_x_angle;
    float camera_rotate_y_angle;
    float camera_rotate_z_angle;

    //Safe Zone Detect
    float pallet_length;
    float pallet_weight;
    float pallet_height;
    float safezone_min_x;
    float safezone_max_x;
    float safezone_min_y;
    float safezone_max_y;
    float safezone_min_z;
    float safezone_max_z;
    bool safezone_show;
    float safezone_show_density;
    size_t safezone_pointCountForObjectExist;

    //Pallet Detect
    float pallet_roi_min_x;
    float pallet_roi_max_x;
    float pallet_roi_min_y;
    float pallet_roi_max_y;
    float pallet_roi_min_z;
    float pallet_roi_max_z;
    bool pallet_roi_show;
    float pallet_roi_show_density;

    //Ground Detect
    float ground_roi_min_x;
    float ground_roi_max_x;
    float ground_roi_min_y;
    float ground_roi_max_y;
    float ground_roi_min_z;
    float ground_roi_max_z;
    bool ground_roi_show;
    float ground_roi_show_density;
    size_t ground_pointCountForGroundExist;
    float ground_increaseCameraHeight;

    Eigen::Vector4f leftcentroidOutput;
    Eigen::Vector4f rightcentroidOutput;

    struct zone_struct safezone;
    struct zone_struct palletzone;
    struct zone_struct groundzone;

private:

};

#endif // BASLER_H
