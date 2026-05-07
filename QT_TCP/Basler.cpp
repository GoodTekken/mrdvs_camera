#include "Basler.h"

Basler::Basler()
{
    safezoneExist = false;
    safezone_max_left =0.0;  //unit:mm
    safezone_max_right =0.0; //unit:mm
    pub_safezoneExist = false;
    pub_safezone_max_left =0.0;  //unit:mm
    pub_safezone_max_right =0.0; //unit:mm

}

Basler::~Basler()
{

}


void Basler::Show_config()
{
    //start to read
    const char ConfigFile[]= "config.txt";
    Config configSettings(ConfigFile);
    debug = configSettings.Read("debug", false);
    kindOfPalletTemplate = configSettings.Read("kindOfPalletTemplate", kindOfPalletTemplate);

    camera_shift_x_distance = configSettings.Read("camera_shift_x_distance", camera_shift_x_distance);
    camera_shift_y_distance = configSettings.Read("camera_shift_y_distance", camera_shift_y_distance);
    camera_shift_z_distance = configSettings.Read("camera_shift_z_distance", camera_shift_z_distance);
    camera_rotate_x_angle = configSettings.Read("camera_rotate_x_angle", camera_rotate_x_angle);
    camera_rotate_y_angle = configSettings.Read("camera_rotate_y_angle", camera_rotate_y_angle);
    camera_rotate_z_angle = configSettings.Read("camera_rotate_z_angle", camera_rotate_z_angle);

    pallet_length = configSettings.Read("pallet_length", pallet_length);
    pallet_weight = configSettings.Read("pallet_weight", pallet_weight);
    pallet_height = configSettings.Read("pallet_height", pallet_height);
    safezone_min_x = configSettings.Read("safezone_min_x", safezone_min_x);
    safezone_max_x = configSettings.Read("safezone_max_x", safezone_max_x);
    safezone_min_y = configSettings.Read("safezone_min_y", safezone_min_y);
    safezone_max_y = configSettings.Read("safezone_max_y", safezone_max_y);
    safezone_min_z = configSettings.Read("safezone_min_z", safezone_min_z);
    safezone_max_z = configSettings.Read("safezone_max_z", safezone_max_z);
    safezone_show = configSettings.Read("safezone_show", safezone_show);
    safezone_show_density = configSettings.Read("safezone_show_density", safezone_show_density);
    safezone_pointCountForObjectExist = configSettings.Read("safezone_pointCountForObjectExist", safezone_pointCountForObjectExist);

    //-----Pallet Detect--------
    pallet_roi_min_x = configSettings.Read("pallet_roi_min_x", pallet_roi_min_x);
    pallet_roi_max_x = configSettings.Read("pallet_roi_max_x", pallet_roi_max_x);
    pallet_roi_min_y = configSettings.Read("pallet_roi_min_y", pallet_roi_min_y);
    pallet_roi_max_y = configSettings.Read("pallet_roi_max_y", pallet_roi_max_y);
    pallet_roi_min_z = configSettings.Read("pallet_roi_min_z", pallet_roi_min_z);
    pallet_roi_max_z = configSettings.Read("pallet_roi_max_z", pallet_roi_max_z);
    pallet_roi_show = configSettings.Read("pallet_roi_show", pallet_roi_show);
    pallet_roi_show_density = configSettings.Read("pallet_roi_show_density", pallet_roi_show_density);

    //-----Ground Detect--------
    ground_roi_min_x = configSettings.Read("ground_roi_min_x", ground_roi_min_x);
    ground_roi_max_x = configSettings.Read("ground_roi_max_x", ground_roi_max_x);
    ground_roi_min_y = configSettings.Read("ground_roi_min_y", ground_roi_min_y);
    ground_roi_max_y = configSettings.Read("ground_roi_max_y", ground_roi_max_y);
    ground_roi_min_z = configSettings.Read("ground_roi_min_z", ground_roi_min_z);
    ground_roi_max_z = configSettings.Read("ground_roi_max_z", ground_roi_max_z);
    ground_roi_show = configSettings.Read("ground_roi_show", ground_roi_show);
    ground_roi_show_density = configSettings.Read("pallet_roi_show_density", pallet_roi_show_density);
    ground_pointCountForGroundExist = configSettings.Read("ground_pointCountForGroundExist", ground_pointCountForGroundExist);
    ground_increaseCameraHeight = configSettings.Read("ground_increaseCameraHeight", ground_increaseCameraHeight);


    port = configSettings.Read("port", 0);
    ipAddress = configSettings.Read("ipAddress", ipAddress);
    username = configSettings.Read("username", username);
    password = configSettings.Read("password", password);
    std::cout<<"debug:"<<debug<<std::endl;
    std::cout<<"kindOfPalletTemplate:"<<kindOfPalletTemplate<<std::endl;

    std::cout<<"camera_shift_x_distance:"<<camera_shift_x_distance<<std::endl;
    std::cout<<"camera_shift_y_distance:"<<camera_shift_y_distance<<std::endl;
    std::cout<<"camera_shift_z_distance:"<<camera_shift_z_distance<<std::endl;

    std::cout<<"camera_rotate_x_angle:"<<camera_rotate_x_angle<<std::endl;
    std::cout<<"camera_rotate_y_angle:"<<camera_rotate_y_angle<<std::endl;
    std::cout<<"camera_rotate_z_angle:"<<camera_rotate_z_angle<<std::endl;

    std::cout<<"pallet_length:"<<pallet_length<<std::endl;
    std::cout<<"pallet_weight:"<<pallet_weight<<std::endl;
    std::cout<<"pallet_height:"<<pallet_height<<std::endl;

    std::cout<<"pallet_length:"<<pallet_length<<std::endl;
    std::cout<<"pallet_weight:"<<pallet_weight<<std::endl;
    std::cout<<"pallet_height:"<<pallet_height<<std::endl;
    std::cout<<"safezone_min_x:"<<safezone_min_x<<std::endl;
    std::cout<<"safezone_max_x:"<<safezone_max_x<<std::endl;
    std::cout<<"safezone_min_y:"<<safezone_min_y<<std::endl;
    std::cout<<"safezone_max_y:"<<safezone_max_y<<std::endl;
    std::cout<<"safezone_min_z:"<<safezone_min_z<<std::endl;
    std::cout<<"safezone_max_z:"<<safezone_max_z<<std::endl;
    std::cout<<"safezone_show:"<<safezone_show<<std::endl;
    std::cout<<"safezone_show_density:"<<safezone_show_density<<std::endl;
    std::cout<<"safezone_pointCountForObjectExist:"<<safezone_pointCountForObjectExist<<std::endl;

    std::cout<<"pallet_roi_min_x:"<<pallet_roi_min_x<<std::endl;
    std::cout<<"pallet_roi_max_x:"<<pallet_roi_max_x<<std::endl;
    std::cout<<"pallet_roi_min_y:"<<pallet_roi_min_y<<std::endl;
    std::cout<<"pallet_roi_max_y:"<<pallet_roi_max_y<<std::endl;
    std::cout<<"pallet_roi_min_z:"<<pallet_roi_min_z<<std::endl;
    std::cout<<"pallet_roi_max_z:"<<pallet_roi_max_z<<std::endl;
    std::cout<<"pallet_roi_show:"<<pallet_roi_show<<std::endl;
    std::cout<<"pallet_roi_show_density:"<<pallet_roi_show_density<<std::endl;

    std::cout<<"ground_roi_min_x:"<<ground_roi_min_x<<std::endl;
    std::cout<<"ground_roi_max_x:"<<ground_roi_max_x<<std::endl;
    std::cout<<"ground_roi_min_y:"<<ground_roi_min_y<<std::endl;
    std::cout<<"ground_roi_max_y:"<<ground_roi_max_y<<std::endl;
    std::cout<<"ground_roi_min_z:"<<ground_roi_min_z<<std::endl;
    std::cout<<"ground_roi_max_z:"<<ground_roi_max_z<<std::endl;
    std::cout<<"ground_roi_show:"<<ground_roi_show<<std::endl;
    std::cout<<"ground_roi_show_density:"<<ground_roi_show_density<<std::endl;
    std::cout<<"ground_pointCountForGroundExist:"<<ground_pointCountForGroundExist<<std::endl;
    std::cout<<"ground_increaseCameraHeight"<<ground_increaseCameraHeight<<std::endl;

    std::cout<<"port:"<<port<<std::endl;
    std::cout<<"ipAddress:"<<ipAddress<<std::endl;
    std::cout<<"username:"<<username<<std::endl;
    std::cout<<"password:"<<password<<std::endl;

    //struct zone_struct safezone;
    safezone.min_x = safezone_min_x;
    safezone.max_x = safezone_max_x;
    safezone.min_y = safezone_min_y;
    safezone.max_y = safezone_max_y;
    safezone.min_z = safezone_min_z;
    safezone.max_z = safezone_max_z;
    safezone.color_red = 0;
    safezone.color_green = 255;
    safezone.color_blue = 0;
    safezone.cloudDensity = safezone_show_density;
    //SetTemplateCloud_Zone(safezone,*safezonecloud);//safe zone

    //struct zone_struct palletzone;
    palletzone.min_x = pallet_roi_min_x;
    palletzone.max_x = pallet_roi_max_x;
    palletzone.min_y = pallet_roi_min_y;
    palletzone.max_y = pallet_roi_max_y;
    palletzone.min_z = pallet_roi_min_z;
    palletzone.max_z = pallet_roi_max_z;
    palletzone.color_red = 0;
    palletzone.color_green = 0;
    palletzone.color_blue = 255;
    palletzone.cloudDensity = pallet_roi_show_density;
    //SetTemplateCloud_Zone(palletzone,*palletzonecloud);//pallet zone

    //struct zone_struct groundzone;
    groundzone.min_x = ground_roi_min_x;
    groundzone.max_x = ground_roi_max_x;
    groundzone.min_y = ground_roi_min_y;
    groundzone.max_y = ground_roi_max_y;
    groundzone.min_z = ground_roi_min_z;
    groundzone.max_z = ground_roi_max_z;
    groundzone.color_red = 255;
    groundzone.color_green = 0;
    groundzone.color_blue = 0;
    groundzone.cloudDensity = ground_roi_show_density;
    //SetTemplateCloud_Zone(groundzone,*groundzonecloud);//ground zone
}

