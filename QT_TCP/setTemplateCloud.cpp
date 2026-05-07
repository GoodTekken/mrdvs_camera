#include "setTemplateCloud.h"

void SetTemplateCloud_Tian(pcl::PointCloud<pcl::PointXYZ>& cloudTemplate)
{
    pcl::PointXYZ templatePoint;
    int cloudDensity = 5;  //点云密度，5mm
    int y_int, z_int;

    templatePoint.x = 1.5;
    for (y_int = -550; y_int <= 550; )
    {
        for (z_int = 120; z_int <= 150;)
        {
            templatePoint.y = (float)y_int / 1000;
            templatePoint.z = (float)z_int / 1000;
            cloudTemplate.push_back(templatePoint);
            z_int = z_int + cloudDensity;
        }
        y_int = y_int + cloudDensity;
    }

    for (y_int = -550; y_int <= 550; )
    {
        for (z_int = 0; z_int <= 30;)
        {
            templatePoint.y = (float)y_int / 1000;
            templatePoint.z = (float)z_int / 1000;
            cloudTemplate.push_back(templatePoint);
            z_int = z_int + cloudDensity;
        }
        y_int = y_int + cloudDensity;
    }


    for (y_int = -550; y_int <= -420;)
    {
        for (z_int = 30; z_int < 120; )
        {
            templatePoint.y = (float)y_int / 1000;
            templatePoint.z = (float)z_int / 1000;
            cloudTemplate.push_back(templatePoint);
            z_int = z_int + cloudDensity;
        }
        y_int = y_int + cloudDensity;
    }

    //for (y = -150; y <= 150; )
    for (y_int = -145; y_int <= 145; )
    {
        for (z_int = 30; z_int < 120; )
        {
            templatePoint.y = (float)y_int / 1000;
            templatePoint.z = (float)z_int / 1000;
            cloudTemplate.push_back(templatePoint);
            z_int = z_int + cloudDensity;
        }
        y_int = y_int + cloudDensity;
    }

    for (y_int = 420; y_int <= 550; )
    {
        for (z_int = 30; z_int < 120; )
        {
            templatePoint.y = (float)y_int / 1000;
            templatePoint.z = (float)z_int / 1000;
            cloudTemplate.push_back(templatePoint);
            z_int = z_int + cloudDensity;
        }
        y_int = y_int + cloudDensity;
    }
}
void SetTemplateCloud_Chuan(pcl::PointCloud<pcl::PointXYZ>& cloudTemplate)
{
    pcl::PointXYZ templatePoint;
    int cloudDensity = 5;  //点云密度，5mm
    int y_int, z_int;

    templatePoint.x = 1.5;
    for (y_int = -550; y_int <= 550; )
    {
        for (z_int = 120; z_int <= 150;)
        {
            templatePoint.y = (float)y_int / 1000;
            templatePoint.z = (float)z_int / 1000;
            cloudTemplate.push_back(templatePoint);
            z_int = z_int + cloudDensity;
        }
        y_int = y_int + cloudDensity;
    }
    for (y_int = -550; y_int <= -420;)
    {
        for (z_int = 0; z_int < 120; )
        {
            templatePoint.y = (float)y_int / 1000;
            templatePoint.z = (float)z_int / 1000;
            cloudTemplate.push_back(templatePoint);
            z_int = z_int + cloudDensity;
        }
        y_int = y_int + cloudDensity;
    }

    //for (y = -150; y <= 150; )
    for (y_int = -65; y_int <= 65; )
    {
        for (z_int = 0; z_int < 120; )
        {
            templatePoint.y = (float)y_int / 1000;
            templatePoint.z = (float)z_int / 1000;
            cloudTemplate.push_back(templatePoint);
            z_int = z_int + cloudDensity;
        }
        y_int = y_int + cloudDensity;
    }

    for (y_int = 420; y_int <= 550; )
    {
        for (z_int = 0; z_int < 120; )
        {
            templatePoint.y = (float)y_int / 1000;
            templatePoint.z = (float)z_int / 1000;
            cloudTemplate.push_back(templatePoint);
            z_int = z_int + cloudDensity;
        }
        y_int = y_int + cloudDensity;
    }
}

void SetTemplateCloud_Zone(float min_x,float max_x,float min_y,float max_y,float min_z,float max_z,
                           int color_red,int color_green,int color_blue, float cloudDensity,
                           pcl::PointCloud<pcl::PointXYZRGB>& cloud_zone)
{
    pcl::PointXYZRGB templatePoint;
    float x_float,y_float, z_float;
    templatePoint.r=color_red;
    templatePoint.g=color_green;
    templatePoint.b=color_blue;

    for(x_float=min_x;x_float<=max_x;)
    {
        templatePoint.x =x_float;
        templatePoint.y = min_y;
        templatePoint.z = min_z;
        cloud_zone.push_back(templatePoint);
        templatePoint.y = min_y;
        templatePoint.z = max_z;
        cloud_zone.push_back(templatePoint);
        templatePoint.y = max_y;
        templatePoint.z = min_z;
        cloud_zone.push_back(templatePoint);
        templatePoint.y = max_y;
        templatePoint.z = max_z;
        cloud_zone.push_back(templatePoint);
        x_float = x_float + cloudDensity;
    }

    for(y_float=min_y;y_float<=max_y;)
    {
        templatePoint.y =y_float;
        templatePoint.x = min_x;
        templatePoint.z = min_z;
        cloud_zone.push_back(templatePoint);
        templatePoint.x = min_x;
        templatePoint.z = max_z;
        cloud_zone.push_back(templatePoint);
        templatePoint.x = max_x;
        templatePoint.z = min_z;
        cloud_zone.push_back(templatePoint);
        templatePoint.x = max_x;
        templatePoint.z = max_z;
        cloud_zone.push_back(templatePoint);
        y_float = y_float + cloudDensity;
    }

    for(z_float=min_z;z_float<=max_z;)
    {
        templatePoint.z =z_float;
        templatePoint.y = min_y;
        templatePoint.x = min_x;
        cloud_zone.push_back(templatePoint);
        templatePoint.y = min_y;
        templatePoint.x = max_x;
        cloud_zone.push_back(templatePoint);
        templatePoint.y = max_y;
        templatePoint.x = min_x;
        cloud_zone.push_back(templatePoint);
        templatePoint.y = max_y;
        templatePoint.x = max_x;
        cloud_zone.push_back(templatePoint);
        z_float = z_float + cloudDensity;
    }
}


void SetTemplateCloud_Zone(zone_struct zone,
                           pcl::PointCloud<pcl::PointXYZRGB>& cloud_zone)
{
    pcl::PointXYZRGB templatePoint;
    float x_float,y_float, z_float;
    templatePoint.r=zone.color_red;
    templatePoint.g=zone.color_green;
    templatePoint.b=zone.color_blue;

    for(x_float=zone.min_x;x_float<=zone.max_x;)
    {
        templatePoint.x =x_float;
        templatePoint.y = zone.min_y;
        templatePoint.z = zone.min_z;
        cloud_zone.push_back(templatePoint);
        templatePoint.y = zone.min_y;
        templatePoint.z = zone.max_z;
        cloud_zone.push_back(templatePoint);
        templatePoint.y = zone.max_y;
        templatePoint.z = zone.min_z;
        cloud_zone.push_back(templatePoint);
        templatePoint.y = zone.max_y;
        templatePoint.z = zone.max_z;
        cloud_zone.push_back(templatePoint);
        x_float = x_float + zone.cloudDensity;
    }

    for(y_float=zone.min_y;y_float<=zone.max_y;)
    {
        templatePoint.y =y_float;
        templatePoint.x = zone.min_x;
        templatePoint.z = zone.min_z;
        cloud_zone.push_back(templatePoint);
        templatePoint.x = zone.min_x;
        templatePoint.z = zone.max_z;
        cloud_zone.push_back(templatePoint);
        templatePoint.x = zone.max_x;
        templatePoint.z = zone.min_z;
        cloud_zone.push_back(templatePoint);
        templatePoint.x = zone.max_x;
        templatePoint.z = zone.max_z;
        cloud_zone.push_back(templatePoint);
        y_float = y_float + zone.cloudDensity;
    }

    for(z_float=zone.min_z;z_float<=zone.max_z;)
    {
        templatePoint.z =z_float;
        templatePoint.y = zone.min_y;
        templatePoint.x = zone.min_x;
        cloud_zone.push_back(templatePoint);
        templatePoint.y = zone.min_y;
        templatePoint.x = zone.max_x;
        cloud_zone.push_back(templatePoint);
        templatePoint.y = zone.max_y;
        templatePoint.x = zone.min_x;
        cloud_zone.push_back(templatePoint);
        templatePoint.y = zone.max_y;
        templatePoint.x = zone.max_x;
        cloud_zone.push_back(templatePoint);
        z_float = z_float + zone.cloudDensity;
    }
}
