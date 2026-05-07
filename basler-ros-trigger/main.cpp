#define VISURLIZE 0

#include <ConsumerImplHelper/BlazeCamera.h>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "Head.h"
#include "Server.h"
#include <QCoreApplication>

#if VISURLIZE
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#endif

using namespace std;
using namespace ros;
using namespace pcl;
using namespace GenTLConsumerImplHelper;

CBlazeCamera m_camera;
Publisher pub_pointcloud;
const string g_frameId = "basler";

bool basler_trigger_enable = false;

#if VISURLIZE
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("third Viewer"));
#endif

// Typedefs for the PCL types used
typedef PointXYZ Point_t;
typedef PointCloud<Point_t> PointCloud_t;
typedef PointCloud_t::Ptr PointCloudPtr;

/*
Convert the Basler point cloud to a PCL point cloud.
*/
PointCloudPtr convertGrabResultToPointCloud(const BufferParts& imgParts)
{
    // An organized point cloud is used, i.e., for each camera pixel there is an entry
    // in the data structure indicating the 3D coordinates calculated from that pixel.
    // If the camera wasn't able to create depth information for a pixel, the x-,y-, and z- coordinates
    // are set to NaN. These NaNs will be retained in the PCL point cloud.

    // Allocate PCL point cloud.
    const size_t width = imgParts[0].width;
    const size_t height = imgParts[0].height;
    PointCloudPtr ptrPointCloud(new PointCloud_t);
    ptrPointCloud->width = width;
    ptrPointCloud->height = height;
    ptrPointCloud->points.resize(width * height);
    ptrPointCloud->is_dense = false; // organized point cloud

    // Create a pointer to the 3D coordinates of the first point.
    // imgParts[0] always refers to the point cloud data.
    CBlazeCamera::Coord3D* pSrcPoint = (CBlazeCamera::Coord3D*) imgParts[0].pData;

    // Create a pointer to the intensity information, stored in the second buffer part.
    uint16_t* pIntensity = (uint16_t*)imgParts[1].pData;

    // Set the points.
    for (size_t i = 0; i < height * width; ++i, ++pSrcPoint, ++pIntensity)
    {
        // Set the X/Y/Z cordinates.
        Point_t& dstPoint = ptrPointCloud->points[i];
        dstPoint.x = pSrcPoint->x;
        dstPoint.y = pSrcPoint->y;
        dstPoint.z = pSrcPoint->z;
    }

    return ptrPointCloud;
}

bool publish(const BufferParts& parts, ros::Time acquisitionTime)
{
    // Run some checks first.
    if (parts.size() != 3)
    {
        ROS_ERROR("[%s] Expected 3 parts, got %zu!", ros::this_node::getName().c_str(), parts.size());
        return EXIT_FAILURE;
    }

    if (parts[0].dataFormat != PFNC_Coord3D_ABC32f)
    {
        ROS_ERROR("[%s] Unexpected data format for the first image part. Coord3D_ABC32f is expected.", ros::this_node::getName().c_str());
        return EXIT_FAILURE;
    }

    if (parts[1].dataFormat != PFNC_Mono16)
    {
        ROS_ERROR("[%s] Unexpected data format for the second image part. Mono16 is expected.", ros::this_node::getName().c_str());
        return EXIT_FAILURE;
    }

    // Processing data.
    const int width = (int)parts[0].width;
    const int height = (int)parts[0].height;
    const int count = width * height;

    PointCloudPtr ptrCloud = convertGrabResultToPointCloud(parts);
    #if VISURLIZE
    viewer->removePointCloud("cloud");
    viewer->addPointCloud<pcl::PointXYZ>(ptrCloud,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(ptrCloud, 255, 0, 255), "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->spinOnce();
#endif
    ptrCloud->header.frame_id = g_frameId;
    ptrCloud->header.stamp = pcl_conversions::toPCL(acquisitionTime);
    ptrCloud->width = width;
    ptrCloud->height = height;
    ptrCloud->is_dense = false;
    ptrCloud->points.resize(count);
    // Point cloud
    if (pub_pointcloud.getNumSubscribers() > 0)
    {
        // Publish
        pub_pointcloud.publish(ptrCloud);
        qDebug()<<"ptrCloud"<<ptrCloud->size();
    }
    /*

  // Get min and max depth values.
  int minDepth = (int) GenApi::CIntegerPtr(m_camera.GetParameter("DepthMin"))->GetValue();
  int maxDepth = (int) GenApi::CIntegerPtr(m_camera.GetParameter("DepthMax"))->GetValue();
  // Intensity
  if (m_intensityPub.getNumSubscribers() > 0)
  {
    cv::Mat intensity = cv::Mat(height, width, CV_16UC1, parts[1].pData);
    // Scale the intensity image since it often looks quite dark.
    double  max;
    cv::minMaxLoc(intensity, NULL, &max);
    intensity /= (max / std::numeric_limits<uint16_t>::max());
    // Publish
    cv_bridge::CvImage intensityCvImg;
    intensityCvImg.header.frame_id  = g_frameId;
    intensityCvImg.header.stamp     = acquisitionTime;
    intensityCvImg.encoding         = sensor_msgs::image_encodings::MONO16;
    intensityCvImg.image            = intensity;
    m_intensityPub.publish(intensityCvImg.toImageMsg());

    sensor_msgs::CameraInfoPtr intensityInfoMsg(new sensor_msgs::CameraInfo(m_intensityInfoManager->getCameraInfo()));
    intensityInfoMsg->header.stamp    = acquisitionTime;
    intensityInfoMsg->header.frame_id = g_frameId;
    m_intensityCIPub.publish(intensityInfoMsg);
  }

  // Range map
  if (m_rangeMapPub.getNumSubscribers() > 0)
  {
    uint16_t* pRangeData = new uint16_t[width * height];
    calculateRangeMap(parts[0], minDepth, maxDepth, pRangeData, m_invalidDataValue);
    cv::Mat rangeMap = cv::Mat(height, width, CV_16UC1, pRangeData);
    // Publish
    cv_bridge::CvImage rangeMapCvImg;
    rangeMapCvImg.header.frame_id  = g_frameId;
    rangeMapCvImg.header.stamp     = acquisitionTime;
    rangeMapCvImg.encoding         = sensor_msgs::image_encodings::MONO16;
    rangeMapCvImg.image            = rangeMap;
    m_rangeMapPub.publish(rangeMapCvImg.toImageMsg());

    sensor_msgs::CameraInfoPtr rangeMapInfoMsg(new sensor_msgs::CameraInfo(m_rangeMapInfoManager->getCameraInfo()));
    rangeMapInfoMsg->header.stamp    = acquisitionTime;
    rangeMapInfoMsg->header.frame_id = g_frameId;
    m_rangeMapCIPub.publish(rangeMapInfoMsg);

    delete[] pRangeData;
  }

  // Range map color
  if (m_rangeMapColorPub.getNumSubscribers() > 0)
  {
    BGR* pRangeDataColor = new BGR[width * height];
    calculateRangeMapColor(parts[0], minDepth, maxDepth, pRangeDataColor, m_invalidDataValue);
    cv::Mat3b rangeMapColor = cv::Mat(height, width, CV_8UC3, pRangeDataColor);
    // Publish
    cv_bridge::CvImage rangeMapColorCvImg;
    rangeMapColorCvImg.header.frame_id  = g_frameId;
    rangeMapColorCvImg.header.stamp     = acquisitionTime;
    rangeMapColorCvImg.encoding         = sensor_msgs::image_encodings::TYPE_8UC3;
    rangeMapColorCvImg.image            = rangeMapColor;
    m_rangeMapColorPub.publish(rangeMapColorCvImg.toImageMsg());

    sensor_msgs::CameraInfoPtr rangeMapColorInfoMsg(new sensor_msgs::CameraInfo(m_rangeMapColorInfoManager->getCameraInfo()));
    rangeMapColorInfoMsg->header.stamp    = acquisitionTime;
    rangeMapColorInfoMsg->header.frame_id = g_frameId;
    m_rangeMapColorCIPub.publish(rangeMapColorInfoMsg);

    delete[] pRangeDataColor;
  }

  // Confidence map
  if (m_confidencePub.getNumSubscribers() > 0)
  {
    cv::Mat confidence = cv::Mat(height, width, CV_16UC1, parts[2].pData);
    // Publish
    cv_bridge::CvImage confidenceCvImg;
    confidenceCvImg.header.frame_id  = g_frameId;
    confidenceCvImg.header.stamp     = acquisitionTime;
    confidenceCvImg.encoding         = sensor_msgs::image_encodings::MONO16;
    confidenceCvImg.image            = confidence;
    m_confidencePub.publish(confidenceCvImg.toImageMsg());

    sensor_msgs::CameraInfoPtr confidenceInfoMsg(new sensor_msgs::CameraInfo(m_confidenceInfoManager->getCameraInfo()));
    confidenceInfoMsg->header.stamp    = acquisitionTime;
    confidenceInfoMsg->header.frame_id = g_frameId;
    m_confidenceCIPub.publish(confidenceInfoMsg);
  }*/

    return true;
}

int run(){
    int exitCode = EXIT_SUCCESS;

    try
    {
//        const size_t nBuffers = 3;  // Number of buffers to be used for grabbing

//        // Allocate the memory buffers and prepare image acquisition.
//        m_camera.PrepareAcquisition(nBuffers);

//        // Enqueue all buffers to be filled with image data.
//        for (size_t i = 0; i < nBuffers; ++i)
//        {
//            m_camera.QueueBuffer(i);
//        }

//        // Start the acquisition engine.
//        m_camera.StartAcquisition();
//        m_camera.IssueAcquisitionStartCommand(); // The camera continuously sends data now.

        // Enable 3D (point cloud) data & intensity data.
        m_camera.SetCameraParameterValue("ComponentSelector", "Range");
        m_camera.SetCameraParameterValue("ComponentEnable", true);
        m_camera.SetCameraParameterValue("PixelFormat", "Coord3D_ABC32f");

        m_camera.SetCameraParameterValue("ComponentSelector", "Intensity");
        m_camera.SetCameraParameterValue("ComponentEnable", true);
        m_camera.SetCameraParameterValue("PixelFormat", "Mono16");

        // Configure the value used for identifiyng missing depth information.
        m_camera.SetCameraParameterValue("Scan3dCoordinateSelector", "CoordinateC"); // Choose z-axis
        // z-coordinate will be set to this value if there is no valid depth data available
        m_camera.SetCameraParameterValue("Scan3dInvalidDataValue",  0.0);

//        m_camera.SetCameraParameterValue("TriggerMode",  "On");

        ROS_INFO("[%s] Acquisition started!", ros::this_node::getName().c_str());
//        std::cout << std::endl;
        ros::Rate rate(1);
        BufferParts parts;
        GrabResultPtr ptrgrabResult;
        while (ros::ok())
        {
            if(basler_trigger_enable)
            {
                ptrgrabResult = m_camera.GrabSingleImage(1000,&parts);

                if(ptrgrabResult->status==GrabResult::Ok)
                {
                    publish(parts, ros::Time::now());
                    basler_trigger_enable = false;
                }
                else
                {
                    std::cerr <<"Failed to grab an image."<< std::endl;
                }
            }
            else
            {
                ;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
    catch(GenICam::GenericException& e)
    {
        ROS_ERROR_STREAM("Exception occurred: " << endl << e.GetDescription());
        exitCode = EXIT_FAILURE;
    }

    if(CBlazeCamera::IsProducerInitialized())
    {
        CBlazeCamera::TerminateProducer();
        std::cerr <<"CBlazeCamera::TerminateProducer();"<< std::endl;
    }

    return exitCode;
}

void *thfrunc(void* arg)
{
    int argc;
    char **argv;
    QCoreApplication a(argc,argv);
    Server server;
    a.exec();
}

int main(int argc, char **argv)
{
    pthread_t pid;
    int res;
    res = pthread_create(&pid,NULL,thfrunc,NULL);
    if(res)
    {
        std::cout <<"Failed to create thread!!!" <<  std::endl;
    }

    ros::init(argc, argv, "basler_ros_trigger");
    NodeHandle nh;
    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/basler/point_cloud",1);
    int exitCode = EXIT_FAILURE;
#if VISURLIZE
    viewer->setBackgroundColor(1, 1, 1);
    viewer->addCoordinateSystem(1000);
#endif

    try
    {
        CBlazeCamera::InitProducer();
        m_camera.OpenFirstCamera();
        m_camera.ParametrizeFromFile("basler-101.pfs");

        // Work with camera...
        run();

        m_camera.Close();
    }
    catch ( GenICam::GenericException& e )
    {
        std::cerr << "Exception occurred: " << std::endl << e.GetDescription() << std::endl;
        // After successfully opening the camera, the IsConnected method can be used
        // to check if the device is still connected.
        if ( m_camera.IsOpen() && ! m_camera.IsConnected() )
        {
            std::cerr << "Camera has been removed." << std::endl;
        }
    }

    // Release the GenTL producer and all of its resources.
    // Note: Don't call TerminateProducer() until the destructor of the CBlazeCamera
    // class has been called. The destructor may require resources which may not
    // be available anymore after TerminateProducer() has been called.
    if ( CBlazeCamera::IsProducerInitialized() )
        CBlazeCamera::TerminateProducer();  // Won't throw any exceptions
    exitCode = EXIT_SUCCESS;

exit:
    std::cout << std::endl << "Press Enter to exit." << std::endl;
    while (std::cin.get() != '\n');

    return exitCode;
}
