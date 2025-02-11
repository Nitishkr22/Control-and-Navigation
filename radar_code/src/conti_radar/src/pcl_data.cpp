#include "pdk/pdk_interface.hpp"
#include "pdk_RadarDetectionImage.pb.h"
#include "pdk_RadarStatus.pb.h"
#include "pdk_RadarObjectList.pb.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "conti_radar/radar_obj.h"
#include <thread>

using namespace std;

ros::Publisher radar_obj_pub;
ros::Publisher radar_pc_pub;

void OnReceiveObjects(const std::string &buffer)
{
    pb::PDK::RadarObjectList obj{};
    obj.ParseFromString(buffer);
    std::cout << "Received " << obj.u_nofusedobjects() << " detections" << std::endl;

    conti_radar::radar_obj radar_msg;
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "radar_frame";
    cloud_msg.height = 1;
    cloud_msg.width = obj.u_nofusedobjects();
    cloud_msg.is_dense = false;
    
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(3, 
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (int i = 0; i < obj.u_nofusedobjects(); i++) {
        pb::PDK::RadarObjectList_RadarObject ob = obj.a_radarobjectlist(i);
        
        radar_msg.f_DistX.push_back(ob.f_distx());
        radar_msg.f_DistY.push_back(ob.f_disty());
        radar_msg.f_DistZ.push_back(ob.f_distz());
        
        // Convert radar detections to Cartesian coordinates (assuming azimuth and range data are provided)
        *iter_x = ob.f_distx();
        *iter_y = ob.f_disty();
        *iter_z = ob.f_distz();

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    radar_obj_pub.publish(radar_msg);
    radar_pc_pub.publish(cloud_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar_ros_obj");
    ros::NodeHandle nh;
    radar_obj_pub = nh.advertise<conti_radar::radar_obj>("radar_obj", 1000);
    radar_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("radar_pointcloud", 1000);

    std::string config = "/opt/pdk/etc/pdk_config.json";
    if (argc > 1) {
        config = argv[1];
        if ((config == "-c") && (argc > 2)) {
            config = argv[2];
        }
    }

    std::cout << "Library version: " << PDK::GetVersion() << std::endl;
    PDK::CInterface::Init(config);
    PDK::CInterface::SetSensorObjectsCallback(OnReceiveObjects);
    std::this_thread::sleep_for(std::chrono::seconds(15));

    while (ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
