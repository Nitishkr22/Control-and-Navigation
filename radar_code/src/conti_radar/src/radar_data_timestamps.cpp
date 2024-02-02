#include "pdk/pdk_interface.hpp"
#include "pdk_RadarDetectionImage.pb.h"
#include "pdk_RadarObjectList.pb.h"
#include <iostream>
#include <cmath> // Include cmath for trigonometric functions
#include <thread>
#include <fstream>
#include "pdk_MsgHeader.pb.h"
#include <ctime>
#include <iomanip>
#include <sstream> 

// void OnReceiveDetections(const std::string &buffer)
// {
//     pb::PDK::RadarDetectionImage radarImage{};
//     radarImage.ParseFromString(buffer);

//     std::cout << "Received Radar Image" << std::endl;

//     for (const auto &detection : radarImage.a_radardetectionlist())
//     {
//         // Check if the f_Range is less than 4 meters
//         if (detection.f_range() < 80.0)
//         {
//             // Calculate x and y components of distances
//             float x_dist = detection.f_range() * std::cos(detection.a_azang_hyp(0)); // Assuming the first azimuth hypothesis
//             float y_dist = detection.f_range() * std::sin(detection.a_azang_hyp(0)); // Assuming the first azimuth hypothesis

//             std::cout << "Detection within 4 meters range:" << std::endl
//                       << "f_Range: " << detection.f_range() << " m" << std::endl
//                       << "f_VrelRad: " << detection.f_vrelrad() << " m/s" << std::endl
//                       << "x_dist: " << x_dist << " m" << std::endl
//                       << "y_dist: " << y_dist << " m" << std::endl
//                       // Include other relevant information
//                       << std::endl;
//         }
//     }
// }


void OnReceiveRadarImage(const std::string &buffer)
{
    pb::PDK::RadarDetectionImage radarImage{};
    radarImage.ParseFromString(buffer);

    if (radarImage.u_nofdetections() == 0)
    {
        return;
    }

    // Open the CSV file in append mode
    std::ofstream outputFile("radar_image_data.csv", std::ios::out | std::ios::app);

    std::cout << "Received " << radarImage.u_nofdetections() << " radar detections" << std::endl;

    for (const auto &detection : radarImage.a_radardetectionlist())
    {
        // Extract timestamp from SensorMsgHeader
        pb::PDK::MsgHeader_Time timestamp = radarImage.t_header().t_commonheader().t_stamp();

        // Get system time in nanoseconds
        auto systemTime = std::chrono::high_resolution_clock::now().time_since_epoch().count();

        // Format system time to a human-readable string
        auto systemTimeInSeconds = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm *timeInfo = std::localtime(&systemTimeInSeconds);
        char buffer[80];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeInfo);
        std::ostringstream formattedTime;
        formattedTime << buffer << "." << std::setw(9) << std::setfill('0') << systemTime % 1000000000;

        // Write the radar detection features along with timestamps to the CSV file
        outputFile << formattedTime.str() << ","
                   << timestamp.u_sec() << "," << timestamp.u_nsec() << ","
                   << detection.f_range() << "," << detection.f_vrelrad() << ","
                   // Include other relevant information
                   << std::endl;
    }

    // Close the CSV file
    outputFile.close();
}


void OnReceiveObjects(const std::string &buffer)
{
    pb::PDK::RadarObjectList obj{};
    obj.ParseFromString(buffer);

    if (obj.u_nofusedobjects() == 0)
    {
        return;
    }

    // Open the CSV file in append mode
    std::ofstream outputFile("radar_data.csv", std::ios::out | std::ios::trunc);
    outputFile << "Timestamp_Sec, Timestamp_Nsec, Formatted_Time, Radar_Epoch_Time, Obj_ID, Dist_X, Dist_Y, Vrel_X, Vrel_Y" << std::endl;

    std::cout << "Received " << obj.u_nofusedobjects() << " objects" << std::endl;

    for (const auto &ro : obj.a_radarobjectlist())
    {
        // Check if the calculated range is less than 10 meters
        double range = std::sqrt(ro.f_distx() * ro.f_distx() + ro.f_disty() * ro.f_disty());

        switch (obj.e_providedkinematics())
        {
            case 0:
                // Handle relative kinematics
                std::cout<<"relative kinmatics"<<std::endl;
                break;

            case 1:
                // Handle absolute kinematics
                std::cout<<"absolute kinmatics"<<std::endl;
                break;

            case 2:
                // Handle both relative and absolute kinematics
                std::cout<<"relative and absolute kinmatics"<<std::endl;
                break;

            default:
                // Handle the default case (if necessary)
                break;
        }

        if (range < 100.0)
        {
            // Extract timestamp from MsgHeader
            pb::PDK::MsgHeader_Time timestamp = obj.t_header().t_commonheader().t_stamp();
            // uint64_t epochTime = static_cast<uint64_t>(timestamp.u_sec()) * 1000000000 + timestamp.u_nsec();
            // std::cout<<"epoch time is: "<<epochTime<<std::endl;

            auto systemTime1 = std::chrono::high_resolution_clock::now().time_since_epoch().count(); // system epoch time in nanosecond
            
            auto systemTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()); // system epoch time in seconds
            uint64_t radar_time_epoch = static_cast<uint64_t>(systemTime) * 1000000000 + timestamp.u_nsec();
            std::cout<<"radar_epoch time is: "<<radar_time_epoch<<std::endl;
            std::cout<<"epoch time is: "<<systemTime<<std::endl;
            std::tm* timeInfo = std::localtime(&systemTime);
            
            char buffer[80];
            std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeInfo);
            
            std::string radar_time_std = buffer;
            radar_time_std += "." + std::to_string(timestamp.u_nsec());
            std::cout<<"formatted time is: "<<radar_time_std<<std::endl;


            // Write the object features along with timestamps to the CSV file
            outputFile << timestamp.u_sec() << "," << timestamp.u_nsec() << ","<<radar_time_std << ","<<radar_time_epoch << ","
                       << ro.u_objid() << "," << ro.f_distx() << "," << ro.f_disty() << ","
                       << ro.f_vrelx() << "," << ro.f_vrely() << ","
                       // Include other relevant information
                       << std::endl;
        }
    }

    // Close the CSV file
    outputFile.close();
}



// void OnReceiveObjects(const std::string &buffer)
// {
//     pb::PDK::RadarObjectList obj{};
//     obj.ParseFromString(buffer);

//     if (obj.u_nofusedobjects() == 0)
//     {
//         return;
//     }

//     std::cout << "Received " << obj.u_nofusedobjects() << " objects" << std::endl;

//     for (const auto &ro : obj.a_radarobjectlist())
//     {
//         // Calculate the range using the square root of f_distx^2 + f_disty^2
//         double range = std::sqrt(ro.f_distx() * ro.f_distx() + ro.f_disty() * ro.f_disty());

//         // Check if the calculated range is less than 10 meters
//         if (range < 10.0)
//         {
//             std::cout << "Object within 10 meters range:" << std::endl
//                       << "u_ObjId " << ro.u_objid() << std::endl
//                       << "f_DistX " << ro.f_distx() << " m" << std::endl
//                       << "f_DistY " << ro.f_disty() << " m" << std::endl
//                       << "Calculated Range: " << range << " m" << std::endl
//                       << "f_VrelX " << ro.f_vrelx() << " m/s" << std::endl
//                       << "f_VrelY " << ro.f_vrely() << " m/s" << std::endl
//                       // Include other relevant information
//                       << std::endl;
//         }
//     }
// }



int main(int argc, char *argv[])
{
    // Initialization and other setup code...
    std::string config{};
    config = "/opt/pdk/etc/pdk_config.json";
    if (argc > 1)
    {
        config = argv[1];
        if ((config == "-c") && (argc > 2))
        {
            config = argv[2];
        }
    }
    std::cout << "Library version: " << PDK::GetVersion() << std::endl;
    PDK::CInterface::Init(config);

    // PDK::CInterface::SetRDICallback(OnReceiveDetections);
    PDK::CInterface::SetRDICallback(OnReceiveRadarImage);
    // PDK::CInterface::SetSensorObjectsCallback(OnReceiveObjects);
    // std::this_thread::sleep_for(std::chrono::seconds(15));

    while (true)
    {
        //std::cout << (PDK::CInterface::PublishVehicleDynamics(vehicle_dynamics) ? "sent" :  "not_sent") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Rest of the code...

    return 0;
}
