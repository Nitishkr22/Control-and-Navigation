#include "ros/ros.h"
#include "std_msgs/String.h"

void publishData1(ros::Publisher& pub1) {
    std_msgs::String msg;
    msg.data = "Hello from Function 1";
    pub1.publish(msg);
}

void publishData2(ros::Publisher& pub2) {
    std_msgs::String msg;
    msg.data = "Hello from Function 2";
    pub2.publish(msg);
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "data_publisher_node");
    
    // Create a node handle
    ros::NodeHandle nh;

    // Create two publishers
    ros::Publisher pub1 = nh.advertise<std_msgs::String>("data_topic_1", 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::String>("data_topic_2", 10);

    // Set the loop rate (adjust as needed)
    ros::Rate loop_rate(1);  // 1 Hz

    while (ros::ok()) {
        // Call the functions to publish data
        publishData1(pub1);
        publishData2(pub2);

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
