#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

//mmc
#include <string>
#include <iostream>
#include <memory>
#include "cppSeaSnakeCommExample/UdpConnection.h"
#include "cppSeaSnakeCommExample/DelimitedMessageConnection.h"
#include "cppSeaSnakeCommExample/protocol/MetaInfo.pb.h"

// IP address of modules.
const char* mmc_axis00="10.10.10.170";

// Port for connecting to modules.
#define TCP_MODULE_PORT 16667
#define UDP_MODULE_PORT 16668



int main(int argc, char **argv)
{
    ros::init(argc, argv, "mmc_comm_node");

    ros::NodeHandle n;
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    //mmc
    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    int count = 0;
    while (ros::ok())
    {
    //std_msgs::String msg;

    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    //chatter_pub.publish(msg);

    ROS_INFO("ROS is OK... %u",count);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    }//end while


    return 0;
}//end main
