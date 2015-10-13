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
const char* MMC_00_IP="10.10.10.170";

// Port for connecting to modules.
#define TCP_MODULE_PORT 16667
#define UDP_MODULE_PORT 16668

float mmc_00_value = 0.5f;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "mmc_comm_node");

    ros::NodeHandle n;
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    //mmc
    ROS_INFO("MMC starting...");
    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    // Create a connection of the given type:
    // Note: std::auto_ptr is a smart pointer that retains sole ownership of
    // an object through a pointer and destroys that object when the unique_ptr
    // goes out of scope. This ensures the connection is cleaned up when we
    // return, and we don't have to call 'delete connection' for every error
    // case. (In C++11, use unique_ptr)
    std::auto_ptr<UdpConnection> connection(
        UdpConnection::create(MMC_00_IP, UDP_MODULE_PORT));
    if (connection.get() == NULL) {
        //std::cerr << "Could not open port; exiting." << std::endl;
        //std::cerr.flush();
        //return -1;
	ROS_INFO("Could not open port!!!");
    }

    // Create a delimited message connection that sends/receives HeaderMessages:
    DelimitedMessageConnection delim_connection(
        (Connection*)connection.get());

    // The message to send to the module.  This message is heirarchical; the
    // HeaderMessage is the top level (see the protocol/MetaInfo.pb.h file for
    // detailed structure).
    protos::HeaderMessage message_out;
    // The incoming message.
    protos::HeaderMessage message_in;
    
    // Note: Each field has appropriately typed 'has', 'clear', 'set', and 'get'
    // methods. All non-required fields can be set or cleared. The library is
    // optimized for message reuse, to it is generally preferred to not create
    // new messages in a sending loop, but modify existing ones.

    // The header message can contain a module command message, which commands
    // position, velocity, etc., depending on what is implemented on the boards.
    // We add it and get a pointer to it:
    protos::ModuleCommand* module_command =
        message_out.mutable_module_command();

    // Add position command.
    module_command->set_position(mmc_00_value);

    // Add other feedback requests:
    // Note: modules always respond with sensor feedback if module_command is
    // set.
    message_out.set_request_ethernet_info(true);

    // Serialize and send message:
    //std::cout << "Sending message.\n";
    ROS_INFO("Sending message...");
    bool success = delim_connection.send_message(&message_out);
    if (!success) {
        //std::cerr << "Failed when sending message; exiting." << std::endl;
        //std::cerr.flush();
        //return -1;
        ROS_INFO("Failed when sending message!!!");
    }

    // Try to receive a message; do a blocking wait for 5 seconds.
    int timeout_ms = 5000;
    success = delim_connection.receive_message(message_in, timeout_ms);
    if (!success){
        //return -1;
        ROS_INFO("Timeout occurred!!!");
    }
    // Print feedback:
    //std::cout << "Position feedback: ";

    if (message_in.has_module_feedback())
        //std::cout << message_in.module_feedback().position() << std::endl;
        ROS_INFO("Position feedback: %s",message_in.module_feedback().position());
    else
        //std::cout << "none" << std::endl;
        ROS_INFO("None feedback!");

    // Delete all global objects allocated by libprotobuf.
    google::protobuf::ShutdownProtobufLibrary();

    //return 0;

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
