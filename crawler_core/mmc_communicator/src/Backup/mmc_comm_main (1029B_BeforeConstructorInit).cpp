#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

// MMC code
#include <string>
#include <iostream>
#include <memory>
#include "cppSeaSnakeCommExample/UdpConnection.h"
#include "cppSeaSnakeCommExample/DelimitedMessageConnection.h"
#include "cppSeaSnakeCommExample/protocol/MetaInfo.pb.h"

// IP address of modules.
const char* MMC_WhlLft_IP="192.168.0.101";
const char* MMC_WhlRft_IP="192.168.0.102";

const char* MMC_CbwLft_IP="192.168.0.103";
const char* MMC_CbwRft_IP="192.168.0.104";

const char* MMC_TrsBas_IP="192.168.0.105";

const char* MMC_ArmYaw_IP="192.168.0.106";
const char* MMC_ArmPic_IP="192.168.0.107";
const char* MMC_ArmPmt_IP="192.168.0.108";

const char* MMC_WrtRol_IP="192.168.0.109";
const char* MMC_WrtPic_IP="192.168.0.110";

// Port for connecting to modules.
#define TCP_MODULE_PORT 16667
#define UDP_MODULE_PORT 16668

float mmc_00_value = 0.7;
int bangbang_state = 0;

// Create class MMC
class MMC_Comm {
    public:
    MMC_Comm(const char*, int);
    float send_and_receive(int tMode, float tValue, int rMode);
};

// MMC_Comm Constructors Function
MMC_Comm::MMC_Comm(const char* hostname, int port) {

   // MMC code
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
        UdpConnection::create(MMC_WhlRft_IP, UDP_MODULE_PORT));
        
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

}

float MMC_Comm::send_and_receive(int tMode, float tValue, int rMode) {
    // Create Reveive Value Var
    float rValue = 1.234;
    
        bool success;
        
        // MMC code
        // Add position command.
        //module_command->set_position(mmc_00_value);
        //module_command->set_position(mmc_00_value);
        
        
        //module_command->set_velocity(mmc_00_value);
        //ROS_INFO("Send set_velocity = %f" , mmc_00_value);
        
        module_command->has_position();
        
        
        // Add other feedback requests:
        // Note: modules always respond with sensor feedback if module_command is
        // set.
        message_out.set_request_ethernet_info(true);

        // Serialize and send message:
        //std::cout << "Sending message.\n";
        success = delim_connection.send_message(&message_out);
        if (!success) {
            //std::cerr << "Failed when sending message; exiting." << std::endl;
            //std::cerr.flush();
            //return -1;
            ROS_INFO("Failed when sending message!!!");
        } else {
            //ROS_INFO("Sucessful sending message.");
            ROS_INFO("Sending message...");
        }
        

        // Try to receive a message; do a blocking wait for 5 seconds.
        int timeout_ms = 5000;
        //ROS_INFO("flag1");
        success = delim_connection.receive_message(message_in, timeout_ms);
        //ROS_INFO("flag2");
        if (!success){
            //return -1;
            ROS_INFO("Timeout occurred!!!");
        } else {
            ROS_INFO("Sucessful sent message. No timeout.");
        }
        // Print feedback:
        //std::cout << "Position feedback: ";

        //ROS_INFO("flag3");
        if (message_in.has_module_feedback()){
            //std::cout << message_in.module_feedback().position() << std::endl;
            //ROS_INFO("flag4");
            ROS_INFO("Position feedback: %f",message_in.module_feedback().position());
            ROS_INFO("Velocity feedback: %f",message_in.module_feedback().velocity());
        } else {
            //std::cout << "none" << std::endl;
            //ROS_INFO("flag5");
            ROS_INFO("None feedback!");
        }
        //ROS_INFO("flag6");
        
        //Change MMC00 var
        if (bangbang_state == 0){
            mmc_00_value += 0.01;
            if (mmc_00_value > 3.0)     bangbang_state = 1;
        } else {
            mmc_00_value -= 0.01;
            if (mmc_00_value < 0.0)     bangbang_state = 0;            
        }
        
    rValue = message_in.module_feedback().position();
        
    ROS_INFO("mmc_comm___________________ %u, %f, %u, %f", tMode, tValue, rMode, rValue);
    return rValue;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "mmc_comm_node");

    ros::NodeHandle n;
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(100);
    
    MMC_Comm mmc_comm(MMC_WhlRft_IP, UDP_MODULE_PORT);

 

    //ROS main while
    ROS_INFO("Wait for 5 sec, then start...");
    ros::Duration(5).sleep(); // sleep for half a second
    
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
 

        
        mmc_comm.send_and_receive(4,5,6);
        
        //ROS
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }//end while

    // Delete all global objects allocated by libprotobuf.
    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}//end main
