#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

// MMC code
#include <string>
#include <iostream>
#include <memory>
#include "UdpConnection.h"
#include "DelimitedMessageConnection.h"
#include "protocol/Core.pb.h"
//v_rep joint state messages
#include <geometry_msgs/Twist.h>

// Crawler Messages
#include "crawler_msgs/JointCmd.h"

// Joint State Msgs
#include <sensor_msgs/JointState.h>

using namespace biorobotics::firmware;

// IP address of modules.
const char* MMC_WhlLft_IP="192.168.0.100";
const char* MMC_WhlRgt_IP="192.168.0.101";

const char* MMC_CbwLft_IP="192.168.0.102";
const char* MMC_CbwRgt_IP="192.168.0.103";

const char* MMC_TrsBas_IP="192.168.0.104";

const char* MMC_ArmYaw_IP="192.168.0.105";
const char* MMC_ArmPic_IP="192.168.0.106";
const char* MMC_ArmPmt_IP="192.168.0.107";

const char* MMC_WrtRol_IP="192.168.0.108";
const char* MMC_WrtPic_IP="192.168.0.109";


// IP to be used if in LAB
// const char* MMC_WhlLft_IP="10.10.10.183";
// const char* MMC_WhlRgt_IP="10.10.10.182";
// const char* MMC_CbwLft_IP="10.10.10.158";
// const char* MMC_CbwRgt_IP="10.10.10.147";
// const char* MMC_TrsBas_IP="10.10.10.135";
// const char* MMC_ArmYaw_IP="10.10.10.137";
// const char* MMC_ArmPic_IP="10.10.10.148";
// const char* MMC_ArmPmt_IP="10.10.10.143";
// const char* MMC_WrtRol_IP="10.10.10.146";
// const char* MMC_WrtPic_IP="10.10.10.193";

#define MMC_WhlLft_JointID  0
#define MMC_WhlRgt_JointID  1

#define MMC_CbwLft_JointID  2
#define MMC_CbwRgt_JointID  3

#define MMC_TrsBas_JointID  4

#define MMC_ArmYaw_JointID  5
#define MMC_ArmPic_JointID  6
#define MMC_ArmPmt_JointID  7

#define MMC_WrtRol_JointID  8
#define MMC_WrtPic_JointID  9

// Port for connecting to modules.
#define TCP_MODULE_PORT 16667
#define UDP_MODULE_PORT 16668

// MMC Comm Mode
#define CMD_SET_POSITION 1
#define CMD_SET_VELOCITY 2
#define CMD_SET_TORQUE 3

// MMC Feedback Mode
#define FB_GET_POSITION 1
#define FB_GET_VELOCITY 2
#define FB_GET_TORQUE 3

float mmc_00_value = 0.7;
int bangbang_state = 0;

int is_joy_updating = 0;

// Create class MMC
class MMC_Comm {
    public:

    float send_and_receive(int tMode, float tValue, int rMode);
    MMC_Comm(const char*, int);

    private:
    // Create a connection of the given type:
    std::auto_ptr<UdpConnection> connection;

    // Create a delimited message connection that sends/receives HeaderMessages:
    DelimitedMessageConnection delim_connection;

    // The message to send to the module.
    protos::RootMessage message_out;
    // The incoming message.
    protos::RootMessage message_in;

    // pointer to message_out
    protos::Command* module_command;

};

// MMC_Comm Constructors Function
MMC_Comm::MMC_Comm(const char* hostname, int port) :
connection(UdpConnection::create(hostname, port)),
delim_connection((Connection*)connection.get()),
module_command(message_out.mutable_command())
{

   // MMC code
    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;


    if (connection.get() == NULL) {
        //std::cerr << "Could not open port; exiting." << std::endl;
        //std::cerr.flush();
        //return -1;
        ROS_INFO("Could not open port!!!");
    }

    ROS_INFO("MMC connected, ip = %s",hostname);

}

float MMC_Comm::send_and_receive(int tMode, float tValue, int rMode) {
    // Create Reveive Value Var
    float rValue = 1.234;

        bool success;

        // MMC code
        switch (tMode) {
        case CMD_SET_POSITION:
            // Add position command.
            module_command->set_position(tValue);
            ROS_INFO("Send set_position = %f" , tValue);
            break;

        case CMD_SET_VELOCITY:
            module_command->set_velocity(tValue);
            ROS_INFO("Send set_velocity = %f" , tValue);
            break;

        case CMD_SET_TORQUE:
            module_command->set_torque(tValue);
            ROS_INFO("Send set_torque = %f" , tValue);
            break;

        default:
            // Code
            break;
        }


        //module_command->has_position();


        // Add other feedback requests:
        // Note: modules always respond with sensor feedback if module_command is
        // set.
        //message_out.set_request_ethernet_info(true);
    	message_out.set_request_feedback(true);

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
            //ROS_INFO("Sending message...");
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
            //ROS_INFO("Sucessful sent message. No timeout.");
            //ROS_INFO("Sucessful sent message to %s", hostname);
        }
        // Print feedback:
        //std::cout << "Position feedback: ";

        //ROS_INFO("flag3");
        if (message_in.has_feedback()){
            //std::cout << message_in.feedback().position() << std::endl;
            //ROS_INFO("flag4");
            switch (rMode) {
            case FB_GET_POSITION:
                rValue = message_in.feedback().position();
                ROS_INFO("...........................Get feedback_position = %f" , rValue);
                break;

            case FB_GET_VELOCITY:
                rValue = message_in.feedback().velocity();
                ROS_INFO("...........................Get feedback_velocity = %f" , rValue);
                break;

            case FB_GET_TORQUE:
                rValue = message_in.feedback().torque();
                ROS_INFO("...........................Get feedback_torque = %f" , rValue);
                break;

            default:
                // Code
                break;
            }
            ROS_INFO("Position feedback: %f",message_in.feedback().position());
            ROS_INFO("Velocity feedback: %f",message_in.feedback().velocity());
        } else {
            //std::cout << "none" << std::endl;
            //ROS_INFO("flag5");
            ROS_INFO("None feedback!");
        }
        //ROS_INFO("flag6");

        //Change MMC00 var
        /*
        if (bangbang_state == 0){
            mmc_00_value += 0.01;
            if (mmc_00_value > 3.0)     bangbang_state = 1;
        } else {
            mmc_00_value -= 0.01;
            if (mmc_00_value < 0.0)     bangbang_state = 0;
        }
        */

    rValue = message_in.feedback().position();

    //ROS_INFO("mmc_comm___________________ %u, %f, %u, %f", tMode, tValue, rMode, rValue);
    return rValue;
}

// Wheel
 MMC_Comm mmc_comm_WhlLft(MMC_WhlLft_IP, UDP_MODULE_PORT);
 MMC_Comm mmc_comm_WhlRgt(MMC_WhlRgt_IP, UDP_MODULE_PORT);

// CBW
 MMC_Comm mmc_comm_CbwLft(MMC_CbwLft_IP, UDP_MODULE_PORT);
 MMC_Comm mmc_comm_CbwRgt(MMC_CbwRgt_IP, UDP_MODULE_PORT);

// Trans Base
 MMC_Comm mmc_comm_TrsBas(MMC_TrsBas_IP, UDP_MODULE_PORT);

// Arm
 MMC_Comm mmc_comm_ArmYaw(MMC_ArmYaw_IP, UDP_MODULE_PORT);
 MMC_Comm mmc_comm_ArmPic(MMC_ArmPic_IP, UDP_MODULE_PORT);
 MMC_Comm mmc_comm_ArmPmt(MMC_ArmPmt_IP, UDP_MODULE_PORT);

// Wrist
 MMC_Comm mmc_comm_WrtRol(MMC_WrtRol_IP, UDP_MODULE_PORT);
 MMC_Comm mmc_comm_WrtPic(MMC_WrtPic_IP, UDP_MODULE_PORT);

// Feedback Values
float fb_values[10];

ros::Time joint_cmd_current_time, joint_cmd_last_time; 

// Call Back Func
void jointCmdCallback(const crawler_msgs::JointCmd::ConstPtr& joint_cmd)
{
    is_joy_updating = 1;
    joint_cmd_current_time = joint_cmd->header.stamp;

    //ROS_INFO("ROS is OK.........WhlLft = %f",joint_cmd->jointCmdVel[MMC_WhlLft_JointID]);
    //ROS_INFO("ROS is OK.........WhlLft = %f",joint_cmd->jointCmdVel[MMC_WhlRgt_JointID]);

    // Wheel
    fb_values[MMC_WhlLft_JointID] = mmc_comm_WhlRgt.send_and_receive(CMD_SET_VELOCITY,
                                    joint_cmd->jointCmdVel[MMC_WhlLft_JointID],
                                    FB_GET_VELOCITY);

    fb_values[MMC_WhlRgt_JointID] = mmc_comm_WhlLft.send_and_receive(CMD_SET_VELOCITY,
                                    joint_cmd->jointCmdVel[MMC_WhlRgt_JointID],
                                    FB_GET_VELOCITY);

    // Arm
    fb_values[MMC_ArmYaw_JointID] = mmc_comm_ArmYaw.send_and_receive(CMD_SET_VELOCITY,
                                    joint_cmd->jointCmdVel[MMC_ArmYaw_JointID],
                                    FB_GET_POSITION);
    fb_values[MMC_ArmPic_JointID] = mmc_comm_ArmPic.send_and_receive(CMD_SET_VELOCITY,
                                    joint_cmd->jointCmdVel[MMC_ArmPic_JointID],
                                    FB_GET_POSITION);
    fb_values[MMC_ArmPmt_JointID] = mmc_comm_ArmPmt.send_and_receive(CMD_SET_VELOCITY,
                                    joint_cmd->jointCmdVel[MMC_ArmPmt_JointID],
                                    FB_GET_POSITION);

    // Trans Base
    fb_values[MMC_TrsBas_JointID] = mmc_comm_TrsBas.send_and_receive(CMD_SET_VELOCITY,
                                    joint_cmd->jointCmdVel[MMC_TrsBas_JointID],
                                    FB_GET_POSITION);
    // CBW
    fb_values[MMC_CbwLft_JointID] = mmc_comm_CbwLft.send_and_receive(CMD_SET_VELOCITY,
                                    joint_cmd->jointCmdVel[MMC_CbwLft_JointID],
                                    FB_GET_POSITION);
    fb_values[MMC_CbwRgt_JointID] = mmc_comm_CbwRgt.send_and_receive(CMD_SET_VELOCITY,
                                    joint_cmd->jointCmdVel[MMC_CbwRgt_JointID],
                                    FB_GET_POSITION);

    // Wrist
    fb_values[MMC_WrtRol_JointID] = mmc_comm_WrtRol.send_and_receive(CMD_SET_TORQUE,
                                    joint_cmd->jointCmdVel[MMC_WrtRol_JointID],
                                    FB_GET_POSITION);
    fb_values[MMC_WrtPic_JointID] = mmc_comm_WrtPic.send_and_receive(CMD_SET_TORQUE,
                                    joint_cmd->jointCmdVel[MMC_WrtPic_JointID],
                                    FB_GET_POSITION);

    joint_cmd_last_time = joint_cmd_current_time;
    is_joy_updating = 0;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mmc_comm_node");

    ros::NodeHandle n;

    ros::Rate loop_rate(100);

    // Subscribe to the CrawlerCMD
    ros::Subscriber joint_cmd_sub_ = n.subscribe("crawler/joint_cmd", 10, jointCmdCallback);

    // Publish Joint State
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    //Publish Joint State to V-rep
    //only use Twist message as a six-size float array
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/v_rep/joint_states", 1);    

    // Joint State message declarations
    sensor_msgs::JointState joint_state;

    //V-rep joint state message
    geometry_msgs::Twist pos;

    //ROS main while
    ROS_INFO("Wait for 3 sec, then start...");
    ros::Duration(3).sleep(); // sleep for half a second

    ROS_INFO("---Ready to Roll---");

    int count = 0;
    while (ros::ok())
    {
        //std_msgs::String msg;

        //std::stringstream ss;
        //ss << "hello world " << count;
        //msg.data = ss.str();

        //ROS_INFO("%s", msg.data.c_str());

        //chatter_pub.publish(msg);

        //ROS_INFO("ROS is OK... %u",count);

        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(10);
        joint_state.position.resize(10);

        joint_state.name[0] ="CmassL_joint";
        joint_state.position[0] = fb_values[MMC_CbwLft_JointID] /100;
	      pos.linear.x = fb_values[MMC_CbwLft_JointID] /100;

        joint_state.name[1] ="CmassR_joint";
        joint_state.position[1] = fb_values[MMC_CbwRgt_JointID]/100;
	      pos.linear.y = fb_values[MMC_CbwRgt_JointID]/100;

        joint_state.name[2] ="TransBase_joint";
        joint_state.position[2] = (17.495 - fb_values[MMC_TrsBas_JointID])/100 * -1;
	      pos.linear.z = (17.495 - fb_values[MMC_TrsBas_JointID])/100 * -1;
        
	      joint_state.name[3] ="Azimuth_joint";
        joint_state.position[3] = fb_values[MMC_ArmYaw_JointID] * -1;
	      pos.angular.x = fb_values[MMC_ArmYaw_JointID] * -1;

        joint_state.name[4] ="Altitude_joint";
        joint_state.position[4] = (fb_values[MMC_ArmPic_JointID] - 0.0514) * -1;
	      pos.angular.y = (fb_values[MMC_ArmPic_JointID] - 0.0514) * -1;

        joint_state.name[5] ="Prismatic_joint";
        joint_state.position[5] = fb_values[MMC_ArmPmt_JointID] / 100 * -1;
	      pos.angular.z = fb_values[MMC_ArmPmt_JointID] / 100 * -1;

        joint_state.name[6] ="WristPivot_joint";
        joint_state.position[6] = (fb_values[MMC_WrtPic_JointID] - 400)
                                    *(0.00872 + 0.43)
                                    / (-2145 - 400) - 0.43 + 3.14 ;

        joint_state.name[7] ="WristLower_joint";
        joint_state.position[7] = fb_values[MMC_WrtRol_JointID] * 0.2617 / 1800;

        joint_state.name[8] ="Wheel_speed_Left";
        joint_state.position[8] = fb_values[MMC_WhlLft_JointID];

        joint_state.name[9] ="Wheel_speed_Right";
        joint_state.position[9] = fb_values[MMC_WhlRgt_JointID];


        // Code to stop the wheel motor if no Joystick update.
        //ROS_INFO("joint_cmd_current_time = %f",joint_cmd_current_time.toSec());
        //ROS_INFO("joint_cmd_last_time = %f",joint_cmd_last_time.toSec());

        if (joint_cmd_current_time == joint_cmd_last_time){

          ROS_INFO("No Joystick update");

          // Wheel
          // fb_values[MMC_WhlLft_JointID] = mmc_comm_WhlRgt.send_and_receive(CMD_SET_VELOCITY,
          //                                 0,
          //                                 FB_GET_VELOCITY);

          // fb_values[MMC_WhlRgt_JointID] = mmc_comm_WhlLft.send_and_receive(CMD_SET_VELOCITY,
          //                                 0,
          //                                 FB_GET_VELOCITY);
        }

        joint_pub.publish(joint_state);
        vel_pub.publish(pos);



        //ROS
        ros::spinOnce();

        loop_rate.sleep();
        ++count;

    }//end while

    mmc_comm_WhlRgt.send_and_receive(CMD_SET_VELOCITY,0,0);
    mmc_comm_WhlLft.send_and_receive(CMD_SET_VELOCITY,0,0);
    // Delete all global objects allocated by libprotobuf.
    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}//end main
