#include <string>
#include <iostream>
#include "UdpConnection.h"
#include "DelimitedMessageConnection.h"
#include "protocol/MetaInfo.pb.h"
#include <memory>

// Port for connecting to modules.
#define TCP_MODULE_PORT 16667
#define UDP_MODULE_PORT 16668

// A good tutorial for basics of C++ protobuf API.
// https://developers.google.com/protocol-buffers/docs/cpptutorial

// Example of synchronous C++ Sea-snake comms
int main(int argc, char* argv[]) {
    if (argc != 2)
    {
        std::cout << "Usage: example <dest_ip>\n";
        std::cout << "Example:\n";
        std::cout << "  ./example 10.10.10.225\n";
        return 1;
    }

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
        UdpConnection::create(argv[1], UDP_MODULE_PORT));
    if (connection.get() == NULL) {
        std::cerr << "Could not open port; exiting." << std::endl;
        std::cerr.flush();
        return -1;
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
    module_command->set_position(0.0f);

    // Add other feedback requests:
    // Note: modules always respond with sensor feedback if module_command is
    // set.
    message_out.set_request_ethernet_info(true);

    // Serialize and send message:
    std::cout << "Sending message.\n";
    bool success = delim_connection.send_message(&message_out);
    if (!success) {
        std::cerr << "Failed when sending message; exiting." << std::endl;
        std::cerr.flush();
        return -1;
    }

    // Try to receive a message; do a blocking wait for 5 seconds.
    int timeout_ms = 5000;
    success = delim_connection.receive_message(message_in, timeout_ms);
    if (!success)
        return -1;

    // Print feedback:
    std::cout << "Position feedback: ";
    if (message_in.has_module_feedback())
        std::cout << message_in.module_feedback().position() << std::endl;
    else
        std::cout << "none" << std::endl;

    // Delete all global objects allocated by libprotobuf.
    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
