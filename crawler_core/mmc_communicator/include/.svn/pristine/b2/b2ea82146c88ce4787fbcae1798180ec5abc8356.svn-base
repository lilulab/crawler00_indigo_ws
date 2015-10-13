#ifndef UDP_CONNECTION_H
#define UDP_CONNECTION_H

#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <poll.h>
#include "Connection.h"

/// UDP socket implementation of Connection abstract base class.
class UdpConnection /* final */ : Connection { // NOTE: enable final if compiling with c++11
    private:
        // Make constructor and copy constructor private; only way to create is
        // from create method.
        UdpConnection(int eth_socket_fd_, struct hostent *hp, int port);
        UdpConnection(UdpConnection& from);
        
        // Information about the connection
        int eth_socket_fd, svr_length;
        struct sockaddr_in server;

        // Information about the sender returned on a recv call.
        struct sockaddr_in from;
        socklen_t from_length;

        // This allows us to poll for data ready to be read before calling
        // recvfrom
        struct pollfd poll_struct;

    public:
        /// Destructor to clean up socket.
        virtual ~UdpConnection();

        /// Returns a new UdpConnection object.  On failure, 'null' is returned.
        /// Note: this returns an object allocated with 'new'; 'delete' must be
        /// called to cleanup!
        static UdpConnection* create(std::string hostname, int port);
        /// Returns a new UdpConnection object.  On failure, 'null' is returned.
        /// Note: this returns an object allocated with 'new'; 'delete' must be
        /// called to cleanup!
        static UdpConnection* create(const char* hostname, int port);

        ////////////////////////////////////////////////////////////////////////
        // Implement members from Connection base class
        ////////////////////////////////////////////////////////////////////////

        bool send(const uint8_t* buffer, int length);
        int receive(uint8_t* buffer, int length);
        int peek(uint8_t* buffer, int length);
        bool wait_for_data(int timeout);
};

#endif // UDP_CONNECTION_H
