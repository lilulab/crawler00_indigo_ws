#include "UdpConnection.h"
#include <iostream>
#include <netdb.h>
#include <strings.h>
#include <unistd.h> // For close()

UdpConnection* UdpConnection::create(std::string hostname, int port) {
    return UdpConnection::create(hostname.c_str(), port);
}

UdpConnection* UdpConnection::create(const char* hostname, int port) {
    // Setup info for UDP transmissions
    int eth_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

    // Negative socket file descriptors indicate errors
    if (eth_socket_fd < 0) {
        std::cerr << "Could not open UDP socket!\n";
        std::cerr.flush();
        return NULL;
    }

    // Get the hostname to send info to:
    struct hostent *hp;
    hp = gethostbyname(hostname);
    if (hp == 0) {
        std::cerr << "Unknown host name.\n";
        std::cerr.flush();
        return NULL;
    }

    // Actually create the object.
    return new UdpConnection(eth_socket_fd, hp, port);
}

// The constructor just fills in the actual data structures.
UdpConnection::UdpConnection(int eth_socket_fd_, struct hostent *hp, int port) {
    eth_socket_fd = eth_socket_fd_;

    svr_length = sizeof(struct sockaddr_in);

    bcopy((unsigned char *)hp->h_addr,
          (unsigned char *)&server.sin_addr,
          hp->h_length);
    server.sin_family = AF_INET;
    server.sin_port = htons(port);

    // This allows us to poll for data ready to be read before calling recvfrom
    poll_struct.fd = eth_socket_fd;
    poll_struct.events = POLLIN;
}

// Destructor to cleanup socket.
UdpConnection::~UdpConnection() {
    close(eth_socket_fd);
}

bool UdpConnection::send(const uint8_t* buffer, int length) {
    // sendto will fail when sending to a bad socket fd, and may not write all
    // bytes if the socket is full. The MSG_DONTWAIT flag is set to ensure
    // nonblocking.
    // If there is an error while sending a packet, return false, but don't
    // disconnect.
    int n = sendto(eth_socket_fd, buffer, length, MSG_DONTWAIT,
            (struct sockaddr *)&server, svr_length);
    if (n < length)
        return false;
    return true;
}

int UdpConnection::receive(uint8_t* buffer, int length) {
    // Since recvfrom can actually change the length variable, we
    // copy it here so we don't modify it.
    socklen_t tmp_from_length = from_length;
    return recvfrom(eth_socket_fd, buffer, length, MSG_DONTWAIT,
        (struct sockaddr *)&from, &tmp_from_length);
}

int UdpConnection::peek(uint8_t* buffer, int length) {
    // Since recvfrom can actually change the length variable, we
    // copy it here so we don't modify it.
    socklen_t tmp_from_length = from_length;
    return recvfrom(eth_socket_fd, buffer, length, MSG_PEEK | MSG_DONTWAIT,
        (struct sockaddr *)&from, &tmp_from_length);
}

// Timeout is in ms.
bool UdpConnection::wait_for_data(int timeout) {
    // Poll the socket
    int num_events = poll(&poll_struct, 1, timeout);
    // Check for timeout (==0) or error (<0)
    return (num_events > 0);
}
