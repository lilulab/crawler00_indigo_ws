#ifndef CONNECTION_H
#define CONNECTION_H

#include <inttypes.h>

/// Abstract base class for Connection objects. Wraps low-level sockets or other
/// two-way communication, providing send, receive, peek, and poll functions,
/// allowing high-level code to be platform and implementation agnostic.
class Connection {
    public:
        /// Send data out on the socket; this is nonblocking.
        /// @param buffer bytes to be sent.
        /// @param length number of bytes to be sent; buffer must be at least
        /// this long to avoid potential memory access errors.
        /// @returns true if successfully sent the entire packet, false
        /// on an error (which could be a partial send).
        virtual bool send(const uint8_t* buffer, int length) = 0;

        /// Receive data from the socket; this is nonblocking.
        /// @param buffer buffer to write incoming bytes to
        /// @param length number of bytes to attempt to read; buffer must be at
        /// least this long to avoid potential memory access errors.
        /// @returns the number of bytes read, or a negative value for an error.
        virtual int receive(uint8_t* buffer, int length) = 0;

        /// Peeks at data from the socket without removing it from the queue;
        /// this is nonblocking.
        /// @param buffer buffer to write incoming bytes to
        /// @param length number of bytes to attempt to peek at; buffer must be
        /// at least this long to avoid potential memory access errors.
        /// @returns the number of bytes peeked at, or a negative value for an
        /// error.
        virtual int peek(uint8_t* buffer, int length) = 0;

        /// Blocks until data is ready to be read.
        /// @param timeout Timeout in ms after which call returns.
        /// @returns true if data is ready, false on timeout.
        virtual bool wait_for_data(int timeout) = 0;
};

#endif // CONNECTION_H
