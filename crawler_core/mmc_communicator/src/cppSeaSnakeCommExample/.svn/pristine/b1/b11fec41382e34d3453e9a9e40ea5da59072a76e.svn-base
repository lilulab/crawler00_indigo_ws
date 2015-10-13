#ifndef DELIMITED_MESSAGE_CONNECTION_H
#define DELIMITED_MESSAGE_CONNECTION_H
#include "Connection.h"
#include "google/protobuf/message_lite.h"

/// Reads and writes entire protobuf messages of a given type, adding and
/// parsing a length delimiting field on send and receive.
class DelimitedMessageConnection {
    private:
        Connection* connection;
        uint8_t* out_buffer;
        int32_t out_buffer_size;
        uint8_t* in_buffer;
        int32_t in_buffer_size;
    public:
        /// Wraps a low-level connection object, and sends and receives
        /// protobuf messages delimited by an initial varint32 size field.
        /// @param connection_ low level connection object; ownership is not
        /// transferred to the DelimitedMessageConnection, and so the object
        /// must be cleaned up by the parent at the conclusion of its use. Once
        /// the connection pointer no longer references a valid object, methods
        /// on this DelimitedMessageConnection are not safe to call.
        /// @param in_buffer_size_ The size of the input buffer; messages over
        /// this size (including delimation) are discarded. Defaults to 1024
        /// bytes.
        /// @param out_buffer_size_ The size of the output buffer; messages over
        /// this size (not including delimation) are not sent. Defaults to 1024
        /// bytes.
        /// @throws bad_alloc exception if in/out buffers cannot be allocated.
        DelimitedMessageConnection(Connection* connection_,
            int in_buffer_size_ = 1024, int out_buffer_size_ = 1024);
        /// Destructor
        ~DelimitedMessageConnection();
        /// Tries to send the message on the socket.
        /// @returns False if there is an error or true on a successful send.
        bool send_message(const google::protobuf::MessageLite* msg_out);
        /// Non-blocking receive; attempts to receive an entire message on the
        /// socket. On failure, clears any remaining data that can be received.
        /// @param msg_in Reference to a message which is filled with the
        /// incoming data. Implemented as a reference to prevent null pointer
        /// exceptions.
        /// @returns True on a successful receive, and writes the message to the
        /// msg_in argument; false on failure.
        bool receive_message(google::protobuf::MessageLite& msg_in);
        /// Blocking receive (with timeout); attempts to receive an entire
        /// message on the socket. On failure, clears any remaining data that
        /// can be received.
        /// @param msg_in Reference to a message which is filled with the
        /// incoming data. Implemented as a reference to prevent null pointer
        /// exceptions.
        /// @param timeout time to wait, in ms, before giving up on connection.
        /// A value of 0 indicated a non-blocking call; a value of -1 indicates
        /// a call that blocks until data is received (or an interruption
        /// occurs).
        /// @returns True on a successful receive, and writes the message to the
        /// msg_in argument; false on failure. 
        bool receive_message(google::protobuf::MessageLite& msg_in, int timeout);
    private:
        /// Consumes all data from the connection, using the input buffer as
        /// temporary space.
        void clear_incoming();
};

#endif // DELIMITED_MESSAGE_CONNECTION_H

