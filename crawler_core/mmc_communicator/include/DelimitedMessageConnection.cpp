#include "DelimitedMessageConnection.h"
#include "google/protobuf/io/coded_stream.h"

// The maximum number of bytes that might be used to encode a varint32 (this
// is defined in google's documentation)
#define MAX_VARINT32_SIZE 10

DelimitedMessageConnection::DelimitedMessageConnection(
    Connection* connection_, int in_buffer_size_, int out_buffer_size_) {

    connection = connection_;

    // Initialize all buffers:
    in_buffer_size = in_buffer_size_; 
    in_buffer = new uint8_t[in_buffer_size];
    out_buffer_size = out_buffer_size_;
    out_buffer = new uint8_t[out_buffer_size];
}

DelimitedMessageConnection::~DelimitedMessageConnection() {

    delete[] in_buffer;
    delete[] out_buffer;
}

bool DelimitedMessageConnection::send_message(
    const google::protobuf::MessageLite* msg_out) {

    // Get size, and the length of that size in a varint encoding.
    uint32_t msg_length = msg_out->ByteSize();
    int varint_length =
        google::protobuf::io::CodedOutputStream::VarintSize32(msg_length);

    // Ensure we can hold all of the data.
    if (varint_length + msg_length > out_buffer_size)
        return false;

    // Write size to the output buffer.
    google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        msg_length, out_buffer);

    // Write message to the output buffer. We can use CachedSizes here because
    // we just called ByteSize.
    msg_out->SerializeWithCachedSizesToArray(out_buffer + varint_length);

    // Send message.
    int send_res = connection->send(out_buffer, varint_length + msg_length);
    return (send_res >= 0);
}

// TODO: allow for fragmented messages? Basically, keep state between calls?
bool DelimitedMessageConnection::receive_message(
    google::protobuf::MessageLite& msg_in) {

    // Peek at bytes up to the biggest possible varint.
    if (in_buffer_size < MAX_VARINT32_SIZE)
        return false;
    int peek_len = connection->peek(in_buffer, MAX_VARINT32_SIZE);
    if (peek_len <= 0)
        return false;
    
    // Convert the incoming buffer to a coded input stream so we can read
    // lengths directly from it and so that we auto-enforce length constraints.
    google::protobuf::io::CodedInputStream input(in_buffer, in_buffer_size);
    // Ensure we only read as much as we were able to peek at:
    google::protobuf::io::CodedInputStream::Limit peek_limit =
        input.PushLimit(peek_len);

    // Attempt to read a varint32 that indicates the delimited message size.
    uint32_t message_size;
    if (!input.ReadVarint32(&message_size)) {
        // Invalid varint 32 (i.e., has overrun allotted 10 bytes). Clear
        // remaining data that we can read so we can start fresh.
        clear_incoming();
        return false; 
    }
    // The number of bytes the message size took.
    int varint_size =
        google::protobuf::io::CodedOutputStream::VarintSize32(message_size);

    // Attempt to read the entire message; return if not available.
    // TODO: for fragments: continue later instead of failing here?
    int recv_len = connection->receive(in_buffer, varint_size + message_size);
    if (recv_len < varint_size + message_size)
        return false;
   
    // Remove the peek len limit, and add one for the amount we actually
    // received.  Note that we have already moved past the varint in our reading
    // so we must take this into account when setting the limit.
    input.PopLimit(peek_limit);
    google::protobuf::io::CodedInputStream::Limit size_limit =
        input.PushLimit(recv_len - varint_size);
   
    // Try to parse message.
    return msg_in.ParseFromCodedStream(&input);
}

bool DelimitedMessageConnection::receive_message(
    google::protobuf::MessageLite& msg_in, int timeout) {

    // Poll the connection first, waiting for data.
    bool has_data = connection->wait_for_data(timeout);
    if (!has_data)
        return false;
    return receive_message(msg_in);
}

void DelimitedMessageConnection::clear_incoming() {

    // Do non-blocking reads into the input buffer until we have no more data.
    int recv_len = 1;
    while (recv_len > 0)
        recv_len = connection->receive(in_buffer, in_buffer_size);
}
