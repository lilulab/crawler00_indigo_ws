all: example_sync

# Include protocol buffers source and messages
PROTOBUF_SRC := google/protobuf/message_lite.cc \
		google/protobuf/generated_message_util.cc \
		google/protobuf/wire_format_lite.cc \
		google/protobuf/io/coded_stream.cc \
		google/protobuf/io/zero_copy_stream.cc \
		google/protobuf/io/zero_copy_stream_impl.cc \
		google/protobuf/io/zero_copy_stream_impl_lite.cc \
        google/protobuf/stubs/common.cc \
        google/protobuf/stubs/once.cc \
        google/protobuf/stubs/atomicops_internals_x86_gcc.cc

# Include generated source files
PROTOBUF_GEN_SRC := $(wildcard protocol/*.cpp)            

CONNECTION_SRC := UdpConnection.cpp DelimitedMessageConnection.cpp

example_sync:
	g++ example_sync.cpp $(PROTOBUF_SRC) $(PROTOBUF_GEN_SRC) $(CONNECTION_SRC) \
                 -I ./ -o example
