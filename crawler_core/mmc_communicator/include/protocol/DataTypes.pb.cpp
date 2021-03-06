// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: DataTypes.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "DataTypes.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
// @@protoc_insertion_point(includes)

namespace biorobotics {
namespace firmware {
namespace protos {

void protobuf_ShutdownFile_DataTypes_2eproto() {
  delete Quaternion::default_instance_;
  delete Vector3::default_instance_;
  delete Color::default_instance_;
}

#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
void protobuf_AddDesc_DataTypes_2eproto_impl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#else
void protobuf_AddDesc_DataTypes_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#endif
  Quaternion::default_instance_ = new Quaternion();
  Vector3::default_instance_ = new Vector3();
  Color::default_instance_ = new Color();
  Quaternion::default_instance_->InitAsDefaultInstance();
  Vector3::default_instance_->InitAsDefaultInstance();
  Color::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_DataTypes_2eproto);
}

#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AddDesc_DataTypes_2eproto_once_);
void protobuf_AddDesc_DataTypes_2eproto() {
  ::google::protobuf::::google::protobuf::GoogleOnceInit(&protobuf_AddDesc_DataTypes_2eproto_once_,
                 &protobuf_AddDesc_DataTypes_2eproto_impl);
}
#else
// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_DataTypes_2eproto {
  StaticDescriptorInitializer_DataTypes_2eproto() {
    protobuf_AddDesc_DataTypes_2eproto();
  }
} static_descriptor_initializer_DataTypes_2eproto_;
#endif

// ===================================================================

#ifndef _MSC_VER
const int Quaternion::kWFieldNumber;
const int Quaternion::kXFieldNumber;
const int Quaternion::kYFieldNumber;
const int Quaternion::kZFieldNumber;
#endif  // !_MSC_VER

Quaternion::Quaternion()
  : ::google::protobuf::MessageLite() {
  SharedCtor();
}

void Quaternion::InitAsDefaultInstance() {
}

Quaternion::Quaternion(const Quaternion& from)
  : ::google::protobuf::MessageLite() {
  SharedCtor();
  MergeFrom(from);
}

void Quaternion::SharedCtor() {
  _cached_size_ = 0;
  w_ = 0;
  x_ = 0;
  y_ = 0;
  z_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Quaternion::~Quaternion() {
  SharedDtor();
}

void Quaternion::SharedDtor() {
  #ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  if (this != &default_instance()) {
  #else
  if (this != default_instance_) {
  #endif
  }
}

void Quaternion::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const Quaternion& Quaternion::default_instance() {
#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  protobuf_AddDesc_DataTypes_2eproto();
#else
  if (default_instance_ == NULL) protobuf_AddDesc_DataTypes_2eproto();
#endif
  return *default_instance_;
}

Quaternion* Quaternion::default_instance_ = NULL;

Quaternion* Quaternion::New() const {
  return new Quaternion;
}

void Quaternion::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    w_ = 0;
    x_ = 0;
    y_ = 0;
    z_ = 0;
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

bool Quaternion::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional float w = 1;
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_FIXED32) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &w_)));
          set_has_w();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(21)) goto parse_x;
        break;
      }

      // optional float x = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_FIXED32) {
         parse_x:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &x_)));
          set_has_x();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(29)) goto parse_y;
        break;
      }

      // optional float y = 3;
      case 3: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_FIXED32) {
         parse_y:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &y_)));
          set_has_y();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(37)) goto parse_z;
        break;
      }

      // optional float z = 4;
      case 4: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_FIXED32) {
         parse_z:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &z_)));
          set_has_z();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectAtEnd()) return true;
        break;
      }

      default: {
      handle_uninterpreted:
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          return true;
        }
        DO_(::google::protobuf::internal::WireFormatLite::SkipField(input, tag));
        break;
      }
    }
  }
  return true;
#undef DO_
}

void Quaternion::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // optional float w = 1;
  if (has_w()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(1, this->w(), output);
  }

  // optional float x = 2;
  if (has_x()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->x(), output);
  }

  // optional float y = 3;
  if (has_y()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->y(), output);
  }

  // optional float z = 4;
  if (has_z()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(4, this->z(), output);
  }

}

int Quaternion::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // optional float w = 1;
    if (has_w()) {
      total_size += 1 + 4;
    }

    // optional float x = 2;
    if (has_x()) {
      total_size += 1 + 4;
    }

    // optional float y = 3;
    if (has_y()) {
      total_size += 1 + 4;
    }

    // optional float z = 4;
    if (has_z()) {
      total_size += 1 + 4;
    }

  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Quaternion::CheckTypeAndMergeFrom(
    const ::google::protobuf::MessageLite& from) {
  MergeFrom(*::google::protobuf::down_cast<const Quaternion*>(&from));
}

void Quaternion::MergeFrom(const Quaternion& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_w()) {
      set_w(from.w());
    }
    if (from.has_x()) {
      set_x(from.x());
    }
    if (from.has_y()) {
      set_y(from.y());
    }
    if (from.has_z()) {
      set_z(from.z());
    }
  }
}

void Quaternion::CopyFrom(const Quaternion& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Quaternion::IsInitialized() const {

  return true;
}

void Quaternion::Swap(Quaternion* other) {
  if (other != this) {
    std::swap(w_, other->w_);
    std::swap(x_, other->x_);
    std::swap(y_, other->y_);
    std::swap(z_, other->z_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::std::string Quaternion::GetTypeName() const {
  return "biorobotics.firmware.protos.Quaternion";
}


// ===================================================================

#ifndef _MSC_VER
const int Vector3::kXFieldNumber;
const int Vector3::kYFieldNumber;
const int Vector3::kZFieldNumber;
#endif  // !_MSC_VER

Vector3::Vector3()
  : ::google::protobuf::MessageLite() {
  SharedCtor();
}

void Vector3::InitAsDefaultInstance() {
}

Vector3::Vector3(const Vector3& from)
  : ::google::protobuf::MessageLite() {
  SharedCtor();
  MergeFrom(from);
}

void Vector3::SharedCtor() {
  _cached_size_ = 0;
  x_ = 0;
  y_ = 0;
  z_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Vector3::~Vector3() {
  SharedDtor();
}

void Vector3::SharedDtor() {
  #ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  if (this != &default_instance()) {
  #else
  if (this != default_instance_) {
  #endif
  }
}

void Vector3::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const Vector3& Vector3::default_instance() {
#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  protobuf_AddDesc_DataTypes_2eproto();
#else
  if (default_instance_ == NULL) protobuf_AddDesc_DataTypes_2eproto();
#endif
  return *default_instance_;
}

Vector3* Vector3::default_instance_ = NULL;

Vector3* Vector3::New() const {
  return new Vector3;
}

void Vector3::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    x_ = 0;
    y_ = 0;
    z_ = 0;
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

bool Vector3::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional float x = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_FIXED32) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &x_)));
          set_has_x();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(29)) goto parse_y;
        break;
      }

      // optional float y = 3;
      case 3: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_FIXED32) {
         parse_y:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &y_)));
          set_has_y();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(37)) goto parse_z;
        break;
      }

      // optional float z = 4;
      case 4: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_FIXED32) {
         parse_z:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &z_)));
          set_has_z();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectAtEnd()) return true;
        break;
      }

      default: {
      handle_uninterpreted:
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          return true;
        }
        DO_(::google::protobuf::internal::WireFormatLite::SkipField(input, tag));
        break;
      }
    }
  }
  return true;
#undef DO_
}

void Vector3::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // optional float x = 2;
  if (has_x()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->x(), output);
  }

  // optional float y = 3;
  if (has_y()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->y(), output);
  }

  // optional float z = 4;
  if (has_z()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(4, this->z(), output);
  }

}

int Vector3::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // optional float x = 2;
    if (has_x()) {
      total_size += 1 + 4;
    }

    // optional float y = 3;
    if (has_y()) {
      total_size += 1 + 4;
    }

    // optional float z = 4;
    if (has_z()) {
      total_size += 1 + 4;
    }

  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Vector3::CheckTypeAndMergeFrom(
    const ::google::protobuf::MessageLite& from) {
  MergeFrom(*::google::protobuf::down_cast<const Vector3*>(&from));
}

void Vector3::MergeFrom(const Vector3& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_x()) {
      set_x(from.x());
    }
    if (from.has_y()) {
      set_y(from.y());
    }
    if (from.has_z()) {
      set_z(from.z());
    }
  }
}

void Vector3::CopyFrom(const Vector3& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Vector3::IsInitialized() const {

  return true;
}

void Vector3::Swap(Vector3* other) {
  if (other != this) {
    std::swap(x_, other->x_);
    std::swap(y_, other->y_);
    std::swap(z_, other->z_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::std::string Vector3::GetTypeName() const {
  return "biorobotics.firmware.protos.Vector3";
}


// ===================================================================

#ifndef _MSC_VER
const int Color::kRFieldNumber;
const int Color::kGFieldNumber;
const int Color::kBFieldNumber;
#endif  // !_MSC_VER

Color::Color()
  : ::google::protobuf::MessageLite() {
  SharedCtor();
}

void Color::InitAsDefaultInstance() {
}

Color::Color(const Color& from)
  : ::google::protobuf::MessageLite() {
  SharedCtor();
  MergeFrom(from);
}

void Color::SharedCtor() {
  _cached_size_ = 0;
  r_ = 0u;
  g_ = 0u;
  b_ = 0u;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Color::~Color() {
  SharedDtor();
}

void Color::SharedDtor() {
  #ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  if (this != &default_instance()) {
  #else
  if (this != default_instance_) {
  #endif
  }
}

void Color::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const Color& Color::default_instance() {
#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  protobuf_AddDesc_DataTypes_2eproto();
#else
  if (default_instance_ == NULL) protobuf_AddDesc_DataTypes_2eproto();
#endif
  return *default_instance_;
}

Color* Color::default_instance_ = NULL;

Color* Color::New() const {
  return new Color;
}

void Color::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    r_ = 0u;
    g_ = 0u;
    b_ = 0u;
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

bool Color::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional uint32 r = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &r_)));
          set_has_r();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(24)) goto parse_g;
        break;
      }

      // optional uint32 g = 3;
      case 3: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
         parse_g:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &g_)));
          set_has_g();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(32)) goto parse_b;
        break;
      }

      // optional uint32 b = 4;
      case 4: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
         parse_b:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &b_)));
          set_has_b();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectAtEnd()) return true;
        break;
      }

      default: {
      handle_uninterpreted:
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          return true;
        }
        DO_(::google::protobuf::internal::WireFormatLite::SkipField(input, tag));
        break;
      }
    }
  }
  return true;
#undef DO_
}

void Color::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // optional uint32 r = 2;
  if (has_r()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->r(), output);
  }

  // optional uint32 g = 3;
  if (has_g()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(3, this->g(), output);
  }

  // optional uint32 b = 4;
  if (has_b()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(4, this->b(), output);
  }

}

int Color::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // optional uint32 r = 2;
    if (has_r()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->r());
    }

    // optional uint32 g = 3;
    if (has_g()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->g());
    }

    // optional uint32 b = 4;
    if (has_b()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->b());
    }

  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Color::CheckTypeAndMergeFrom(
    const ::google::protobuf::MessageLite& from) {
  MergeFrom(*::google::protobuf::down_cast<const Color*>(&from));
}

void Color::MergeFrom(const Color& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_r()) {
      set_r(from.r());
    }
    if (from.has_g()) {
      set_g(from.g());
    }
    if (from.has_b()) {
      set_b(from.b());
    }
  }
}

void Color::CopyFrom(const Color& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Color::IsInitialized() const {

  return true;
}

void Color::Swap(Color* other) {
  if (other != this) {
    std::swap(r_, other->r_);
    std::swap(g_, other->g_);
    std::swap(b_, other->b_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::std::string Color::GetTypeName() const {
  return "biorobotics.firmware.protos.Color";
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace protos
}  // namespace firmware
}  // namespace biorobotics

// @@protoc_insertion_point(global_scope)
