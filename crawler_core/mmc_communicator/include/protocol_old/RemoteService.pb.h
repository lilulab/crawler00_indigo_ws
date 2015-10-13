// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: RemoteService.proto

#ifndef PROTOBUF_RemoteService_2eproto__INCLUDED
#define PROTOBUF_RemoteService_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message_lite.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
// @@protoc_insertion_point(includes)

namespace protos {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_RemoteService_2eproto();
void protobuf_AssignDesc_RemoteService_2eproto();
void protobuf_ShutdownFile_RemoteService_2eproto();

class RemoteService;
class CalibrateEncoders;
class CalibrateCurrent;

// ===================================================================

class RemoteService : public ::google::protobuf::MessageLite {
 public:
  RemoteService();
  virtual ~RemoteService();

  RemoteService(const RemoteService& from);

  inline RemoteService& operator=(const RemoteService& from) {
    CopyFrom(from);
    return *this;
  }

  static const RemoteService& default_instance();

  #ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  // Returns the internal default instance pointer. This function can
  // return NULL thus should not be used by the user. This is intended
  // for Protobuf internal code. Please use default_instance() declared
  // above instead.
  static inline const RemoteService* internal_default_instance() {
    return default_instance_;
  }
  #endif

  void Swap(RemoteService* other);

  // implements Message ----------------------------------------------

  RemoteService* New() const;
  void CheckTypeAndMergeFrom(const ::google::protobuf::MessageLite& from);
  void CopyFrom(const RemoteService& from);
  void MergeFrom(const RemoteService& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::std::string GetTypeName() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional .protos.CalibrateEncoders calibrateEncoders = 10;
  inline bool has_calibrateencoders() const;
  inline void clear_calibrateencoders();
  static const int kCalibrateEncodersFieldNumber = 10;
  inline const ::protos::CalibrateEncoders& calibrateencoders() const;
  inline ::protos::CalibrateEncoders* mutable_calibrateencoders();
  inline ::protos::CalibrateEncoders* release_calibrateencoders();
  inline void set_allocated_calibrateencoders(::protos::CalibrateEncoders* calibrateencoders);

  // optional .protos.CalibrateCurrent calibrateCurrent = 11;
  inline bool has_calibratecurrent() const;
  inline void clear_calibratecurrent();
  static const int kCalibrateCurrentFieldNumber = 11;
  inline const ::protos::CalibrateCurrent& calibratecurrent() const;
  inline ::protos::CalibrateCurrent* mutable_calibratecurrent();
  inline ::protos::CalibrateCurrent* release_calibratecurrent();
  inline void set_allocated_calibratecurrent(::protos::CalibrateCurrent* calibratecurrent);

  // @@protoc_insertion_point(class_scope:protos.RemoteService)
 private:
  inline void set_has_calibrateencoders();
  inline void clear_has_calibrateencoders();
  inline void set_has_calibratecurrent();
  inline void clear_has_calibratecurrent();

  ::protos::CalibrateEncoders* calibrateencoders_;
  ::protos::CalibrateCurrent* calibratecurrent_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(2 + 31) / 32];

  #ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  friend void  protobuf_AddDesc_RemoteService_2eproto_impl();
  #else
  friend void  protobuf_AddDesc_RemoteService_2eproto();
  #endif
  friend void protobuf_AssignDesc_RemoteService_2eproto();
  friend void protobuf_ShutdownFile_RemoteService_2eproto();

  void InitAsDefaultInstance();
  static RemoteService* default_instance_;
};
// -------------------------------------------------------------------

class CalibrateEncoders : public ::google::protobuf::MessageLite {
 public:
  CalibrateEncoders();
  virtual ~CalibrateEncoders();

  CalibrateEncoders(const CalibrateEncoders& from);

  inline CalibrateEncoders& operator=(const CalibrateEncoders& from) {
    CopyFrom(from);
    return *this;
  }

  static const CalibrateEncoders& default_instance();

  #ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  // Returns the internal default instance pointer. This function can
  // return NULL thus should not be used by the user. This is intended
  // for Protobuf internal code. Please use default_instance() declared
  // above instead.
  static inline const CalibrateEncoders* internal_default_instance() {
    return default_instance_;
  }
  #endif

  void Swap(CalibrateEncoders* other);

  // implements Message ----------------------------------------------

  CalibrateEncoders* New() const;
  void CheckTypeAndMergeFrom(const ::google::protobuf::MessageLite& from);
  void CopyFrom(const CalibrateEncoders& from);
  void MergeFrom(const CalibrateEncoders& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::std::string GetTypeName() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // @@protoc_insertion_point(class_scope:protos.CalibrateEncoders)
 private:


  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[1];

  #ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  friend void  protobuf_AddDesc_RemoteService_2eproto_impl();
  #else
  friend void  protobuf_AddDesc_RemoteService_2eproto();
  #endif
  friend void protobuf_AssignDesc_RemoteService_2eproto();
  friend void protobuf_ShutdownFile_RemoteService_2eproto();

  void InitAsDefaultInstance();
  static CalibrateEncoders* default_instance_;
};
// -------------------------------------------------------------------

class CalibrateCurrent : public ::google::protobuf::MessageLite {
 public:
  CalibrateCurrent();
  virtual ~CalibrateCurrent();

  CalibrateCurrent(const CalibrateCurrent& from);

  inline CalibrateCurrent& operator=(const CalibrateCurrent& from) {
    CopyFrom(from);
    return *this;
  }

  static const CalibrateCurrent& default_instance();

  #ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  // Returns the internal default instance pointer. This function can
  // return NULL thus should not be used by the user. This is intended
  // for Protobuf internal code. Please use default_instance() declared
  // above instead.
  static inline const CalibrateCurrent* internal_default_instance() {
    return default_instance_;
  }
  #endif

  void Swap(CalibrateCurrent* other);

  // implements Message ----------------------------------------------

  CalibrateCurrent* New() const;
  void CheckTypeAndMergeFrom(const ::google::protobuf::MessageLite& from);
  void CopyFrom(const CalibrateCurrent& from);
  void MergeFrom(const CalibrateCurrent& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::std::string GetTypeName() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // @@protoc_insertion_point(class_scope:protos.CalibrateCurrent)
 private:


  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[1];

  #ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  friend void  protobuf_AddDesc_RemoteService_2eproto_impl();
  #else
  friend void  protobuf_AddDesc_RemoteService_2eproto();
  #endif
  friend void protobuf_AssignDesc_RemoteService_2eproto();
  friend void protobuf_ShutdownFile_RemoteService_2eproto();

  void InitAsDefaultInstance();
  static CalibrateCurrent* default_instance_;
};
// ===================================================================


// ===================================================================

// RemoteService

// optional .protos.CalibrateEncoders calibrateEncoders = 10;
inline bool RemoteService::has_calibrateencoders() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void RemoteService::set_has_calibrateencoders() {
  _has_bits_[0] |= 0x00000001u;
}
inline void RemoteService::clear_has_calibrateencoders() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void RemoteService::clear_calibrateencoders() {
  if (calibrateencoders_ != NULL) calibrateencoders_->::protos::CalibrateEncoders::Clear();
  clear_has_calibrateencoders();
}
inline const ::protos::CalibrateEncoders& RemoteService::calibrateencoders() const {
#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  return calibrateencoders_ != NULL ? *calibrateencoders_ : *default_instance().calibrateencoders_;
#else
  return calibrateencoders_ != NULL ? *calibrateencoders_ : *default_instance_->calibrateencoders_;
#endif
}
inline ::protos::CalibrateEncoders* RemoteService::mutable_calibrateencoders() {
  set_has_calibrateencoders();
  if (calibrateencoders_ == NULL) calibrateencoders_ = new ::protos::CalibrateEncoders;
  return calibrateencoders_;
}
inline ::protos::CalibrateEncoders* RemoteService::release_calibrateencoders() {
  clear_has_calibrateencoders();
  ::protos::CalibrateEncoders* temp = calibrateencoders_;
  calibrateencoders_ = NULL;
  return temp;
}
inline void RemoteService::set_allocated_calibrateencoders(::protos::CalibrateEncoders* calibrateencoders) {
  delete calibrateencoders_;
  calibrateencoders_ = calibrateencoders;
  if (calibrateencoders) {
    set_has_calibrateencoders();
  } else {
    clear_has_calibrateencoders();
  }
}

// optional .protos.CalibrateCurrent calibrateCurrent = 11;
inline bool RemoteService::has_calibratecurrent() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void RemoteService::set_has_calibratecurrent() {
  _has_bits_[0] |= 0x00000002u;
}
inline void RemoteService::clear_has_calibratecurrent() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void RemoteService::clear_calibratecurrent() {
  if (calibratecurrent_ != NULL) calibratecurrent_->::protos::CalibrateCurrent::Clear();
  clear_has_calibratecurrent();
}
inline const ::protos::CalibrateCurrent& RemoteService::calibratecurrent() const {
#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  return calibratecurrent_ != NULL ? *calibratecurrent_ : *default_instance().calibratecurrent_;
#else
  return calibratecurrent_ != NULL ? *calibratecurrent_ : *default_instance_->calibratecurrent_;
#endif
}
inline ::protos::CalibrateCurrent* RemoteService::mutable_calibratecurrent() {
  set_has_calibratecurrent();
  if (calibratecurrent_ == NULL) calibratecurrent_ = new ::protos::CalibrateCurrent;
  return calibratecurrent_;
}
inline ::protos::CalibrateCurrent* RemoteService::release_calibratecurrent() {
  clear_has_calibratecurrent();
  ::protos::CalibrateCurrent* temp = calibratecurrent_;
  calibratecurrent_ = NULL;
  return temp;
}
inline void RemoteService::set_allocated_calibratecurrent(::protos::CalibrateCurrent* calibratecurrent) {
  delete calibratecurrent_;
  calibratecurrent_ = calibratecurrent;
  if (calibratecurrent) {
    set_has_calibratecurrent();
  } else {
    clear_has_calibratecurrent();
  }
}

// -------------------------------------------------------------------

// CalibrateEncoders

// -------------------------------------------------------------------

// CalibrateCurrent


// @@protoc_insertion_point(namespace_scope)

}  // namespace protos

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_RemoteService_2eproto__INCLUDED