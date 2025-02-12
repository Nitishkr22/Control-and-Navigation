// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: pdk_SensorMsgHeader.proto

#include "pdk_SensorMsgHeader.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace protobuf_pdk_5fMsgHeader_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_pdk_5fMsgHeader_2eproto ::google::protobuf::internal::SCCInfo<1> scc_info_MsgHeader;
}  // namespace protobuf_pdk_5fMsgHeader_2eproto
namespace pb {
namespace PDK {
class SensorMsgHeaderDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<SensorMsgHeader>
      _instance;
} _SensorMsgHeader_default_instance_;
}  // namespace PDK
}  // namespace pb
namespace protobuf_pdk_5fSensorMsgHeader_2eproto {
static void InitDefaultsSensorMsgHeader() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::PDK::_SensorMsgHeader_default_instance_;
    new (ptr) ::pb::PDK::SensorMsgHeader();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::PDK::SensorMsgHeader::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_SensorMsgHeader =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsSensorMsgHeader}, {
      &protobuf_pdk_5fMsgHeader_2eproto::scc_info_MsgHeader.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_SensorMsgHeader.base);
}

::google::protobuf::Metadata file_level_metadata[1];
const ::google::protobuf::EnumDescriptor* file_level_enum_descriptors[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::pb::PDK::SensorMsgHeader, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::pb::PDK::SensorMsgHeader, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::pb::PDK::SensorMsgHeader, t_commonheader_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::pb::PDK::SensorMsgHeader, u_sensorid_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::pb::PDK::SensorMsgHeader, e_signalstatus_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::pb::PDK::SensorMsgHeader, e_sensortype_),
  0,
  1,
  2,
  3,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::pb::PDK::SensorMsgHeader)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::pb::PDK::_SensorMsgHeader_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "pdk_SensorMsgHeader.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, file_level_enum_descriptors, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\031pdk_SensorMsgHeader.proto\022\006pb.PDK\032\023pdk"
      "_MsgHeader.proto\"\250\003\n\017SensorMsgHeader\022)\n\016"
      "t_CommonHeader\030\001 \001(\0132\021.pb.PDK.MsgHeader\022"
      "\022\n\nu_SensorId\030\002 \001(\r\022Q\n\016e_SignalStatus\030\003 "
      "\001(\0162$.pb.PDK.SensorMsgHeader.SignalStatu"
      "s:\023EM_SIGSTATE_INVALID\022A\n\014e_SensorType\030\004"
      " \001(\0162\".pb.PDK.SensorMsgHeader.SensorType"
      ":\007UNKNOWN\"Q\n\014SignalStatus\022\024\n\020EM_SIGSTATE"
      "_INIT\020\000\022\022\n\016EM_SIGSTATE_OK\020\001\022\027\n\023EM_SIGSTA"
      "TE_INVALID\020\002\"m\n\nSensorType\022\014\n\010ARS430EO\020\000"
      "\022\014\n\010ARS430DI\020\001\022\014\n\010SRR520CO\020\003\022\016\n\nARS540DE"
      "MO\020\004\022\014\n\010SRR520DI\020\005\022\n\n\006ARS548\020\006\022\013\n\007UNKNOW"
      "N\020c"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 483);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "pdk_SensorMsgHeader.proto", &protobuf_RegisterTypes);
  ::protobuf_pdk_5fMsgHeader_2eproto::AddDescriptors();
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_pdk_5fSensorMsgHeader_2eproto
namespace pb {
namespace PDK {
const ::google::protobuf::EnumDescriptor* SensorMsgHeader_SignalStatus_descriptor() {
  protobuf_pdk_5fSensorMsgHeader_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_pdk_5fSensorMsgHeader_2eproto::file_level_enum_descriptors[0];
}
bool SensorMsgHeader_SignalStatus_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const SensorMsgHeader_SignalStatus SensorMsgHeader::EM_SIGSTATE_INIT;
const SensorMsgHeader_SignalStatus SensorMsgHeader::EM_SIGSTATE_OK;
const SensorMsgHeader_SignalStatus SensorMsgHeader::EM_SIGSTATE_INVALID;
const SensorMsgHeader_SignalStatus SensorMsgHeader::SignalStatus_MIN;
const SensorMsgHeader_SignalStatus SensorMsgHeader::SignalStatus_MAX;
const int SensorMsgHeader::SignalStatus_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900
const ::google::protobuf::EnumDescriptor* SensorMsgHeader_SensorType_descriptor() {
  protobuf_pdk_5fSensorMsgHeader_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_pdk_5fSensorMsgHeader_2eproto::file_level_enum_descriptors[1];
}
bool SensorMsgHeader_SensorType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 3:
    case 4:
    case 5:
    case 6:
    case 99:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const SensorMsgHeader_SensorType SensorMsgHeader::ARS430EO;
const SensorMsgHeader_SensorType SensorMsgHeader::ARS430DI;
const SensorMsgHeader_SensorType SensorMsgHeader::SRR520CO;
const SensorMsgHeader_SensorType SensorMsgHeader::ARS540DEMO;
const SensorMsgHeader_SensorType SensorMsgHeader::SRR520DI;
const SensorMsgHeader_SensorType SensorMsgHeader::ARS548;
const SensorMsgHeader_SensorType SensorMsgHeader::UNKNOWN;
const SensorMsgHeader_SensorType SensorMsgHeader::SensorType_MIN;
const SensorMsgHeader_SensorType SensorMsgHeader::SensorType_MAX;
const int SensorMsgHeader::SensorType_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

// ===================================================================

void SensorMsgHeader::InitAsDefaultInstance() {
  ::pb::PDK::_SensorMsgHeader_default_instance_._instance.get_mutable()->t_commonheader_ = const_cast< ::pb::PDK::MsgHeader*>(
      ::pb::PDK::MsgHeader::internal_default_instance());
}
void SensorMsgHeader::clear_t_commonheader() {
  if (t_commonheader_ != NULL) t_commonheader_->Clear();
  clear_has_t_commonheader();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int SensorMsgHeader::kTCommonHeaderFieldNumber;
const int SensorMsgHeader::kUSensorIdFieldNumber;
const int SensorMsgHeader::kESignalStatusFieldNumber;
const int SensorMsgHeader::kESensorTypeFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

SensorMsgHeader::SensorMsgHeader()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_pdk_5fSensorMsgHeader_2eproto::scc_info_SensorMsgHeader.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.PDK.SensorMsgHeader)
}
SensorMsgHeader::SensorMsgHeader(const SensorMsgHeader& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_t_commonheader()) {
    t_commonheader_ = new ::pb::PDK::MsgHeader(*from.t_commonheader_);
  } else {
    t_commonheader_ = NULL;
  }
  ::memcpy(&u_sensorid_, &from.u_sensorid_,
    static_cast<size_t>(reinterpret_cast<char*>(&e_sensortype_) -
    reinterpret_cast<char*>(&u_sensorid_)) + sizeof(e_sensortype_));
  // @@protoc_insertion_point(copy_constructor:pb.PDK.SensorMsgHeader)
}

void SensorMsgHeader::SharedCtor() {
  ::memset(&t_commonheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&u_sensorid_) -
      reinterpret_cast<char*>(&t_commonheader_)) + sizeof(u_sensorid_));
  e_signalstatus_ = 2;
  e_sensortype_ = 99;
}

SensorMsgHeader::~SensorMsgHeader() {
  // @@protoc_insertion_point(destructor:pb.PDK.SensorMsgHeader)
  SharedDtor();
}

void SensorMsgHeader::SharedDtor() {
  if (this != internal_default_instance()) delete t_commonheader_;
}

void SensorMsgHeader::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* SensorMsgHeader::descriptor() {
  ::protobuf_pdk_5fSensorMsgHeader_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_pdk_5fSensorMsgHeader_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const SensorMsgHeader& SensorMsgHeader::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_pdk_5fSensorMsgHeader_2eproto::scc_info_SensorMsgHeader.base);
  return *internal_default_instance();
}


void SensorMsgHeader::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.PDK.SensorMsgHeader)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(t_commonheader_ != NULL);
    t_commonheader_->Clear();
  }
  if (cached_has_bits & 14u) {
    u_sensorid_ = 0u;
    e_signalstatus_ = 2;
    e_sensortype_ = 99;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool SensorMsgHeader::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:pb.PDK.SensorMsgHeader)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .pb.PDK.MsgHeader t_CommonHeader = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_t_commonheader()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional uint32 u_SensorId = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {
          set_has_u_sensorid();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &u_sensorid_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .pb.PDK.SensorMsgHeader.SignalStatus e_SignalStatus = 3 [default = EM_SIGSTATE_INVALID];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::pb::PDK::SensorMsgHeader_SignalStatus_IsValid(value)) {
            set_e_signalstatus(static_cast< ::pb::PDK::SensorMsgHeader_SignalStatus >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                3, static_cast< ::google::protobuf::uint64>(value));
          }
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .pb.PDK.SensorMsgHeader.SensorType e_SensorType = 4 [default = UNKNOWN];
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(32u /* 32 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::pb::PDK::SensorMsgHeader_SensorType_IsValid(value)) {
            set_e_sensortype(static_cast< ::pb::PDK::SensorMsgHeader_SensorType >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                4, static_cast< ::google::protobuf::uint64>(value));
          }
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:pb.PDK.SensorMsgHeader)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:pb.PDK.SensorMsgHeader)
  return false;
#undef DO_
}

void SensorMsgHeader::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:pb.PDK.SensorMsgHeader)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.PDK.MsgHeader t_CommonHeader = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->_internal_t_commonheader(), output);
  }

  // optional uint32 u_SensorId = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->u_sensorid(), output);
  }

  // optional .pb.PDK.SensorMsgHeader.SignalStatus e_SignalStatus = 3 [default = EM_SIGSTATE_INVALID];
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      3, this->e_signalstatus(), output);
  }

  // optional .pb.PDK.SensorMsgHeader.SensorType e_SensorType = 4 [default = UNKNOWN];
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      4, this->e_sensortype(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:pb.PDK.SensorMsgHeader)
}

::google::protobuf::uint8* SensorMsgHeader::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:pb.PDK.SensorMsgHeader)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.PDK.MsgHeader t_CommonHeader = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->_internal_t_commonheader(), deterministic, target);
  }

  // optional uint32 u_SensorId = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->u_sensorid(), target);
  }

  // optional .pb.PDK.SensorMsgHeader.SignalStatus e_SignalStatus = 3 [default = EM_SIGSTATE_INVALID];
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      3, this->e_signalstatus(), target);
  }

  // optional .pb.PDK.SensorMsgHeader.SensorType e_SensorType = 4 [default = UNKNOWN];
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      4, this->e_sensortype(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.PDK.SensorMsgHeader)
  return target;
}

size_t SensorMsgHeader::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.PDK.SensorMsgHeader)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 15u) {
    // optional .pb.PDK.MsgHeader t_CommonHeader = 1;
    if (has_t_commonheader()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *t_commonheader_);
    }

    // optional uint32 u_SensorId = 2;
    if (has_u_sensorid()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->u_sensorid());
    }

    // optional .pb.PDK.SensorMsgHeader.SignalStatus e_SignalStatus = 3 [default = EM_SIGSTATE_INVALID];
    if (has_e_signalstatus()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::EnumSize(this->e_signalstatus());
    }

    // optional .pb.PDK.SensorMsgHeader.SensorType e_SensorType = 4 [default = UNKNOWN];
    if (has_e_sensortype()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::EnumSize(this->e_sensortype());
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void SensorMsgHeader::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.PDK.SensorMsgHeader)
  GOOGLE_DCHECK_NE(&from, this);
  const SensorMsgHeader* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const SensorMsgHeader>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.PDK.SensorMsgHeader)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.PDK.SensorMsgHeader)
    MergeFrom(*source);
  }
}

void SensorMsgHeader::MergeFrom(const SensorMsgHeader& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.PDK.SensorMsgHeader)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 15u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_t_commonheader()->::pb::PDK::MsgHeader::MergeFrom(from.t_commonheader());
    }
    if (cached_has_bits & 0x00000002u) {
      u_sensorid_ = from.u_sensorid_;
    }
    if (cached_has_bits & 0x00000004u) {
      e_signalstatus_ = from.e_signalstatus_;
    }
    if (cached_has_bits & 0x00000008u) {
      e_sensortype_ = from.e_sensortype_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void SensorMsgHeader::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.PDK.SensorMsgHeader)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SensorMsgHeader::CopyFrom(const SensorMsgHeader& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.PDK.SensorMsgHeader)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SensorMsgHeader::IsInitialized() const {
  return true;
}

void SensorMsgHeader::Swap(SensorMsgHeader* other) {
  if (other == this) return;
  InternalSwap(other);
}
void SensorMsgHeader::InternalSwap(SensorMsgHeader* other) {
  using std::swap;
  swap(t_commonheader_, other->t_commonheader_);
  swap(u_sensorid_, other->u_sensorid_);
  swap(e_signalstatus_, other->e_signalstatus_);
  swap(e_sensortype_, other->e_sensortype_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata SensorMsgHeader::GetMetadata() const {
  protobuf_pdk_5fSensorMsgHeader_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_pdk_5fSensorMsgHeader_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace PDK
}  // namespace pb
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::pb::PDK::SensorMsgHeader* Arena::CreateMaybeMessage< ::pb::PDK::SensorMsgHeader >(Arena* arena) {
  return Arena::CreateInternal< ::pb::PDK::SensorMsgHeader >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
