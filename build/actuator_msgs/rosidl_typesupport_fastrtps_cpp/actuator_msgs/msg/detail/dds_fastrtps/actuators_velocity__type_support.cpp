// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from actuator_msgs:msg/ActuatorsVelocity.idl
// generated code does not contain a copyright notice
#include "actuator_msgs/msg/detail/actuators_velocity__rosidl_typesupport_fastrtps_cpp.hpp"
#include "actuator_msgs/msg/detail/actuators_velocity__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace actuator_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const actuator_msgs::msg::ActuatorsAngularVelocity &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  actuator_msgs::msg::ActuatorsAngularVelocity &);
size_t get_serialized_size(
  const actuator_msgs::msg::ActuatorsAngularVelocity &,
  size_t current_alignment);
size_t
max_serialized_size_ActuatorsAngularVelocity(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace actuator_msgs

namespace actuator_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const actuator_msgs::msg::ActuatorsLinearVelocity &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  actuator_msgs::msg::ActuatorsLinearVelocity &);
size_t get_serialized_size(
  const actuator_msgs::msg::ActuatorsLinearVelocity &,
  size_t current_alignment);
size_t
max_serialized_size_ActuatorsLinearVelocity(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace actuator_msgs


namespace actuator_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_actuator_msgs
cdr_serialize(
  const actuator_msgs::msg::ActuatorsVelocity & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: angular
  actuator_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.angular,
    cdr);
  // Member: linear
  actuator_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.linear,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_actuator_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  actuator_msgs::msg::ActuatorsVelocity & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: angular
  actuator_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.angular);

  // Member: linear
  actuator_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.linear);

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_actuator_msgs
get_serialized_size(
  const actuator_msgs::msg::ActuatorsVelocity & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: angular

  current_alignment +=
    actuator_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.angular, current_alignment);
  // Member: linear

  current_alignment +=
    actuator_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.linear, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_actuator_msgs
max_serialized_size_ActuatorsVelocity(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: angular
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        actuator_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_ActuatorsAngularVelocity(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: linear
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        actuator_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_ActuatorsLinearVelocity(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = actuator_msgs::msg::ActuatorsVelocity;
    is_plain =
      (
      offsetof(DataType, linear) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ActuatorsVelocity__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const actuator_msgs::msg::ActuatorsVelocity *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ActuatorsVelocity__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<actuator_msgs::msg::ActuatorsVelocity *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ActuatorsVelocity__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const actuator_msgs::msg::ActuatorsVelocity *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ActuatorsVelocity__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ActuatorsVelocity(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ActuatorsVelocity__callbacks = {
  "actuator_msgs::msg",
  "ActuatorsVelocity",
  _ActuatorsVelocity__cdr_serialize,
  _ActuatorsVelocity__cdr_deserialize,
  _ActuatorsVelocity__get_serialized_size,
  _ActuatorsVelocity__max_serialized_size
};

static rosidl_message_type_support_t _ActuatorsVelocity__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ActuatorsVelocity__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace actuator_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_actuator_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<actuator_msgs::msg::ActuatorsVelocity>()
{
  return &actuator_msgs::msg::typesupport_fastrtps_cpp::_ActuatorsVelocity__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, actuator_msgs, msg, ActuatorsVelocity)() {
  return &actuator_msgs::msg::typesupport_fastrtps_cpp::_ActuatorsVelocity__handle;
}

#ifdef __cplusplus
}
#endif
