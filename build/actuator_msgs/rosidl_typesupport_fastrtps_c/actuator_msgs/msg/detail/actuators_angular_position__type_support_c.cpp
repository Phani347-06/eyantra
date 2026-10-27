// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from actuator_msgs:msg/ActuatorsAngularPosition.idl
// generated code does not contain a copyright notice
#include "actuator_msgs/msg/detail/actuators_angular_position__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "actuator_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "actuator_msgs/msg/detail/actuators_angular_position__struct.h"
#include "actuator_msgs/msg/detail/actuators_angular_position__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // position
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // position

// forward declare type support functions


using _ActuatorsAngularPosition__ros_msg_type = actuator_msgs__msg__ActuatorsAngularPosition;

static bool _ActuatorsAngularPosition__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ActuatorsAngularPosition__ros_msg_type * ros_message = static_cast<const _ActuatorsAngularPosition__ros_msg_type *>(untyped_ros_message);
  // Field name: position
  {
    size_t size = ros_message->position.size;
    auto array_ptr = ros_message->position.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _ActuatorsAngularPosition__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ActuatorsAngularPosition__ros_msg_type * ros_message = static_cast<_ActuatorsAngularPosition__ros_msg_type *>(untyped_ros_message);
  // Field name: position
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->position.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->position);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->position, size)) {
      fprintf(stderr, "failed to create array for field 'position'");
      return false;
    }
    auto array_ptr = ros_message->position.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_actuator_msgs
size_t get_serialized_size_actuator_msgs__msg__ActuatorsAngularPosition(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ActuatorsAngularPosition__ros_msg_type * ros_message = static_cast<const _ActuatorsAngularPosition__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name position
  {
    size_t array_size = ros_message->position.size;
    auto array_ptr = ros_message->position.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ActuatorsAngularPosition__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_actuator_msgs__msg__ActuatorsAngularPosition(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_actuator_msgs
size_t max_serialized_size_actuator_msgs__msg__ActuatorsAngularPosition(
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

  // member: position
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = actuator_msgs__msg__ActuatorsAngularPosition;
    is_plain =
      (
      offsetof(DataType, position) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ActuatorsAngularPosition__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_actuator_msgs__msg__ActuatorsAngularPosition(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ActuatorsAngularPosition = {
  "actuator_msgs::msg",
  "ActuatorsAngularPosition",
  _ActuatorsAngularPosition__cdr_serialize,
  _ActuatorsAngularPosition__cdr_deserialize,
  _ActuatorsAngularPosition__get_serialized_size,
  _ActuatorsAngularPosition__max_serialized_size
};

static rosidl_message_type_support_t _ActuatorsAngularPosition__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ActuatorsAngularPosition,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, actuator_msgs, msg, ActuatorsAngularPosition)() {
  return &_ActuatorsAngularPosition__type_support;
}

#if defined(__cplusplus)
}
#endif
