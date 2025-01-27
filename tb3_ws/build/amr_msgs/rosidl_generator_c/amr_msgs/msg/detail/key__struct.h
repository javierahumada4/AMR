// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from amr_msgs:msg/Key.idl
// generated code does not contain a copyright notice

#ifndef AMR_MSGS__MSG__DETAIL__KEY__STRUCT_H_
#define AMR_MSGS__MSG__DETAIL__KEY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'key'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Key in the package amr_msgs.
typedef struct amr_msgs__msg__Key
{
  rosidl_runtime_c__String key;
} amr_msgs__msg__Key;

// Struct for a sequence of amr_msgs__msg__Key.
typedef struct amr_msgs__msg__Key__Sequence
{
  amr_msgs__msg__Key * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} amr_msgs__msg__Key__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AMR_MSGS__MSG__DETAIL__KEY__STRUCT_H_
