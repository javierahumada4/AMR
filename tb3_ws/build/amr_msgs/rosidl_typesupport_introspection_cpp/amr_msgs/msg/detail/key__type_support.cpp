// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from amr_msgs:msg/Key.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "amr_msgs/msg/detail/key__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace amr_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Key_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) amr_msgs::msg::Key(_init);
}

void Key_fini_function(void * message_memory)
{
  auto typed_message = static_cast<amr_msgs::msg::Key *>(message_memory);
  typed_message->~Key();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Key_message_member_array[1] = {
  {
    "key",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(amr_msgs::msg::Key, key),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Key_message_members = {
  "amr_msgs::msg",  // message namespace
  "Key",  // message name
  1,  // number of fields
  sizeof(amr_msgs::msg::Key),
  Key_message_member_array,  // message members
  Key_init_function,  // function to initialize message memory (memory has to be allocated)
  Key_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Key_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Key_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace amr_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<amr_msgs::msg::Key>()
{
  return &::amr_msgs::msg::rosidl_typesupport_introspection_cpp::Key_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, amr_msgs, msg, Key)() {
  return &::amr_msgs::msg::rosidl_typesupport_introspection_cpp::Key_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
