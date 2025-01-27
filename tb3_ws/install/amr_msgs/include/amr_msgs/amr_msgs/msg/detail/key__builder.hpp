// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from amr_msgs:msg/Key.idl
// generated code does not contain a copyright notice

#ifndef AMR_MSGS__MSG__DETAIL__KEY__BUILDER_HPP_
#define AMR_MSGS__MSG__DETAIL__KEY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "amr_msgs/msg/detail/key__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace amr_msgs
{

namespace msg
{

namespace builder
{

class Init_Key_key
{
public:
  Init_Key_key()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::amr_msgs::msg::Key key(::amr_msgs::msg::Key::_key_type arg)
  {
    msg_.key = std::move(arg);
    return std::move(msg_);
  }

private:
  ::amr_msgs::msg::Key msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::amr_msgs::msg::Key>()
{
  return amr_msgs::msg::builder::Init_Key_key();
}

}  // namespace amr_msgs

#endif  // AMR_MSGS__MSG__DETAIL__KEY__BUILDER_HPP_
