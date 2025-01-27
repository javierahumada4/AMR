// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from amr_msgs:msg/Key.idl
// generated code does not contain a copyright notice

#ifndef AMR_MSGS__MSG__DETAIL__KEY__STRUCT_HPP_
#define AMR_MSGS__MSG__DETAIL__KEY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__amr_msgs__msg__Key __attribute__((deprecated))
#else
# define DEPRECATED__amr_msgs__msg__Key __declspec(deprecated)
#endif

namespace amr_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Key_
{
  using Type = Key_<ContainerAllocator>;

  explicit Key_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->key = "";
    }
  }

  explicit Key_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : key(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->key = "";
    }
  }

  // field types and members
  using _key_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _key_type key;

  // setters for named parameter idiom
  Type & set__key(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->key = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    amr_msgs::msg::Key_<ContainerAllocator> *;
  using ConstRawPtr =
    const amr_msgs::msg::Key_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<amr_msgs::msg::Key_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<amr_msgs::msg::Key_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      amr_msgs::msg::Key_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<amr_msgs::msg::Key_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      amr_msgs::msg::Key_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<amr_msgs::msg::Key_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<amr_msgs::msg::Key_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<amr_msgs::msg::Key_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__amr_msgs__msg__Key
    std::shared_ptr<amr_msgs::msg::Key_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__amr_msgs__msg__Key
    std::shared_ptr<amr_msgs::msg::Key_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Key_ & other) const
  {
    if (this->key != other.key) {
      return false;
    }
    return true;
  }
  bool operator!=(const Key_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Key_

// alias to use template instance with default allocator
using Key =
  amr_msgs::msg::Key_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace amr_msgs

#endif  // AMR_MSGS__MSG__DETAIL__KEY__STRUCT_HPP_
