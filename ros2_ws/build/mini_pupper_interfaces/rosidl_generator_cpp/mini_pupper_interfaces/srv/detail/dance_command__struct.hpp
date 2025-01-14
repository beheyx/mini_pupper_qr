// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mini_pupper_interfaces:srv/DanceCommand.idl
// generated code does not contain a copyright notice

#ifndef MINI_PUPPER_INTERFACES__SRV__DETAIL__DANCE_COMMAND__STRUCT_HPP_
#define MINI_PUPPER_INTERFACES__SRV__DETAIL__DANCE_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mini_pupper_interfaces__srv__DanceCommand_Request __attribute__((deprecated))
#else
# define DEPRECATED__mini_pupper_interfaces__srv__DanceCommand_Request __declspec(deprecated)
#endif

namespace mini_pupper_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DanceCommand_Request_
{
  using Type = DanceCommand_Request_<ContainerAllocator>;

  explicit DanceCommand_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  explicit DanceCommand_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  // field types and members
  using _data_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mini_pupper_interfaces::srv::DanceCommand_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mini_pupper_interfaces::srv::DanceCommand_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mini_pupper_interfaces::srv::DanceCommand_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mini_pupper_interfaces::srv::DanceCommand_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mini_pupper_interfaces::srv::DanceCommand_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mini_pupper_interfaces::srv::DanceCommand_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mini_pupper_interfaces::srv::DanceCommand_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mini_pupper_interfaces::srv::DanceCommand_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mini_pupper_interfaces::srv::DanceCommand_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mini_pupper_interfaces::srv::DanceCommand_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mini_pupper_interfaces__srv__DanceCommand_Request
    std::shared_ptr<mini_pupper_interfaces::srv::DanceCommand_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mini_pupper_interfaces__srv__DanceCommand_Request
    std::shared_ptr<mini_pupper_interfaces::srv::DanceCommand_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DanceCommand_Request_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const DanceCommand_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DanceCommand_Request_

// alias to use template instance with default allocator
using DanceCommand_Request =
  mini_pupper_interfaces::srv::DanceCommand_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mini_pupper_interfaces


#ifndef _WIN32
# define DEPRECATED__mini_pupper_interfaces__srv__DanceCommand_Response __attribute__((deprecated))
#else
# define DEPRECATED__mini_pupper_interfaces__srv__DanceCommand_Response __declspec(deprecated)
#endif

namespace mini_pupper_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DanceCommand_Response_
{
  using Type = DanceCommand_Response_<ContainerAllocator>;

  explicit DanceCommand_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->executed = false;
    }
  }

  explicit DanceCommand_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->executed = false;
    }
  }

  // field types and members
  using _executed_type =
    bool;
  _executed_type executed;

  // setters for named parameter idiom
  Type & set__executed(
    const bool & _arg)
  {
    this->executed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mini_pupper_interfaces::srv::DanceCommand_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mini_pupper_interfaces::srv::DanceCommand_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mini_pupper_interfaces::srv::DanceCommand_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mini_pupper_interfaces::srv::DanceCommand_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mini_pupper_interfaces::srv::DanceCommand_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mini_pupper_interfaces::srv::DanceCommand_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mini_pupper_interfaces::srv::DanceCommand_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mini_pupper_interfaces::srv::DanceCommand_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mini_pupper_interfaces::srv::DanceCommand_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mini_pupper_interfaces::srv::DanceCommand_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mini_pupper_interfaces__srv__DanceCommand_Response
    std::shared_ptr<mini_pupper_interfaces::srv::DanceCommand_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mini_pupper_interfaces__srv__DanceCommand_Response
    std::shared_ptr<mini_pupper_interfaces::srv::DanceCommand_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DanceCommand_Response_ & other) const
  {
    if (this->executed != other.executed) {
      return false;
    }
    return true;
  }
  bool operator!=(const DanceCommand_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DanceCommand_Response_

// alias to use template instance with default allocator
using DanceCommand_Response =
  mini_pupper_interfaces::srv::DanceCommand_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mini_pupper_interfaces

namespace mini_pupper_interfaces
{

namespace srv
{

struct DanceCommand
{
  using Request = mini_pupper_interfaces::srv::DanceCommand_Request;
  using Response = mini_pupper_interfaces::srv::DanceCommand_Response;
};

}  // namespace srv

}  // namespace mini_pupper_interfaces

#endif  // MINI_PUPPER_INTERFACES__SRV__DETAIL__DANCE_COMMAND__STRUCT_HPP_
