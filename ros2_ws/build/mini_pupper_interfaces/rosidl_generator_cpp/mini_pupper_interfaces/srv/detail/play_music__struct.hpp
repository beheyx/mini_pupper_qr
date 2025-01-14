// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mini_pupper_interfaces:srv/PlayMusic.idl
// generated code does not contain a copyright notice

#ifndef MINI_PUPPER_INTERFACES__SRV__DETAIL__PLAY_MUSIC__STRUCT_HPP_
#define MINI_PUPPER_INTERFACES__SRV__DETAIL__PLAY_MUSIC__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mini_pupper_interfaces__srv__PlayMusic_Request __attribute__((deprecated))
#else
# define DEPRECATED__mini_pupper_interfaces__srv__PlayMusic_Request __declspec(deprecated)
#endif

namespace mini_pupper_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlayMusic_Request_
{
  using Type = PlayMusic_Request_<ContainerAllocator>;

  explicit PlayMusic_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->file_name = "";
      this->start_second = 0.0f;
      this->duration = 0.0f;
    }
  }

  explicit PlayMusic_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : file_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->file_name = "";
      this->start_second = 0.0f;
      this->duration = 0.0f;
    }
  }

  // field types and members
  using _file_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _file_name_type file_name;
  using _start_second_type =
    float;
  _start_second_type start_second;
  using _duration_type =
    float;
  _duration_type duration;

  // setters for named parameter idiom
  Type & set__file_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->file_name = _arg;
    return *this;
  }
  Type & set__start_second(
    const float & _arg)
  {
    this->start_second = _arg;
    return *this;
  }
  Type & set__duration(
    const float & _arg)
  {
    this->duration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mini_pupper_interfaces::srv::PlayMusic_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mini_pupper_interfaces::srv::PlayMusic_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mini_pupper_interfaces::srv::PlayMusic_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mini_pupper_interfaces::srv::PlayMusic_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mini_pupper_interfaces::srv::PlayMusic_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mini_pupper_interfaces::srv::PlayMusic_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mini_pupper_interfaces::srv::PlayMusic_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mini_pupper_interfaces::srv::PlayMusic_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mini_pupper_interfaces::srv::PlayMusic_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mini_pupper_interfaces::srv::PlayMusic_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mini_pupper_interfaces__srv__PlayMusic_Request
    std::shared_ptr<mini_pupper_interfaces::srv::PlayMusic_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mini_pupper_interfaces__srv__PlayMusic_Request
    std::shared_ptr<mini_pupper_interfaces::srv::PlayMusic_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlayMusic_Request_ & other) const
  {
    if (this->file_name != other.file_name) {
      return false;
    }
    if (this->start_second != other.start_second) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlayMusic_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlayMusic_Request_

// alias to use template instance with default allocator
using PlayMusic_Request =
  mini_pupper_interfaces::srv::PlayMusic_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mini_pupper_interfaces


#ifndef _WIN32
# define DEPRECATED__mini_pupper_interfaces__srv__PlayMusic_Response __attribute__((deprecated))
#else
# define DEPRECATED__mini_pupper_interfaces__srv__PlayMusic_Response __declspec(deprecated)
#endif

namespace mini_pupper_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlayMusic_Response_
{
  using Type = PlayMusic_Response_<ContainerAllocator>;

  explicit PlayMusic_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit PlayMusic_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mini_pupper_interfaces::srv::PlayMusic_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mini_pupper_interfaces::srv::PlayMusic_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mini_pupper_interfaces::srv::PlayMusic_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mini_pupper_interfaces::srv::PlayMusic_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mini_pupper_interfaces::srv::PlayMusic_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mini_pupper_interfaces::srv::PlayMusic_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mini_pupper_interfaces::srv::PlayMusic_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mini_pupper_interfaces::srv::PlayMusic_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mini_pupper_interfaces::srv::PlayMusic_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mini_pupper_interfaces::srv::PlayMusic_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mini_pupper_interfaces__srv__PlayMusic_Response
    std::shared_ptr<mini_pupper_interfaces::srv::PlayMusic_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mini_pupper_interfaces__srv__PlayMusic_Response
    std::shared_ptr<mini_pupper_interfaces::srv::PlayMusic_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlayMusic_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlayMusic_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlayMusic_Response_

// alias to use template instance with default allocator
using PlayMusic_Response =
  mini_pupper_interfaces::srv::PlayMusic_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mini_pupper_interfaces

namespace mini_pupper_interfaces
{

namespace srv
{

struct PlayMusic
{
  using Request = mini_pupper_interfaces::srv::PlayMusic_Request;
  using Response = mini_pupper_interfaces::srv::PlayMusic_Response;
};

}  // namespace srv

}  // namespace mini_pupper_interfaces

#endif  // MINI_PUPPER_INTERFACES__SRV__DETAIL__PLAY_MUSIC__STRUCT_HPP_
