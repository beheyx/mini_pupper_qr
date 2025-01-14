// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mini_pupper_interfaces:srv/PlayMusic.idl
// generated code does not contain a copyright notice

#ifndef MINI_PUPPER_INTERFACES__SRV__DETAIL__PLAY_MUSIC__BUILDER_HPP_
#define MINI_PUPPER_INTERFACES__SRV__DETAIL__PLAY_MUSIC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mini_pupper_interfaces/srv/detail/play_music__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mini_pupper_interfaces
{

namespace srv
{

namespace builder
{

class Init_PlayMusic_Request_duration
{
public:
  explicit Init_PlayMusic_Request_duration(::mini_pupper_interfaces::srv::PlayMusic_Request & msg)
  : msg_(msg)
  {}
  ::mini_pupper_interfaces::srv::PlayMusic_Request duration(::mini_pupper_interfaces::srv::PlayMusic_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mini_pupper_interfaces::srv::PlayMusic_Request msg_;
};

class Init_PlayMusic_Request_start_second
{
public:
  explicit Init_PlayMusic_Request_start_second(::mini_pupper_interfaces::srv::PlayMusic_Request & msg)
  : msg_(msg)
  {}
  Init_PlayMusic_Request_duration start_second(::mini_pupper_interfaces::srv::PlayMusic_Request::_start_second_type arg)
  {
    msg_.start_second = std::move(arg);
    return Init_PlayMusic_Request_duration(msg_);
  }

private:
  ::mini_pupper_interfaces::srv::PlayMusic_Request msg_;
};

class Init_PlayMusic_Request_file_name
{
public:
  Init_PlayMusic_Request_file_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlayMusic_Request_start_second file_name(::mini_pupper_interfaces::srv::PlayMusic_Request::_file_name_type arg)
  {
    msg_.file_name = std::move(arg);
    return Init_PlayMusic_Request_start_second(msg_);
  }

private:
  ::mini_pupper_interfaces::srv::PlayMusic_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mini_pupper_interfaces::srv::PlayMusic_Request>()
{
  return mini_pupper_interfaces::srv::builder::Init_PlayMusic_Request_file_name();
}

}  // namespace mini_pupper_interfaces


namespace mini_pupper_interfaces
{

namespace srv
{

namespace builder
{

class Init_PlayMusic_Response_message
{
public:
  explicit Init_PlayMusic_Response_message(::mini_pupper_interfaces::srv::PlayMusic_Response & msg)
  : msg_(msg)
  {}
  ::mini_pupper_interfaces::srv::PlayMusic_Response message(::mini_pupper_interfaces::srv::PlayMusic_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mini_pupper_interfaces::srv::PlayMusic_Response msg_;
};

class Init_PlayMusic_Response_success
{
public:
  Init_PlayMusic_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlayMusic_Response_message success(::mini_pupper_interfaces::srv::PlayMusic_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PlayMusic_Response_message(msg_);
  }

private:
  ::mini_pupper_interfaces::srv::PlayMusic_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mini_pupper_interfaces::srv::PlayMusic_Response>()
{
  return mini_pupper_interfaces::srv::builder::Init_PlayMusic_Response_success();
}

}  // namespace mini_pupper_interfaces

#endif  // MINI_PUPPER_INTERFACES__SRV__DETAIL__PLAY_MUSIC__BUILDER_HPP_
