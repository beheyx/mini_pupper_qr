// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mini_pupper_interfaces:srv/StopMusic.idl
// generated code does not contain a copyright notice

#ifndef MINI_PUPPER_INTERFACES__SRV__DETAIL__STOP_MUSIC__BUILDER_HPP_
#define MINI_PUPPER_INTERFACES__SRV__DETAIL__STOP_MUSIC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mini_pupper_interfaces/srv/detail/stop_music__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mini_pupper_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mini_pupper_interfaces::srv::StopMusic_Request>()
{
  return ::mini_pupper_interfaces::srv::StopMusic_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace mini_pupper_interfaces


namespace mini_pupper_interfaces
{

namespace srv
{

namespace builder
{

class Init_StopMusic_Response_message
{
public:
  explicit Init_StopMusic_Response_message(::mini_pupper_interfaces::srv::StopMusic_Response & msg)
  : msg_(msg)
  {}
  ::mini_pupper_interfaces::srv::StopMusic_Response message(::mini_pupper_interfaces::srv::StopMusic_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mini_pupper_interfaces::srv::StopMusic_Response msg_;
};

class Init_StopMusic_Response_success
{
public:
  Init_StopMusic_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StopMusic_Response_message success(::mini_pupper_interfaces::srv::StopMusic_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_StopMusic_Response_message(msg_);
  }

private:
  ::mini_pupper_interfaces::srv::StopMusic_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mini_pupper_interfaces::srv::StopMusic_Response>()
{
  return mini_pupper_interfaces::srv::builder::Init_StopMusic_Response_success();
}

}  // namespace mini_pupper_interfaces

#endif  // MINI_PUPPER_INTERFACES__SRV__DETAIL__STOP_MUSIC__BUILDER_HPP_
