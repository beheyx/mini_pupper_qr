// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mini_pupper_interfaces:srv/DanceCommand.idl
// generated code does not contain a copyright notice

#ifndef MINI_PUPPER_INTERFACES__SRV__DETAIL__DANCE_COMMAND__BUILDER_HPP_
#define MINI_PUPPER_INTERFACES__SRV__DETAIL__DANCE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mini_pupper_interfaces/srv/detail/dance_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mini_pupper_interfaces
{

namespace srv
{

namespace builder
{

class Init_DanceCommand_Request_data
{
public:
  Init_DanceCommand_Request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mini_pupper_interfaces::srv::DanceCommand_Request data(::mini_pupper_interfaces::srv::DanceCommand_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mini_pupper_interfaces::srv::DanceCommand_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mini_pupper_interfaces::srv::DanceCommand_Request>()
{
  return mini_pupper_interfaces::srv::builder::Init_DanceCommand_Request_data();
}

}  // namespace mini_pupper_interfaces


namespace mini_pupper_interfaces
{

namespace srv
{

namespace builder
{

class Init_DanceCommand_Response_executed
{
public:
  Init_DanceCommand_Response_executed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mini_pupper_interfaces::srv::DanceCommand_Response executed(::mini_pupper_interfaces::srv::DanceCommand_Response::_executed_type arg)
  {
    msg_.executed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mini_pupper_interfaces::srv::DanceCommand_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mini_pupper_interfaces::srv::DanceCommand_Response>()
{
  return mini_pupper_interfaces::srv::builder::Init_DanceCommand_Response_executed();
}

}  // namespace mini_pupper_interfaces

#endif  // MINI_PUPPER_INTERFACES__SRV__DETAIL__DANCE_COMMAND__BUILDER_HPP_
