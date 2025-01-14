// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mini_pupper_interfaces:srv/PlayMusic.idl
// generated code does not contain a copyright notice

#ifndef MINI_PUPPER_INTERFACES__SRV__DETAIL__PLAY_MUSIC__TRAITS_HPP_
#define MINI_PUPPER_INTERFACES__SRV__DETAIL__PLAY_MUSIC__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mini_pupper_interfaces/srv/detail/play_music__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mini_pupper_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlayMusic_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: file_name
  {
    out << "file_name: ";
    rosidl_generator_traits::value_to_yaml(msg.file_name, out);
    out << ", ";
  }

  // member: start_second
  {
    out << "start_second: ";
    rosidl_generator_traits::value_to_yaml(msg.start_second, out);
    out << ", ";
  }

  // member: duration
  {
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlayMusic_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: file_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "file_name: ";
    rosidl_generator_traits::value_to_yaml(msg.file_name, out);
    out << "\n";
  }

  // member: start_second
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_second: ";
    rosidl_generator_traits::value_to_yaml(msg.start_second, out);
    out << "\n";
  }

  // member: duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlayMusic_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mini_pupper_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mini_pupper_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mini_pupper_interfaces::srv::PlayMusic_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mini_pupper_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mini_pupper_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const mini_pupper_interfaces::srv::PlayMusic_Request & msg)
{
  return mini_pupper_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mini_pupper_interfaces::srv::PlayMusic_Request>()
{
  return "mini_pupper_interfaces::srv::PlayMusic_Request";
}

template<>
inline const char * name<mini_pupper_interfaces::srv::PlayMusic_Request>()
{
  return "mini_pupper_interfaces/srv/PlayMusic_Request";
}

template<>
struct has_fixed_size<mini_pupper_interfaces::srv::PlayMusic_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mini_pupper_interfaces::srv::PlayMusic_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mini_pupper_interfaces::srv::PlayMusic_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mini_pupper_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlayMusic_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlayMusic_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlayMusic_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mini_pupper_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mini_pupper_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mini_pupper_interfaces::srv::PlayMusic_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mini_pupper_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mini_pupper_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const mini_pupper_interfaces::srv::PlayMusic_Response & msg)
{
  return mini_pupper_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mini_pupper_interfaces::srv::PlayMusic_Response>()
{
  return "mini_pupper_interfaces::srv::PlayMusic_Response";
}

template<>
inline const char * name<mini_pupper_interfaces::srv::PlayMusic_Response>()
{
  return "mini_pupper_interfaces/srv/PlayMusic_Response";
}

template<>
struct has_fixed_size<mini_pupper_interfaces::srv::PlayMusic_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mini_pupper_interfaces::srv::PlayMusic_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mini_pupper_interfaces::srv::PlayMusic_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mini_pupper_interfaces::srv::PlayMusic>()
{
  return "mini_pupper_interfaces::srv::PlayMusic";
}

template<>
inline const char * name<mini_pupper_interfaces::srv::PlayMusic>()
{
  return "mini_pupper_interfaces/srv/PlayMusic";
}

template<>
struct has_fixed_size<mini_pupper_interfaces::srv::PlayMusic>
  : std::integral_constant<
    bool,
    has_fixed_size<mini_pupper_interfaces::srv::PlayMusic_Request>::value &&
    has_fixed_size<mini_pupper_interfaces::srv::PlayMusic_Response>::value
  >
{
};

template<>
struct has_bounded_size<mini_pupper_interfaces::srv::PlayMusic>
  : std::integral_constant<
    bool,
    has_bounded_size<mini_pupper_interfaces::srv::PlayMusic_Request>::value &&
    has_bounded_size<mini_pupper_interfaces::srv::PlayMusic_Response>::value
  >
{
};

template<>
struct is_service<mini_pupper_interfaces::srv::PlayMusic>
  : std::true_type
{
};

template<>
struct is_service_request<mini_pupper_interfaces::srv::PlayMusic_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mini_pupper_interfaces::srv::PlayMusic_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MINI_PUPPER_INTERFACES__SRV__DETAIL__PLAY_MUSIC__TRAITS_HPP_
