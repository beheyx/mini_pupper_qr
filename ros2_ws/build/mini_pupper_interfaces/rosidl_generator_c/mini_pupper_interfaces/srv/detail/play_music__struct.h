// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mini_pupper_interfaces:srv/PlayMusic.idl
// generated code does not contain a copyright notice

#ifndef MINI_PUPPER_INTERFACES__SRV__DETAIL__PLAY_MUSIC__STRUCT_H_
#define MINI_PUPPER_INTERFACES__SRV__DETAIL__PLAY_MUSIC__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'file_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PlayMusic in the package mini_pupper_interfaces.
typedef struct mini_pupper_interfaces__srv__PlayMusic_Request
{
  /// The name of the song file
  rosidl_runtime_c__String file_name;
  /// Optional offset (in seconds) to start loading the audio file
  float start_second;
  /// Optional duration to be played
  float duration;
} mini_pupper_interfaces__srv__PlayMusic_Request;

// Struct for a sequence of mini_pupper_interfaces__srv__PlayMusic_Request.
typedef struct mini_pupper_interfaces__srv__PlayMusic_Request__Sequence
{
  mini_pupper_interfaces__srv__PlayMusic_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mini_pupper_interfaces__srv__PlayMusic_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PlayMusic in the package mini_pupper_interfaces.
typedef struct mini_pupper_interfaces__srv__PlayMusic_Response
{
  /// Indicates whether the command was executed successfully
  bool success;
  /// Additional information or error message
  rosidl_runtime_c__String message;
} mini_pupper_interfaces__srv__PlayMusic_Response;

// Struct for a sequence of mini_pupper_interfaces__srv__PlayMusic_Response.
typedef struct mini_pupper_interfaces__srv__PlayMusic_Response__Sequence
{
  mini_pupper_interfaces__srv__PlayMusic_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mini_pupper_interfaces__srv__PlayMusic_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MINI_PUPPER_INTERFACES__SRV__DETAIL__PLAY_MUSIC__STRUCT_H_
