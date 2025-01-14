// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mini_pupper_interfaces:srv/DanceCommand.idl
// generated code does not contain a copyright notice

#ifndef MINI_PUPPER_INTERFACES__SRV__DETAIL__DANCE_COMMAND__STRUCT_H_
#define MINI_PUPPER_INTERFACES__SRV__DETAIL__DANCE_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/DanceCommand in the package mini_pupper_interfaces.
typedef struct mini_pupper_interfaces__srv__DanceCommand_Request
{
  rosidl_runtime_c__String data;
} mini_pupper_interfaces__srv__DanceCommand_Request;

// Struct for a sequence of mini_pupper_interfaces__srv__DanceCommand_Request.
typedef struct mini_pupper_interfaces__srv__DanceCommand_Request__Sequence
{
  mini_pupper_interfaces__srv__DanceCommand_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mini_pupper_interfaces__srv__DanceCommand_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/DanceCommand in the package mini_pupper_interfaces.
typedef struct mini_pupper_interfaces__srv__DanceCommand_Response
{
  bool executed;
} mini_pupper_interfaces__srv__DanceCommand_Response;

// Struct for a sequence of mini_pupper_interfaces__srv__DanceCommand_Response.
typedef struct mini_pupper_interfaces__srv__DanceCommand_Response__Sequence
{
  mini_pupper_interfaces__srv__DanceCommand_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mini_pupper_interfaces__srv__DanceCommand_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MINI_PUPPER_INTERFACES__SRV__DETAIL__DANCE_COMMAND__STRUCT_H_
