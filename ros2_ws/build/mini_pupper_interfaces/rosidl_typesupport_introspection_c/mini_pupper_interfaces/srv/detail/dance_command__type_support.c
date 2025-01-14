// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mini_pupper_interfaces:srv/DanceCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mini_pupper_interfaces/srv/detail/dance_command__rosidl_typesupport_introspection_c.h"
#include "mini_pupper_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mini_pupper_interfaces/srv/detail/dance_command__functions.h"
#include "mini_pupper_interfaces/srv/detail/dance_command__struct.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mini_pupper_interfaces__srv__DanceCommand_Request__rosidl_typesupport_introspection_c__DanceCommand_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mini_pupper_interfaces__srv__DanceCommand_Request__init(message_memory);
}

void mini_pupper_interfaces__srv__DanceCommand_Request__rosidl_typesupport_introspection_c__DanceCommand_Request_fini_function(void * message_memory)
{
  mini_pupper_interfaces__srv__DanceCommand_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mini_pupper_interfaces__srv__DanceCommand_Request__rosidl_typesupport_introspection_c__DanceCommand_Request_message_member_array[1] = {
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mini_pupper_interfaces__srv__DanceCommand_Request, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mini_pupper_interfaces__srv__DanceCommand_Request__rosidl_typesupport_introspection_c__DanceCommand_Request_message_members = {
  "mini_pupper_interfaces__srv",  // message namespace
  "DanceCommand_Request",  // message name
  1,  // number of fields
  sizeof(mini_pupper_interfaces__srv__DanceCommand_Request),
  mini_pupper_interfaces__srv__DanceCommand_Request__rosidl_typesupport_introspection_c__DanceCommand_Request_message_member_array,  // message members
  mini_pupper_interfaces__srv__DanceCommand_Request__rosidl_typesupport_introspection_c__DanceCommand_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  mini_pupper_interfaces__srv__DanceCommand_Request__rosidl_typesupport_introspection_c__DanceCommand_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mini_pupper_interfaces__srv__DanceCommand_Request__rosidl_typesupport_introspection_c__DanceCommand_Request_message_type_support_handle = {
  0,
  &mini_pupper_interfaces__srv__DanceCommand_Request__rosidl_typesupport_introspection_c__DanceCommand_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mini_pupper_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mini_pupper_interfaces, srv, DanceCommand_Request)() {
  if (!mini_pupper_interfaces__srv__DanceCommand_Request__rosidl_typesupport_introspection_c__DanceCommand_Request_message_type_support_handle.typesupport_identifier) {
    mini_pupper_interfaces__srv__DanceCommand_Request__rosidl_typesupport_introspection_c__DanceCommand_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mini_pupper_interfaces__srv__DanceCommand_Request__rosidl_typesupport_introspection_c__DanceCommand_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mini_pupper_interfaces/srv/detail/dance_command__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mini_pupper_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mini_pupper_interfaces/srv/detail/dance_command__functions.h"
// already included above
// #include "mini_pupper_interfaces/srv/detail/dance_command__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mini_pupper_interfaces__srv__DanceCommand_Response__rosidl_typesupport_introspection_c__DanceCommand_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mini_pupper_interfaces__srv__DanceCommand_Response__init(message_memory);
}

void mini_pupper_interfaces__srv__DanceCommand_Response__rosidl_typesupport_introspection_c__DanceCommand_Response_fini_function(void * message_memory)
{
  mini_pupper_interfaces__srv__DanceCommand_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mini_pupper_interfaces__srv__DanceCommand_Response__rosidl_typesupport_introspection_c__DanceCommand_Response_message_member_array[1] = {
  {
    "executed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mini_pupper_interfaces__srv__DanceCommand_Response, executed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mini_pupper_interfaces__srv__DanceCommand_Response__rosidl_typesupport_introspection_c__DanceCommand_Response_message_members = {
  "mini_pupper_interfaces__srv",  // message namespace
  "DanceCommand_Response",  // message name
  1,  // number of fields
  sizeof(mini_pupper_interfaces__srv__DanceCommand_Response),
  mini_pupper_interfaces__srv__DanceCommand_Response__rosidl_typesupport_introspection_c__DanceCommand_Response_message_member_array,  // message members
  mini_pupper_interfaces__srv__DanceCommand_Response__rosidl_typesupport_introspection_c__DanceCommand_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  mini_pupper_interfaces__srv__DanceCommand_Response__rosidl_typesupport_introspection_c__DanceCommand_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mini_pupper_interfaces__srv__DanceCommand_Response__rosidl_typesupport_introspection_c__DanceCommand_Response_message_type_support_handle = {
  0,
  &mini_pupper_interfaces__srv__DanceCommand_Response__rosidl_typesupport_introspection_c__DanceCommand_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mini_pupper_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mini_pupper_interfaces, srv, DanceCommand_Response)() {
  if (!mini_pupper_interfaces__srv__DanceCommand_Response__rosidl_typesupport_introspection_c__DanceCommand_Response_message_type_support_handle.typesupport_identifier) {
    mini_pupper_interfaces__srv__DanceCommand_Response__rosidl_typesupport_introspection_c__DanceCommand_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mini_pupper_interfaces__srv__DanceCommand_Response__rosidl_typesupport_introspection_c__DanceCommand_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mini_pupper_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mini_pupper_interfaces/srv/detail/dance_command__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mini_pupper_interfaces__srv__detail__dance_command__rosidl_typesupport_introspection_c__DanceCommand_service_members = {
  "mini_pupper_interfaces__srv",  // service namespace
  "DanceCommand",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // mini_pupper_interfaces__srv__detail__dance_command__rosidl_typesupport_introspection_c__DanceCommand_Request_message_type_support_handle,
  NULL  // response message
  // mini_pupper_interfaces__srv__detail__dance_command__rosidl_typesupport_introspection_c__DanceCommand_Response_message_type_support_handle
};

static rosidl_service_type_support_t mini_pupper_interfaces__srv__detail__dance_command__rosidl_typesupport_introspection_c__DanceCommand_service_type_support_handle = {
  0,
  &mini_pupper_interfaces__srv__detail__dance_command__rosidl_typesupport_introspection_c__DanceCommand_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mini_pupper_interfaces, srv, DanceCommand_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mini_pupper_interfaces, srv, DanceCommand_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mini_pupper_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mini_pupper_interfaces, srv, DanceCommand)() {
  if (!mini_pupper_interfaces__srv__detail__dance_command__rosidl_typesupport_introspection_c__DanceCommand_service_type_support_handle.typesupport_identifier) {
    mini_pupper_interfaces__srv__detail__dance_command__rosidl_typesupport_introspection_c__DanceCommand_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mini_pupper_interfaces__srv__detail__dance_command__rosidl_typesupport_introspection_c__DanceCommand_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mini_pupper_interfaces, srv, DanceCommand_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mini_pupper_interfaces, srv, DanceCommand_Response)()->data;
  }

  return &mini_pupper_interfaces__srv__detail__dance_command__rosidl_typesupport_introspection_c__DanceCommand_service_type_support_handle;
}
