// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mini_pupper_interfaces:srv/DanceCommand.idl
// generated code does not contain a copyright notice

#ifndef MINI_PUPPER_INTERFACES__SRV__DETAIL__DANCE_COMMAND__FUNCTIONS_H_
#define MINI_PUPPER_INTERFACES__SRV__DETAIL__DANCE_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mini_pupper_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "mini_pupper_interfaces/srv/detail/dance_command__struct.h"

/// Initialize srv/DanceCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mini_pupper_interfaces__srv__DanceCommand_Request
 * )) before or use
 * mini_pupper_interfaces__srv__DanceCommand_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
bool
mini_pupper_interfaces__srv__DanceCommand_Request__init(mini_pupper_interfaces__srv__DanceCommand_Request * msg);

/// Finalize srv/DanceCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
void
mini_pupper_interfaces__srv__DanceCommand_Request__fini(mini_pupper_interfaces__srv__DanceCommand_Request * msg);

/// Create srv/DanceCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mini_pupper_interfaces__srv__DanceCommand_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
mini_pupper_interfaces__srv__DanceCommand_Request *
mini_pupper_interfaces__srv__DanceCommand_Request__create();

/// Destroy srv/DanceCommand message.
/**
 * It calls
 * mini_pupper_interfaces__srv__DanceCommand_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
void
mini_pupper_interfaces__srv__DanceCommand_Request__destroy(mini_pupper_interfaces__srv__DanceCommand_Request * msg);

/// Check for srv/DanceCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
bool
mini_pupper_interfaces__srv__DanceCommand_Request__are_equal(const mini_pupper_interfaces__srv__DanceCommand_Request * lhs, const mini_pupper_interfaces__srv__DanceCommand_Request * rhs);

/// Copy a srv/DanceCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
bool
mini_pupper_interfaces__srv__DanceCommand_Request__copy(
  const mini_pupper_interfaces__srv__DanceCommand_Request * input,
  mini_pupper_interfaces__srv__DanceCommand_Request * output);

/// Initialize array of srv/DanceCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * mini_pupper_interfaces__srv__DanceCommand_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
bool
mini_pupper_interfaces__srv__DanceCommand_Request__Sequence__init(mini_pupper_interfaces__srv__DanceCommand_Request__Sequence * array, size_t size);

/// Finalize array of srv/DanceCommand messages.
/**
 * It calls
 * mini_pupper_interfaces__srv__DanceCommand_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
void
mini_pupper_interfaces__srv__DanceCommand_Request__Sequence__fini(mini_pupper_interfaces__srv__DanceCommand_Request__Sequence * array);

/// Create array of srv/DanceCommand messages.
/**
 * It allocates the memory for the array and calls
 * mini_pupper_interfaces__srv__DanceCommand_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
mini_pupper_interfaces__srv__DanceCommand_Request__Sequence *
mini_pupper_interfaces__srv__DanceCommand_Request__Sequence__create(size_t size);

/// Destroy array of srv/DanceCommand messages.
/**
 * It calls
 * mini_pupper_interfaces__srv__DanceCommand_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
void
mini_pupper_interfaces__srv__DanceCommand_Request__Sequence__destroy(mini_pupper_interfaces__srv__DanceCommand_Request__Sequence * array);

/// Check for srv/DanceCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
bool
mini_pupper_interfaces__srv__DanceCommand_Request__Sequence__are_equal(const mini_pupper_interfaces__srv__DanceCommand_Request__Sequence * lhs, const mini_pupper_interfaces__srv__DanceCommand_Request__Sequence * rhs);

/// Copy an array of srv/DanceCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
bool
mini_pupper_interfaces__srv__DanceCommand_Request__Sequence__copy(
  const mini_pupper_interfaces__srv__DanceCommand_Request__Sequence * input,
  mini_pupper_interfaces__srv__DanceCommand_Request__Sequence * output);

/// Initialize srv/DanceCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mini_pupper_interfaces__srv__DanceCommand_Response
 * )) before or use
 * mini_pupper_interfaces__srv__DanceCommand_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
bool
mini_pupper_interfaces__srv__DanceCommand_Response__init(mini_pupper_interfaces__srv__DanceCommand_Response * msg);

/// Finalize srv/DanceCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
void
mini_pupper_interfaces__srv__DanceCommand_Response__fini(mini_pupper_interfaces__srv__DanceCommand_Response * msg);

/// Create srv/DanceCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mini_pupper_interfaces__srv__DanceCommand_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
mini_pupper_interfaces__srv__DanceCommand_Response *
mini_pupper_interfaces__srv__DanceCommand_Response__create();

/// Destroy srv/DanceCommand message.
/**
 * It calls
 * mini_pupper_interfaces__srv__DanceCommand_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
void
mini_pupper_interfaces__srv__DanceCommand_Response__destroy(mini_pupper_interfaces__srv__DanceCommand_Response * msg);

/// Check for srv/DanceCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
bool
mini_pupper_interfaces__srv__DanceCommand_Response__are_equal(const mini_pupper_interfaces__srv__DanceCommand_Response * lhs, const mini_pupper_interfaces__srv__DanceCommand_Response * rhs);

/// Copy a srv/DanceCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
bool
mini_pupper_interfaces__srv__DanceCommand_Response__copy(
  const mini_pupper_interfaces__srv__DanceCommand_Response * input,
  mini_pupper_interfaces__srv__DanceCommand_Response * output);

/// Initialize array of srv/DanceCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * mini_pupper_interfaces__srv__DanceCommand_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
bool
mini_pupper_interfaces__srv__DanceCommand_Response__Sequence__init(mini_pupper_interfaces__srv__DanceCommand_Response__Sequence * array, size_t size);

/// Finalize array of srv/DanceCommand messages.
/**
 * It calls
 * mini_pupper_interfaces__srv__DanceCommand_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
void
mini_pupper_interfaces__srv__DanceCommand_Response__Sequence__fini(mini_pupper_interfaces__srv__DanceCommand_Response__Sequence * array);

/// Create array of srv/DanceCommand messages.
/**
 * It allocates the memory for the array and calls
 * mini_pupper_interfaces__srv__DanceCommand_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
mini_pupper_interfaces__srv__DanceCommand_Response__Sequence *
mini_pupper_interfaces__srv__DanceCommand_Response__Sequence__create(size_t size);

/// Destroy array of srv/DanceCommand messages.
/**
 * It calls
 * mini_pupper_interfaces__srv__DanceCommand_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
void
mini_pupper_interfaces__srv__DanceCommand_Response__Sequence__destroy(mini_pupper_interfaces__srv__DanceCommand_Response__Sequence * array);

/// Check for srv/DanceCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
bool
mini_pupper_interfaces__srv__DanceCommand_Response__Sequence__are_equal(const mini_pupper_interfaces__srv__DanceCommand_Response__Sequence * lhs, const mini_pupper_interfaces__srv__DanceCommand_Response__Sequence * rhs);

/// Copy an array of srv/DanceCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mini_pupper_interfaces
bool
mini_pupper_interfaces__srv__DanceCommand_Response__Sequence__copy(
  const mini_pupper_interfaces__srv__DanceCommand_Response__Sequence * input,
  mini_pupper_interfaces__srv__DanceCommand_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MINI_PUPPER_INTERFACES__SRV__DETAIL__DANCE_COMMAND__FUNCTIONS_H_
