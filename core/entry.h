#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/// @brief For you to implement.
void init();

/// @brief Exposed to the runtime, do not call.
void __attribute__((visibility("default"))) __initHandler();

#ifdef __cplusplus
}
#endif
