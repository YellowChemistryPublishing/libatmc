#pragma once

#include <runtime_headers.h>

#ifdef STM32H7xx
#define __dma_rw __attribute__((section(".dma_data")))
#else
#define __dma_rw
#endif

#ifdef __cplusplus
namespace atmc
{
	/// @brief The status of a hardware operation.
	enum class HardwareStatus : uint_least8_t
	{
		Ok = HAL_OK,
		Error = HAL_ERROR,
		Busy = HAL_BUSY,
		Timeout = HAL_TIMEOUT
	};
}
#endif

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
