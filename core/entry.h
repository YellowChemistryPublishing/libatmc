#pragma once

#include <module/sys>
#include <runtime_headers.h> // NOLINT(misc-include-cleaner)

#include <Target.h>

#if _libatmc_target_stm32 && defined(STM32H7xx)
#define _dma_rw __attribute__((section(".dma_data")))
#else
#define _dma_rw
#endif

#ifdef __cplusplus
namespace atmc
{
#if _libatmc_target_stm32
    enum class HardwareStatus : byte
    {
        Ok = HAL_OK,
        Error = HAL_ERROR,
        Busy = HAL_BUSY,
        Timeout = HAL_TIMEOUT
    };
#else
    /// @brief The status of a hardware operation.
    enum class HardwareStatus : byte
    {
        Ok = 0,
        Error = 1,
        Busy = 2,
        Timeout = 3
    };
#endif
} // namespace atmc
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /// @brief For you to implement.
    void init();

    /// @brief Exposed to the runtime, do not call.
    void __attribute__((visibility("default"))) __initHandler(); // NOLINT(bugprone-reserved-identifier, readability-identifier-naming)

#ifdef __cplusplus
}
#endif
