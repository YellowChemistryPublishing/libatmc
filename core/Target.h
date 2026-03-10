#pragma once

/// @file

/// @defgroup hardware_targets Target Hardware Detection Constants
/// @{

#define _libatmc_target_stm32 0
#define _libatmc_target_hosted 0

/// @}

#if defined(STM32) && STM32
#undef _libatmc_target_stm32
#define _libatmc_target_stm32 1
#elif defined(HOSTED) && HOSTED
#undef _libatmc_target_hosted
#define _libatmc_target_hosted 1
#else
#error "Unsupported target hardware."
#endif
