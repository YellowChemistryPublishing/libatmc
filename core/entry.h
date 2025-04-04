#pragma once

#include <runtime_headers.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/// @brief For you to implement.
void init();
/// @brief For you to implement.
void tick();

/// @brief Exposed to the runtime, do not call.
void __attribute__((visibility("default"))) __initHandler();

#ifdef __cplusplus
}
#endif
