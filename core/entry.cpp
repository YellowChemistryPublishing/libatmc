#include <cxxutil.h>
#include <entry.h>
#include <exception>
#include <print>
#include <runtime_headers.h>
#include <FreeRTOS.h>

/* import core.Concurrency; */
#include <GPIOManager.hpp>
#include <Task.hpp>
/* import core.Fundamental; */
#include <cxxutil.hpp>
#include <Exception.hpp>

using namespace atmc;

__weak I2C_HandleTypeDef hi2c1;
__weak I2C_HandleTypeDef hi2c2;
__weak I2C_HandleTypeDef hi2c3;

__weak SPI_HandleTypeDef hspi1;
__weak SPI_HandleTypeDef hspi2;
__weak SPI_HandleTypeDef hspi3;
__weak SPI_HandleTypeDef hspi4;

__weak UART_HandleTypeDef huart1;
__weak UART_HandleTypeDef huart2;
__weak UART_HandleTypeDef huart3;

__weak ADC_HandleTypeDef hadc1;
__weak DMA_HandleTypeDef hdma_adc1;
__weak ADC_HandleTypeDef hadc2;
__weak DMA_HandleTypeDef hdma_adc2;
__weak ADC_HandleTypeDef hadc3;
__weak DMA_HandleTypeDef hdma_adc3;

void* operator new (size_t sz)
{
    void* ret = pvPortMalloc(sz);
    if (!ret)
        throw std::bad_alloc();
    else return ret;
}
void operator delete (void* p) noexcept
{
    vPortFree(p);
}
void operator delete (void* p, size_t) noexcept
{
    vPortFree(p);
}

extern "C" __weak void init()
{ }

// `try`-`catch` blocks are rarely used in embedded systems, and I've never gotten them to work properly.
// If they are, they should be used to catch exceptions thrown by the `init` and `tick` functions.
// Otherwise, sufficient error messages are printed to `stderr` anyways.

void __initHandler()
{
    __push_nowarn(__clWarn_use_after_free);
    try
    {
        init();

        vTaskStartScheduler();
    }
    catch (const std::exception& ex)
    {
        std::println(stderr, "Exception thrown during call to `extern \"C\" void init()`: {}", ex.what());
        __throw(TerminateException());
    }
    catch (...)
    {
        std::println(stderr, "Unmanaged exception of type `{}` thrown during call to `extern \"C\" void init()`.", exceptionTypeName(std::current_exception()).get());
        __throw(TerminateException());
    }
    __pop_nowarn();
}

extern "C" void xPortSysTickHandler();

#ifdef STM32
extern "C" void HardFault_Handler()
{
    std::println("Hardfault!");
    while (true);
}
extern "C" void SysTick_Handler()
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) [[likely]]
        xPortSysTickHandler();
}

#ifdef STM32H753xx
extern "C" int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif
#else
#error "Unsupported board!"
#endif
