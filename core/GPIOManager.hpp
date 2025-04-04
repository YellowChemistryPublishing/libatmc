/* module; */
#pragma once

#include <atomic>
#include <bit>
#include <coroutine>
#include <cxxutil.h>
#include <runtime_headers.h>

#include <Config.h>
#include <SpinLock.h>

#define __gpio_declare_pin(port, pin) static const GPIOPin P##port##pin
#define __gpio_declare_port_pins(port) \
__gpio_declare_pin(port, 0); \
__gpio_declare_pin(port, 1); \
__gpio_declare_pin(port, 2); \
__gpio_declare_pin(port, 3); \
__gpio_declare_pin(port, 4); \
__gpio_declare_pin(port, 5); \
__gpio_declare_pin(port, 6); \
__gpio_declare_pin(port, 7); \
__gpio_declare_pin(port, 8); \
__gpio_declare_pin(port, 9); \
__gpio_declare_pin(port, 10); \
__gpio_declare_pin(port, 11); \
__gpio_declare_pin(port, 12); \
__gpio_declare_pin(port, 13); \
__gpio_declare_pin(port, 14); \
__gpio_declare_pin(port, 15)
#define __gpio_declare_ports() \
__gpio_declare_port_pins(A); \
__gpio_declare_port_pins(B); \
__gpio_declare_port_pins(C); \
__gpio_declare_port_pins(D); \
__gpio_declare_port_pins(E); \
__gpio_declare_port_pins(F); \
__gpio_declare_port_pins(G); \
__gpio_declare_port_pins(H); \
__gpio_declare_port_pins(I); \
__gpio_declare_port_pins(J); \
__gpio_declare_port_pins(K)
#define __gpio_define_pin(port, pin) constexpr GPIOPin GPIOPin::P##port##pin(GPIO##port##_BASE, GPIO_PIN_##pin)
#define __gpio_define_port_pins(port) \
__gpio_define_pin(port, 0); \
__gpio_define_pin(port, 1); \
__gpio_define_pin(port, 2); \
__gpio_define_pin(port, 3); \
__gpio_define_pin(port, 4); \
__gpio_define_pin(port, 5); \
__gpio_define_pin(port, 6); \
__gpio_define_pin(port, 7); \
__gpio_define_pin(port, 8); \
__gpio_define_pin(port, 9); \
__gpio_define_pin(port, 10); \
__gpio_define_pin(port, 11); \
__gpio_define_pin(port, 12); \
__gpio_define_pin(port, 13); \
__gpio_define_pin(port, 14); \
__gpio_define_pin(port, 15)
#define __gpio_define_ports() \
__gpio_define_port_pins(A); \
__gpio_define_port_pins(B); \
__gpio_define_port_pins(C); \
__gpio_define_port_pins(D); \
__gpio_define_port_pins(E); \
__gpio_define_port_pins(F); \
__gpio_define_port_pins(G); \
__gpio_define_port_pins(H); \
__gpio_define_port_pins(I); \
__gpio_define_port_pins(J); \
__gpio_define_port_pins(K)

/* export module core.IO.Embedded; */

/* import core.Concurrency; */
#include <Task.hpp>

/* export */ namespace atmc
{
    class GPIOManager;

    /// @brief Any GPIO pin.
    /// @note Pass `byval`.
    struct GPIOPin
    {
        __gpio_declare_ports();

        /// @brief The state of a GPIO pin.
        using State = int;
        /// @brief Set a pin high.
        constexpr static State High = GPIO_PIN_SET;
        /// @brief Set a pin low.
        constexpr static State Low = GPIO_PIN_RESET;

        /// @brief Construct a GPIO pin.
        /// @param portAddr The address to the internal handle of the GPIO port.
        /// @param pin The pin number.
        constexpr GPIOPin(uintptr_t portAddr, uint16_t pin) : port(portAddr), pin(pin)
        { }

        friend class atmc::GPIOManager;
    private:
        uintptr_t port;
        uint16_t pin;
    };

    __gpio_define_ports();

    /// @brief GPIO manager.
    /// @note Static class.
    class GPIOManager final
    {
    public:
        static std::atomic_flag pinFlag[atmc::Config::PinCountGPIO];

        GPIOManager() = delete;

        /// @brief Await a pin interrupt, as preconfigured.
        /// @param pin The pin number to await.
        inline static Task<> pinInterrupt(uint16_t pin)
        {
            int pinIndex = std::bit_width(pin) - 1;
            GPIOManager::pinFlag[pinIndex].test_and_set();
            while (GPIOManager::pinFlag[pinIndex].test())
                co_await Task<>::yield();
            co_return;
        }

        /// @brief Read a pin.
        /// @param pin The pin to read.
        /// @return The state of the pin.
        inline static GPIOPin::State readPin(GPIOPin pin)
        {
            return HAL_GPIO_ReadPin((GPIO_TypeDef*)pin.port, pin.pin);
        }
        /// @brief Set a pin.
        /// @param pin The pin to set.
        /// @param state The state to set the pin to.
        inline static void writePin(GPIOPin pin, GPIOPin::State state)
        {
            HAL_GPIO_WritePin((GPIO_TypeDef*)pin.port, pin.pin, (GPIO_PinState)state);
        }
    };
}
