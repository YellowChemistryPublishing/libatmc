#pragma once

#include <atomic>
#include <bit>
#include <board.h>
#include <cxxutil.h>
#include <entry.h>
#include <runtime_headers.h>

#include <Config.h>
#include <Exception.h>
#include <InplaceAtomicSet.h>
#include <InplaceVector.h>
#include <Result.h>
#include <Result.hpp>
#include <SpinLock.h>

/* export module core.IO.Embedded; */

/* import core.Concurrency; */
#include <Task.h>

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

namespace atmc
{
    using ADCChannel = int;

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

    struct AnalogPin final
    {
        constexpr AnalogPin(int adc, int rank) : adcIndex(adc), rank(rank)
        { }

        constexpr bool operator==(const AnalogPin& other) const = default;

        friend class atmc::GPIOManager;
    private:
        int adcIndex;
        int rank;
        
        inline ADC_HandleTypeDef* internalHandle()
        {
            switch (this->adcIndex)
            {
            case 0:
                return &hadc1;
            case 1:
                return &hadc2;
            case 2:
                return &hadc3;
            default:
                return nullptr;
            }
        }
    };

    struct PWMPin final
    {
        constexpr PWMPin(int timer, int channel) : timerIndex(timer), channel([&]() -> int
        {
            switch (channel)
            {
            case 0:
                return TIM_CHANNEL_1;
            case 1:
                return TIM_CHANNEL_2;
            case 2:
                return TIM_CHANNEL_3;
            case 3:
                return TIM_CHANNEL_4;
            case 4:
                return TIM_CHANNEL_5;
            case 5:
                return TIM_CHANNEL_6;
            default:
                return -1;
            }
        }())
        { }

        friend class atmc::GPIOManager;
    private:
        int timerIndex;
        int channel;

        inline TIM_HandleTypeDef* internalHandle()
        {
            switch (this->timerIndex)
            {
            case 0:
                return &htim1;
            case 1:
                return &htim2;
            case 2:
                return &htim3;
            case 3:
                return &htim4;
            case 4:
                return &htim5;
            case 5:
                return &htim6;
            case 6:
                return &htim7;
            case 7:
                return &htim8;
            case 11:
                return &htim12;
            case 12:
                return &htim13;
            case 13:
                return &htim14;
            case 14:
                return &htim15;
            case 15:
                return &htim16;
            case 16:
                return &htim17;
            default:
                return nullptr;
            }
        }
    };

    /// @brief GPIO manager.
    /// @note Static class.
    class GPIOManager final
    {
        static std::atomic_flag pinFlag[Config::PinCountGPIO];

        static std::atomic_flag adcFlags[Config::AnalogConverterCount];
        static __dma_rw volatile uint16_t adcRaw[Config::AnalogConverterCount][Config::MaxADCChannels];
    public:
        GPIOManager() = delete;
        
        /// @brief Await a pin interrupt, as preconfigured.
        /// @param pin The pin number to await.
        inline static sys::Task<> pinInterrupt(uint16_t pin)
        {
            int pinIndex = std::bit_width(pin) - 1;
            GPIOManager::pinFlag[pinIndex].test_and_set();
            while (GPIOManager::pinFlag[pinIndex].test())
                co_await sys::Task<>::yield();
            co_return;
        }

        /// @brief Read a pin.
        /// @param pin The pin to read.
        /// @return The state of the pin.
        inline static GPIOPin::State digitalRead(GPIOPin pin)
        {
            return HAL_GPIO_ReadPin(__reic(GPIO_TypeDef*, pin.port), pin.pin);
        }
        inline static sys::Task<sys::Result<float, HardwareStatus>> analogRead(AnalogPin pin)
        {
            ADC_HandleTypeDef* hadc = pin.internalHandle();
            __fence_value_co_return(HardwareStatus::Error, !hadc);

            int resolution;
            switch (ADC_GET_RESOLUTION(hadc))
            {
            case ADC_RESOLUTION_8B:
                resolution = 8;
                break;
            case ADC_RESOLUTION_10B:
                resolution = 10;
                break;
            case ADC_RESOLUTION_12B:
            case ADC_RESOLUTION_12B_OPT:
                resolution = 12;
                break;
            case ADC_RESOLUTION_14B:
            case ADC_RESOLUTION_14B_OPT:
                resolution = 14;
                break;
            case ADC_RESOLUTION_16B:
                resolution = 16;
                break;
            default:
                co_return HardwareStatus::Error;
            }

            if (!GPIOManager::adcFlags[pin.adcIndex].test_and_set())
            {
                HardwareStatus res = HardwareStatus(HAL_ADC_Start_DMA(hadc, __reic(uint32_t*, __cstc(uint16_t*, GPIOManager::adcRaw[pin.adcIndex])), hadc->Init.NbrOfConversion));
                if (res != HardwareStatus::Ok)
                {
                    GPIOManager::adcFlags[pin.adcIndex].clear();
                    co_return res;
                }
                // `HAL_ADC_Stop_DMA(hadc)` called in ISR.
            }

            co_await sys::Task<>::waitUntil([&]
            {
                return !GPIOManager::adcFlags[pin.adcIndex].test();
            });

            float maxVal = (1u << resolution) - 1;
            co_return float(GPIOManager::adcRaw[pin.adcIndex][pin.rank]) / maxVal;
        }
        
        /// @brief Set a pin.
        /// @param pin The pin to set.
        /// @param state The state to set the pin to.
        inline static void digitalWrite(GPIOPin pin, GPIOPin::State state)
        {
            HAL_GPIO_WritePin(__reic(GPIO_TypeDef*, pin.port), pin.pin, GPIO_PinState(state));
        }

        inline static HardwareStatus pwmWrite(PWMPin pin, float duty)
        {
            __fence_value_return(HardwareStatus::Error, pin.channel == -1 || duty < 0.0f || duty > 1.0f);

            TIM_HandleTypeDef* htim = pin.internalHandle();
            __fence_value_return(HardwareStatus::Error, !htim);

            htim->Instance->CCR1 = uint32_t(htim->Instance->ARR * duty + 0.5f);

            return HardwareStatus(HAL_TIM_PWM_Start(htim, uint32_t(pin.channel)));
        }
        inline static HardwareStatus pwmClear(PWMPin pin)
        {
            __fence_value_return(HardwareStatus::Error, pin.channel == -1);

            TIM_HandleTypeDef* htim = pin.internalHandle();
            __fence_value_return(HardwareStatus::Error, !htim);

            return HardwareStatus(HAL_TIM_PWM_Stop(htim, uint32_t(pin.channel)));
        }

        friend void ::HAL_GPIO_EXTI_Callback(uint16_t pin);
        friend void ::HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
    };
}
