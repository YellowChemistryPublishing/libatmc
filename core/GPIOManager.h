#pragma once

/// @file

#include <atomic>
#include <bit>
#include <board.h> // NOLINT(misc-include-cleaner)
#include <cstdint>
#include <entry.h>
#include <runtime_headers.h> // NOLINT(misc-include-cleaner)

#include <module/sys>
#include <module/sys.Threading>

#include <Config.h>
#include <Target.h>

#if _libatmc_target_stm32
#define _internal_gpio_declare_pin(port, pin) static const GPIOPin P##port##pin
#define _internal_gpio_declare_port_pins(port) \
    _internal_gpio_declare_pin(port, 0);       \
    _internal_gpio_declare_pin(port, 1);       \
    _internal_gpio_declare_pin(port, 2);       \
    _internal_gpio_declare_pin(port, 3);       \
    _internal_gpio_declare_pin(port, 4);       \
    _internal_gpio_declare_pin(port, 5);       \
    _internal_gpio_declare_pin(port, 6);       \
    _internal_gpio_declare_pin(port, 7);       \
    _internal_gpio_declare_pin(port, 8);       \
    _internal_gpio_declare_pin(port, 9);       \
    _internal_gpio_declare_pin(port, 10);      \
    _internal_gpio_declare_pin(port, 11);      \
    _internal_gpio_declare_pin(port, 12);      \
    _internal_gpio_declare_pin(port, 13);      \
    _internal_gpio_declare_pin(port, 14);      \
    _internal_gpio_declare_pin(port, 15)
#define _internal_gpio_declare_ports()   \
    _internal_gpio_declare_port_pins(A); \
    _internal_gpio_declare_port_pins(B); \
    _internal_gpio_declare_port_pins(C); \
    _internal_gpio_declare_port_pins(D); \
    _internal_gpio_declare_port_pins(E); \
    _internal_gpio_declare_port_pins(F); \
    _internal_gpio_declare_port_pins(G); \
    _internal_gpio_declare_port_pins(H); \
    _internal_gpio_declare_port_pins(I); \
    _internal_gpio_declare_port_pins(J); \
    _internal_gpio_declare_port_pins(K)
#define _internal_gpio_define_pin(port, pin) constexpr GPIOPin GPIOPin::P##port##pin(GPIO##port##_BASE, u16(GPIO_PIN_##pin))
#define _internal_gpio_define_port_pins(port) \
    _internal_gpio_define_pin(port, 0);       \
    _internal_gpio_define_pin(port, 1);       \
    _internal_gpio_define_pin(port, 2);       \
    _internal_gpio_define_pin(port, 3);       \
    _internal_gpio_define_pin(port, 4);       \
    _internal_gpio_define_pin(port, 5);       \
    _internal_gpio_define_pin(port, 6);       \
    _internal_gpio_define_pin(port, 7);       \
    _internal_gpio_define_pin(port, 8);       \
    _internal_gpio_define_pin(port, 9);       \
    _internal_gpio_define_pin(port, 10);      \
    _internal_gpio_define_pin(port, 11);      \
    _internal_gpio_define_pin(port, 12);      \
    _internal_gpio_define_pin(port, 13);      \
    _internal_gpio_define_pin(port, 14);      \
    _internal_gpio_define_pin(port, 15)
#define _internal_gpio_define_ports()   \
    _internal_gpio_define_port_pins(A); \
    _internal_gpio_define_port_pins(B); \
    _internal_gpio_define_port_pins(C); \
    _internal_gpio_define_port_pins(D); \
    _internal_gpio_define_port_pins(E); \
    _internal_gpio_define_port_pins(F); \
    _internal_gpio_define_port_pins(G); \
    _internal_gpio_define_port_pins(H); \
    _internal_gpio_define_port_pins(I); \
    _internal_gpio_define_port_pins(J); \
    _internal_gpio_define_port_pins(K)
#else
#define _internal_gpio_declare_ports() static const GPIOPin PZ9;
#define _internal_gpio_define_ports() constexpr GPIOPin GPIOPin::PZ9(0, 0);
#endif

namespace atmc
{
#if _libatmc_target_stm32
    using ADCNativeHandle = ADC_HandleTypeDef;
    using HwTimerNativeHandle = TIM_HandleTypeDef;
#else
    using ADCNativeHandle = byte;
    using HwTimerNativeHandle = byte;
#endif

    using ADCChannel = int;

    class GPIOManager;

    /// @brief Any GPIO pin.
    /// @note Pass `byval`.
    struct GPIOPin
    {
        _internal_gpio_declare_ports(); // NOLINT(bugprone-dynamic-static-initializers)

        /// @brief The state of a GPIO pin.
        using State = int;

#if defined(STM32) && STM32
        static constexpr State High = GPIO_PIN_SET;
        static constexpr State Low = GPIO_PIN_RESET;
#else
        /// @brief Set a pin high.
        static constexpr State High = 1;
        /// @brief Set a pin low.
        static constexpr State Low = 0;
#endif

        /// @brief Construct a GPIO pin.
        /// @param portAddr The address to the internal handle of the GPIO port.
        /// @param pin The pin number.
        constexpr GPIOPin(uintptr_t portAddr, u16 pin) : port(portAddr), pin(pin) { }

        friend class atmc::GPIOManager;
    private:
        uintptr_t port;
        u16 pin;
    };

    _internal_gpio_define_ports();

    struct AnalogPin final
    {
        constexpr AnalogPin(int adc, int rank) : adcIndex(adc), rank(rank) { }

        constexpr bool operator==(const AnalogPin& other) const = default;

        friend class atmc::GPIOManager;
    private:
        int adcIndex;
        int rank;

        [[nodiscard]] ADCNativeHandle* internalHandle() const
        {
            switch (this->adcIndex)
            {
#if _libatmc_target_stm32
            case 0: return &hadc1;
            case 1: return &hadc2;
            case 2: return &hadc3;
#endif

            default: return nullptr;
            }
        }
    };

    struct PWMPin final
    {
        constexpr PWMPin(int timer, int channel) :
            timerIndex(timer), channel([&]() -> int
        {
            switch (channel) // NOLINT(hicpp-multiway-paths-covered)
            {
#if _libatmc_target_stm32
            case 0: return TIM_CHANNEL_1;
            case 1: return TIM_CHANNEL_2;
            case 2: return TIM_CHANNEL_3;
            case 3: return TIM_CHANNEL_4;
            case 4: return TIM_CHANNEL_5;
            // NOLINTNEXTLINE(readability-magic-numbers)
            case 5: return TIM_CHANNEL_6;
#endif

            default: return -1;
            }
        }())
        { }

        friend class atmc::GPIOManager;
    private:
        int timerIndex;
        int channel;

        [[nodiscard]] HwTimerNativeHandle* internalHandle() const
        {
            // NOLINTBEGIN(readability-magic-numbers)
            switch (this->timerIndex)
            {
#if _libatmc_target_stm32
            case 0: return &htim1;
            case 1: return &htim2;
            case 2: return &htim3;
            case 3: return &htim4;
            case 4: return &htim5;
            case 5: return &htim6;
            case 6: return &htim7;
            case 7: return &htim8;
            case 11: return &htim12;
            case 12: return &htim13;
            case 13: return &htim14;
            case 14: return &htim15;
            case 15: return &htim16;
            case 16: return &htim17;

#endif

            default: return nullptr;
            }
            // NOLINTEND(readability-magic-numbers)
        }
    };

    /// @brief GPIO manager.
    /// @note Static class.
    class GPIOManager final
    {
        static std::atomic_flag pinFlag[Config::PinCountGPIO]; // NOLINT(bugprone-dynamic-static-initializers)

        static std::atomic_flag adcFlags[Config::AnalogConverterCount];                                // NOLINT(bugprone-dynamic-static-initializers)
        static _dma_rw volatile uint16_t adcRaw[Config::AnalogConverterCount][Config::MaxADCChannels]; // NOLINT(bugprone-dynamic-static-initializers)
    public:
        GPIOManager() = delete;

        /// @brief Await a pin interrupt, as preconfigured.
        /// @param pin The pin number to await.
        static sys::task<> pinInterrupt(u16 pin)
        {
            const sz pinIndex(std::bit_width(*pin) - 1);
            _coretif(, pinIndex >= sz(Config::PinCountGPIO));

            // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
            GPIOManager::pinFlag[*pinIndex].test_and_set();
            while (GPIOManager::pinFlag[*pinIndex].test())
                co_await sys::task<>::yield();
            // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)

            co_return;
        }

        /// @brief Read a pin.
        /// @param pin The pin to read.
        /// @return The state of the pin.
        static GPIOPin::State digitalRead(GPIOPin pin)
        {
#if _libatmc_target_stm32
            return HAL_GPIO_ReadPin(_asr(GPIO_TypeDef*, pin.port), *pin.pin);
#else
            (void)pin;
            return GPIOPin::Low;
#endif
        }
        static sys::task<sys::result<float, HardwareStatus>> analogRead(AnalogPin pin)
        {
#if _libatmc_target_stm32
            ADCNativeHandle* hadc = pin.internalHandle();
            _coretif(HardwareStatus::Error, !hadc);

            int resolution = 0;
            // NOLINTBEGIN(readability-magic-numbers)
            switch (ADC_GET_RESOLUTION(hadc))
            {
            case ADC_RESOLUTION_8B: resolution = 8; break;
            case ADC_RESOLUTION_10B: resolution = 10; break;
            case ADC_RESOLUTION_12B:
            case ADC_RESOLUTION_12B_OPT: resolution = 12; break;
            case ADC_RESOLUTION_14B:
            case ADC_RESOLUTION_14B_OPT: resolution = 14; break;
            case ADC_RESOLUTION_16B: resolution = 16; break;
            default: co_return HardwareStatus::Error;
            }
            // NOLINTEND(readability-magic-numbers)

            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            if (!GPIOManager::adcFlags[pin.adcIndex].test_and_set())
            {
                _nowarn_begin_one_gcc(_clwarn_gcc_cast_align);
                _nowarn_begin_one_clang(_clwarn_clang_cast_align);
                // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index, cppcoreguidelines-pro-type-const-cast)
                HardwareStatus res = _as(HardwareStatus, HAL_ADC_Start_DMA(hadc, _asr(uint32_t*, _asc(uint16_t*, GPIOManager::adcRaw[pin.adcIndex])), hadc->Init.NbrOfConversion));
                _nowarn_end_clang();
                _nowarn_end_gcc();
                if (res != HardwareStatus::Ok)
                {
                    GPIOManager::adcFlags[pin.adcIndex].clear(); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                    co_return res;
                }
                // `HAL_ADC_Stop_DMA(hadc)` called in ISR.
            }

            co_await sys::task<>::wait_until([&] { return !GPIOManager::adcFlags[pin.adcIndex].test(); });

            float maxVal = _as(float, (1u << _as(unsigned, resolution)) - 1);
            co_return _as(float, GPIOManager::adcRaw[pin.adcIndex][pin.rank]) / maxVal;
#else
            (void)pin;
            co_return 0.0f;
#endif
        }

        /// @brief Set a pin.
        /// @param pin The pin to set.
        /// @param state The state to set the pin to.
        static void digitalWrite(GPIOPin pin, GPIOPin::State state)
        {
#if _libatmc_target_stm32
            HAL_GPIO_WritePin(_asr(GPIO_TypeDef*, pin.port), *pin.pin, GPIO_PinState(state));
#else
            (void)pin;
            (void)state;
#endif
        }

        static HardwareStatus pwmWrite(PWMPin pin, float duty)
        {
#if _libatmc_target_stm32
            _retif(HardwareStatus::Error, pin.channel == -1 || duty < 0.0f || duty > 1.0f);

            HwTimerNativeHandle* htim = pin.internalHandle();
            _retif(HardwareStatus::Error, !htim);

            htim->Instance->CCR1 = _as(uint32_t, _as(float, htim->Instance->ARR) * duty + 0.5f);

            return HardwareStatus(HAL_TIM_PWM_Start(htim, _as(uint32_t, pin.channel)));
#else
            (void)pin;
            (void)duty;
            return HardwareStatus::Ok;
#endif
        }
        static HardwareStatus pwmClear(PWMPin pin)
        {
#if _libatmc_target_stm32
            _retif(HardwareStatus::Error, pin.channel == -1);

            HwTimerNativeHandle* htim = pin.internalHandle();
            _retif(HardwareStatus::Error, !htim);

            return HardwareStatus(HAL_TIM_PWM_Stop(htim, _as(uint32_t, pin.channel)));
#else
            (void)pin;
            return HardwareStatus::Ok;
#endif
        }

#if _libatmc_target_stm32
        friend void ::HAL_GPIO_EXTI_Callback(uint16_t pin);
        friend void ::HAL_ADC_ConvCpltCallback(atmc::ADCNativeHandle* hadc);
#endif
    };
} // namespace atmc
