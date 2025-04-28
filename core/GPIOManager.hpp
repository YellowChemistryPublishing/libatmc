/* module; */
#pragma once

#include <atomic>
#include <bit>
#include <coroutine>
#include <cstring>
#include <cxxutil.h>
#include <runtime_headers.h>

#include <Config.h>
#include <Exception.hpp>
#include <InplaceAtomicSet.h>
#include <InplaceVector.h>
#include <Result.h>
#include <Result.hpp>
#include <SpinLock.h>

/* export module core.IO.Embedded; */

/* import core.Concurrency; */
#include <Task.hpp>

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

extern "C" void DMA1_Stream0_IRQHandler();
extern "C" void ADC_IRQHandler();

extern "C" void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);

/* export */ namespace atmc
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
        constexpr AnalogPin(int adc, int channel) : adcIndex(adc), channel(channel)
        { }

        constexpr bool operator==(const AnalogPin& other) const = default;
        
        friend class atmc::GPIOManager;
    private:
        uintptr_t adcIndex;
        int channel;
    };

    /// @brief GPIO manager.
    /// @note Static class.
    class GPIOManager final
    {
        static std::atomic_flag pinFlag[Config::PinCountGPIO];

        static InplaceVector<int, Config::MaxADCChannels> adcChannels[Config::AnalogConverterCount];
        static DMA_HandleTypeDef hdmaADC[Config::AnalogConverterCount];
        static ADC_HandleTypeDef hadc[Config::AnalogConverterCount];
        static std::atomic_flag adcFlags[Config::AnalogConverterCount];
        static __dma_rw volatile uint16_t adcRaw[Config::AnalogConverterCount][Config::MaxADCChannels];
    public:
        GPIOManager() = delete;
        
        inline static HardwareStatus initADC(std::span<AnalogPin> pins)
        {
            for (auto inputPin : pins)
            {
                if (inputPin.adcIndex < 0 || inputPin.adcIndex >= Config::AnalogConverterCount ||
                    GPIOManager::adcChannels[inputPin.adcIndex].find(inputPin.channel))
                    continue;

                if (!GPIOManager::adcChannels[inputPin.adcIndex].pushBack(inputPin.channel))
                {
                    for (int i = 0; i < Config::AnalogConverterCount; i++)
                        GPIOManager::adcChannels[i].clear();
                    return HardwareStatus::Error;
                }
            }

            __HAL_RCC_DMA1_CLK_ENABLE();

            HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

            GPIOManager::hadc[0].Instance = ADC1;
            GPIOManager::hadc[0].Init.ScanConvMode = ADC_SCAN_ENABLE;
            GPIOManager::hadc[0].Init.EOCSelection = ADC_EOC_SEQ_CONV;
            GPIOManager::hadc[0].Init.LowPowerAutoWait = DISABLE;
            GPIOManager::hadc[0].Init.ContinuousConvMode = DISABLE;
            GPIOManager::hadc[0].Init.NbrOfConversion = GPIOManager::adcChannels[0].size();
            GPIOManager::hadc[0].Init.DiscontinuousConvMode = DISABLE;
            GPIOManager::hadc[0].Init.ExternalTrigConv = ADC_SOFTWARE_START;
            GPIOManager::hadc[0].Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
            GPIOManager::hadc[0].Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
            GPIOManager::hadc[0].Init.Overrun = ADC_OVR_DATA_PRESERVED;
            GPIOManager::hadc[0].Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
            GPIOManager::hadc[0].Init.OversamplingMode = DISABLE;
            GPIOManager::hadc[0].Init.Oversampling.Ratio = 1;
            GPIOManager::hadc[0].Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
            GPIOManager::hadc[0].Init.Resolution = ADC_RESOLUTION_16B;
            HardwareStatus res = (HardwareStatus)HAL_ADC_Init(&GPIOManager::hadc[0]);
            __fence_value_return(res, res != HardwareStatus::Ok);

            ADC_MultiModeTypeDef multimode;
            memset(&multimode, 0, sizeof(ADC_MultiModeTypeDef));
            ADC_ChannelConfTypeDef sConfig;
            memset(&sConfig, 0, sizeof(ADC_ChannelConfTypeDef));

            multimode.Mode = ADC_MODE_INDEPENDENT;
            res = (HardwareStatus)HAL_ADCEx_MultiModeConfigChannel(&GPIOManager::hadc[0], &multimode);
            __fence_value_return(res, res != HardwareStatus::Ok);

            sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
            sConfig.SingleDiff = ADC_SINGLE_ENDED;
            sConfig.OffsetNumber = ADC_OFFSET_NONE;
            sConfig.Offset = 0;
            sConfig.OffsetSignedSaturation = DISABLE;
            
            for (int i = 0; i < GPIOManager::adcChannels[0].size(); i++)
            {
                switch (GPIOManager::adcChannels[0][i])
                {
                case 0:
                    sConfig.Channel = ADC_CHANNEL_0;
                    break;
                case 1:
                    sConfig.Channel = ADC_CHANNEL_1;
                    break;
                case 2:
                    sConfig.Channel = ADC_CHANNEL_2;
                    break;
                case 3:
                    sConfig.Channel = ADC_CHANNEL_3;
                    break;
                case 4:
                    sConfig.Channel = ADC_CHANNEL_4;
                    break;
                case 5:
                    sConfig.Channel = ADC_CHANNEL_5;
                    break;
                case 6:
                    sConfig.Channel = ADC_CHANNEL_6;
                    break;
                case 7:
                    sConfig.Channel = ADC_CHANNEL_7;
                    break;
                case 8:
                    sConfig.Channel = ADC_CHANNEL_8;
                    break;
                case 9:
                    sConfig.Channel = ADC_CHANNEL_9;
                    break;
                case 10:
                    sConfig.Channel = ADC_CHANNEL_10;
                    break;
                case 11:
                    sConfig.Channel = ADC_CHANNEL_11;
                    break;
                case 12:
                    sConfig.Channel = ADC_CHANNEL_12;
                    break;
                case 13:
                    sConfig.Channel = ADC_CHANNEL_13;
                    break;
                case 14:
                    sConfig.Channel = ADC_CHANNEL_14;
                    break;
                case 15:
                    sConfig.Channel = ADC_CHANNEL_15;
                    break;
                case 16:
                    sConfig.Channel = ADC_CHANNEL_16;
                    break;
                case 17:
                    sConfig.Channel = ADC_CHANNEL_17;
                    break;
                case 18:
                    sConfig.Channel = ADC_CHANNEL_18;
                    break;
                case 19:
                    sConfig.Channel = ADC_CHANNEL_19;
                    break;
                default:
                    return HardwareStatus::Error;
                }
                switch (i)
                {
                case 0:
                    sConfig.Rank = ADC_REGULAR_RANK_1;
                    break;
                case 1:
                    sConfig.Rank = ADC_REGULAR_RANK_2;
                    break;
                case 2:
                    sConfig.Rank = ADC_REGULAR_RANK_3;
                    break;
                case 3:
                    sConfig.Rank = ADC_REGULAR_RANK_4;
                    break;
                case 4:
                    sConfig.Rank = ADC_REGULAR_RANK_5;
                    break;
                case 5:
                    sConfig.Rank = ADC_REGULAR_RANK_6;
                    break;
                case 6:
                    sConfig.Rank = ADC_REGULAR_RANK_7;
                    break;
                case 7:
                    sConfig.Rank = ADC_REGULAR_RANK_8;
                    break;
                case 8:
                    sConfig.Rank = ADC_REGULAR_RANK_9;
                    break;
                case 9:
                    sConfig.Rank = ADC_REGULAR_RANK_10;
                    break;
                case 10:
                    sConfig.Rank = ADC_REGULAR_RANK_11;
                    break;
                case 11:
                    sConfig.Rank = ADC_REGULAR_RANK_12;
                    break;
                case 12:
                    sConfig.Rank = ADC_REGULAR_RANK_13;
                    break;
                case 13:
                    sConfig.Rank = ADC_REGULAR_RANK_14;
                    break;
                case 14:
                    sConfig.Rank = ADC_REGULAR_RANK_15;
                    break;
                case 15:
                    sConfig.Rank = ADC_REGULAR_RANK_16;
                    break;
                default:
                    return HardwareStatus::Error;
                }
                res = (HardwareStatus)HAL_ADC_ConfigChannel(&GPIOManager::hadc[0], &sConfig);
                __fence_value_return(res, res != HardwareStatus::Ok);
            }

            return HardwareStatus::Ok;
        }

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
        inline static GPIOPin::State digitalRead(GPIOPin pin)
        {
            return HAL_GPIO_ReadPin((GPIO_TypeDef*)pin.port, pin.pin);
        }
        inline static Task<Result<float, HardwareStatus>> analogRead(AnalogPin pin)
        {
            int resolution;
            switch (ADC_GET_RESOLUTION(&GPIOManager::hadc[0]))
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
                HAL_ADC_Start_DMA(&GPIOManager::hadc[0], __reic(uint32_t*, __cstc(uint16_t*, GPIOManager::adcRaw[pin.adcIndex])), 2);
            // `HAL_ADC_Stop_DMA(hadc)` called in ISR.

            co_await Task<>::waitUntil([&]
            {
                return !GPIOManager::adcFlags[pin.adcIndex].test();
            });

            int* rank = GPIOManager::adcChannels[pin.adcIndex].find(pin.channel);
            __fence_value_co_return(HardwareStatus::Error, !rank);

            auto rankIndex = rank - &GPIOManager::adcChannels[pin.adcIndex][0];
            float maxVal = (1u << resolution) - 1;

            co_return __sc(float, GPIOManager::adcRaw[pin.adcIndex][rankIndex]) / maxVal;
        }
        
        /// @brief Set a pin.
        /// @param pin The pin to set.
        /// @param state The state to set the pin to.
        inline static void digitalWrite(GPIOPin pin, GPIOPin::State state)
        {
            HAL_GPIO_WritePin((GPIO_TypeDef*)pin.port, pin.pin, (GPIO_PinState)state);
        }
        inline static void pwmWrite()
        {
            
        }

        friend void ::HAL_GPIO_EXTI_Callback(uint16_t pin);

        friend void ::DMA1_Stream0_IRQHandler();
        friend void ::ADC_IRQHandler();
        friend void ::HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
        friend void ::HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
    };
}
