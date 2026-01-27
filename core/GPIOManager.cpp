#include "GPIOManager.h"

#include <atomic>
#include <cstdint>
#include <entry.h>

#include <Config.h>
#include <Target.h>

using namespace atmc;

std::atomic_flag GPIOManager::pinFlag[Config::PinCountGPIO];

std::atomic_flag GPIOManager::adcFlags[Config::AnalogConverterCount];
_dma_rw alignas(uint32_t) volatile uint16_t GPIOManager::adcRaw[Config::AnalogConverterCount][Config::MaxADCChannels];

#if _libatmc_target_stm32

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    int pinIndex = std::bit_width(pin) - 1;
    atmc::GPIOManager::pinFlag[pinIndex].clear();
}

extern "C" void HAL_ADC_ConvCpltCallback(ADCNativeHandle* hadc)
{
    HAL_ADC_Stop_DMA(hadc);
    switch (_asr(uintptr_t, hadc->Instance))
    {
    case ADC1_BASE: GPIOManager::adcFlags[0].clear(); break;
    case ADC2_BASE: GPIOManager::adcFlags[1].clear(); break;
    case ADC3_BASE: GPIOManager::adcFlags[2].clear(); break;
    }
}

#endif
