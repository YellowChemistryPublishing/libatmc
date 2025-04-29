#include "GPIOManager.hpp"

using namespace atmc;

std::atomic_flag GPIOManager::pinFlag[Config::PinCountGPIO];

std::atomic_flag GPIOManager::adcFlags[Config::AnalogConverterCount];
__dma_rw volatile uint16_t GPIOManager::adcRaw[Config::AnalogConverterCount][Config::MaxADCChannels];
