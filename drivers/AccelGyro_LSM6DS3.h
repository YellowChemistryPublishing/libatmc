#pragma once

/// @file AccelGyro_LSM6DS3.cppm
/// @brief Accelerometer and gyroscope driver for the LSM6DS3.

#include <concepts>
#include <limits>
#include <print>
#include <span>

#include <LanguageSupport.h>
#include <Result.h>
#include <SerialInterfaceDevice.h>
#include <TaskEx.h>
#include <Vector.h>


namespace atmc
{
    namespace LSM6DS3
    {
        static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "This LSM6DS3 driver requires a little-endian byte order.");

        // v Basic data definitions.

        constexpr int AddrLow = 0x6A;
        constexpr int AddrHigh = 0x6B;
        constexpr int DeviceID = 0x6A; // Result of reading register LSM6DS3_REGISTER_WHO_AM_I.

        // ^ Basic data definitions. / Register definitions. v

        /// @brief Register addresses.
        /// @note Static class.
        struct RegAddr
        {
            using RegisterAddress = int;
            constexpr static int Size = sizeof(uint8_t);

            // Page 49/115 DocID030071 Rev 3
            constexpr static RegisterAddress FuncCfgAccess = 0x01;
            constexpr static RegisterAddress SensorSyncTimeFrame = 0x04;
            constexpr static RegisterAddress SensorSyncResRatio = 0x05;
            constexpr static RegisterAddress FIFOCtrl1 = 0x06;
            constexpr static RegisterAddress FIFOCtrl2 = 0x07;
            constexpr static RegisterAddress FIFOCtrl3 = 0x08;
            constexpr static RegisterAddress FIFOCtrl4 = 0x09;
            constexpr static RegisterAddress FIFOCtrl5 = 0x0A;
            constexpr static RegisterAddress DRdyPulseCfgG = 0x0B;
            constexpr static RegisterAddress Int1Ctrl = 0x0D;
            constexpr static RegisterAddress Int2Ctrl = 0x0E;
            constexpr static RegisterAddress WhoAmI = 0x0F;
            constexpr static RegisterAddress Ctrl1XL = 0x10;
            constexpr static RegisterAddress Ctrl2G = 0x11;
            constexpr static RegisterAddress Ctrl3C = 0x12;
            constexpr static RegisterAddress Ctrl4C = 0x13;
            constexpr static RegisterAddress Ctrl5C = 0x14;
            constexpr static RegisterAddress Ctrl6C = 0x15;
            constexpr static RegisterAddress Ctrl7G = 0x16;
            constexpr static RegisterAddress Ctrl8XL = 0x17;
            constexpr static RegisterAddress Ctrl9XL = 0x18;
            constexpr static RegisterAddress Ctrl10C = 0x19;

            // Page 50/115 DocID030071 Rev 3
            constexpr static RegisterAddress MasterConfig = 0x1A;
            constexpr static RegisterAddress WakeUpSrc = 0x1B;
            constexpr static RegisterAddress TapSrc = 0x1C;
            constexpr static RegisterAddress D6DSrc = 0x1D;
            constexpr static RegisterAddress StatusReg = 0x1E;
            constexpr static RegisterAddress OutTempL = 0x20;
            constexpr static RegisterAddress OutTempH = 0x21;
            constexpr static RegisterAddress OutXLGyro = 0x22;
            constexpr static RegisterAddress OutXHGyro = 0x23;
            constexpr static RegisterAddress OutYLGyro = 0x24;
            constexpr static RegisterAddress OutYHGyro = 0x25;
            constexpr static RegisterAddress OutZLGyro = 0x26;
            constexpr static RegisterAddress OutZHGyro = 0x27;
            constexpr static RegisterAddress OutXLAccel = 0x28;
            constexpr static RegisterAddress OutXHAccel = 0x29;
            constexpr static RegisterAddress OutYLAccel = 0x2A;
            constexpr static RegisterAddress OutYHAccel = 0x2B;
            constexpr static RegisterAddress OutZLAccel = 0x2C;
            constexpr static RegisterAddress OutZHAccel = 0x2D;
            constexpr static RegisterAddress SensorHub1Reg = 0x2E;
            constexpr static RegisterAddress SensorHub2Reg = 0x2F;
            constexpr static RegisterAddress SensorHub3Reg = 0x30;
            constexpr static RegisterAddress SensorHub4Reg = 0x31;
            constexpr static RegisterAddress SensorHub5Reg = 0x32;
            constexpr static RegisterAddress SensorHub6Reg = 0x33;
            constexpr static RegisterAddress SensorHub7Reg = 0x34;
            constexpr static RegisterAddress SensorHub8Reg = 0x35;
            constexpr static RegisterAddress SensorHub9Reg = 0x36;
            constexpr static RegisterAddress SensorHub10Reg = 0x37;
            constexpr static RegisterAddress SensorHub11Reg = 0x38;
            constexpr static RegisterAddress SensorHub12Reg = 0x39;

            // Page 51/115 DocID030071 Rev 3
            constexpr static RegisterAddress FIFOStatus1 = 0x3A;
            constexpr static RegisterAddress FIFOStatus2 = 0x3B;
            constexpr static RegisterAddress FIFOStatus3 = 0x3C;
            constexpr static RegisterAddress FIFOStatus4 = 0x3D;
            constexpr static RegisterAddress FIFODataOutL = 0x3E;
            constexpr static RegisterAddress FIFODataOutH = 0x3F;
            constexpr static RegisterAddress Timestamp0Reg = 0x40;
            constexpr static RegisterAddress Timestamp1Reg = 0x41;
            constexpr static RegisterAddress Timestamp2Reg = 0x42;
            constexpr static RegisterAddress StepTimestampL = 0x49;
            constexpr static RegisterAddress StepTimestampH = 0x4A;
            constexpr static RegisterAddress StepCounterL = 0x4B;
            constexpr static RegisterAddress StepCounterH = 0x4C;
            constexpr static RegisterAddress SensorHub13Reg = 0x4D;
            constexpr static RegisterAddress SensorHub14Reg = 0x4E;
            constexpr static RegisterAddress SensorHub15Reg = 0x4F;
            constexpr static RegisterAddress SensorHub16Reg = 0x50;
            constexpr static RegisterAddress SensorHub17Reg = 0x51;
            constexpr static RegisterAddress SensorHub18Reg = 0x52;
            constexpr static RegisterAddress FuncSrc1 = 0x53;
            constexpr static RegisterAddress FuncSrc2 = 0x54;
            constexpr static RegisterAddress WristTiltIA = 0x55;
            constexpr static RegisterAddress TapCfg = 0x58;
            constexpr static RegisterAddress TapThs6D = 0x59;
            constexpr static RegisterAddress IntDur2 = 0x5A;
            constexpr static RegisterAddress WakeUpThs = 0x5B;
            constexpr static RegisterAddress WakeUpDur = 0x5C;
            constexpr static RegisterAddress FreeFall = 0x5D;
            constexpr static RegisterAddress MD1Cfg = 0x5E;
            constexpr static RegisterAddress MD2Cfg = 0x5F;
            constexpr static RegisterAddress MasterCmdCode = 0x60;

            // Page 52/115 DocID030071 Rev 3
            constexpr static RegisterAddress SensSyncSPIErrorCode = 0x61;
            constexpr static RegisterAddress OutMagRawXL = 0x66;
            constexpr static RegisterAddress OutMagRawXH = 0x67;
            constexpr static RegisterAddress OutMagRawYL = 0x68;
            constexpr static RegisterAddress OutMagRawYH = 0x69;
            constexpr static RegisterAddress OutMagRawZL = 0x6A;
            constexpr static RegisterAddress OutMagRawZH = 0x6B;
            constexpr static RegisterAddress XOfsUsr = 0x73;
            constexpr static RegisterAddress YOfsUsr = 0x74;
            constexpr static RegisterAddress ZOfsUsr = 0x75;

            // Page 97/115 DocID030071 Rev 3
            constexpr static RegisterAddress Slv0Add = 0x02;
            constexpr static RegisterAddress Slv0SubAdd = 0x03;
            constexpr static RegisterAddress Slave0Config = 0x04;
            constexpr static RegisterAddress Slv1Add = 0x05;
            constexpr static RegisterAddress Slv1SubAdd = 0x06;
            constexpr static RegisterAddress Slave1Config = 0x07;
            constexpr static RegisterAddress Slv2Add = 0x08;
            constexpr static RegisterAddress Slv2SubAdd = 0x09;
            constexpr static RegisterAddress Slave2Config = 0x0A;
            constexpr static RegisterAddress Slv3Add = 0x0B;
            constexpr static RegisterAddress Slv3SubAdd = 0x0C;
            constexpr static RegisterAddress Slave3Config = 0x0D;
            constexpr static RegisterAddress DataWriteSrcModeSubSlv0 = 0x0E;
            constexpr static RegisterAddress ConfigPedoThsMin = 0x0F;
            constexpr static RegisterAddress SMThs = 0x13;
            constexpr static RegisterAddress PedoDebReg = 0x14;
            constexpr static RegisterAddress StepCountDelta = 0x15;
            constexpr static RegisterAddress MagSI_XX = 0x24;
            constexpr static RegisterAddress MagSI_XY = 0x25;
            constexpr static RegisterAddress MagSI_XZ = 0x26;
            constexpr static RegisterAddress MagSI_YX = 0x27;
            constexpr static RegisterAddress MagSI_YY = 0x28;

            // Page 98/115 DocID030071 Rev 3
            constexpr static RegisterAddress MagSI_YZ = 0x29;
            constexpr static RegisterAddress MagSI_ZX = 0x2A;
            constexpr static RegisterAddress MagSI_ZY = 0x2B;
            constexpr static RegisterAddress MagSI_ZZ = 0x2C;
            constexpr static RegisterAddress MagOffXL = 0x2D;
            constexpr static RegisterAddress MagOffXH = 0x2E;
            constexpr static RegisterAddress MagOffYL = 0x2F;
            constexpr static RegisterAddress MagOffYH = 0x30;
            constexpr static RegisterAddress MagOffZL = 0x31;
            constexpr static RegisterAddress MagOffZH = 0x32;
            constexpr static RegisterAddress AWristTiltLat = 0x50;
            constexpr static RegisterAddress AWristTiltThs = 0x54;
            constexpr static RegisterAddress AWristTiltMask = 0x59;

            RegAddr() = delete;
        };

        // ^ Register definitions. / Sensitivity constants. v

        constexpr float AccelSensitivityForFS2G = 0.000061f;
        constexpr float AccelSensitivityForFS4G = 0.000122f;
        constexpr float AccelSensitivityForFS8G = 0.000244f;
        constexpr float AccelSensitivityForFS16G = 0.000488f;

        constexpr float GyroSensitivityForFS125DPS = 0.004375f;
        constexpr float GyroSensitivityForFS245DPS = 0.00875f;
        constexpr float GyroSensitivityForFS500DPS = 0.0175f;
        constexpr float GyroSensitivityForFS1000DPS = 0.035f;
        constexpr float GyroSensitivityForFS2000DPS = 0.07f;

        // ^ Sensitivity constants. / Default configuration settings. v

#if LSM6DS3_TIMEOUT
        constexpr int TimeoutDuration = LSM6DS3_TIMEOUT;
#else
        constexpr int TimeoutDuration = 10000;
#endif

#if LSM6DS3_FIFO_CHUNK_SIZE
        constexpr int FIFOChunkSize = LSM6DS3_FIFO_CHUNK_SIZE;
#else
        constexpr int FIFOChunkSize = 256;
#endif

        // ^ Default configuration settings.

#pragma pack(push, 1)

        enum class FeatureConfigBanksEnabled : uint_least8_t
        {
            Neither = 0b00,
            BankA = 0b10,
            BankB = 0b11
        };

        struct RegisterFeatureConfig
        {
            uint8_t _rsv2 : 5 = 0;
            uint8_t _noDirectAccessFuncConfigEnabledB : 1 = 0;
            uint8_t _rsv1 : 1 = 0;
            uint8_t _noDirectAccessFuncConfigEnabled : 1 = 0;

            constexpr FeatureConfigBanksEnabled banksEnabled() const noexcept
            {
                uint_fast8_t banks = this->_noDirectAccessFuncConfigEnabledB;
                banks |= this->_noDirectAccessFuncConfigEnabled << 1;
                return FeatureConfigBanksEnabled(banks);
            }
            constexpr void setBanksEnabled(FeatureConfigBanksEnabled which)
            {
                this->_noDirectAccessFuncConfigEnabled = (uint_fast8_t(which) & 0b10) >> 1;
                this->_noDirectAccessFuncConfigEnabledB = uint_fast8_t(which) & 0b01;
            }

            constexpr bool operator==(const RegisterFeatureConfig& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterFeatureConfig) == 1, "Type `RegisterFeatureConfig` must be packed to one byte.");

        constexpr uint16_t thresholdBitsToBytes(uint8_t msb, uint8_t lsb)
        {
            return uint16_t(((uint16_t(msb) << 8) | uint16_t(lsb)) * 2);
        }
        constexpr uint16_t thresholdBytesToBits(uint16_t bytes)
        {
            return bytes / 2;
        }

        struct RegisterFIFOCtrl1
        {
            uint8_t threshold = 0xff;

            constexpr bool operator==(const RegisterFIFOCtrl1& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterFIFOCtrl1) == 1, "Type `RegisterFIFOCtrl1` must be packed to one byte.");

        enum class FIFOWriteMode : uint_least8_t
        {
            OnDataReady = 0,
            OnStepCount = 1
        };

        struct RegisterFIFOCtrl2
        {
            uint8_t thresholdHighBits : 3 = 0b111;
            bool tempStoreEnabled : 1 = false;
            uint8_t _rsv : 2 = 0;
            FIFOWriteMode writeMode : 1 = FIFOWriteMode::OnDataReady;
            bool fifoTimerStepCounterEnabled : 1 = true;

            constexpr bool operator==(const RegisterFIFOCtrl2& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterFIFOCtrl2) == 1, "Type `RegisterFIFOCtrl2` must be packed to one byte.");

        constexpr uint16_t registerThresholdBits(const RegisterFIFOCtrl1& ctrl1, const RegisterFIFOCtrl2& ctrl2)
        {
            return thresholdBytesToBits(thresholdBitsToBytes(ctrl2.thresholdHighBits, ctrl1.threshold));
        }
        constexpr void setRegisterThresholdBits(RegisterFIFOCtrl1& ctrl1, RegisterFIFOCtrl2& ctrl2, uint16_t bits)
        {
            ctrl1.threshold = uint8_t(bits);
            _push_nowarn_gcc(_clWarn_gcc_conversion);
            ctrl2.thresholdHighBits = uint8_t(bits >> 8);
            _pop_nowarn_gcc();
        }

        constexpr uint8_t decimationBitsForNoSensor = 0;
        constexpr uint8_t decimationFactorToBits(int factor)
        {
            switch (factor)
            {
            case 0: return 0b001;
            case 2: return 0b010;
            case 3: return 0b011;
            case 4: return 0b100;
            case 8: return 0b101;
            case 16: return 0b110;
            case 32: return 0b111;
            default: return decimationBitsForNoSensor;
            }
        }

        struct RegisterFIFOCtrl3
        {
            uint8_t accelDecimation : 3 = decimationFactorToBits(3);
            uint8_t gyroDecimation : 3 = decimationFactorToBits(3);
            uint8_t _rsv : 2 = 0;

            constexpr bool operator==(const RegisterFIFOCtrl3& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterFIFOCtrl3) == 1, "Type `RegisterFIFOCtrl3` must be packed to one byte.");

        struct RegisterFIFOCtrl4
        {
            uint8_t data3Decimation : 3 = decimationBitsForNoSensor;
            uint8_t data4Decimation : 3 = decimationFactorToBits(0);
            bool onlyHighData : 1 = false;
            bool stopOnThreshold : 1 = false;

            constexpr bool operator==(const RegisterFIFOCtrl4& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterFIFOCtrl4) == 1, "Type `RegisterFIFOCtrl4` must be packed to one byte.");

        enum class FIFOMode : uint_least8_t
        {
            Bypass = 0b000,
            FIFO = 0b001,
            ContinuousThenFIFO = 0b011,
            BypassThenContinuous = 0b100,
            Continuous = 0b110
        };

        constexpr uint8_t outputDataRateFrequencyToBits(float hz)
        {
            if (hz == 12.5f)
                return 0b0001;
            else if (hz == 26.0f)
                return 0b0010;
            else if (hz == 52.0f)
                return 0b0011;
            else if (hz == 104.0f)
                return 0b0100;
            else if (hz == 208.0f)
                return 0b0101;
            else if (hz == 416.0f)
                return 0b0110;
            else if (hz == 833.0f)
                return 0b0111;
            else if (hz == 1660.0f)
                return 0b1000;
            else if (hz == 3330.0f)
                return 0b1001;
            else if (hz == 6660.0f)
                return 0b1010;
            else // if (hz == 0.0f)
                return 0b0000;
        }
        constexpr float outputDataRateBitsToFrequency(uint8_t bits)
        {
            switch (bits)
            {
            case 0b0001: return 12.5f;
            case 0b0010: return 26.0f;
            case 0b0011: return 52.0f;
            case 0b0100: return 104.0f;
            case 0b0101: return 208.0f;
            case 0b0110: return 416.0f;
            case 0b0111: return 833.0f;
            case 0b1000: return 1660.0f;
            case 0b1001: return 3330.0f;
            case 0b1010: return 6660.0f;
            case 0b0000:
            default: return 0.0f;
            }
        }

        struct RegisterFIFOCtrl5
        {
            FIFOMode mode : 3 = FIFOMode::Bypass;
            uint8_t outputDataRate : 4 = outputDataRateFrequencyToBits(12.5f);
            uint8_t _rsv : 1 = 0;

            constexpr bool operator==(const RegisterFIFOCtrl5& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterFIFOCtrl5) == 1, "Type `RegisterFIFOCtrl5` must be packed to one byte.");

        enum class DataReadyPulsedMode : uint_least8_t
        {
            Latched = 0,
            Pulsed = 1
        };

        struct RegisterDataReadyPulseConfigGyro
        {
            bool int2WristTiltIRQ : 1 = false;
            uint8_t _rsv : 6 = 0;
            DataReadyPulsedMode mode : 1 = DataReadyPulsedMode::Latched;

            constexpr bool operator==(const RegisterDataReadyPulseConfigGyro& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterDataReadyPulseConfigGyro) == 1, "Type `RegisterDataReadyPulseConfigGyro` must be packed to one byte.");

        struct RegisterInt1Ctrl
        {
            bool dataReadyAccelIRQ : 1 = false;
            bool dataReadyGyroIRQ : 1 = false;
            bool bootStatusIRQ : 1 = false;
            bool fifoThresholdIRQ : 1 = false;
            bool fifoOverrunIRQ : 1 = true;
            bool fifoFullIRQ : 1 = false;
            bool significantMotionIRQ : 1 = false;
            bool stepDetectorIRQ : 1 = false;

            constexpr bool operator==(const RegisterInt1Ctrl& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterInt1Ctrl) == 1, "Type `RegisterInt1Ctrl` must be packed to one byte.");

        struct RegisterInt2Ctrl
        {
            bool dataReadyAccelIRQ : 1 = false;
            bool dataReadyGyroIRQ : 1 = false;
            bool dataReadyTempIRQ : 1 = false;
            bool fifoThresholdIRQ : 1 = false;
            bool fifoOverrunIRQ : 1 = false;
            bool fifoFullIRQ : 1 = true;
            bool stepCounterOverflowIRQ : 1 = false;
            bool stepRecogOnDeltaTimeIRQ : 1 = false;

            constexpr bool operator==(const RegisterInt2Ctrl& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterInt2Ctrl) == 1, "Type `RegisterInt2Ctrl` must be packed to one byte.");

        enum class FrequencySelection : uint_least8_t
        {
            HalfOutputDataRate = 0,
            QuarterOutputDataRate = 1
        };

        struct RegisterCtrl1Accel
        {
            uint8_t analogChainBandwidth : 1 = RegisterCtrl1Accel::analogChainBandwidthToBits(1.5f);
            FrequencySelection _noDirectAccessLPF1Bandwidth : 1 = FrequencySelection::QuarterOutputDataRate;
            uint8_t fullScaleConfig : 2 = RegisterCtrl1Accel::fullScaleConfigRangeToBits(16);
            uint8_t outputDataRate : 4 = outputDataRateFrequencyToBits(12.5f);

            constexpr static uint8_t fullScaleConfigRangeToBits(int rangeUpper)
            {
                switch (rangeUpper)
                {
                case 16: return 0b01;
                case 4: return 0b10;
                case 8: return 0b11;
                case 2:
                default: return 0b00;
                }
            }
            constexpr static float fullScaleConfigToGeePerLSB(uint8_t fullScaleConfig)
            {
                switch (fullScaleConfig)
                {
                case RegisterCtrl1Accel::fullScaleConfigRangeToBits(4): return LSM6DS3::AccelSensitivityForFS4G;
                case RegisterCtrl1Accel::fullScaleConfigRangeToBits(8): return LSM6DS3::AccelSensitivityForFS8G;
                case RegisterCtrl1Accel::fullScaleConfigRangeToBits(16): return LSM6DS3::AccelSensitivityForFS16G;
                case RegisterCtrl1Accel::fullScaleConfigRangeToBits(2):
                default: return LSM6DS3::AccelSensitivityForFS2G;
                }
            }

            constexpr static uint8_t analogChainBandwidthToBits(float kHz)
            {
                if (kHz == 0.4f)
                    return 0b1;
                else // if (kHz == 1.5f)
                    return 0b0;
            }

            constexpr bool operator==(const RegisterCtrl1Accel& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl1Accel) == 1, "Type `RegisterCtrl1Accel` must be packed to one byte.");

        struct RegisterCtrl2Gyro
        {
            uint8_t _rsv : 1 = 0;
            uint8_t fullScaleConfig : 3 = RegisterCtrl2Gyro::fullScaleConfigRangeToBits(2000);
            uint8_t outputDataRate : 4 = outputDataRateFrequencyToBits(12.5f);

            constexpr static uint8_t fullScaleConfigRangeToBits(int dps)
            {
                switch (dps)
                {
                case 125: return 0b001;
                case 500: return 0b010;
                case 1000: return 0b100;
                case 2000: return 0b110;
                case 245:
                default: return 0b000;
                }
            }
            constexpr static float fullScaleConfigToDPSPerLSB(uint8_t fullScaleConfig)
            {
                switch (fullScaleConfig)
                {
                case RegisterCtrl2Gyro::fullScaleConfigRangeToBits(125): return LSM6DS3::GyroSensitivityForFS125DPS;
                case RegisterCtrl2Gyro::fullScaleConfigRangeToBits(500): return LSM6DS3::GyroSensitivityForFS500DPS;
                case RegisterCtrl2Gyro::fullScaleConfigRangeToBits(1000): return LSM6DS3::GyroSensitivityForFS1000DPS;
                case RegisterCtrl2Gyro::fullScaleConfigRangeToBits(2000): return LSM6DS3::GyroSensitivityForFS2000DPS;
                case RegisterCtrl2Gyro::fullScaleConfigRangeToBits(245):
                default: return LSM6DS3::GyroSensitivityForFS245DPS;
                }
            }

            constexpr bool operator==(const RegisterCtrl2Gyro& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl2Gyro) == 1, "Type `RegisterCtrl2Gyro` must be packed to one byte.");

        enum class BlockDataUpdate : uint_least8_t
        {
            Continuous = 0,
            UntilRead = 1
        };
        enum class InterruptActivationLevel : uint_least8_t
        {
            ActiveHigh = 0,
            ActiveLow = 1
        };
        enum class IRQPadMode : uint_least8_t
        {
            PushPull = 0,
            OpenDrain = 1
        };
        enum class SPIInterfaceType : uint_least8_t
        {
            FourWire = 0,
            ThreeWire = 1
        };
        enum class Endianness : uint_least8_t
        {
            LowerAddressLSB = 0,
            LowerAddressMSB = 1
        };

        struct RegisterCtrl3
        {
            bool swReset : 1 = false;
            Endianness _rsvEndianness : 1 = Endianness::LowerAddressLSB; // Don't touch this one.
            bool _rsvIncRegAddrMultiAccess : 1 = true;                   // Don't touch this one.
            SPIInterfaceType spiInterface : 1 = SPIInterfaceType::FourWire;
            IRQPadMode int12PadMode : 1 = IRQPadMode::PushPull;
            InterruptActivationLevel activationLevelIRQ : 1 = InterruptActivationLevel::ActiveHigh;
            BlockDataUpdate blockDataUpdate : 1 = BlockDataUpdate::UntilRead;
            bool rebootMemory : 1 = false;

            constexpr bool operator==(const RegisterCtrl3& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl3) == 1, "Type `RegisterCtrl3` must be packed to one byte.");

        struct RegisterCtrl4
        {
            uint8_t _rsv : 1 = 0;
            bool lpf1GyroEnabled : 1 = true;
            bool disableI2C : 1 = false;
            bool dataReadyMask : 1 = true;
            bool allIRQOnInt1 : 1 = false;
            bool dataReadyDENInt1 : 1 = false;
            bool gyroSleepMode : 1 = false;
            bool extendDENToAccel : 1 = false;

            constexpr bool operator==(const RegisterCtrl4& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl4) == 1, "Type `RegisterCtrl4` must be packed to one byte.");

        enum class RegisterRoundingPattern : uint_least8_t
        {
            NoRounding = 0b000,
            AccelOnly = 0b001,
            GyroOnly = 0b010,
            AccelAndGyro = 0b011,
            FromSensorHub1To6 = 0b100,
            AccelAndFromSensorHub1To6 = 0b101,
            AccelAndGyroAndFromSensorHub1To12 = 0b110,
            AccelAndGyroAndFromSensorHub1To6 = 0b111
        };
        enum class DENActivationLevel : uint_least8_t
        {
            ActiveLow = 0,
            ActiveHigh = 1
        };
        enum class AngularRateSelfTestConfig : uint_least8_t
        {
            Normal = 0b00,
            PositiveSign = 0b01,
            NegativeSign = 0b11
        };
        enum class LinAccelSelfTestConfig : uint_least8_t
        {
            Normal = 0b00,
            PositiveSign = 0b01,
            NegativeSign = 0b10
        };

        struct RegisterCtrl5
        {
            LinAccelSelfTestConfig accelSelfTest : 2 = LinAccelSelfTestConfig::Normal;
            AngularRateSelfTestConfig gyroSelfTest : 2 = AngularRateSelfTestConfig::Normal;
            DENActivationLevel activationLevelDEN : 1 = DENActivationLevel::ActiveLow;
            RegisterRoundingPattern rounding : 3 = RegisterRoundingPattern::NoRounding;

            constexpr bool operator==(const RegisterCtrl5& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl5) == 1, "Type `RegisterCtrl5` must be packed to one byte.");

        enum class TriggerMode : uint_least8_t
        {
            EdgeSensitive = 0b100,
            LevelSensitive = 0b010,
            LevelSensitiveLatched = 0b011,
            LevelSensitiveFIFOEnable = 0b110
        };
        enum class InvertedBool : uint_least8_t
        {
            True = 0,
            False = 1
        };

        struct RegisterCtrl6
        {
            uint8_t lpf1Bandwidth : 2 = RegisterCtrl6::lpf1BandwidthToBits(173);
            uint8_t _rsv : 1 = 0;
            uint8_t accelOffsetWeight : 1 = RegisterCtrl6::accelOffsetWeightToBit(2e-10f);
            InvertedBool accelHighPerfMode : 1 = InvertedBool::True;
            TriggerMode triggerMode : 3 = TriggerMode::LevelSensitive;

            constexpr static uint8_t accelOffsetWeightToBit(float weight)
            {
                if (weight == 2e-6f)
                    return 0b1;
                else // if (weight == 2e-10f)
                    return 0b0;
            }
            constexpr static float accelOffsetBitToWeight(uint8_t bit)
            {
                if (bit == 0b1)
                    return 2e-6f;
                else // if (bit == 0b0)
                    return 2e-10f;
            }

            constexpr static uint8_t lpf1BandwidthToBits(int hz)
            {
                if (hz == 195 || hz == 224 || hz == 234 || hz == 237)
                    return 0b01;
                else if (hz == 155 || hz == 168 || hz == 172 || hz == 173)
                    return 0b10;
                else if (hz == 293 || hz == 505 || hz == 925 || hz == 937)
                    return 0b11;
                else // if (hz == 245 || hz == 315 || hz == 343 || hz == 351)
                    return 0b00;
            }

            constexpr bool operator==(const RegisterCtrl6& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl6) == 1, "Type `RegisterCtrl6` must be packed to one byte.");

        struct RegisterCtrl7Gyro
        {
            uint8_t _rsv2 : 2 = 0;
            bool roundSrcRegisters : 1 = false;
            uint8_t _rsv1 : 1 = 0;
            uint8_t gyroHighPassFilterCutoff : 2 = RegisterCtrl7Gyro::gyroHighPassFilterCutoffToBits(0.016f);
            bool gyroHighPassFilter : 1 = true;
            InvertedBool gyroHighPerfMode : 1 = InvertedBool::True;

            constexpr static uint8_t gyroHighPassFilterCutoffToBits(float hz)
            {
                if (hz == 0.065f)
                    return 0b01;
                else if (hz == 0.26f)
                    return 0b10;
                else if (hz == 1.04f)
                    return 0b11;
                else // if (hz == 0.016f)
                    return 0b00;
            }

            constexpr bool operator==(const RegisterCtrl7Gyro& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl7Gyro) == 1, "Type `RegisterCtrl7Gyro` must be packed to one byte.");

        enum class FilterSelection : uint_least8_t
        {
            LowPass = 0,
            HighPass = 1
        };

        enum class InputPreference : uint_least8_t
        {
            LowLatency = 0,
            LowNoise = 1
        };

        struct RegisterCtrl8Accel
        {
            bool lpf2OnOrientationFeatureSel : 1 = false;
            uint8_t _rsv : 1 = 0;
            FilterSelection _noDirectAccessFilterSelection : 1 = FilterSelection::LowPass;
            InputPreference _noDirectAccessInputComposite : 1 = InputPreference::LowNoise;
            bool highPerfFilterRefMode : 1 = false;
            uint8_t _noDirectAccessAccelLPF2Cutoff : 2 = 0b10;
            bool _noDirectAccessEnableAccelLPF2 : 1 = true;

            constexpr bool operator==(const RegisterCtrl8Accel& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl8Accel) == 1, "Type `RegisterCtrl8Accel` must be packed to one byte.");

        constexpr void setAccelRegisterSettingsForBandwidth(RegisterCtrl1Accel& ctrl1, RegisterCtrl8Accel& ctrl8, FilterSelection sel, float odrMultiplierBw,
                                                            InputPreference pref = InputPreference::LowNoise)
        {
            ctrl8._noDirectAccessFilterSelection = sel;
            if (sel == FilterSelection::LowPass)
            {
                if (odrMultiplierBw == 1.0f / 50.0f || odrMultiplierBw == 1.0f / 100.0f || odrMultiplierBw == 1.0f / 9.0f || odrMultiplierBw == 1.0f / 400.0f)
                {
                    ctrl8._noDirectAccessEnableAccelLPF2 = true;
                    ctrl1._noDirectAccessLPF1Bandwidth = FrequencySelection::HalfOutputDataRate;
                    if (odrMultiplierBw == 1.0f / 50.0f)
                        ctrl8._noDirectAccessAccelLPF2Cutoff = 0b00;
                    else if (odrMultiplierBw == 1.0f / 100.0f)
                        ctrl8._noDirectAccessAccelLPF2Cutoff = 0b01;
                    else if (odrMultiplierBw == 1.0f / 400.0f)
                        ctrl8._noDirectAccessAccelLPF2Cutoff = 0b11;
                    else // if (odrMultiplierBw == 1.0f / 9.0f)
                        ctrl8._noDirectAccessAccelLPF2Cutoff = 0b10;
                    ctrl8._noDirectAccessInputComposite = pref;
                }
                else // if (odrMultiplierBw == 1.0f / 2.0f || odrMultiplierBw == 1.0f / 4.0f)
                {
                    ctrl8._noDirectAccessEnableAccelLPF2 = false;
                    ctrl1._noDirectAccessLPF1Bandwidth = odrMultiplierBw == 1.0f / 4.0f ? FrequencySelection::QuarterOutputDataRate : FrequencySelection::HalfOutputDataRate;
                    ctrl8._noDirectAccessAccelLPF2Cutoff = 0;
                    ctrl8._noDirectAccessInputComposite = InputPreference(0);
                }
            }
            else // if (sel == FilterSelection::HighPass)
            {
                // if (odrMultiplierBw == 1.0f / 4.0f || odrMultiplierBw == 1.0f / 100.0f || odrMultiplierBw == 1.0f / 9.0f || odrMultiplierBw == 1.0f / 400.0f)
                {
                    ctrl8._noDirectAccessEnableAccelLPF2 = false;
                    ctrl1._noDirectAccessLPF1Bandwidth = FrequencySelection::HalfOutputDataRate;
                    if (odrMultiplierBw == 1.0f / 100.0f)
                        ctrl8._noDirectAccessAccelLPF2Cutoff = 0b01;
                    else if (odrMultiplierBw == 1.0f / 9.0f)
                        ctrl8._noDirectAccessAccelLPF2Cutoff = 0b10;
                    else if (odrMultiplierBw == 1.0f / 400.0f)
                        ctrl8._noDirectAccessAccelLPF2Cutoff = 0b11;
                    else // if (odrMultiplierBw == 1.0f / 4.0f)
                        ctrl8._noDirectAccessAccelLPF2Cutoff = 0b00;
                    ctrl8._noDirectAccessInputComposite = InputPreference(0);
                }
            }
        }

        enum class SensorType : uint_least8_t
        {
            Gyro = 0,
            Accel = 1
        };

        struct RegisterCtrl9Accel
        {
            uint8_t _rsv2 : 2 = 0;
            bool magnetometerSoftIronCorrection : 1 = false;
            uint8_t _rsv1 : 1 = 0;
            SensorType stampDENInfo : 1 = SensorType::Gyro;
            bool zStoreDEN : 1 = true;
            bool yStoreDEN : 1 = true;
            bool xStoreDEN : 1 = true;

            constexpr bool operator==(const RegisterCtrl9Accel& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl9Accel) == 1, "Type `RegisterCtrl9Accel` must be packed to one byte.");

        struct RegisterCtrl10
        {
            bool enableSignificantMotionDetect : 1 = false;
            bool stepCounterReset : 1 = false;
            bool enableEmbeddedFeatures : 1 = false;
            bool enableTiltCalc : 1 = false;
            bool enableStepCounter : 1 = false;
            bool enableTimestamp : 1 = false;
            uint8_t _rsv : 1 = 0;
            bool enableWristTilt : 1 = false;

            constexpr bool operator==(const RegisterCtrl10& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl10) == 1, "Type `RegisterCtrl10` must be packed to one byte.");

        enum class DataReadySource : uint_least8_t
        {
            AccelGyroOrStepCounter = 0,
            SensorHub = 1
        };
        enum class SensorHubTriggerSource : uint_least8_t
        {
            AccelGyroDataReady = 0,
            Int2External = 1
        };

        struct RegisterMasterConfig
        {
            bool sensorHubI2CEnabled : 1 = false;
            bool magnetometerHardIronCorrection : 1 = false;
            bool i2cInterfacePassthrough : 1 = false;
            bool i2cAuxiliaryPullup : 1 = false;
            SensorHubTriggerSource startConfig : 1 = SensorHubTriggerSource::AccelGyroDataReady;
            uint8_t _rsv : 1 = 0;
            DataReadySource fifoValidSignal : 1 = DataReadySource::AccelGyroOrStepCounter;
            bool dataReadyOnInt1 : 1 = false;

            constexpr bool operator==(const RegisterMasterConfig& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterMasterConfig) == 1, "Type `RegisterMasterConfig` must be packed to one byte.");

        struct RegisterWakeupSrc
        {
            bool zAxisWakeUpDetected : 1 = false;
            bool yAxisWakeUpDetected : 1 = false;
            bool xAxisWakeUpDetected : 1 = false;
            bool wakeupEventDetected : 1 = false;
            bool sleepEventDetected : 1 = false;
            bool freeFallDetected : 1 = false;
            uint8_t _rsv : 2 = 0;

            constexpr bool operator==(const RegisterWakeupSrc& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterWakeupSrc) == 1, "Type `RegisterWakeupSrc` must be packed to one byte.");

        enum class NumericSign : uint_least8_t
        {
            Positive = 0,
            Negative = 1
        };

        struct RegisterTapSrc
        {
            bool zAxisTapDetected : 1 = false;
            bool yAxisTapDetected : 1 = false;
            bool xAxisTapDetected : 1 = false;
            NumericSign tapSign : 1 = NumericSign::Positive;
            bool doubleTapDetected : 1 = false;
            bool singleTapDetected : 1 = false;
            bool tapDetected : 1 = false;
            uint8_t _rsv : 1 = 0;

            constexpr bool operator==(const RegisterTapSrc& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterTapSrc) == 1, "Type `RegisterTapSrc` must be packed to one byte.");

        struct RegisterOrientationSrc
        {
            bool xAxisUnder : 1 = false;
            bool xAxisOver : 1 = false;
            bool yAxisUnder : 1 = false;
            bool yAxisOver : 1 = false;
            bool zAxisUnder : 1 = false;
            bool zAxisOver : 1 = false;
            bool orientationChangeDetected : 1 = false;
            bool dataReadyDEN : 1 = false;

            constexpr bool operator==(const RegisterOrientationSrc& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterOrientationSrc) == 1, "Type `RegisterOrientationSrc` must be packed to one byte.");

        struct RegisterStatus
        {
            bool accelDataAvail : 1 = false;
            bool gyroDataAvail : 1 = false;
            bool tempDataAvail : 1 = false;
            uint8_t _rsv : 5 = 0;

            constexpr bool operator==(const RegisterStatus& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterStatus) == 1, "Type `RegisterStatus` must be packed to one byte.");

        struct RegisterFIFOStatus2
        {
            uint8_t _rsvUnreadCountHighBits : 3 = 0;
            uint8_t _rsv : 1 = 0;
            bool empty : 1 = false;
            bool overrunOnNextData : 1 = false;
            bool overrunStatus : 1 = false;
            bool watermarkStatus : 1 = false;

            constexpr bool operator==(const RegisterFIFOStatus2& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterFIFOStatus2) == 1, "Type `RegisterFIFOStatus2` must be packed to one byte.");

        struct RegisterFeatureSrc1
        {
            bool sensorHubCommunicationDone : 1 = false;
            bool ironCalibrationDone : 1 = false;
            bool ironCalibrationFail : 1 = false;
            bool stepCounterOverflow : 1 = false;
            bool stepDetected : 1 = false;
            bool tiltDetected : 1 = false;
            bool significantMotionDetected : 1 = false;
            bool stepRecognisedDuringDeltaTime : 1 = false;

            constexpr bool operator==(const RegisterFeatureSrc1& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterFeatureSrc1) == 1, "Type `RegisterFeatureSrc1` must be packed to one byte.");

        struct RegisterFeatureSrc2
        {
            bool wristTiltDetected : 1 = false;
            uint8_t _rsv2 : 2 = 0;
            bool slave0NACK : 1 = false;
            bool slave1NACK : 1 = false;
            bool slave2NACK : 1 = false;
            bool slave3NACK : 1 = false;
            uint8_t _rsv1 : 1 = 0;

            constexpr bool operator==(const RegisterFeatureSrc2& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterFeatureSrc2) == 1, "Type `RegisterFeatureSrc2` must be packed to one byte.");

        struct RegisterWristTiltIRQ
        {
            uint8_t _rsv : 2 = 0;
            bool zTiltNveAxis : 1 = false;
            bool zTiltPveAxis : 1 = false;
            bool yTiltNveAxis : 1 = false;
            bool yTiltPveAxis : 1 = false;
            bool xTiltNveAxis : 1 = false;
            bool xTiltPveAxis : 1 = false;

            constexpr bool operator==(const RegisterWristTiltIRQ& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterWristTiltIRQ) == 1, "Type `RegisterWristTiltIRQ` must be packed to one byte.");

        enum class InactivityBehaviour : uint_least8_t
        {
            Disabled = 0b00,
            AccelLowPower = 0b01,
            AccelLowPowerGyroSleep = 0b10,
            AccelLowPowerGyroPowerDown = 0b11
        };
        enum class FilterType : uint_least8_t
        {
            SlopeFilter = 0,
            HighPassFilter = 1
        };

        struct RegisterTapConfig
        {
            bool latchedIRQ : 1 = false;
            bool zTapDirEnabled : 1 = false;
            bool yTapDirEnabled : 1 = false;
            bool xTapDirEnabled : 1 = false;
            FilterType filterType : 1 = FilterType::SlopeFilter;
            InactivityBehaviour inactivityFn : 2 = InactivityBehaviour::Disabled;
            bool enableIRQs : 1 = false;

            constexpr bool operator==(const RegisterTapConfig& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterTapConfig) == 1, "Type `RegisterTapConfig` must be packed to one byte.");

        struct RegisterTapThreshold
        {
            uint8_t tapThreshold : 5 = 0;
            uint8_t angularThreshold : 2 = RegisterTapThreshold::angularThresholdToBits(80.0f);
            InvertedBool orientationDetectWithoutZ : 1 = InvertedBool::True;

            constexpr static uint8_t angularThresholdToBits(float deg)
            {
                if (deg == 70.0f)
                    return 0b01;
                else if (deg == 60.0f)
                    return 0b10;
                else if (deg == 50.0f)
                    return 0b11;
                else // if (deg == 80.0f)
                    return 0b00;
            }

            constexpr bool operator==(const RegisterTapThreshold& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterTapThreshold) == 1, "Type `RegisterTapThreshold` must be packed to one byte.");

        struct RegisterTapRecognitionConfig
        {
            uint8_t maxDurationForOverthreshold : 2 = 0;
            uint8_t quietDurationAfterTap : 2 = 0;
            uint8_t maxDurationForDoubleTap : 4 = 0;

            constexpr bool operator==(const RegisterTapRecognitionConfig& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterTapRecognitionConfig) == 1, "Type `RegisterTapRecognitionConfig` must be packed to one byte.");

        enum class WakeupTapTrigger : uint_least8_t
        {
            SingleTap = 0,
            Both = 1
        };

        struct RegisterWakeupThreshold
        {
            uint8_t wakeupThreshold : 6 = 0;
            uint8_t _rsv : 1 = 0;
            WakeupTapTrigger wakeupTrigger : 1 = WakeupTapTrigger::SingleTap;

            constexpr bool operator==(const RegisterWakeupThreshold& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterWakeupThreshold) == 1, "Type `RegisterWakeupThreshold` must be packed to one byte.");

        struct RegisterWakeupDuration
        {
            uint8_t durationToEnterSleepMode : 4 = 0;
            uint8_t timestampResolution : 1 = RegisterWakeupDuration::timestampResolutionToBit(0.025f);
            uint8_t wakeupDuration : 2 = 0;
            uint8_t freeFallDurationHighBit : 1 = 0;

            constexpr static uint8_t timestampResolutionToBit(float ms)
            {
                if (ms == 0.025f)
                    return 0b1;
                else // if (ms == 6.4f)
                    return 0b0;
            }
            constexpr static float timestampBitToSecPerLSB(uint8_t bit)
            {
                if (bit == 0b1)
                    return 0.025e-3f;
                else // if (bit == 0b0)
                    return 6.4e-3f;
            }

            constexpr bool operator==(const RegisterWakeupDuration& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterWakeupDuration) == 1, "Type `RegisterWakeupDuration` must be packed to one byte.");

        struct RegisterFreeFall
        {
            uint8_t freeFallThreshold : 3 = RegisterFreeFall::freeFallThresholdForceToBits(156.0f);
            uint8_t freeFallDuration : 5 = 0;

            constexpr static uint8_t freeFallThresholdForceToBits(float milliGee)
            {
                if (milliGee == 219.0f)
                    return 0b001;
                else if (milliGee == 250.0f)
                    return 0b010;
                else if (milliGee == 312.0f)
                    return 0b011;
                else if (milliGee == 344.0f)
                    return 0b100;
                else if (milliGee == 406.0f)
                    return 0b101;
                else if (milliGee == 469.0f)
                    return 0b110;
                else if (milliGee == 500.0f)
                    return 0b111;
                else // if (milliGee == 156.0f)
                    return 0b000;
            }

            constexpr bool operator==(const RegisterFreeFall& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterFreeFall) == 1, "Type `RegisterFreeFall` must be packed to one byte.");

        struct RegisterInt1Config
        {
            bool timerEndCounterRouting : 1 = false;
            bool tiltEventRouting : 1 = false;
            bool orientationChangeRouting : 1 = false;
            bool doubleTapRouting : 1 = false;
            bool freeFallEventRouting : 1 = false;
            bool wakeupEventRouting : 1 = false;
            bool singleTapRouting : 1 = false;
            bool inactivityRouting : 1 = false;

            constexpr bool operator==(const RegisterInt1Config& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterInt1Config) == 1, "Type `RegisterInt1Config` must be packed to one byte.");

        struct RegisterInt2Config
        {
            bool ironCalibrationEndRouting : 1 = false;
            bool tiltEventRouting : 1 = false;
            bool orientationChangeRouting : 1 = false;
            bool doubleTapRouting : 1 = false;
            bool freeFallEventRouting : 1 = false;
            bool wakeupEventRouting : 1 = false;
            bool singleTapRouting : 1 = false;
            bool inactivityRouting : 1 = false;

            constexpr bool operator==(const RegisterInt2Config& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterInt2Config) == 1, "Type `RegisterInt2Config` must be packed to one byte.");

        enum class SerialOperation : uint_least8_t
        {
            Write = 0,
            Read = 1
        };

        struct RegisterSlave0Addr
        {
            SerialOperation serialOperation : 1 = SerialOperation::Write;
            uint8_t addr : 7 = 0;

            constexpr bool operator==(const RegisterSlave0Addr& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterSlave0Addr) == 1, "Type `RegisterSlave0Addr` must be packed to one byte.");

        struct RegisterSlavexAddr
        {
            bool readEnabled : 1 = false;
            uint8_t addr : 7 = 0;

            constexpr bool operator==(const RegisterSlavexAddr& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterSlavexAddr) == 1, "Type `RegisterSlavexAddr` must be packed to one byte.");

        constexpr uint8_t slaveDecimationToBits(int dec)
        {
            switch (dec)
            {
            case 2: return 0b01;
            case 3: return 0b10;
            case 4: return 0b11;
            case 0:
            default: return 0b00;
            }
        }

        struct RegisterSlave0Config
        {
            uint8_t numReadOperations : 3 = 0;
            bool conditionedRead : 1 = false;
            uint8_t externalSensorCount : 2 = RegisterSlave0Config::sensorCountToBits(1);
            uint8_t decimationRate : 2 = slaveDecimationToBits(0);

            constexpr static uint8_t sensorCountToBits(int count)
            {
                switch (count)
                {
                case 2: return 0b01;
                case 3: return 0b10;
                case 4: return 0b11;
                case 1:
                default: return 0b00;
                }
            }

            constexpr bool operator==(const RegisterSlave0Config& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterSlave0Config) == 1, "Type `RegisterSlave0Config` must be packed to one byte.");

        struct RegisterSlave1Config
        {
            uint8_t numReadOperations : 3 = 0;
            uint8_t _rsv : 2 = 0;
            bool writeOnceOnly : 1 = false;
            uint8_t decimationRate : 2 = slaveDecimationToBits(0);

            constexpr bool operator==(const RegisterSlave1Config& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterSlave1Config) == 1, "Type `RegisterSlave1Config` must be packed to one byte.");

        struct RegisterSlavexConfig
        {
            uint8_t numReadOperations : 3 = 0;
            uint8_t _rsv : 3 = 0;
            uint8_t decimationRate : 2 = slaveDecimationToBits(0);

            constexpr bool operator==(const RegisterSlavexConfig& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterSlavexConfig) == 1, "Type `RegisterSlavexConfig` must be packed to one byte.");

        enum class StepCounterDataElaboration : uint_least8_t
        {
            TwoGee = 0,
            FourGee = 1,
        };

        struct RegisterStepCounterMinThreshold
        {
            uint8_t minThreshold : 5 = 0x10;
            uint8_t _rsv : 2 = 0;
            StepCounterDataElaboration dataElaboration : 1 = StepCounterDataElaboration::FourGee;

            constexpr bool operator==(const RegisterStepCounterMinThreshold& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterStepCounterMinThreshold) == 1, "Type `RegisterStepCounterMinThreshold` must be packed to one byte.");

        struct RegisterStepCounterDebounce
        {
            uint8_t debounceStepThreshold : 3 = 0b110;
            uint8_t debounceTime : 5 = RegisterStepCounterDebounce::debounceTimeToBits(13.0f * 80.0f / 1000.0f);

            constexpr static uint8_t debounceTimeToBits(float s)
            {
                return uint8_t(s / (80.0f / 1000.0f) + 0.5f);
            }

            constexpr bool operator==(const RegisterStepCounterDebounce& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterStepCounterDebounce) == 1, "Type `RegisterStepCounterDebounce` must be packed to one byte.");

        struct RegisterWristTiltMask
        {
            uint8_t _rsv : 2 = 0;
            uint8_t zNve : 1 = 0;
            uint8_t zPve : 1 = 0;
            uint8_t yNve : 1 = 0;
            uint8_t yPve : 1 = 0;
            uint8_t xNve : 1 = 1;
            uint8_t xPve : 1 = 1;

            constexpr bool operator==(const RegisterWristTiltMask& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterWristTiltMask) == 1, "Type `RegisterWristTiltMask` must be packed to one byte.");

#pragma pack(pop)

        template <typename T>
        concept ConfigRegisterType = std::same_as<T, RegisterFeatureConfig> || std::same_as<T, RegisterFIFOCtrl1> || std::same_as<T, RegisterFIFOCtrl2> ||
            std::same_as<T, RegisterFIFOCtrl3> || std::same_as<T, RegisterFIFOCtrl4> || std::same_as<T, RegisterFIFOCtrl5> || std::same_as<T, RegisterDataReadyPulseConfigGyro> ||
            std::same_as<T, RegisterInt1Ctrl> || std::same_as<T, RegisterInt2Ctrl> || std::same_as<T, RegisterCtrl1Accel> || std::same_as<T, RegisterCtrl2Gyro> ||
            std::same_as<T, RegisterCtrl3> || std::same_as<T, RegisterCtrl4> || std::same_as<T, RegisterCtrl5> || std::same_as<T, RegisterCtrl6> ||
            std::same_as<T, RegisterCtrl7Gyro> || std::same_as<T, RegisterCtrl8Accel> || std::same_as<T, RegisterCtrl9Accel> || std::same_as<T, RegisterCtrl10> ||
            std::same_as<T, RegisterMasterConfig> || std::same_as<T, RegisterTapConfig> || std::same_as<T, RegisterTapThreshold> || std::same_as<T, RegisterTapRecognitionConfig> ||
            std::same_as<T, RegisterWakeupThreshold> || std::same_as<T, RegisterWakeupDuration> || std::same_as<T, RegisterFreeFall> || std::same_as<T, RegisterInt1Config> ||
            std::same_as<T, RegisterInt2Config> || std::same_as<T, RegisterSlave0Addr> || std::same_as<T, RegisterSlave0Config> || std::same_as<T, RegisterSlave1Config> ||
            std::same_as<T, RegisterStepCounterMinThreshold> || std::same_as<T, RegisterStepCounterDebounce> || std::same_as<T, RegisterWristTiltMask>;

        template <typename T>
        concept DataRegisterType =
            std::same_as<T, RegisterWakeupSrc> || std::same_as<T, RegisterTapSrc> || std::same_as<T, RegisterOrientationSrc> || std::same_as<T, RegisterStatus> ||
            std::same_as<T, RegisterFIFOStatus2> || std::same_as<T, RegisterFeatureSrc1> || std::same_as<T, RegisterFeatureSrc2> || std::same_as<T, RegisterWristTiltIRQ>;

        template <typename T>
        requires ConfigRegisterType<T> || DataRegisterType<T>
        constexpr uint16_t registerAddressOf()
        {
            if constexpr (std::same_as<T, RegisterFeatureConfig>)
                return LSM6DS3::RegAddr::FuncCfgAccess;
            else if constexpr (std::same_as<T, RegisterFIFOCtrl1>)
                return LSM6DS3::RegAddr::FIFOCtrl1;
            else if constexpr (std::same_as<T, RegisterFIFOCtrl2>)
                return LSM6DS3::RegAddr::FIFOCtrl2;
            else if constexpr (std::same_as<T, RegisterFIFOCtrl3>)
                return LSM6DS3::RegAddr::FIFOCtrl3;
            else if constexpr (std::same_as<T, RegisterFIFOCtrl4>)
                return LSM6DS3::RegAddr::FIFOCtrl4;
            else if constexpr (std::same_as<T, RegisterFIFOCtrl5>)
                return LSM6DS3::RegAddr::FIFOCtrl5;
            else if constexpr (std::same_as<T, RegisterDataReadyPulseConfigGyro>)
                return LSM6DS3::RegAddr::DRdyPulseCfgG;
            else if constexpr (std::same_as<T, RegisterInt1Ctrl>)
                return LSM6DS3::RegAddr::Int1Ctrl;
            else if constexpr (std::same_as<T, RegisterInt2Ctrl>)
                return LSM6DS3::RegAddr::Int2Ctrl;
            else if constexpr (std::same_as<T, RegisterCtrl1Accel>)
                return LSM6DS3::RegAddr::Ctrl1XL;
            else if constexpr (std::same_as<T, RegisterCtrl2Gyro>)
                return LSM6DS3::RegAddr::Ctrl2G;
            else if constexpr (std::same_as<T, RegisterCtrl3>)
                return LSM6DS3::RegAddr::Ctrl3C;
            else if constexpr (std::same_as<T, RegisterCtrl4>)
                return LSM6DS3::RegAddr::Ctrl4C;
            else if constexpr (std::same_as<T, RegisterCtrl5>)
                return LSM6DS3::RegAddr::Ctrl5C;
            else if constexpr (std::same_as<T, RegisterCtrl6>)
                return LSM6DS3::RegAddr::Ctrl6C;
            else if constexpr (std::same_as<T, RegisterCtrl7Gyro>)
                return LSM6DS3::RegAddr::Ctrl7G;
            else if constexpr (std::same_as<T, RegisterCtrl8Accel>)
                return LSM6DS3::RegAddr::Ctrl8XL;
            else if constexpr (std::same_as<T, RegisterCtrl9Accel>)
                return LSM6DS3::RegAddr::Ctrl9XL;
            else if constexpr (std::same_as<T, RegisterCtrl10>)
                return LSM6DS3::RegAddr::Ctrl10C;
            else if constexpr (std::same_as<T, RegisterMasterConfig>)
                return LSM6DS3::RegAddr::MasterConfig;
            else if constexpr (std::same_as<T, RegisterTapConfig>)
                return LSM6DS3::RegAddr::TapCfg;
            else if constexpr (std::same_as<T, RegisterTapThreshold>)
                return LSM6DS3::RegAddr::TapThs6D;
            else if constexpr (std::same_as<T, RegisterTapRecognitionConfig>)
                return LSM6DS3::RegAddr::IntDur2;
            else if constexpr (std::same_as<T, RegisterWakeupThreshold>)
                return LSM6DS3::RegAddr::WakeUpThs;
            else if constexpr (std::same_as<T, RegisterWakeupDuration>)
                return LSM6DS3::RegAddr::WakeUpDur;
            else if constexpr (std::same_as<T, RegisterFreeFall>)
                return LSM6DS3::RegAddr::FreeFall;
            else if constexpr (std::same_as<T, RegisterInt1Config>)
                return LSM6DS3::RegAddr::Int1Ctrl;
            else if constexpr (std::same_as<T, RegisterInt2Config>)
                return LSM6DS3::RegAddr::Int2Ctrl;
            else if constexpr (std::same_as<T, RegisterSlave0Addr>)
                return LSM6DS3::RegAddr::Slv0Add;
            else if constexpr (std::same_as<T, RegisterSlave0Config>)
                return LSM6DS3::RegAddr::Slave0Config;
            else if constexpr (std::same_as<T, RegisterSlave1Config>)
                return LSM6DS3::RegAddr::Slave1Config;
            else if constexpr (std::same_as<T, RegisterStepCounterMinThreshold>)
                return LSM6DS3::RegAddr::ConfigPedoThsMin;
            else if constexpr (std::same_as<T, RegisterStepCounterDebounce>)
                return LSM6DS3::RegAddr::PedoDebReg;
            else if constexpr (std::same_as<T, RegisterWristTiltMask>)
                return LSM6DS3::RegAddr::AWristTiltMask;
            else if constexpr (std::same_as<T, RegisterWakeupSrc>)
                return LSM6DS3::RegAddr::WakeUpSrc;
            else if constexpr (std::same_as<T, RegisterTapSrc>)
                return LSM6DS3::RegAddr::TapSrc;
            else if constexpr (std::same_as<T, RegisterOrientationSrc>)
                return LSM6DS3::RegAddr::D6DSrc;
            else if constexpr (std::same_as<T, RegisterStatus>)
                return LSM6DS3::RegAddr::StatusReg;
            else if constexpr (std::same_as<T, RegisterFIFOStatus2>)
                return LSM6DS3::RegAddr::FIFOStatus2;
            else if constexpr (std::same_as<T, RegisterFeatureSrc1>)
                return LSM6DS3::RegAddr::FuncSrc1;
            else if constexpr (std::same_as<T, RegisterFeatureSrc2>)
                return LSM6DS3::RegAddr::FuncSrc2;
            else if constexpr (std::same_as<T, RegisterWristTiltIRQ>)
                return LSM6DS3::RegAddr::WristTiltIA;
            else
                []<bool _false = false>
                {
                    static_assert(_false, "Unknown register type.");
                }();
        }

        struct LiterallyEveryConfigRegister
        {
            RegisterFeatureConfig featureConfig;
            float sensorSyncTimeFrame = 0.0f;
            int sensorSyncResRatioTwoTo = 14;
            RegisterFIFOCtrl1 fifoCtrl1;
            RegisterFIFOCtrl2 fifoCtrl2;
            RegisterFIFOCtrl3 fifoCtrl3;
            RegisterFIFOCtrl4 fifoCtrl4;
            RegisterFIFOCtrl5 fifoCtrl5;
            RegisterDataReadyPulseConfigGyro dataReadyPulseConfigGyro;
            RegisterInt1Ctrl int1Ctrl;
            RegisterInt2Ctrl int2Ctrl;
            RegisterCtrl1Accel ctrl1;
            RegisterCtrl2Gyro ctrl2;
            RegisterCtrl3 ctrl3;
            RegisterCtrl4 ctrl4;
            RegisterCtrl5 ctrl5;
            RegisterCtrl6 ctrl6;
            RegisterCtrl7Gyro ctrl7;
            RegisterCtrl8Accel ctrl8;
            RegisterCtrl9Accel ctrl9;
            RegisterCtrl10 ctrl10;
            RegisterMasterConfig masterConfig;
            RegisterTapConfig tapConfig;
            RegisterTapThreshold tapThreshold;
            RegisterTapRecognitionConfig tapRecognitionConfig;
            RegisterWakeupThreshold wakeupThreshold;
            RegisterWakeupDuration wakeupDuration;
            RegisterFreeFall freeFall;
            RegisterInt1Config int1Config;
            RegisterInt2Config int2Config;
            RegisterSlave0Addr slave0Addr;
            RegisterSlave0Config slave0Config;
            RegisterSlave1Config slave1Config;
            RegisterStepCounterMinThreshold stepCounterMinThreshold;
            RegisterStepCounterDebounce stepCounterDebounce;
            RegisterWristTiltMask wristTiltMask;
        };

        enum class FIFOPatternWordType : uint_least8_t
        {
            GyroX,
            GyroY,
            GyroZ,
            AccelX,
            AccelY,
            AccelZ,
            MagX,
            MagY,
            MagZ,
            TimestampLow,
            TimestampHigh,
            StepCounter,
            Temperature,
            Discard
        };
        struct FIFOData
        {
            union
            {
                float gx;
                float gy;
                float gz;
                float ax;
                float ay;
                float az;
                int16_t mx;
                int16_t my;
                int16_t mz;
                uint16_t timestampLow;
                uint16_t timestampHigh;
                uint16_t stepCounter;
                float temperature;
            };
            FIFOPatternWordType type;
        };
    } // namespace LSM6DS3

    class AccelGyro_LSM6DS3
    {
        SerialInterfaceDevice* device;

        float _durSecPerLSB = std::numeric_limits<float>::quiet_NaN();
        float _dpsPerLSB = std::numeric_limits<float>::quiet_NaN();
        float _geePerLSB = std::numeric_limits<float>::quiet_NaN();
        float _accelOffsetWeight = std::numeric_limits<float>::quiet_NaN();
    public:
        inline AccelGyro_LSM6DS3() = default;

        inline sys::task<HardwareStatus> begin(SerialInterfaceDevice* device, [[maybe_unused]] LSM6DS3::LiterallyEveryConfigRegister conf = LSM6DS3::LiterallyEveryConfigRegister())
        {
            this->device = device;

            HardwareStatus res = this->device->waitReadySync(4, LSM6DS3::TimeoutDuration);
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            uint8_t id;
            _fence_result_co_return(co_await this->deviceID(), id);
            _fence_value_co_return(HardwareStatus::Error, id != LSM6DS3::DeviceID); // Probably faulty.

            // res = co_await this->writeConfigRegister(conf.featureConfig);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->setSensorSyncTimeFrame(conf.sensorSyncTimeFrame);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->setSensorSyncResRatioTwoTo(conf.sensorSyncResRatioTwoTo);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.fifoCtrl1);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.fifoCtrl2);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.fifoCtrl3);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.fifoCtrl4);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.fifoCtrl5);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.dataReadyPulseConfigGyro);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.int1Ctrl);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.int2Ctrl);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.ctrl1);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.ctrl2);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.ctrl3);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.ctrl4);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.ctrl5);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.ctrl6);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.ctrl7);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.ctrl8);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.ctrl9);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.ctrl10);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.masterConfig);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.tapConfig);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.tapThreshold);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.tapRecognitionConfig);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.wakeupThreshold);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.wakeupDuration);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.freeFall);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.int1Config);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.int2Config);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.slave0Addr);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.slave0Config);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.slave1Config);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // // This register doesn't work for some reason.
            // // res = co_await this->writeConfigRegister(conf.stepCounterMinThreshold);
            // // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.stepCounterDebounce);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);
            // res = co_await this->writeConfigRegister(conf.wristTiltMask);
            // _fence_value_co_return(res, res != HardwareStatus::Ok);

            co_return HardwareStatus::Ok;
        }

        inline sys::task<sys::result<uint8_t, HardwareStatus>> deviceID()
        {
            return this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::WhoAmI);
        }

        inline float durSecPerLSB() const noexcept
        {
            return this->_durSecPerLSB;
        }
        inline float dpsPerLSB() const noexcept
        {
            return this->_dpsPerLSB;
        }
        inline float geePerLSB() const noexcept
        {
            return this->_geePerLSB;
        }
        inline float accelOffsetWeight() const noexcept
        {
            return this->_accelOffsetWeight;
        }

        template <LSM6DS3::DataRegisterType T>
        inline sys::task<sys::result<T, HardwareStatus>> readDataRegister()
        {
            return this->device->readMemoryAs<T>(LSM6DS3::registerAddressOf<T>());
        }
        template <LSM6DS3::ConfigRegisterType T>
        inline sys::task<sys::result<T, HardwareStatus>> readConfigRegister()
        {
            return this->device->readMemoryAs<T>(LSM6DS3::registerAddressOf<T>());
        }
        template <LSM6DS3::ConfigRegisterType T>
        inline sys::task<HardwareStatus> writeConfigRegister(T value)
        {
            if constexpr (std::is_same<T, LSM6DS3::RegisterCtrl1Accel>::value)
            {
                HardwareStatus res = co_await this->device->writeMemoryChecked(LSM6DS3::registerAddressOf<T>(), value);
                _fence_value_co_return(res, res != HardwareStatus::Ok);
                this->_geePerLSB = LSM6DS3::RegisterCtrl1Accel::fullScaleConfigToGeePerLSB(value.fullScaleConfig);
                co_return HardwareStatus::Ok;
            }
            else if constexpr (std::is_same<T, LSM6DS3::RegisterCtrl2Gyro>::value)
            {
                HardwareStatus res = co_await this->device->writeMemoryChecked(LSM6DS3::registerAddressOf<T>(), value);
                _fence_value_co_return(res, res != HardwareStatus::Ok);
                this->_dpsPerLSB = LSM6DS3::RegisterCtrl2Gyro::fullScaleConfigToDPSPerLSB(value.fullScaleConfig);
                co_return HardwareStatus::Ok;
            }
            else if constexpr (std::is_same<T, LSM6DS3::RegisterCtrl6>::value)
            {
                HardwareStatus res = co_await this->device->writeMemoryChecked(LSM6DS3::registerAddressOf<T>(), value);
                _fence_value_co_return(res, res != HardwareStatus::Ok);
                this->_accelOffsetWeight = LSM6DS3::RegisterCtrl6::accelOffsetBitToWeight(value.accelOffsetWeight);
                co_return HardwareStatus::Ok;
            }
            else if constexpr (std::is_same<T, LSM6DS3::RegisterWakeupDuration>::value)
            {
                HardwareStatus res = co_await this->device->writeMemoryChecked(LSM6DS3::registerAddressOf<T>(), value);
                _fence_value_co_return(res, res != HardwareStatus::Ok);
                this->_durSecPerLSB = LSM6DS3::RegisterWakeupDuration::timestampBitToSecPerLSB(value.timestampResolution);
                co_return HardwareStatus::Ok;
            }
            else
                co_return co_await this->device->writeMemoryChecked(LSM6DS3::registerAddressOf<T>(), value);
        }

        inline sys::task<sys::result<float, HardwareStatus>> sensorSyncTimeFrame()
        {
            co_return (co_await this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::SensorSyncTimeFrame)) * 0.5f;
        }
        inline sys::task<HardwareStatus> setSensorSyncTimeFrame(float timeFr)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::SensorSyncTimeFrame, uint8_t(uint8_t(timeFr * 2.0f + 0.5f) & 0b00001111));
        }
        inline sys::task<sys::result<int, HardwareStatus>> sensorSyncResRatioTwoTo()
        {
            uint8_t ret;
            _fence_result_co_return(co_await this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::SensorSyncResRatio), ret);
            co_return ret + 11;
        }
        inline sys::task<HardwareStatus> setSensorSyncResRatioTwoTo(int consequent)
        {
            _fence_contract_enforce(consequent >= 11 && consequent <= 14 && "Invalid sync resolution ratio.");
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::SensorSyncResRatio, uint8_t(consequent - 11));
        }

        inline sys::task<sys::result<float, HardwareStatus>> readTemperature()
        {
            i16 ret;
            _fence_result_co_return(co_await this->device->readInt16LSBFirst(LSM6DS3::RegAddr::OutTempL), ret);
            co_return float(ret) / 256.0f + 25.0f;
        }
        inline sys::task<sys::result<sysm::vector3, HardwareStatus>> readGyro()
        {
            uint8_t data[6];
            HardwareStatus res = co_await this->device->readMemory(LSM6DS3::RegAddr::OutXLGyro, data);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            co_return sysm::vector3(sys::s16fb2(data[1], data[0]) * this->_dpsPerLSB, sys::s16fb2(data[3], data[2]) * this->_dpsPerLSB,
                                    sys::s16fb2(data[5], data[4]) * this->_dpsPerLSB);
        }
        inline sys::task<sys::result<sysm::vector3, HardwareStatus>> readAccel()
        {
            uint8_t data[6];
            HardwareStatus res = co_await this->device->readMemory(LSM6DS3::RegAddr::OutXLAccel, data);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            co_return sysm::vector3(sys::s16fb2(data[1], data[0]) * this->_geePerLSB, sys::s16fb2(data[3], data[2]) * this->_geePerLSB,
                                    sys::s16fb2(data[5], data[4]) * this->_geePerLSB);
        }
        inline sys::task<sys::result<sysm::vector3i16, HardwareStatus>> readExternalMagnetometer()
        {
            uint8_t data[6];
            HardwareStatus res = co_await this->device->readMemory(LSM6DS3::RegAddr::OutMagRawXL, data);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            co_return sysm::vector3i16(sys::s16fb2(data[1], data[0]), sys::s16fb2(data[3], data[2]), sys::s16fb2(data[5], data[4]));
        }

        inline sys::task<sys::result<sysm::vector3i8, HardwareStatus>> accelOffset()
        {
            return this->device->readMemoryAs<sysm::vector3i8>(LSM6DS3::RegAddr::XOfsUsr, 3);
        }
        inline sys::task<HardwareStatus> setAccelOffset(sysm::vector3i8 offset)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::XOfsUsr, reinterpret_cast<uint8_t*>(&offset), 3);
        }

        template <uint8_t SensorHub>
        requires (SensorHub > 0 && SensorHub <= 18)
        inline sys::task<sys::result<uint8_t, HardwareStatus>> readSensorHubRegister()
        {
            if constexpr (SensorHub <= 12)
                return this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::SensorHub1Reg + SensorHub - 1);
            else // if constexpr (SensorHub <= 18)
                return this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::SensorHub13Reg + SensorHub - 13);
        }
        template <uint8_t SensorHub, size_t N>
        requires (SensorHub > 0 && SensorHub <= 18)
        inline sys::task<HardwareStatus> readSensorHubRegistersFrom(uint8_t (&data)[N])
        {
            HardwareStatus res;
            if constexpr (SensorHub <= 12)
                res = co_await this->device->readMemory(LSM6DS3::RegAddr::SensorHub1Reg + SensorHub - 1, data, N);
            else // if constexpr (SensorHub <= 18)
                res = co_await this->device->readMemory(LSM6DS3::RegAddr::SensorHub13Reg + SensorHub - 13, data, N);
            co_return res;
        }

        inline sys::task<sys::result<uint16_t, HardwareStatus>> fifoUnreadCount()
        {
            uint8_t data[2];
            HardwareStatus res = co_await this->device->readMemory(LSM6DS3::RegAddr::FIFOStatus1, data);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            data[1] &= 0b00000111;
            co_return uint16_t((uint16_t(data[1]) << 8) | data[0]);
        }
        inline sys::task<sys::result<u16, HardwareStatus>> fifoRecursivePatternAtNextRead()
        {
            return this->device->readUInt16LSBFirst(LSM6DS3::RegAddr::FIFOStatus3);
        }
        inline sys::task<sys::result<u16, HardwareStatus>> readFIFOData()
        {
            return this->device->readUInt16LSBFirst(LSM6DS3::RegAddr::FIFODataOutL);
        }
        template <uint16_t N>
        inline sys::task<sys::result<size_t, HardwareStatus>> readFIFOData(const LSM6DS3::FIFOPatternWordType (&fifoPattern)[N], uint16_t outSize, LSM6DS3::FIFOData* out)
        {
            uint8_t fifoStatus1;
            _fence_result_co_return(co_await this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::FIFOStatus1), fifoStatus1);
            LSM6DS3::RegisterFIFOStatus2 fifoStatus2;
            _fence_result_co_return(co_await this->readDataRegister<LSM6DS3::RegisterFIFOStatus2>(), fifoStatus2);
            uint16_t unreadCount = fifoStatus2.overrunStatus ? UINT16_MAX : uint16_t(uint16_t(uint8_t(reinterpret_cast<uint8_t&>(fifoStatus2) & 0b00000111) << 8) | fifoStatus1);

            u16 pattern;
            _fence_result_co_return(co_await this->fifoRecursivePatternAtNextRead(), pattern);
            uint16_t readCount = std::min(uint16_t(LSM6DS3::FIFOChunkSize), unreadCount);
            _fence_value_co_return(0u, readCount == 0);

            uint8_t data[LSM6DS3::FIFOChunkSize * 2];
            HardwareStatus res = co_await this->device->readMemory(LSM6DS3::RegAddr::FIFODataOutL, std::span(data, readCount * 2));
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            uint16_t outNext = 0;
            for (uint16_t i = 0; i < readCount && outNext < outSize; i++)
            {
                if (pattern < N)
                {
                    switch (fifoPattern[+pattern])
                    {
                    case LSM6DS3::FIFOPatternWordType::GyroX:
                        out[outNext].gx = sys::s16fb2(data[i * 2 + 1], data[i * 2]) * this->_dpsPerLSB;
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::GyroX;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::GyroY:
                        out[outNext].gy = sys::s16fb2(data[i * 2 + 1], data[i * 2]) * this->_dpsPerLSB;
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::GyroY;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::GyroZ:
                        out[outNext].gz = sys::s16fb2(data[i * 2 + 1], data[i * 2]) * this->_dpsPerLSB;
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::GyroZ;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::AccelX:
                        out[outNext].ax = sys::s16fb2(data[i * 2 + 1], data[i * 2]) * this->_geePerLSB;
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::AccelX;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::AccelY:
                        out[outNext].ay = sys::s16fb2(data[i * 2 + 1], data[i * 2]) * this->_geePerLSB;
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::AccelY;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::AccelZ:
                        out[outNext].az = sys::s16fb2(data[i * 2 + 1], data[i * 2]) * this->_geePerLSB;
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::AccelZ;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::MagX:
                        out[outNext].mx = +sys::s16fb2(data[i * 2 + 1], data[i * 2]);
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::MagX;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::MagY:
                        out[outNext].my = +sys::s16fb2(data[i * 2 + 1], data[i * 2]);
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::MagY;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::MagZ:
                        out[outNext].mz = +sys::s16fb2(data[i * 2 + 1], data[i * 2]);
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::MagZ;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::TimestampLow:
                        out[outNext].timestampLow = data[i * 2 + 1];
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::TimestampLow;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::TimestampHigh:
                        out[outNext].timestampHigh = uint16_t(uint16_t(data[i * 2 + 1]) << 8) | data[i * 2];
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::TimestampHigh;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::StepCounter:
                        out[outNext].stepCounter = uint16_t(uint16_t(data[i * 2 + 1]) << 8) | data[i * 2];
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::StepCounter;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::Temperature:
                        out[outNext].temperature = sys::s16fb2(data[i * 2 + 1], data[i * 2]) / 256.0f + 25.0f;
                        out[outNext].type = LSM6DS3::FIFOPatternWordType::Temperature;
                        ++outNext;
                        break;
                    case LSM6DS3::FIFOPatternWordType::Discard:
                    default: break;
                    }
                }

                if (++pattern == N)
                    pattern = 0;
            }
            co_return outNext;
        }
        inline sys::task<sys::result<float, HardwareStatus>> readTimestamp()
        {
            uint8_t data[3];
            HardwareStatus res = co_await this->device->readMemory(LSM6DS3::RegAddr::Timestamp0Reg, data);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            co_return ((uint32_t(data[2]) << 16) | (uint32_t(data[1]) << 8) | data[0]) * this->_durSecPerLSB;
        }
        inline sys::task<sys::result<float, HardwareStatus>> readStepTimestamp()
        {
            co_return (co_await this->device->readUInt16LSBFirst(LSM6DS3::RegAddr::StepTimestampL)) * this->_durSecPerLSB;
        }
        inline sys::task<sys::result<u16, HardwareStatus>> readStepCounter()
        {
            return this->device->readUInt16LSBFirst(LSM6DS3::RegAddr::StepCounterL);
        }

        inline sys::task<sys::result<uint8_t, HardwareStatus>> significantMotionThreshold()
        {
            return this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::SMThs);
        }
        inline sys::task<HardwareStatus> setSignificantMotionThreshold(uint8_t threshold)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::SMThs, threshold);
        }
        inline sys::task<sys::result<float, HardwareStatus>> stepCounterDeltaTime()
        {
            co_return (co_await this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::StepCountDelta) * 1.6384f);
        }
        inline sys::task<HardwareStatus> setStepCounterDeltaTime(float time)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::StepCountDelta, uint8_t(time / 1.6384f + 0.5f));
        }

        inline sys::task<sys::result<sysm::matrix3x3u8, HardwareStatus>> magnetometerSoftIronCorrection()
        {
            return this->device->readMemoryAs<sysm::matrix3x3u8>(LSM6DS3::RegAddr::MagSI_XX, sizeof(uint8_t) * 9);
        }
        inline sys::task<HardwareStatus> setMagnetometerSoftIronCorrection(sysm::matrix3x3u8 correction)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::MagSI_XX, reinterpret_cast<uint8_t*>(&correction), sizeof(uint8_t) * 9);
        }
        inline sys::task<sys::result<sysm::vector3i16, HardwareStatus>> magnetometerHardIronCorrection()
        {
            uint8_t data[6];
            HardwareStatus res = co_await this->device->readMemory(LSM6DS3::RegAddr::MagOffXL, data);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            co_return sysm::vector3i16(sys::s16fb2(data[1], data[0]), sys::s16fb2(data[3], data[2]), sys::s16fb2(data[5], data[4]));
        }

        inline sys::task<sys::result<float, HardwareStatus>> wristTiltLatency()
        {
            co_return (co_await this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::AWristTiltLat) * 0.040f);
        }
        inline sys::task<HardwareStatus> setWristTiltLatency(float latency)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::AWristTiltLat, uint8_t(latency / 0.040f + 0.5f));
        }
        inline sys::task<sys::result<float, HardwareStatus>> absoluteWristTiltThreshold()
        {
            co_return (co_await this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::AWristTiltThs) * 15.625f);
        }
        inline sys::task<HardwareStatus> setAbsoluteWristTiltThreshold(float threshold)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::AWristTiltThs, uint8_t(threshold / 15.625f + 0.5f));
        }

        inline sys::task<sys::result<uint8_t, HardwareStatus>> sensorSyncMasterCommandCode()
        {
            return this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::MasterCmdCode);
        }
        inline sys::task<HardwareStatus> setSensorSyncMasterCommandCode(uint8_t code)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::MasterCmdCode, code);
        }
        inline sys::task<sys::result<uint8_t, HardwareStatus>> sensorSyncSPIErrorCode()
        {
            return this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::SensSyncSPIErrorCode);
        }
        inline sys::task<HardwareStatus> setSensorSyncSPIErrorCode(uint8_t code)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::SensSyncSPIErrorCode, code);
        }

        inline sys::task<sys::result<uint8_t, HardwareStatus>> slave0WriteData()
        {
            return this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::DataWriteSrcModeSubSlv0);
        }
        inline sys::task<HardwareStatus> setSlave0WriteData(uint8_t data)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::DataWriteSrcModeSubSlv0, data);
        }
        template <int SensorIndex>
        requires (SensorIndex >= 1 && SensorIndex <= 3)
        inline sys::task<sys::result<LSM6DS3::RegisterSlavexAddr, HardwareStatus>> slaveI2CAddress()
        {
            return this->device->readMemoryAs<LSM6DS3::RegisterSlavexAddr>(LSM6DS3::RegAddr::Slv0Add + SensorIndex * 3);
        }
        template <int SensorIndex>
        requires (SensorIndex >= 1 && SensorIndex <= 3)
        inline sys::task<HardwareStatus> setSlaveI2CAddress(LSM6DS3::RegisterSlavexAddr value)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::Slv0SubAdd + SensorIndex * 3, value);
        }
        template <int SensorIndex>
        requires (SensorIndex >= 0 && SensorIndex <= 3)
        inline sys::task<sys::result<uint8_t, HardwareStatus>> slaveI2CRegisterAddress()
        {
            return this->device->readMemoryAs<uint8_t>(LSM6DS3::RegAddr::Slv0SubAdd + SensorIndex * 3);
        }
        template <int SensorIndex>
        requires (SensorIndex >= 0 && SensorIndex <= 3)
        inline sys::task<HardwareStatus> setSlaveI2CRegisterAddress(uint8_t address)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::Slv0SubAdd + SensorIndex * 3, address);
        }
        template <int SensorIndex>
        requires (SensorIndex >= 2 && SensorIndex <= 3)
        inline sys::task<sys::result<LSM6DS3::RegisterSlavexConfig, HardwareStatus>> slaveI2CConfig()
        {
            return this->device->readMemoryAs<LSM6DS3::RegisterSlavexConfig>(LSM6DS3::RegAddr::Slave0Config + SensorIndex * 3);
        }
        template <int SensorIndex>
        requires (SensorIndex >= 2 && SensorIndex <= 3)
        inline sys::task<HardwareStatus> setSlaveI2CConfig(LSM6DS3::RegisterSlavexConfig value)
        {
            return this->device->writeMemoryChecked(LSM6DS3::RegAddr::Slave0Config + SensorIndex * 3, value);
        }
    };
} // namespace atmc
