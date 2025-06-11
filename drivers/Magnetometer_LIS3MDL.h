#pragma once

/// @file Magnetometer_LIS3MDL.cppm
/// @brief Magnetometer driver for the LIS3MDL.

#if __has_include(<config.h>)
    #include <config.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <Result.h>
#include <SerialInterfaceDevice.h>
#include <Task.h>
#include <Vector.h>

namespace atmc
{
    namespace LIS3MDL
    {
        static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "This LIS3MDL driver requires a little-endian byte order.");

        // v Basic data definitions.

        constexpr int AddrLow = 0x1C;
        constexpr int AddrHigh = 0x1E;
        constexpr int DeviceID = 0x3D; // Result of reading register LIS3MDL_REGISTER_WHO_AM_I.

        // ^ Basic data definitions. / Register definitions. v

        /// @brief Register addresses.
        /// @note Static class.
        struct RegAddr
        {
            using RegisterAddress = int;
            constexpr static int Size = sizeof(uint8_t);
            constexpr static RegisterAddress OffsetXRegLM = 0x05;
            constexpr static RegisterAddress OffsetXRegHM = 0x06;
            constexpr static RegisterAddress OffsetYRegLM = 0x07;
            constexpr static RegisterAddress OffsetYRegHM = 0x08;
            constexpr static RegisterAddress OffsetZRegLM = 0x09;
            constexpr static RegisterAddress OffsetZRegHM = 0x0A;
            constexpr static RegisterAddress WhoAmI = 0x0F;
            constexpr static RegisterAddress CtrlReg1 = 0x20;
            constexpr static RegisterAddress CtrlReg2 = 0x21;
            constexpr static RegisterAddress CtrlReg3 = 0x22;
            constexpr static RegisterAddress CtrlReg4 = 0x23;
            constexpr static RegisterAddress CtrlReg5 = 0x24;
            constexpr static RegisterAddress StatusReg = 0x27;
            constexpr static RegisterAddress OutXL = 0x28;
            constexpr static RegisterAddress OutXH = 0x29;
            constexpr static RegisterAddress OutYL = 0x2A;
            constexpr static RegisterAddress OutYH = 0x2B;
            constexpr static RegisterAddress OutZL = 0x2C;
            constexpr static RegisterAddress OutZH = 0x2D;
            constexpr static RegisterAddress TempOutL = 0x2E;
            constexpr static RegisterAddress TempOutH = 0x2F;
            constexpr static RegisterAddress IntCfg = 0x30;
            constexpr static RegisterAddress IntSrc = 0x31;
            constexpr static RegisterAddress IntThsL = 0x32;
            constexpr static RegisterAddress IntThsH = 0x33;

            RegAddr() = delete;
        };

        // ^ Register definitions. / Magnetometer sensitivity constants. v

        constexpr float SensitivityForFS4G = 1.0f / 6842.0f;  // Sensitivity value for 4 gauss full scale.
        constexpr float SensitivityForFS8G = 1.0f / 3421.0f;  // Sensitivity value for 8 gauss full scale.
        constexpr float SensitivityForFS12G = 1.0f / 2281.0f; // Sensitivity value for 12 gauss full scale.
        constexpr float SensitivityForFS16G = 1.0f / 1711.0f; // Sensitivity value for 16 gauss full scale.

        // ^ Magnetometer sensitivity constants. / Default configuration settings. v

#if LIS3MDL_TIMEOUT
        constexpr int TimeoutDuration = LIS3MDL_TIMEOUT;
#else
        constexpr int TimeoutDuration = 10000;
#endif

        // ^ Default configuration settings.

#pragma pack(push, 1)

        /// @brief Operating mode settings.
        /// @see `RegisterCtrl1::operatingModeXY` and `RegisterCtrl4::operatingModeZ`.
        enum class OperatingMode : uint_least8_t
        {
            LowPower = 0b00,
            MediumPerformance = 0b01,
            HighPerformance = 0b10,
            UltraHighPerformance = 0b11
        };

        /// @brief Control register 1 settings.
        struct RegisterCtrl1
        {
            bool selfTest : 1 = false;
            bool fastDataRate : 1 = true;
            uint8_t outputDataRate : 3 = RegisterCtrl1::outputDataRateFreqToBits(80.0f);
            OperatingMode operatingModeXY : 2 = OperatingMode::UltraHighPerformance;
            bool tempEnabled : 1 = true;

            /// @brief Convert output data rate frequency to its bit representation.
            /// @param hz Output data rate frequency in Hz.
            /// @return Bit representation of the output data rate frequency.
            /// @note If the frequency is not supported, the function will default to taking 10 Hz.
            /// @see `RegisterCtrl1::outputDataRate`.
            constexpr static uint8_t outputDataRateFreqToBits(float hz)
            {
                if (hz == 0.625f)
                    return 0b000;
                else if (hz == 1.25f)
                    return 0b001;
                else if (hz == 2.5f)
                    return 0b010;
                else if (hz == 5.0f)
                    return 0b011;
                else if (hz == 20.0f)
                    return 0b101;
                else if (hz == 40.0f)
                    return 0b110;
                else if (hz == 80.0f)
                    return 0b111;
                else // if (hz == 10.0f)
                    return 0b100;
            }

            constexpr bool operator==(const RegisterCtrl1& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl1) == 1, "Type `RegisterCtrl1` must be packed to one byte.");

        /// @brief Control register 2 settings.
        struct RegisterCtrl2
        {
            uint8_t _rsv3 : 2 = 0;
            bool softReset : 1 = false;
            bool reboot : 1 = false;
            uint8_t _rsv2 : 1 = 0;
            uint8_t fullScaleConfig : 2 = RegisterCtrl2::fullScaleConfigRangeToBits(16);
            uint8_t _rsv1 : 1 = 0;

            /// @brief Convert a magnetometer data range to its bit representation. The range of the magnetometer will be [ -rangeUpper, rangeUpper ] * 10^-1 mT.
            /// @param rangeUpper The upper limit of the range in Gauss (milliTesla).
            /// @return Bit representation of the magnetometer data range.
            /// @note If the range is not supported, the function will default to taking 4 Gauss.
            /// @see `RegisterCtrl2::fullScaleConfig`.
            constexpr static uint8_t fullScaleConfigRangeToBits(int rangeUpper)
            {
                switch (rangeUpper)
                {
                case 8: return 0b01;
                case 12: return 0b10;
                case 16: return 0b11;
                case 4:
                default: return 0b00;
                }
            }
            /// @brief Obtain the sensitivity multiplier for the magnetometer data range by its bit representation.
            /// @param fullScaleConfig The bit representation of the magnetometer data range.
            /// @return The sensitivity multiplier for the magnetometer data range.
            /// @note If the bit representation is not supported, the function will default to taking 4 Gauss.
            /// @see `RegisterCtrl2::fullScaleConfig`.
            constexpr static float fullScaleConfigToGaussPerLSB(uint8_t fullScaleConfig)
            {
                switch (fullScaleConfig)
                {
                case RegisterCtrl2::fullScaleConfigRangeToBits(8): return LIS3MDL::SensitivityForFS8G;
                case RegisterCtrl2::fullScaleConfigRangeToBits(12): return LIS3MDL::SensitivityForFS12G;
                case RegisterCtrl2::fullScaleConfigRangeToBits(16): return LIS3MDL::SensitivityForFS16G;
                case RegisterCtrl2::fullScaleConfigRangeToBits(4):
                default: return LIS3MDL::SensitivityForFS4G;
                }
            }

            constexpr bool operator==(const RegisterCtrl2& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl1) == 1, "Type `RegisterCtrl2` must be packed to one byte.");

        /// @brief SPI interface type settings.
        /// @see `RegisterCtrl3::spiInterface`.
        enum class SPIInterfaceType : uint_least8_t
        {
            FourWire = 0,
            ThreeWire = 1
        };
        /// @brief System operating mode settings.
        /// @see `RegisterCtrl3::systemOperatingMode`.
        enum class SystemOperatingMode : uint_least8_t
        {
            ContinuousConversion = 0b00,
            SingleConversion = 0b01,
            PowerDown = 0b11
        };

        /// @brief Control register 3 settings.
        struct RegisterCtrl3
        {
            SystemOperatingMode systemOperatingMode : 2 = SystemOperatingMode::ContinuousConversion;
            SPIInterfaceType spiInterface : 1 = SPIInterfaceType::FourWire;
            uint8_t _rsv2 : 2 = 0;
            bool lowPower : 1 = false;
            uint8_t _rsv1 : 2 = 0;

            constexpr bool operator==(const RegisterCtrl3& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl3) == 1, "Type `RegisterCtrl3` must be packed to one byte.");

        /// @brief Endianness settings.
        /// @warning Do not touch `RegisterCtrl4::_rsvEndianness`. This driver will always assume the lower address is the least significant byte.
        /// @see `RegisterCtrl4::_rsvEndianness`.
        enum class Endianness : uint_least8_t
        {
            LowerAddressLSB = 0,
            LowerAddressMSB = 1
        };

        /// @brief Control register 4 settings.
        struct RegisterCtrl4
        {
            uint8_t _rsv2 : 1 = 0;
            Endianness _rsvEndianness : 1 = Endianness::LowerAddressLSB; // Don't touch this one.
            OperatingMode operatingModeZ : 2 = OperatingMode::UltraHighPerformance;
            uint8_t _rsv1 : 4 = 0;

            constexpr bool operator==(const RegisterCtrl4& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl4) == 1, "Type `RegisterCtrl4` must be packed to one byte.");

        /// @brief Block data update settings.
        /// @see `RegisterCtrl5::blockDataUpdate`.
        enum class BlockDataUpdate : uint_least8_t
        {
            Continuous = 0,
            UntilRead = 1
        };

        /// @brief Control register 5 settings.
        struct RegisterCtrl5
        {
            uint8_t _rsv : 6 = 0;
            BlockDataUpdate blockDataUpdate : 1 = BlockDataUpdate::UntilRead;
            bool fastRead : 1 = false;

            constexpr bool operator==(const RegisterCtrl5& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterCtrl5) == 1, "Type `RegisterCtrl5` must be packed to one byte.");

        /// @brief Status register settings.
        struct RegisterStatus
        {
            bool xNewDataAvail : 1 = false;
            bool yNewDataAvail : 1 = false;
            bool zNewDataAvail : 1 = false;
            bool xyzNewDataAvail : 1 = false;
            bool xOverrun : 1 = false;
            bool yOverrun : 1 = false;
            bool zOverrun : 1 = false;
            bool xyzOverrun : 1 = false;

            constexpr bool operator==(const RegisterStatus& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterStatus) == 1, "Type `RegisterStatus` must be packed to one byte.");

        enum class PolarityIRQ : uint_least8_t
        {
            ActiveLow = 0,
            ActiveHigh = 1
        };
        /// @brief Latch interrupt request settings.
        /// @see `RegisterInterruptConfig::latchIRQ`.
        enum class LatchIRQMode : uint_least8_t
        {
            Latched = 0,
            NotLatched = 1
        };

        /// @brief Interrupt configuration register settings.
        struct RegisterIRQConfig
        {
            bool intPinInterrupt : 1 = false;
            LatchIRQMode latchIRQ : 1 = LatchIRQMode::Latched;
            PolarityIRQ iea : 1 = PolarityIRQ::ActiveLow;
            uint8_t _rsv2 : 1 = 1;
            uint8_t _rsv1 : 1 = 0;
            bool zIRQ : 1 = true;
            bool yIRQ : 1 = true;
            bool xIRQ : 1 = true;

            constexpr bool operator==(const RegisterIRQConfig& other) const noexcept = default;
        };
        static_assert(sizeof(RegisterIRQConfig) == 1, "Type `RegisterIRQConfig` must be packed to one byte.");

        /// @brief Interrupt source register settings.
        struct RegisterIRQSrc
        {
            bool interruptOccurred : 1; // Indeterminate unless obtained.
            bool measRangeMagValOverflow : 1 = false;
            bool zExceedsNveThreshold : 1 = false;
            bool yExceedsNveThreshold : 1 = false;
            bool xExceedsNveThreshold : 1 = false;
            bool zExceedsPveThreshold : 1 = false;
            bool yExceedsPveThreshold : 1 = false;
            bool xExceedsPveThreshold : 1 = false;
        };
        static_assert(sizeof(RegisterIRQSrc) == 1, "Type `RegisterIRQSrc` must be packed to one byte.");

        template <typename T>
        concept ConfigRegisterType = std::same_as<T, RegisterCtrl1> || std::same_as<T, RegisterCtrl2> || std::same_as<T, RegisterCtrl3> || std::same_as<T, RegisterCtrl4> ||
            std::same_as<T, RegisterCtrl5> || std::same_as<T, RegisterIRQConfig>;

        template <typename T>
        concept DataRegisterType = std::same_as<T, RegisterStatus> || std::same_as<T, RegisterIRQSrc>;

        template <typename T>
        requires ConfigRegisterType<T> || DataRegisterType<T>
        constexpr uint16_t registerAddressOf()
        {
            if constexpr (std::same_as<T, RegisterCtrl1>)
                return LIS3MDL::RegAddr::CtrlReg1;
            else if constexpr (std::same_as<T, RegisterCtrl2>)
                return LIS3MDL::RegAddr::CtrlReg2;
            else if constexpr (std::same_as<T, RegisterCtrl3>)
                return LIS3MDL::RegAddr::CtrlReg3;
            else if constexpr (std::same_as<T, RegisterCtrl4>)
                return LIS3MDL::RegAddr::CtrlReg4;
            else if constexpr (std::same_as<T, RegisterCtrl5>)
                return LIS3MDL::RegAddr::CtrlReg5;
            else if constexpr (std::same_as<T, RegisterIRQConfig>)
                return LIS3MDL::RegAddr::IntCfg;
            else if constexpr (std::same_as<T, RegisterStatus>)
                return LIS3MDL::RegAddr::StatusReg;
            else if constexpr (std::same_as<T, RegisterIRQSrc>)
                return LIS3MDL::RegAddr::IntSrc;
            else
                []<bool _false = false>
                {
                    static_assert(_false, "Unknown register type.");
                }();
        }

#pragma pack(pop)
    } // namespace LIS3MDL

    /// @brief Magnetometer driver for the LIS3MDL.
    /// @note
    /// Please read the datasheet and application note provided by STMicroelectronics. It will help you understand the registers and how to use them.
    ///
    /// You must call Magnetometer_LIS3MDL::begin() and verify its successful initialization, once, and _only once_, per instance of this driver, before using any other enclosed
    /// functions.
    ///
    /// #### Calibration:
    ///
    /// The calibration process is as follows:
    ///
    /// 1. Set the magnetometer to the desired settings.
    ///
    /// 2. Call `Magnetometer_LIS3MDL::beginCalibration()` to start the calibration process.
    ///
    /// 3. Repeatedly call `Magnetometer_LIS3MDL::calibrationSingleCycle()`, moving the magnetometer into as many different orientations as possible.
    ///
    /// 4. When you are satisfied with the calibration, call `Magnetometer_LIS3MDL::endCalibration()`.
    ///
    /// 5. The hard-iron calibration data will be set as the offset of the magnetometer.
    ///
    /// Alternatively, you can set the hard-iron calibration data manually, or, for convenience, use the `Magnetometer_LIS3MDL::calibrateUntil()` function.
    /// @warning Single-threaded read/write only.
    class Magnetometer_LIS3MDL final
    {
        SerialInterfaceDevice* device;

        float _gaussPerLSB = std::numeric_limits<float>::quiet_NaN();

        // Iron calibration data.
        float readXMin = std::numeric_limits<float>::quiet_NaN(), readXMax = std::numeric_limits<float>::quiet_NaN();
        float readYMin = std::numeric_limits<float>::quiet_NaN(), readYMax = std::numeric_limits<float>::quiet_NaN();
        float readZMin = std::numeric_limits<float>::quiet_NaN(), readZMax = std::numeric_limits<float>::quiet_NaN();
        float xSI = 1.0f, ySI = 1.0f, zSI = 1.0f;
    public:
        /// @brief Create an empty instance of the LIS3MDL magnetometer driver.
        inline Magnetometer_LIS3MDL() = default;

        /// @brief Start reading from the magnetometer.
        /// @tparam SerialInterfaceHandle `SerialInterfaceHandle == I2C_HandleTypeDef || SerialInterfaceHandle == SPI_HandleTypeDef`
        /// @param handle Serial interface handle.
        /// @param ctrl1 Control register 1.
        /// @param ctrl2 Control register 2.
        /// @param ctrl3 Control register 3.
        /// @param ctrl4 Control register 4.
        /// @param ctrl5 Control register 5.
        /// @return Whether initialization was successful.
        /// @attention Lifetime assumptions! `handle->decltype(*handle)()` < `this->begin(...)` and `this->...` <  `handle->~decltype(*handle)()`.
        inline sys::task<HardwareStatus> begin(SerialInterfaceDevice* device, LIS3MDL::RegisterCtrl1 ctrl1 = LIS3MDL::RegisterCtrl1(),
                                               LIS3MDL::RegisterCtrl2 ctrl2 = LIS3MDL::RegisterCtrl2(), LIS3MDL::RegisterCtrl3 ctrl3 = LIS3MDL::RegisterCtrl3(),
                                               LIS3MDL::RegisterCtrl4 ctrl4 = LIS3MDL::RegisterCtrl4(), LIS3MDL::RegisterCtrl5 ctrl5 = LIS3MDL::RegisterCtrl5())
        {
            this->device = device;

            HardwareStatus res = this->device->waitReadySync(4, LIS3MDL::TimeoutDuration);
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            uint8_t id;
            _fence_result_co_return(co_await this->deviceID(), id);
            _fence_value_co_return(HardwareStatus::Error, id != LIS3MDL::DeviceID); // Probably faulty.

            res = co_await this->writeConfigRegister<LIS3MDL::RegisterCtrl1>(ctrl1);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            res = co_await this->writeConfigRegister<LIS3MDL::RegisterCtrl2>(ctrl2);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            res = co_await this->writeConfigRegister<LIS3MDL::RegisterCtrl3>(ctrl3);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            res = co_await this->writeConfigRegister<LIS3MDL::RegisterCtrl4>(ctrl4);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            res = co_await this->writeConfigRegister<LIS3MDL::RegisterCtrl5>(ctrl5);
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            co_return HardwareStatus::Ok;
        }

        /// @brief Obtain the sensitivity multiplier for the current magnetometer data range.
        /// @return The sensitivity multiplier, in Gauss / LSB.
        inline float gaussPerLSB() const noexcept
        {
            return this->_gaussPerLSB;
        }
        /// @brief Obtain the iron calibration center currently held during calibration.
        /// @return The iron center, in Gauss.
        /// @attention Lifetime assumptions! `this->beginCalibration()` < `this->hardIronOffsets()` and `this->this->hardIronOffsets()` <  `this->endCalibration()`.
        inline sysm::vector3 debugIronOffsets() const noexcept
        {
            return sysm::vector3((this->readXMax + this->readXMin) * 0.5f, (this->readYMax + this->readYMin) * 0.5f, (this->readZMax + this->readZMin) * 0.5f);
        }

        /// @brief Obtain the value of a data register.
        /// @tparam T `T is LIS3MDL::DataRegisterType`
        /// @return The value of the data register, or an error code.
        template <LIS3MDL::DataRegisterType T>
        inline sys::task<sys::result<T, HardwareStatus>> readDataRegister()
        {
            return this->device->readMemoryAs<T>(LIS3MDL::registerAddressOf<T>());
        }
        /// @brief Obtain the value of a config register.
        /// @tparam T `T is LIS3MDL::ConfigRegisterType`
        /// @return The value of the config register, or an error code.
        template <LIS3MDL::ConfigRegisterType T>
        inline sys::task<sys::result<T, HardwareStatus>> readConfigRegister()
        {
            return this->device->readMemoryAs<T>(LIS3MDL::registerAddressOf<T>());
        }
        /// @brief Write a value to a config register.
        /// @tparam T `T is LIS3MDL::ConfigRegisterType`
        /// @param value The value to write to the config register.
        /// @return Whether the operation was successful.
        template <LIS3MDL::ConfigRegisterType T>
        inline sys::task<HardwareStatus> writeConfigRegister(T value)
        {
            if constexpr (std::is_same<T, LIS3MDL::RegisterCtrl2>::value)
            {
                HardwareStatus res = co_await this->device->writeMemoryChecked<T>(LIS3MDL::registerAddressOf<T>(), value);
                _fence_value_co_return(res, res != HardwareStatus::Ok);
                this->_gaussPerLSB = LIS3MDL::RegisterCtrl2::fullScaleConfigToGaussPerLSB(value.fullScaleConfig);
                co_return HardwareStatus::Ok;
            }
            else
                co_return co_await this->device->writeMemoryChecked<T>(LIS3MDL::registerAddressOf<T>(), value);
        }

        /// @brief Read the device ID.
        /// @return The device ID, which should be `LIS3MDL_DEVICE_ID`, or an error code.
        inline sys::task<sys::result<uint8_t, HardwareStatus>> deviceID()
        {
            return this->device->readMemoryAs<uint8_t>(LIS3MDL::RegAddr::WhoAmI);
        }
        inline sys::task<sys::result<bool, HardwareStatus>> selfTest()
        {
            LIS3MDL::RegisterCtrl1 oldCtrl1;
            _fence_result_co_return(co_await this->readConfigRegister<LIS3MDL::RegisterCtrl1>(), oldCtrl1);
            LIS3MDL::RegisterCtrl2 oldCtrl2;
            _fence_result_co_return(co_await this->readConfigRegister<LIS3MDL::RegisterCtrl2>(), oldCtrl2);
            LIS3MDL::RegisterCtrl3 oldCtrl3;
            _fence_result_co_return(co_await this->readConfigRegister<LIS3MDL::RegisterCtrl3>(), oldCtrl3);

            LIS3MDL::RegisterCtrl1 ctrl1;
            reinterpret_cast<uint8_t&>(ctrl1) = 0x1C;
            HardwareStatus res = co_await this->writeConfigRegister<LIS3MDL::RegisterCtrl1>(ctrl1);
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            LIS3MDL::RegisterCtrl2 ctrl2;
            reinterpret_cast<uint8_t&>(ctrl2) = 0x40;
            res = co_await this->writeConfigRegister<LIS3MDL::RegisterCtrl2>(ctrl2);
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            co_await sys::task<>::delay(20);

            LIS3MDL::RegisterCtrl3 ctrl3;
            reinterpret_cast<uint8_t&>(ctrl3) = 0x00;
            res = co_await this->writeConfigRegister<LIS3MDL::RegisterCtrl3>(ctrl3);
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            co_await sys::task<>::delay(20);

            auto readOut = [this]() -> sys::task<sys::result<sysm::vector3, HardwareStatus>>
            {
                LIS3MDL::RegisterStatus status;
                do
                {
                    _fence_result_co_return(co_await this->readDataRegister<LIS3MDL::RegisterStatus>(), status);
                }
                while (status.xyzNewDataAvail == false);

                [[maybe_unused]] sysm::vector3i16 _;
                _fence_result_co_return(co_await this->readRaw(), _);

                sysm::vector3 out = sysm::vector3 { 0.0f, 0.0f, 0.0f };

                for (int i = 0; i < 5; i++)
                {
                    LIS3MDL::RegisterStatus status;
                    do
                    {
                        _fence_result_co_return(co_await this->readDataRegister<LIS3MDL::RegisterStatus>(), status);
                    }
                    while (status.xyzNewDataAvail == false);

                    sysm::vector3 read;
                    _fence_result_co_return(co_await this->read(), read);
                    out.x += read.x;
                    out.y += read.y;
                    out.z += read.z;
                }

                out.x /= 5.0f;
                out.y /= 5.0f;
                out.z /= 5.0f;

                co_return out;
            };

            sysm::vector3 outNoST;
            _fence_result_co_return(co_await readOut(), outNoST);

            reinterpret_cast<uint8_t&>(ctrl1) = 0x1D;
            res = co_await this->writeConfigRegister<LIS3MDL::RegisterCtrl1>(ctrl1);
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            co_await sys::task<>::delay(60);

            sysm::vector3 outST;
            _fence_result_co_return(co_await readOut(), outST);

            bool pass = 1.0f <= std::abs(outST.x - outNoST.x) && std::abs(outST.x - outNoST.x) <= 3.0f && 1.0f <= std::abs(outST.y - outNoST.y) &&
                std::abs(outST.y - outNoST.y) <= 3.0f && 0.1f <= std::abs(outST.z - outNoST.z) && std::abs(outST.z - outNoST.z) <= 1.0f;

            res = co_await this->writeConfigRegister<LIS3MDL::RegisterCtrl1>(oldCtrl1);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            res = co_await this->writeConfigRegister<LIS3MDL::RegisterCtrl2>(oldCtrl2);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            res = co_await this->writeConfigRegister<LIS3MDL::RegisterCtrl3>(oldCtrl3);
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            co_return pass;
        }

        /// @brief Read the offset values of the magnetometer.
        /// @return The offset values, or an error code.
        inline sys::task<sys::result<sysm::vector3i16, HardwareStatus>> offset()
        {
            uint8_t data[6];
            HardwareStatus res = co_await this->device->readMemory(LIS3MDL::RegAddr::OffsetXRegLM, data);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            co_return sysm::vector3i16(sys::s16fb2(data[1], data[0]), sys::s16fb2(data[3], data[2]), sys::s16fb2(data[5], data[4]));
        }
        /// @brief Set the hard-iron offset values of the magnetometer.
        /// @param offset Offset values.
        /// @return Whether the operation was successful.
        inline sys::task<HardwareStatus> setOffset(sysm::vector3i16 offset)
        {
            u8 data[6] { sys::lbfs16(offset.x), sys::hbfs16(offset.x), sys::lbfs16(offset.y), sys::hbfs16(offset.y), sys::lbfs16(offset.z), sys::hbfs16(offset.z) };
            co_return co_await this->device->writeMemoryChecked(LIS3MDL::RegAddr::OffsetXRegLM, data);
        }
        inline sysm::vector3 softIronScale() const noexcept
        {
            return sysm::vector3(this->xSI, this->ySI, this->zSI);
        }
        /// @brief Set the soft-iron scale factors of the magnetometer.
        /// @param si Scale factors.
        inline void setSoftIronScale(sysm::vector3 si)
        {
            this->xSI = si.x;
            this->ySI = si.y;
            this->zSI = si.z;
        }

        /// @brief Read the raw magnetometer data.
        /// @return The magnetometer data, or an error code.
        inline sys::task<sys::result<sysm::vector3i16, HardwareStatus>> readRaw()
        {
            uint8_t data[6];
            HardwareStatus res = co_await this->device->readMemory(LIS3MDL::RegAddr::OutXL, data);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            co_return sysm::vector3i16(sys::s16fb2(data[1], data[0]), sys::s16fb2(data[3], data[2]), sys::s16fb2(data[5], data[4]));
        }
        /// @brief Read the magnetometer data.
        /// @return The magnetometer data, or an error code.
        inline sys::task<sys::result<sysm::vector3, HardwareStatus>> read()
        {
            sysm::vector3i16 raw;
            _fence_result_co_return(co_await this->readRaw(), raw);
            co_return sysm::vector3(raw.x * this->_gaussPerLSB * this->xSI, raw.y * this->_gaussPerLSB * this->ySI, raw.z * this->_gaussPerLSB * this->zSI);
        }
        /// @brief Read the onboard temperature data.
        /// @return The temperature data, in degC, or an error code.
        inline sys::task<sys::result<float, HardwareStatus>> readTemperature()
        {
            uint8_t data[2];
            HardwareStatus res = co_await this->device->readMemory(LIS3MDL::RegAddr::TempOutL, data);
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            co_return float(sys::s16fb2(data[1], data[0])) / 8.0f + 25.0f;
        }

        /// @brief Read the interrupt threshold value.
        /// @return The interrupt threshold value, or an error code.
        inline sys::task<sys::result<u16, HardwareStatus>> interruptThreshold()
        {
            return this->device->readUInt16LSBFirst(LIS3MDL::RegAddr::IntThsL);
        }
        /// @brief Set the interrupt threshold value.
        /// @param threshold Threshold value.
        /// @return Whether the operation was successful.
        inline sys::task<HardwareStatus> setInterruptThreshold(uint16_t threshold)
        {
            threshold &= 0b0111111111111111;
            uint8_t* inputData = reinterpret_cast<uint8_t*>(&threshold);
            uint8_t data[2] { inputData[1], inputData[0] };
            co_return co_await this->device->writeMemoryChecked(LIS3MDL::RegAddr::IntThsL, data);
        }

        /// @brief To start calibrating the magnetometer, call this function first.
        /// @return Whether the operation was successful.
        inline sys::task<HardwareStatus> beginCalibration()
        {
            HardwareStatus res = co_await this->setOffset(sysm::vector3i16 { 0, 0, 0 });
            _fence_value_co_return(res, res != HardwareStatus::Ok);

            this->readXMin = std::numeric_limits<float>::quiet_NaN();
            this->readYMin = std::numeric_limits<float>::quiet_NaN();
            this->readZMin = std::numeric_limits<float>::quiet_NaN();
            this->readXMax = std::numeric_limits<float>::quiet_NaN();
            this->readYMax = std::numeric_limits<float>::quiet_NaN();
            this->readZMax = std::numeric_limits<float>::quiet_NaN();
            this->xSI = 1.0f;
            this->ySI = 1.0f;
            this->zSI = 1.0f;

            co_return HardwareStatus::Ok;
        }
        /// @brief Run this function multiple times (strictly, at least once) to calibrate the magnetometer.
        /// @return Whether the operation was successful.
        inline sys::task<HardwareStatus> calibrationSingleCycle()
        {
            sysm::vector3 readVal;
            _fence_result_co_return(co_await this->read(), readVal);
            if (std::isnan(this->readXMin) || std::isnan(this->readYMin) || std::isnan(this->readZMin) || std::isnan(this->readXMax) || std::isnan(this->readYMax) ||
                std::isnan(this->readZMax))
            {
                // Initial values.
                this->readXMin = readVal.x;
                this->readYMin = readVal.y;
                this->readZMin = readVal.z;
                this->readXMax = readVal.x;
                this->readYMax = readVal.y;
                this->readZMax = readVal.z;
            }
            else
            {
                this->readXMin = std::min(this->readXMin, readVal.x);
                this->readYMin = std::min(this->readYMin, readVal.y);
                this->readZMin = std::min(this->readZMin, readVal.z);
                this->readXMax = std::max(this->readXMax, readVal.x);
                this->readYMax = std::max(this->readYMax, readVal.y);
                this->readZMax = std::max(this->readZMax, readVal.z);
            }
            co_return HardwareStatus::Ok;
        }
        /// @brief To end calibrating the magnetometer, call this function when you are done.
        /// @return Whether the operation was successful.
        inline sys::task<HardwareStatus> endCalibration()
        {
            float xDiff = this->readXMax - this->readXMin;
            float yDiff = this->readYMax - this->readYMin;
            float zDiff = this->readZMax - this->readZMin;
            float avgDiff = (xDiff + yDiff + zDiff) / 3.0f;
            this->xSI = avgDiff / xDiff;
            this->ySI = avgDiff / yDiff;
            this->zSI = avgDiff / zDiff;
            co_return co_await this->setOffset(sysm::vector3i16 { int16_t((this->readXMax + this->readXMin) * 0.5f / this->_gaussPerLSB + 0.5f),
                                                                  int16_t((this->readYMax + this->readYMin) * 0.5f / this->_gaussPerLSB + 0.5f),
                                                                  int16_t((this->readZMax + this->readZMin) * 0.5f / this->_gaussPerLSB + 0.5f) });
        }

        /// @brief Useful function to calibrate the magnetometer until a certain condition is met.
        /// @tparam Pred `Pred()() -> bool || co_await Pred()() -> bool`.
        /// @param pred A predicate.
        /// @return Whether the operation was successful.
        template <typename Pred>
        inline sys::task<HardwareStatus> calibrateUntil(Pred&& pred)
        {
            HardwareStatus res = co_await this->beginCalibration();
            _fence_value_co_return(res, res != HardwareStatus::Ok);
            if constexpr (!std::convertible_to<decltype(pred()), bool>)
            {
                do
                {
                    HardwareStatus res = co_await this->calibrationSingleCycle();
                    _fence_value_co_return(res, res != HardwareStatus::Ok);
                }
                while (!co_await pred());
            }
            else
            {
                do
                {
                    HardwareStatus res = co_await this->calibrationSingleCycle();
                    _fence_value_co_return(res, res != HardwareStatus::Ok);
                }
                while (!pred());
            }
            co_return co_await this->endCalibration();
        }
    };
} // namespace atmc
