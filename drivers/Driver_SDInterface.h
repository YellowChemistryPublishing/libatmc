#pragma once

#include <coroutine>
#include <cstdint>
#include <cxxutil.h>
#include <cxxutil.hpp>
#include <span>

#include <Task.h>
#include <SPIDevice.h>

namespace atmc
{
    namespace SDI
    {
        static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "This SD card interface driver requires a little-endian byte order.");

        using CmdRegAddress = int;

        struct Cmd
        {
            constexpr static CmdRegAddress GoIdleState = 0;
            constexpr static CmdRegAddress SendOpCond = 1;
            constexpr static CmdRegAddress SendIfCond = 8;
            constexpr static CmdRegAddress SendCSD = 9;
            constexpr static CmdRegAddress SendCID = 10;
            constexpr static CmdRegAddress StopTransmission = 12;
            constexpr static CmdRegAddress SendStatus = 13;
            constexpr static CmdRegAddress SetBlockLen = 16;
            constexpr static CmdRegAddress ReadSingleBlock = 17;
            constexpr static CmdRegAddress ReadMultipleBlock = 18;
            constexpr static CmdRegAddress WriteBlock = 24;
            constexpr static CmdRegAddress WriteMultipleBlock = 25;
            constexpr static CmdRegAddress AppCmd = 55;
            constexpr static CmdRegAddress ReadOCR = 58;

            Cmd() = delete;
        };
        
        struct ACmd
        {
            constexpr static CmdRegAddress SDStatus = 13;
            constexpr static CmdRegAddress SendOpCond = 41;

            ACmd() = delete;
        };
        
        #pragma pack(push, 1)

        struct ResponseR1
        {
            bool idle : 1 = false;
            bool eraseReset : 1 = false;
            bool illCmd : 1 = false;
            bool cmdCRCErr : 1 = false;
            bool eraseSeqErr : 1 = false;
            bool addrErr : 1 = false;
            bool paramErr : 1 = false;
            uint8_t _rsv : 1 = 0;

            constexpr bool operator==(const ResponseR1& other) const = default;

            constexpr bool isErrored() const
            {
                return this->illCmd || this->cmdCRCErr || this->eraseSeqErr || this->addrErr || this->paramErr;
            }
        };
        static_assert(sizeof(ResponseR1) == 1, "Type `ResponseR1` must be packed to one byte.");

        struct Response
        {
            union
            {
                uint8_t r1Raw = 0;
                SDI::ResponseR1 r1;
            };
            uint32_t ocr;

            constexpr Response() = default;
        };
        static_assert(sizeof(Response) == 5, "Type `Response` must be packed to five byte.");

        #pragma pack(pop)

        constexpr uint8_t cmdIndexToCmd(uint8_t index)
        {
            return 0b01000000 | (index & 0b00111111);
        }
    }

    class Driver_SDInterface
    {
        SPIDevice* device;
        uint8_t crcTable[256];
 
        inline void generateCRCTable()
        {
            uint8_t crcPoly = 0x89;
            for (size_t i = 0; i < 256; i++)
            {
                this->crcTable[i] = (i & 0x80) ? i ^ crcPoly : i;
                for (size_t j = 1; j < 8; j++)
                {
                    this->crcTable[i] <<= 1;
                    if (this->crcTable[i] & 0x80)
                        this->crcTable[i] ^= crcPoly;
                }
            }
        }
        inline uint8_t crcAdd(uint8_t crc, uint8_t messageByte)
        {
            return this->crcTable[(crc << 1) ^ messageByte];
        }
        inline uint8_t crcOf(uint8_t message[], size_t length)
        {
            uint8_t crc = 0;
            for (size_t i = 0; i < length; i++)
                crc = this->crcAdd(crc, message[i]);
            return crc;
        }

        inline sys::Task<HardwareStatus> waitBytesUnchecked(size_t count)
        {
            uint8_t block[16];
            while (count > 16)
            {
                HardwareStatus res = co_await this->device->readMemoryUnchecked(block);
                __fence_value_co_return(res, res != HardwareStatus::Ok);
                count -= 16;
            }
            co_return co_await this->device->readMemoryUnchecked(std::span(block, count));
        }

        inline sys::Task<sys::Result<SDI::Response, HardwareStatus>> sendCommandUnchecked(uint8_t cmd, uint32_t arg, bool acmd = false)
        {
            if (acmd)
            {
                uint8_t preData[6] = { 0b01000000 | SDI::Cmd::AppCmd, 0, 0, 0, 0, 0 };
                preData[5] = (this->crcOf(preData, 5) << 1) | 1;
                HardwareStatus res = co_await this->device->writeMemoryUnchecked(preData);
                __fence_value_co_return(res, res != HardwareStatus::Ok);
            }

            uint8_t _cmd = 0b01000000 | (cmd & 0b00111111);
            uint8_t data[6];
            data[0] = _cmd;
            data[1] = (arg >> 24) & 0xFF;
            data[2] = (arg >> 16) & 0xFF;
            data[3] = (arg >> 8) & 0xFF;
            data[4] = arg & 0xFF;
            data[5] = (this->crcOf(data, 5) << 1) | 1;

            HardwareStatus res = co_await this->device->writeMemoryUnchecked(data);
            __fence_value_co_return(res, res != HardwareStatus::Ok);
            
            co_return co_await this->waitBytesUnchecked(1);
        }
    public:
        /// @brief 
        /// @param device 
        /// @attention Lifetime assumptions!
        /// ```cpp
        /// (&*device)->decltype(*&*device)();
        /// ...
        /// this->Driver_SDInterface(device);
        /// ...
        /// this->~Driver_SDInterface();
        /// ...
        /// (&*device)->~decltype(*&*device)();
        /// ```
        inline Driver_SDInterface(sys::FencedPointer<SPIDevice> device) : device(&*device /* Contract implied: `device != nullptr`. */)
        {
            this->generateCRCTable();
        }

        /// @brief 
        /// @return 
        /// @attention Lifetime assumptions!
        /// ```cpp
        /// this->device->begin();
        /// ...
        /// this->begin();
        /// ...
        /// this->device->~decltype(*this->device)();
        /// ```
        inline sys::Task<HardwareStatus> begin()
        {
            // `this->device` is technically not-owned, but just to be safe...
            this->device->begin();
            HardwareStatus res = this->device->waitReadySync(4, sys::Task<>::MaxDelay);
            __fence_value_co_return(res, res != HardwareStatus::Ok);

            this->generateCRCTable();
            
        }
    };
}