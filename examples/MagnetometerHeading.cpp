#include <cstdio>
#include <cstring>
#include <entry.h>
#include <print>

#include <AccelGyro_LSM6DS3.h>
#include <I2CDevice.h>
#include <LanguageSupport.h>
#include <Magnetometer_LIS3MDL.h>
#include <SPIDevice.h>
#include <SerialInterfaceDevice.h>
#include <Task.h>
#include <Vector.h>

using namespace atmc;
using namespace sys;
using namespace sysm;

// void init()
// {
//     []() -> __async(void)
//     {
//         try
//         {
//             _fence_contract_enforce(0 == 1);
//         }
//         catch (const std::exception& ex)
//         {
//             std::println("Caught exception: {}", ex.what());
//         }
//         catch (...)
//         {
//             std::println("Caught unknown exception");
//         }
//         co_return;
//     }();
// }

I2CDevice imuIF { &hi2c1, LSM6DS3::AddrLow, LSM6DS3::RegAddr::Size };
I2CDevice magIF { &hi2c1, LIS3MDL::AddrLow, LIS3MDL::RegAddr::Size };

AccelGyro_LSM6DS3 imu;
Magnetometer_LIS3MDL mag;
vector3 pos(0.0f, 0.0f, 0.0f);
vector3 vel(0.0f, 0.0f, 0.0f);

async imuTest()
{
    try
    {
        _fence_contract_enforce(1 == 0);
    }
    catch (...)
    { }
    printf("beans\n");
    std::println("started");
    _fence_contract_enforce(co_await imu.begin(&imuIF) == HardwareStatus::Ok);
    std::println("IMU detected");
    _fence_contract_enforce(co_await mag.begin(&magIF) == HardwareStatus::Ok);

    std::println("IMU and mag detected");

    LSM6DS3::RegisterFIFOCtrl5 fifoCtrl5;
    fifoCtrl5.outputDataRate = LSM6DS3::outputDataRateFrequencyToBits(6660.0f);
    _fence_contract_enforce(co_await imu.writeConfigRegister(fifoCtrl5) == HardwareStatus::Ok);

    LSM6DS3::RegisterMasterConfig masterConfig;
    masterConfig.fifoValidSignal = LSM6DS3::DataReadySource::AccelGyroOrStepCounter;
    LSM6DS3::RegisterFIFOCtrl2 fifoCtrl2;
    fifoCtrl2.writeMode = LSM6DS3::FIFOWriteMode::OnDataReady;
    fifoCtrl2.fifoTimerStepCounterEnabled = true;
    _fence_contract_enforce(co_await imu.writeConfigRegister(masterConfig) == HardwareStatus::Ok);
    _fence_contract_enforce(co_await imu.writeConfigRegister(fifoCtrl2) == HardwareStatus::Ok);

    LSM6DS3::RegisterCtrl1Accel ctrl1;
    ctrl1.outputDataRate = LSM6DS3::outputDataRateFrequencyToBits(104.0f);
    _fence_contract_enforce(co_await imu.writeConfigRegister(ctrl1) == HardwareStatus::Ok);

    LSM6DS3::RegisterCtrl2Gyro ctrl2;
    ctrl2.outputDataRate = LSM6DS3::outputDataRateFrequencyToBits(104.0f);
    _fence_contract_enforce(co_await imu.writeConfigRegister(ctrl2) == HardwareStatus::Ok);

    LSM6DS3::RegisterFIFOCtrl3 fifoCtrl3;
    fifoCtrl3.accelDecimation = LSM6DS3::decimationFactorToBits(0);
    fifoCtrl3.gyroDecimation = LSM6DS3::decimationFactorToBits(0);
    _fence_contract_enforce(co_await imu.writeConfigRegister(fifoCtrl3) == HardwareStatus::Ok);

    LSM6DS3::RegisterFIFOCtrl4 fifoCtrl4;
    fifoCtrl4.data3Decimation = LSM6DS3::decimationBitsForNoSensor;
    fifoCtrl4.data4Decimation = LSM6DS3::decimationFactorToBits(0);

    LSM6DS3::RegisterCtrl10 ctrl10;
    ctrl10.enableEmbeddedFeatures = true;
    ctrl10.enableTimestamp = true;
    ctrl10.enableStepCounter = true;
    _fence_contract_enforce(co_await imu.writeConfigRegister(ctrl10) == HardwareStatus::Ok);

    _fence_contract_enforce(co_await imu.writeConfigRegister(fifoCtrl4) == HardwareStatus::Ok);
    _fence_contract_enforce(co_await imu.writeConfigRegister(fifoCtrl2) == HardwareStatus::Ok);
    _fence_contract_enforce(co_await imu.writeConfigRegister(fifoCtrl5) == HardwareStatus::Ok);

    auto wakeupDur = (co_await imu.readConfigRegister<LSM6DS3::RegisterWakeupDuration>()).expect();
    wakeupDur.wakeupDuration = LSM6DS3::RegisterWakeupDuration::timestampResolutionToBit(0.025f);
    _fence_contract_enforce(co_await imu.writeConfigRegister(wakeupDur) == HardwareStatus::Ok);

    fifoCtrl5.mode = LSM6DS3::FIFOMode::Continuous;
    _fence_contract_enforce(co_await imu.writeConfigRegister(fifoCtrl5) == HardwareStatus::Ok);

    constexpr LSM6DS3::FIFOPatternWordType pattern[] { LSM6DS3::FIFOPatternWordType::GyroX,         LSM6DS3::FIFOPatternWordType::GyroY,
                                                       LSM6DS3::FIFOPatternWordType::GyroZ,         LSM6DS3::FIFOPatternWordType::AccelX,
                                                       LSM6DS3::FIFOPatternWordType::AccelY,        LSM6DS3::FIFOPatternWordType::AccelZ,
                                                       LSM6DS3::FIFOPatternWordType::TimestampHigh, LSM6DS3::FIFOPatternWordType::TimestampLow,
                                                       LSM6DS3::FIFOPatternWordType::StepCounter };

    u16 readRemainder = LSM6DS3::FIFOChunkSize;
    LSM6DS3::FIFOData data[LSM6DS3::FIFOChunkSize];
    static uint16_t lastTimestamp = 0;
    vector3 curAccel;
    vector3 curVel = vector3(0.0f, 0.0f, 0.0f);
    vector3 curPos = vector3(0.0f, 0.0f, 0.0f);
    while (true)
    {
        printf("Begin cycle...\n");
        auto rA = (co_await imu.readFIFOData(pattern, +readRemainder, &data[+(LSM6DS3::FIFOChunkSize - readRemainder)]));
        if (!rA)
        {
            switch (rA.err())
            {
            case HardwareStatus::Timeout: printf("Timeout\n"); break;
            case HardwareStatus::Error: printf("Error\n"); break;
            case HardwareStatus::Busy: printf("Busy\n"); break;
            default: break;
            }
            continue;
        }
        uint16_t readCount = rA.move();
        int lastFinalIndex = -1;
        for (uint16_t i = 0; i < readCount; i++)
        {
            switch (data[i].type)
            {
            case LSM6DS3::FIFOPatternWordType::GyroZ:
                printf("Gyro: { %f, %f, %f }\n", double(data[i - 2].gx), double(data[i - 1].gy), double(data[i].gz));
                lastFinalIndex = i;
                break;
            case LSM6DS3::FIFOPatternWordType::AccelZ:
                fprintf(stderr, "Accel: { %f, %f, %f }\n", double(data[i - 2].ax), double(data[i - 1].ay), double(data[i].az));
                curAccel = vector3(data[i - 2].ax, data[i - 1].ay, data[i].az);
                lastFinalIndex = i;
                break;
            case LSM6DS3::FIFOPatternWordType::TimestampLow:
                {
                    printf("Timestamp: %f\n", double(uint16_t((data[i - 1].timestampHigh << 8) | data[i].timestampLow) * imu.durSecPerLSB()));
                    if (lastTimestamp != 0)
                    {
                        float dT = (uint16_t((data[i - 1].timestampHigh << 8) | data[i].timestampLow) - lastTimestamp) * imu.durSecPerLSB();
                        printf("DeltaT: %f\n", double(dT));
                        curVel += curAccel * dT;
                        curPos += curVel * dT;
                        printf("pos: { %f, %f, %f }\n", double(curPos.x), double(curPos.y), double(curPos.z));
                        lastFinalIndex = i;
                    }
                    lastTimestamp = (data[i - 1].timestampHigh << 8) | data[i].timestampLow;
                }
                break;
            case LSM6DS3::FIFOPatternWordType::StepCounter:
                printf("Step Counter: %u\n", data[i].stepCounter);
                lastFinalIndex = i;
                break;
            case LSM6DS3::FIFOPatternWordType::GyroX:
            case LSM6DS3::FIFOPatternWordType::GyroY:
            case LSM6DS3::FIFOPatternWordType::AccelX:
            case LSM6DS3::FIFOPatternWordType::AccelY:
            case LSM6DS3::FIFOPatternWordType::TimestampHigh:
            default: break;
            }
        }
        std::memmove(data, &data[lastFinalIndex + 1], unsigned(_as(int, readCount) - lastFinalIndex - 1) * sizeof(LSM6DS3::FIFOData));
        readRemainder = LSM6DS3::FIFOChunkSize - (_as(int, readCount) - lastFinalIndex - 1);
    }
}

void init()
{
    imuTest();
}
void tick()
{ }

// Task<> nested2()
// {
//     printf("nested 2\n");
//     co_return;
// }
// Task<> nested1()
// {
//     printf("nested\n");
//     co_await nested2();
//     printf("nested\n");
//     co_await nested2();
//     printf("nested\n");
//     co_return;
// }
//
// Task<> test()
// {
//     printf("test\n");
//     co_await nested1();
//     printf("test\n");
//     co_await nested1();
//     printf("done\n");
//     co_return;
// }
//
// Task<int> mainLoop()
// {
//     while (true)
//     {
//         auto testTemp = co_await mag.readConfigRegister<LIS3MDL::RegisterCtrl1>();
//         assert(!testTemp || testTemp.takeValue().tempEnabled);
//         if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
//         {
//             co_await mag.calibrateUntil([]
//             {
//                 auto[x, y, z] = mag.debugIronOffsets();
//                 printf("%f %f %f\n", x, y, z);
//                 //printf("%f %f %f\n", mag.offsetSync().takeValue().x * mag.gaussPerLSB, mag.offsetSync().takeValue().y * mag.gaussPerLSB, mag.offsetSync().takeValue().z *
//                 mag.gaussPerLSB); return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET;
//             });
//         }
//         else
//         {
//             auto readRes = co_await mag.read();
//             auto tempRes = co_await mag.readTemperature();
//             if (readRes && tempRes)
//             {
//                 vector3 readVal = readRes.takeValue();
//                 float temp = tempRes.takeValue();
//                 printf("%.4f\t%.4f\t%.4f\t%.4f\n", readVal.x, readVal.y, readVal.z, temp);
//                 printf("heading: %.4f, offx: %f, offy: %f, offz: %f\n", atan2f((readVal.y), (readVal.x)) * 180.0f / float(std::numbers::pi), mag.softIronScale().x,
//                 mag.softIronScale().y, mag.softIronScale().z);
//             }
//             else
//             {
//                 readRes.takeError();
//                 tempRes.takeError();
//             }
//         }
//
//         co_await Task<>::delay(100);
//     }
//     co_return 0;
// }
//
// void init()
// {
//     printf("begin setup\n");
//     []() -> __async(void)
//     {
//         assert(co_await mag.begin(&hi2c2) == HardwareStatus::Ok && "Failed to start / detect magnetometer!");
//         auto stRes = co_await mag.selfTest();
//         assert(stRes && stRes.takeValue() && "Failed self-test!");
//         co_await mag.setOffset(vector3i16 { int16_t(-0.133255f / mag.gaussPerLSB() + 0.5f), int16_t(-0.101987 / mag.gaussPerLSB() + 0.5f), int16_t(-0.468147 /
//         mag.gaussPerLSB() + 0.5f) }); co_await mainLoop();
//     }();
//     printf("end setup\n");
// }
//
// void tick()
// { }
