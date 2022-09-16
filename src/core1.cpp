#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"
#include "SEGGER_RTT.h"

extern "C" spin_lock_t* spin_lock;

enum class GainFactor
{
    k1x = 0,
    k3x = 1,
    k6x = 2,
    k9x = 3,
    k18x = 4
};

enum class LEDPulseFrequency
{
    k60kHz = 0x18,
    k70kHz = 0x40,
    k80kHz = 0x28,
    k90kHz = 0x30,
    k100kHz = 0x38,
};

enum class LEDCurrent
{
    kPulse2mA = 0,
    kPulse5mA = 1,
    kPulse10mA = 2,
    kPulse25mA = 3,
    kPulse50mA = 4,
    kPulse75mA = 5,
    kPulse100mA = 6,
    kPulse125mA = 7,
};

enum class ProximityResolution
{
    k8bit = 0x00,
    k9bit = 0x08,
    k10bit = 0x10,
    k11bit = 0x18,
};

enum class ProximityMeasurementRate
{
    k6ms = 1,
    k12ms = 2,
    k25ms = 3,
    k50ms = 4,
    k100ms = 5,
    k200ms = 6,
    k400ms = 7,
};

enum class ColorResolution
{
    k20bit = 0x00,
    k19bit = 0x10,
    k18bit = 0x20,
    k17bit = 0x30,
    k16bit = 0x40,
    k13bit = 0x50,
};

enum class ColorMeasurementRate
{
    k25ms = 0,
    k50ms = 1,
    k100ms = 2,
    k200ms = 3,
    k500ms = 4,
    k1000ms = 5,
    k2000ms = 7,
};

enum class Register
{
    kMainCtrl = 0x00,
    kProximitySensorLED = 0x01,
    kProximitySensorPulses = 0x02,
    kProximitySensorRate = 0x03,
    kLightSensorMeasurementRate = 0x04,
    kLightSensorGain = 0x05,
    kPartID = 0x06,
    kMainStatus = 0x07,
    kProximityData = 0x08,
    kDataInfrared = 0x0A,
    kDataGreen = 0x0D,
    kDataBlue = 0x10,
    kDataRed = 0x13
};

enum class MainCtrlFields
{
    kProximitySensorEnable = 0x01,
    kLightSensorEnable = 0x02,
    kRGBMode = 0x04
};

static constexpr int kAddress = 0x52;
static constexpr int kExpectedPartID = 0xC2;
// static constexpr GainFactor kDefaultGain =
//     GainFactor::k3x;
// static constexpr LEDPulseFrequency kDefaultPulseFreq =
//     LEDPulseFrequency::k60kHz;
// static constexpr LEDCurrent kDefaultLEDCurrent =
//     LEDCurrent::kPulse100mA;
// static constexpr uint8_t kDefaultPulses = 32;
// static constexpr ProximityResolution kDefaultProxRes =
//     ProximityResolution::k11bit;
// static constexpr ProximityMeasurementRate kDefaultProxRate =
//     ProximityMeasurementRate::k50ms;
// static constexpr ColorResolution kDefaultColorRes =
//     ColorResolution::k18bit;
// static constexpr ColorMeasurementRate kDefaultColorRate =
//     ColorMeasurementRate::k100ms;

bool read_i2c_data(i2c_inst_t *i2c, Register reg, uint8_t *buffer, uint8_t readLen)
{
    buffer[0] = static_cast<uint8_t>(reg);
    int result = i2c_write_timeout_us(i2c, kAddress, buffer, 1, true, 25000);
    if (result != 1)
    {
        return false;
    }
    result = i2c_read_timeout_us(i2c, kAddress, buffer, readLen, false, 25000);
    return result == readLen;
}

void init_device(i2c_inst_t *i2c, uint8_t *i2cBuffer)
{
    i2cBuffer[0] = static_cast<uint8_t>(Register::kMainCtrl);
    i2cBuffer[1] = static_cast<uint8_t>(MainCtrlFields::kRGBMode) |
                   static_cast<uint8_t>(MainCtrlFields::kLightSensorEnable) |
                   static_cast<uint8_t>(MainCtrlFields::kProximitySensorEnable);
    i2c_write_timeout_us(i2c, kAddress, i2cBuffer, 2, false, 25000);

    i2cBuffer[0] = static_cast<uint8_t>(Register::kProximitySensorRate);
    i2cBuffer[1] = static_cast<uint8_t>(ProximityResolution::k11bit) | static_cast<uint8_t>(ProximityMeasurementRate::k100ms);
    i2c_write_timeout_us(i2c, kAddress, i2cBuffer, 2, false, 25000);

    i2cBuffer[0] = static_cast<uint8_t>(Register::kProximitySensorPulses);
    i2cBuffer[1] = 32;
    i2c_write_timeout_us(i2c, kAddress, i2cBuffer, 2, false, 25000);

    // i2cBuffer[0] = static_cast<uint8_t>(Register::kLightSensorMeasurementRate);
    // i2cBuffer[1] = static_cast<uint8_t>(kDefaultColorRes) | static_cast<uint8_t>(kDefaultColorRate);
    // i2c_write_timeout_us(i2c, kAddress, i2cBuffer, 2, false, 25000);

    // i2cBuffer[0] = static_cast<uint8_t>(Register::kLightSensorGain);
    // i2cBuffer[1] = static_cast<uint8_t>(kDefaultGain);
    // i2c_write_timeout_us(i2c, kAddress, i2cBuffer, 2, false, 25000);
}

unsigned int values0[10];
unsigned int values1[10];
unsigned int values2[10];

static unsigned int* valuesCurrentWrite = values0;
static unsigned int* valuesCurrentCache = values1;
static unsigned int* valuesCurrentRead = values2;
static bool has_new = false;

extern "C" unsigned int* get_current_values() {
    unsigned int* tmp = valuesCurrentRead;
    uint32_t irq = spin_lock_blocking(spin_lock);
    if (has_new) {
        valuesCurrentRead = valuesCurrentCache;
        valuesCurrentCache = tmp;
        tmp = valuesCurrentRead;
        has_new = false;
    }
    spin_unlock(spin_lock, irq);
    return tmp;
}

extern "C" void core1_main()
{
    // Init I2c 0
    i2c_init(i2c0, 400000);
    gpio_set_function(8, GPIO_FUNC_I2C);
    gpio_set_function(9, GPIO_FUNC_I2C);
    gpio_pull_up(8);
    gpio_pull_up(9);

    // Init I2c 1
    i2c_init(i2c1, 400000);
    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    gpio_pull_up(6);
    gpio_pull_up(7);

    char outputBuffer[512];

    uint8_t i2cBuffer[20];
    bool currentValid0 = false;
    bool currentValid1 = false;
    absolute_time_t loopTime;

    sleep_ms(100);

    init_device(i2c0, i2cBuffer);
    read_i2c_data(i2c0, Register::kMainStatus, i2cBuffer, 1);

    init_device(i2c1, i2cBuffer);
    read_i2c_data(i2c1, Register::kMainStatus, i2cBuffer, 1);

    memset(values0, 0, sizeof(values0));
    memset(values1, 0, sizeof(values1));
    memset(values2, 0, sizeof(values2));

    while (1)
    {
        loopTime = make_timeout_time_ms(100);

        currentValid0 = read_i2c_data(i2c0, Register::kMainStatus, i2cBuffer, 15);
        if (currentValid0)
        {
            if ((i2cBuffer[0] & 0x20) != 0)
            {
                init_device(i2c0, i2cBuffer);
            }
            else
            {
                valuesCurrentWrite[4] = ((i2cBuffer[1] & 0xFF) | ((i2cBuffer[2] & 0xFF) << 8)) & 0x7FF;
                valuesCurrentWrite[3] = ((i2cBuffer[3] & 0xFF) | ((i2cBuffer[4] & 0xFF) << 8) | ((i2cBuffer[5] & 0xFF) << 16)) & 0x03FFFF;
                valuesCurrentWrite[2] = ((i2cBuffer[6] & 0xFF) | ((i2cBuffer[7] & 0xFF) << 8) | ((i2cBuffer[8] & 0xFF) << 16)) & 0x03FFFF;
                valuesCurrentWrite[1] = ((i2cBuffer[9] & 0xFF) | ((i2cBuffer[10] & 0xFF) << 8) | ((i2cBuffer[11] & 0xFF) << 16)) & 0x03FFFF;
                valuesCurrentWrite[0] = ((i2cBuffer[12] & 0xFF) | ((i2cBuffer[13] & 0xFF) << 8) | ((i2cBuffer[14] & 0xFF) << 16)) & 0x03FFFF;
            }
        }

        currentValid1 = read_i2c_data(i2c1, Register::kMainStatus, i2cBuffer, 15);
        if (currentValid1)
        {
            if ((i2cBuffer[0] & 0x20) != 0)
            {
                init_device(i2c1, i2cBuffer);
            }
            else
            {
                valuesCurrentWrite[9] = ((i2cBuffer[1] & 0xFF) | ((i2cBuffer[2] & 0xFF) << 8)) & 0x7FF;
                valuesCurrentWrite[8] = ((i2cBuffer[3] & 0xFF) | ((i2cBuffer[4] & 0xFF) << 8) | ((i2cBuffer[5] & 0xFF) << 16)) & 0x03FFFF;
                valuesCurrentWrite[7] = ((i2cBuffer[6] & 0xFF) | ((i2cBuffer[7] & 0xFF) << 8) | ((i2cBuffer[8] & 0xFF) << 16)) & 0x03FFFF;
                valuesCurrentWrite[6] = ((i2cBuffer[9] & 0xFF) | ((i2cBuffer[10] & 0xFF) << 8) | ((i2cBuffer[11] & 0xFF) << 16)) & 0x03FFFF;
                valuesCurrentWrite[5] = ((i2cBuffer[12] & 0xFF) | ((i2cBuffer[13] & 0xFF) << 8) | ((i2cBuffer[14] & 0xFF) << 16)) & 0x03FFFF;
            }
        }

        snprintf(outputBuffer, sizeof(outputBuffer), "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                 currentValid0, currentValid1, valuesCurrentWrite[0], valuesCurrentWrite[1], valuesCurrentWrite[2], valuesCurrentWrite[3], valuesCurrentWrite[4], valuesCurrentWrite[5], valuesCurrentWrite[6], valuesCurrentWrite[7], valuesCurrentWrite[8], valuesCurrentWrite[9]);

        SEGGER_RTT_printf(0, "%s", outputBuffer);

        unsigned int* tmp = valuesCurrentWrite;
        uint32_t irq = spin_lock_blocking(spin_lock);
        valuesCurrentWrite = valuesCurrentCache;
        valuesCurrentCache = tmp;
        has_new = true;
        spin_unlock(spin_lock, irq);

        sleep_until(loopTime);
    }
}
