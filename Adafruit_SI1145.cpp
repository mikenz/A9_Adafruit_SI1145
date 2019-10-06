/***************************************************
  This is a library for the Si1145 UV/IR/Visible Light Sensor

  Designed specifically to work with the Si1145 sensor in the
  adafruit shop
  ----> https://www.adafruit.com/products/1777

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_SI1145.h"
#include "api_debug.h"
#include "api_os.h"

Adafruit_SI1145::Adafruit_SI1145(uint8_t addr) : _addr(addr)
{
}

bool Adafruit_SI1145::begin(I2C_ID_t i2c)
{
    _i2c = i2c;
    _i2cConfig.freq = I2C_FREQ_100K;
    I2C_Init(_i2c, _i2cConfig);
    uint8_t id = read8(SI1145_REG_PARTID);
    if (id != 0x45)
        return false; // look for SI1145

    reset();

    /***********************************/
    // enable UVindex measurement coefficients!
    write8(SI1145_REG_UCOEFF0, 0x29);
    write8(SI1145_REG_UCOEFF1, 0x89);
    write8(SI1145_REG_UCOEFF2, 0x02);
    write8(SI1145_REG_UCOEFF3, 0x00);

    // enable UV sensor
    writeParam(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |
                                        SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS |
                                        SI1145_PARAM_CHLIST_ENPS1);
    // enable interrupt on every sample
    write8(SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);
    write8(SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);

    /****************************** Prox Sense 1 */

    // program LED current
    write8(SI1145_REG_PSLED21, 0x03); // 20mA for LED 1 only
    writeParam(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
    // prox sensor #1 uses LED #1
    writeParam(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
    // fastest clocks, clock div 1
    writeParam(SI1145_PARAM_PSADCGAIN, 0);
    // take 511 clocks to measure
    writeParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    // in prox mode, high range
    writeParam(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE |
                                           SI1145_PARAM_PSADCMISC_PSMODE);

    writeParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);
    // fastest clocks, clock div 1
    writeParam(SI1145_PARAM_ALSIRADCGAIN, 0);
    // take 511 clocks to measure
    writeParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    // in high range mode
    writeParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);

    // fastest clocks, clock div 1
    writeParam(SI1145_PARAM_ALSVISADCGAIN, 0);
    // take 511 clocks to measure
    writeParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    // in high range mode (not normal signal)
    writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);

    /************************/

    // measurement rate for auto
    write8(SI1145_REG_MEASRATE0, 0xFF); // 255 * 31.25uS = 8ms

    // auto run
    write8(SI1145_REG_COMMAND, SI1145_PSALS_AUTO);

    return true;
}

void Adafruit_SI1145::reset()
{
    write8(SI1145_REG_MEASRATE0, 0);
    write8(SI1145_REG_MEASRATE1, 0);
    write8(SI1145_REG_IRQEN, 0);
    write8(SI1145_REG_IRQMODE1, 0);
    write8(SI1145_REG_IRQMODE2, 0);
    write8(SI1145_REG_INTCFG, 0);
    write8(SI1145_REG_IRQSTAT, 0xFF);

    write8(SI1145_REG_COMMAND, SI1145_RESET);
    OS_Sleep(10);
    write8(SI1145_REG_HWKEY, 0x17);

    OS_Sleep(10);
}

//////////////////////////////////////////////////////

// returns the UV index * 100 (divide by 100 to get the index)
uint16_t Adafruit_SI1145::readUV(void)
{
    return read16(SI1145_REG_UVINDEX0);
}

// returns visible+IR light levels
uint16_t Adafruit_SI1145::readVisible(void)
{
    return read16(SI1145_REG_ALSVISDATA0);
}

// returns IR light levels
uint16_t Adafruit_SI1145::readIR(void)
{
    return read16(SI1145_REG_ALSIRDATA0);
}

// returns "Proximity" - assumes an IR LED is attached to LED
uint16_t Adafruit_SI1145::readProx(void)
{
    return read16(SI1145_REG_PS1DATA0);
}

/*********************************************************************/

uint8_t Adafruit_SI1145::writeParam(uint8_t p, uint8_t v)
{
    //Serial.print("Param 0x"); Serial.print(p, HEX);
    //Serial.print(" = 0x"); Serial.println(v, HEX);

    write8(SI1145_REG_PARAMWR, v);
    write8(SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
    return read8(SI1145_REG_PARAMRD);
}

uint8_t Adafruit_SI1145::readParam(uint8_t p)
{
    write8(SI1145_REG_COMMAND, p | SI1145_PARAM_QUERY);
    return read8(SI1145_REG_PARAMRD);
}

/*********************************************************************/

uint8_t Adafruit_SI1145::read8(uint8_t reg)
{
    I2C_Error_t error = I2C_Transmit(_i2c, _addr, &reg, 1, I2C_DEFAULT_TIME_OUT);
    if (error != I2C_ERROR_NONE)
    {
        Trace(1, "Adafruit_SI1145::read8 transmit error: 0X%02x", error);
        return 0;
    }

    uint8_t res = 0;
    error = I2C_Receive(_i2c, _addr, &res, 1, I2C_DEFAULT_TIME_OUT);
    if (error != I2C_ERROR_NONE)
    {
        Trace(1, "Adafruit_SI1145::read8 recieve error: 0X%02x", error);
        return 0;
    }
    return res;
}

uint16_t Adafruit_SI1145::read16(uint8_t reg)
{
    I2C_Error_t error = I2C_Transmit(_i2c, _addr, &reg, 1, I2C_DEFAULT_TIME_OUT);
    if (error != I2C_ERROR_NONE)
    {
        Trace(1, "Adafruit_SI1145::read16 transmit error: 0X%02x", error);
        return 0;
    }

    uint8_t res[2];
    error = I2C_Receive(_i2c, _addr, &res[0], 2, I2C_DEFAULT_TIME_OUT);
    if (error != I2C_ERROR_NONE)
    {
        Trace(1, "Adafruit_SI1145::read16 recieve error: 0X%02x", error);
        return 0;
    }

    // Shift values to create properly formed integer
    return res[0] | (res[1] << 8);
}

void Adafruit_SI1145::write8(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = {
        reg,         // Register
        val        // Value
    };
    I2C_Error_t error = I2C_Transmit(_i2c, _addr, &data[0], 2, I2C_DEFAULT_TIME_OUT);
    if (error != I2C_ERROR_NONE)
    {
        Trace(1, "Adafruit_SI1145::write8 transmit error: 0X%02x", error);
    }
}
