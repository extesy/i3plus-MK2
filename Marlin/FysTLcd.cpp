#include "FysTLcd.h"
#include "HardwareSerial.h"
#if !defined(DISABLED_SERIAL_RXISR)
#error You must set a serial for fysTLcd interface.
#elif DISABLED_SERIAL_RXISR != FYSTLCD_SER_PORT
#error The FYSTLCD_SER_PORT is not adapted to the unuse serial port.
#endif
volatile bool FysTLcd::GLOBAL_var_V006 = false;
uint8_t FysTLcd::ftDataLen = 0;
uint16_t FysTLcd::ftAddr = 0;
uint8_t FysTLcd::ftData[128];
void FysTLcd::FunV001()
{
    uint16_t baud_setting = (F_CPU / 4 / FYSTLCD_SER_BAUD - 1) / 2;
    FYSTLCD_UCSRxA = 1 << FYSTLCD_U2Xx;
    #if F_CPU == 16000000UL&&FYSTLCD_SER_BAUD==57600
    FYSTLCD_UCSRxA = 0;
    baud_setting = (F_CPU / 8 / FYSTLCD_SER_BAUD - 1) / 2;
    #endif
    FYSTLCD_UBRRxH = baud_setting >> 8;
    FYSTLCD_UBRRxL = baud_setting;
    #if defined(__AVR_ATmega8__)
    FYSTLCD_UCSRxC = 0x86; 
    #else
    FYSTLCD_UCSRxC = 0x06;
    #endif
    SBI(FYSTLCD_UCSRxB, FYSTLCD_RXENx);
    SBI(FYSTLCD_UCSRxB, FYSTLCD_TXENx);
    SBI(FYSTLCD_UCSRxB, FYSTLCD_RXCIEx);
    CBI(FYSTLCD_UCSRxB, FYSTLCD_UDRIEx);
}
void FysTLcd::FunV002()
{
    CBI(FYSTLCD_UCSRxB, FYSTLCD_RXENx);
    CBI(FYSTLCD_UCSRxB, FYSTLCD_TXENx);
    CBI(FYSTLCD_UCSRxB, FYSTLCD_RXCIEx);
    CBI(FYSTLCD_UCSRxB, FYSTLCD_UDRIEx);
}
bool FysTLcd::waitData(uint8_t*dat)
{
    uint16_t t=10000;
    while (t && !GLOBAL_var_V006)
    {
        t--;
        DELAY_10US;
    }
    GLOBAL_var_V006 = false;
    if (t)
    {
        for (t = 0; t<ftDataLen; t++)dat[t] = ftData[t];
        return true;
    }
    return false;
}
void FysTLcd::FunV012(u8 dat)
{
    static bool ifGetData = false;
    static int8_t num = 0, n = 0, step = 0;
#ifdef FYSTLCD_FYS_CUSTOMIZED
    if(dat==FYSTLCD_FRAM_HEAD1&&step == 0)step = 1;
    else if(step==1)
    {
        if(dat==FYSTLCD_READ_VAR)step=2;
        else step=0;
        num=0;
    }
    else if(step==2)
    {
        ++num;
        switch(num)
        {
        case 1:
            ftAddr = dat;
            break;
        case 2:
            ftAddr = (ftAddr << 8) | dat;
            break;
        case 3:
            n=dat<<1;
            break;
        default:
            if (n > 0)
            {
                ftData[num - 4] = dat;
                n--;
                ifGetData = true;
            }
            break;
        }
        if (ifGetData&&n == 0)
        {
            GLOBAL_var_V006 = true;
            ifGetData = false;
            step = 0;
            ftDataLen = num - 3;
            num = 0;
        }
    }
    else
    {
        num = 0;
        step = 0;
        n = 0;
    }
#else
    if (dat == FYSTLCD_FRAM_HEAD1&&step == 0)step = 1;
    else if (dat == FYSTLCD_FRAM_HEAD2&&step == 1)step = 2;
    else if (step == 2)step++; 
    else if (step == 3)
    {
        switch (dat)
        {
        case FYSTLCD_READ_REG:step = 5; break;
        case FYSTLCD_READ_VAR:step = 6;  break;
                #if FYSTLCD_VER==FYSTLCD_VER_2016
                default:  break;
                #else
        default:step = 0;  break;
                #endif
        }
        num = 0;
    }
    else if (step == 5)
    {
        num++;
        switch (num)
        {
        case 0x01:
            ftAddr = dat;
            break;
        case 0x02:
            n = dat;
            break;
        default:
            if (n > 0)
            {
                ftData[num - 3] = dat;
                n--;
                ifGetData = true;
            }
            break;
        }
        if (ifGetData&&n == 0)
        {
            GLOBAL_var_V006 = true;
            ifGetData = false;
            step = 0;
            ftDataLen = num - 2;
            num = 0;
        }
    }
    else if (step == 6)
    {
        ++num;
        switch (num)
        {
        case 1:
            ftAddr = dat;
            break;
        case 2:
            ftAddr = (ftAddr << 8) | dat;
            break;
        case 3:
            n = dat << 1;
            break;
        default:
            if (n > 0)
            {
                ftData[num - 4] = dat;
                n--;
                ifGetData = true;
            }
            break;
        }
        if (ifGetData&&n == 0)
        {
            GLOBAL_var_V006 = true;
            ifGetData = false;
            step = 0;
            ftDataLen = num - 3;
            num = 0;
        }
    }
    else
    {
        num = 0;
        step = 0;
    }
#endif
}
#if (DISABLED_SERIAL_RXISR==0)&&defined(USART0_RX_vect)
ISR(USART0_RX_vect)
{
    uint8_t dat = FYSTLCD_UDRx;
    FysTLcd::ftRxlrq(dat);
}
#elif (DISABLED_SERIAL_RXISR==1)&&defined(USART1_RX_vect)
ISR(USART1_RX_vect)
{
    uint8_t dat = FYSTLCD_UDRx;
    FysTLcd::ftRxlrq(dat);
}
#elif (DISABLED_SERIAL_RXISR==2)&&defined(USART2_RX_vect)
ISR(USART2_RX_vect)
{
    uint8_t dat = FYSTLCD_UDRx;
    FysTLcd::FunV012(dat);
}
#elif (DISABLED_SERIAL_RXISR==3)&&defined(USART3_RX_vect)
ISR(USART3_RX_vect)
{
    uint8_t dat = FYSTLCD_UDRx;
    FysTLcd::ftRxlrq(dat);
}
#endif
#if FYSTLCD_VER==FYSTLCD_VER_2016
void FysTLcd::FunV016(const uint8_t& regAddr, const uint8_t *data, uint8_t len)
{
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_FRAM_HEAD2);
    FunV00E(len + 2);
    FunV00E(FYSTLCD_WRITE_REG);
    FunV00E(regAddr);
    while (len-- > 0)FunV00E(*data++);
}
bool FysTLcd::FunV017(const uint8_t& regAddr, uint8_t*tdata, uint8_t len)
{
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_FRAM_HEAD2);
    FunV00E(0x03);
    FunV00E(FYSTLCD_READ_REG);
    FunV00E(regAddr);
    FunV00E(len);
    return waitData(tdata);
}
void FysTLcd::FunV017(const uint8_t& regAddr, uint8_t len)
{
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_FRAM_HEAD2);
    FunV00E(0x03);
    FunV00E(FYSTLCD_READ_REG);
    FunV00E(regAddr);
    FunV00E(len);
}
#endif
void FysTLcd::FunV01A(const uint16_t& varAddr, const uint8_t *varData, uint8_t len)
{
    if (len == 0)return;
#if defined(FYSTLCD_FYS_CUSTOMIZED)
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_WRITE_VAR);
    FunV00E((uint8_t)(varAddr >> 8));
    FunV00E((uint8_t)varAddr);
    FunV00E(len>>1);
#else
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_FRAM_HEAD2);
    FunV00E(len + 3);
    FunV00E(FYSTLCD_WRITE_VAR);
    FunV00E((uint8_t)(varAddr >> 8));
    FunV00E((uint8_t)varAddr);
#endif
    while (len-- > 0)FunV00E(*varData++);
}
void FysTLcd::FunV01A(const uint16_t& varAddr, const uint16_t& data)
{
#if defined(FYSTLCD_FYS_CUSTOMIZED)
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_WRITE_VAR);
    FunV00E((uint8_t)(varAddr >> 8));
    FunV00E((uint8_t)varAddr);
    FunV00E(0x01);
#else
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_FRAM_HEAD2);
    FunV00E(0x05);
    FunV00E(FYSTLCD_WRITE_VAR);
    FunV00E((uint8_t)(varAddr >> 8));
    FunV00E((uint8_t)varAddr);
#endif
    FunV00E((uint8_t)(data >> 8));
    FunV00E((uint8_t)data);
}
bool FysTLcd::FunV01D(const uint16_t& varAddr, uint8_t*tdata, uint8_t len)
{
    GLOBAL_var_V006 = false;
#if defined(FYSTLCD_FYS_CUSTOMIZED)
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_READ_VAR);
    FunV00E(uint8_t(varAddr >> 8));
    FunV00E(uint8_t(varAddr));
#else
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_FRAM_HEAD2);
    FunV00E(0x04);
    FunV00E(FYSTLCD_READ_VAR);
    FunV00E(uint8_t(varAddr >> 8));
    FunV00E(uint8_t(varAddr));
#endif
    FunV00E(len >> 1);
    return waitData(tdata);
}
void FysTLcd::FunV01D(const uint16_t& varAddr, uint8_t len)
{
    GLOBAL_var_V006 = false;
#if defined(FYSTLCD_FYS_CUSTOMIZED)
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_READ_VAR);
    FunV00E(uint8_t(varAddr >> 8));
    FunV00E(uint8_t(varAddr));
#else
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_FRAM_HEAD2);
    FunV00E(0x04);
    FunV00E(FYSTLCD_READ_VAR);
    FunV00E(uint8_t(varAddr >> 8));
    FunV00E(uint8_t(varAddr));
#endif
    FunV00E(len >> 1);
}
void FysTLcd::FunV01E()
{
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
    uint8_t cmd[4] = {0x55,0xAA,0x5A,0x5A};
    FunV01A(0x0004,cmd,4);
#elif FYSTLCD_VER==FYSTLCD_VER_2016
    uint8_t cmd[2]={0x5A,0xA5};
    FunV016(0xEE,cmd,2);
#endif
}
void FysTLcd::FunV024(uint16_t page)
{
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
    uint8_t cmd[4] = { 0x5A, 0x01, (page >> 8) & 0xFF, page & 0xFF };
    FunV01A(0x0084, cmd, 4);
#elif FYSTLCD_VER==FYSTLCD_VER_2016
    uint8_t cmd[2] = { (page >> 8) & 0xFF, page & 0xFF };
    FunV016(0x03, cmd, 2);
#endif
}
uint16_t FysTLcd::FunV025()
{
    uint8_t cmd[4] = { 0 };
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
    if (FunV01D(0x0014, cmd, 2))
    {
        uint16_t t = cmd[0];
        return (t << 8) | cmd[1];
    }
#elif FYSTLCD_VER==FYSTLCD_VER_2016
    if (FysTLcd::FunV017(0x03, cmd, 2))
    {
        uint16_t t = cmd[0];
        return (t << 8) | cmd[1];
    }
#endif
    return 0xFFFF;
}
void FysTLcd::FunV022(uint8_t val)
{
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
    if (val > FYSTLCD_BRIGHTNESS_MAX)val = FYSTLCD_BRIGHTNESS_MAX;
    uint8_t cmd[2] = { val, 0x00 };
    FunV01A(0x0082, cmd, 2);
#elif FYSTLCD_VER==FYSTLCD_VER_2016
    if(val>FYSTLCD_BRIGHTNESS_MAX)val=FYSTLCD_BRIGHTNESS_MAX;
    FunV016(0x01,&val,1);
#endif
}
void FysTLcd::FunV028(uint8_t id, uint8_t segments, uint8_t volume)
{
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
    uint8_t cmd[4] = { id, segments, volume, 0x80 };
    FunV01A(0x00A0, cmd, 4);
#endif
}
void FysTLcd::FunV015(const uint16_t& varAddr, const char*str, uint8_t len)
{
    if (len == 0)
    while (str[len])len++;
#if defined(FYSTLCD_FYS_CUSTOMIZED)
    len++;
    len >>= 1;
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_WRITE_VAR);
    FunV00E((uint8_t)(varAddr >> 8));
    FunV00E((uint8_t)varAddr);
    FunV00E(len);
    len <<= 1;
    while (len--)
    {
        if(*str)FunV00E(*str++);
        else FunV00E(0);
    }
#else
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_FRAM_HEAD2);
    FunV00E(len + 3);
    FunV00E(FYSTLCD_WRITE_VAR);
    FunV00E((uint8_t)(varAddr >> 8));
    FunV00E((uint8_t)varAddr);
    while (len-- > 0)
    {
        if(*str)FunV00E(*str++);
        else FunV00E(0);
    }
#endif
}
void FysTLcd::ftPutsPGM(const uint16_t& varAddr, const char*str, uint8_t len)
{
    const char*t = str;
    char tlen = 0, ch;
    while (pgm_read_byte(t))
    {
        tlen++;
        t++;
    }
    if (len == 0)len = tlen;
#if defined(FYSTLCD_FYS_CUSTOMIZED)
    len++;
    len >>= 1;
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_WRITE_VAR);
    FunV00E((uint8_t)(varAddr >> 8));
    FunV00E((uint8_t)varAddr);
    FunV00E(len);
    len <<= 1;
    ch = pgm_read_byte(str);
    while (len-- > 0)
    {
        if (ch)
        {
            FunV00E(ch);
            str++;
            ch = pgm_read_byte(str);
        }
        else FunV00E(0);
    }
#else
    FunV00E(FYSTLCD_FRAM_HEAD1);
    FunV00E(FYSTLCD_FRAM_HEAD2);
    FunV00E(len + 3);
    FunV00E(FYSTLCD_WRITE_VAR);
    FunV00E((uint8_t)(varAddr >> 8));
    FunV00E((uint8_t)varAddr);
    ch = pgm_read_byte(str);
    while (len-- > 0)
    {
        if (ch)
        {
            FunV00E(ch);
            str++;
            ch = pgm_read_byte(str);
        }
        else FunV00E(0);
    }
#endif
}
void FysTLcd::FunV049(millis_t& pt)
{
    uint8_t t;
    t = pt / 3600;
    if (t >= 100)
    {
        cdt[n++] = t / 100 + '0';
        t %= 100;
    }
    if (t >= 10)
    {
        cdt[n++] = t / 10 + '0';
        t %= 10;
    }
    cdt[n++] = t + '0';
    cdt[n++] = ':';
    pt %= 3600;
    t = pt / 60;
    if (t >= 10)
    {
        cdt[n++] = t / 10 + '0';
        t %= 10;
    }
    cdt[n++] = t + '0';
    cdt[n++] = ':';
    t = pt % 60;
    if (t >= 10)
    {
        cdt[n++] = t / 10 + '0';
        t %= 10;
    }
    cdt[n++] = t + '0';
    for (; n < 8; n++)cdt[n] = ' ';
}
void FysTLcd::FunV04A(const char* str)
{
    while (char ch = pgm_read_byte(str++)) cdt[n++] = ch;
    for (; n < 8; n++)cdt[n] = ' ';
}
void FysTLcd::FunV00F(const char*tar) 
{
    char c, ch, tt = 0;
    for (; (*tar >= 'A'&&*tar <= 'F') || (*tar >= 'a'&&*tar <= 'f') || (*tar >= '0'&&*tar <= '9') || *tar == ' '; tar++)
    {
        MYSERIAL.write(*tar);
        if (*tar == ' ')continue;
        if (*tar >= 'A'&&*tar <= 'F')
            ch = *tar - 'A' + 10;
        else if (*tar >= 'a'&&*tar <= 'f')
            ch = *tar - 'a' + 10;
        else
            ch = *tar - '0';
        if (tt == 0)
        {
            tt = 1;
            c = ch << 4;
        }
        else
        if (tt == 1)
        {
            tt = 2;
            c |= ch;
        }
        if (tt == 2)
        {
            tt = 0;
            FunV00E(c);
        }
    }
}
const char* FysTLcd::FunV05C(uint8_t& stringLen)
{
    if (ftDataLen<4)
    {
        stringLen = 0;
        return nullptr;
    }
    else
    {
        stringLen = ftDataLen - 2;
        if (ftData[stringLen - 1] == 0xFF)stringLen--;
        return (const char*)ftData;
    }
}
