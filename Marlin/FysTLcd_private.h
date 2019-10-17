#ifndef Y_FYSTLCD_Private_H
#define Y_FYSTLCD_Private_H
#define     FYSTLCD_VER_2016 0
#define     FYSTLCD_VER_T5V1 1
#define     FYSTLCD_VER FYSTLCD_VER_2016 
#define FYSTLCD_WRITE_REG                   0x80 
#define FYSTLCD_READ_REG                    0x81
#ifdef FYSTLCD_FYS_CUSTOMIZED
#define FYSTLCD_FRAM_HEAD1                  0xCA
#define FYSTLCD_WRITE_VAR                   0xA1 
#define FYSTLCD_READ_VAR                    0xA2
#else
#define FYSTLCD_FRAM_HEAD1                  0x5A
#define FYSTLCD_FRAM_HEAD2                  0xA5
#define FYSTLCD_WRITE_VAR                   0x82 
#define FYSTLCD_READ_VAR                    0x83
#endif
#if FYSTLCD_VER==FYSTLCD_VER_2016
#define     FYSTLCD_BRIGHTNESS_MAX                      0x40 
#elif FYSTLCD_VER==FYSTLCD_VER_T5V1
#define     FYSTLCD_BRIGHTNESS_MAX                      0x64 
#endif
#define _TNAME(X,Y,Z)                   X##Y##Z
#define TNAME(X,Y,Z)                    _TNAME(X,Y,Z)
#define FYSTLCD_SERIAL_RX_VECT              TNAME(USART,FYSTLCD_SER_PORT,_RX_vect)
#define FYSTLCD_UCSRxA                      TNAME(UCSR,FYSTLCD_SER_PORT,A)
#define FYSTLCD_UCSRxB                      TNAME(UCSR,FYSTLCD_SER_PORT,B)
#define FYSTLCD_UCSRxC                      TNAME(UCSR,FYSTLCD_SER_PORT,C)
#define FYSTLCD_UBRRxH                      TNAME(UBRR,FYSTLCD_SER_PORT,H)
#define FYSTLCD_UBRRxL                      TNAME(UBRR,FYSTLCD_SER_PORT,L)
#define FYSTLCD_UDRx                        TNAME(UDR,FYSTLCD_SER_PORT,)
#define FYSTLCD_U2Xx                        TNAME(U2X,FYSTLCD_SER_PORT,)
#define FYSTLCD_RXENx                       TNAME(RXEN,FYSTLCD_SER_PORT,)
#define FYSTLCD_TXENx                       TNAME(TXEN,FYSTLCD_SER_PORT,)
#define FYSTLCD_TXCx                        TNAME(TXC,FYSTLCD_SER_PORT,)
#define FYSTLCD_RXCIEx                      TNAME(RXCIE,FYSTLCD_SER_PORT,)
#define FYSTLCD_UDRIEx                      TNAME(UDRIE,FYSTLCD_SER_PORT,)
#define FYSTLCD_UDREx                       TNAME(UDRE,FYSTLCD_SER_PORT,)
class FysTLcd
{
private:
    volatile static bool GLOBAL_var_V006;
    static uint8_t ftDataLen;
    static uint8_t ftData[128];
    static inline void FunV00E(uint8_t dat)
    {
        while (!(FYSTLCD_UCSRxA&(1 << FYSTLCD_UDREx)));
        FYSTLCD_UDRx = dat;
    }
    static bool waitData(uint8_t*dat);
#if FYSTLCD_VER==FYSTLCD_VER_2016
    static void FunV016(const uint8_t& regAddr, const uint8_t *data, uint8_t len);
    static bool FunV017(const uint8_t& regAddr, uint8_t*data, uint8_t len);
    static void FunV017(const uint8_t& varAddr, uint8_t len);
#endif
    static void FunV01A(const uint16_t& varAddr, const uint16_t& data);
    static void FunV01A(const uint16_t& varAddr, const uint8_t *data, uint8_t len = 2);
    static void FunV01D(const uint16_t& varAddr, uint8_t len = 2);
    static bool FunV01D(const uint16_t& varAddr, uint8_t*data, uint8_t len = 2);
public:
    static uint16_t ftAddr;
    static void FunV001();
    static void FunV002();
    static bool FunV059(){ bool ifCmd = GLOBAL_var_V006; GLOBAL_var_V006 = false; return ifCmd; }
    static inline uint16_t FunV05A(){ uint16_t val = ftData[0]; val = (val << 8) | ftData[1]; return val; }
    static inline uint32_t FunV05B(){ uint32_t val = ftData[0]; val = (val << 8) | ftData[1]; val = (val << 8) | ftData[2]; val = (val << 8) | ftData[3]; return val; }
    static const char* FunV05C(uint8_t& stringLen);
    static void FunV012(u8 dat);
    static void FunV01E();
    static void FunV022(uint8_t val);
    static void FunV024(uint16_t page);
    static void FunV028(uint8_t id, uint8_t segments, uint8_t volume);
    static void FunV015(const uint16_t& ftAddr, const char*str, uint8_t len = 0);
    static void ftPutsPGM(const uint16_t& ftAddr, const char*str, uint8_t len = 0);
    static uint16_t FunV025();
    static void FunV00F(const char*tar); 
private:
    uint8_t n;
    uint8_t cdt[39];
    uint16_t cAddr;
    inline void FunV02A(){ CBI(FYSTLCD_UCSRxB, FYSTLCD_RXCIEx); }
    inline void FunV02B(){ SBI(FYSTLCD_UCSRxB, FYSTLCD_RXCIEx); }
public:
    inline void FunV02C(const uint16_t& ftAddr)
    {
        cAddr = ftAddr;
        memset(cdt, 0, 39);
        n = 0;
    }
    inline void FunV02E(uint8_t jump=0)
    {
        FunV01A(cAddr, cdt, n);
        memset(cdt, 0, n);
        if (jump)cAddr += jump;
        else cAddr += (n >> 1);
        n = 0;
    }
    inline bool FunV027(uint8_t num)
    {
        bool ifGet=FunV01D(cAddr, cdt, num);
        cAddr += (num >> 1);
        n = 0;
        return ifGet;
    }
    inline void ftCmdClear(){ memset(cdt, 0, 39);}
    inline void FunV026(uint8_t byteNum){ n += byteNum; }
    inline void FunV031(const int16_t& r)
    {
        cdt[n++] = (r >> 8) & 0xFF;
        cdt[n++] = (uint8_t)r;
    }
    inline void FunV032(const float& r)
    {
        uint16_t t = r* FYSTLCD_DOT_TEN_MUL;
        cdt[n++] = (t >> 8) & 0xFF;
        cdt[n++] = (uint8_t)t;
    }
    inline void ftPutF16_2(const float& r) 
    {
        uint16_t t = r* FYSTLCD_DOT_HUND_MUL;
        cdt[n++] = (t >> 8) & 0xFF;
        cdt[n++] = (uint8_t)t;
    }
    inline void FunV037(const int32_t& r)
    {
        cdt[n++] = (r >> 24) & 0xFF;
        cdt[n++] = (r >> 16) & 0xFF;
        cdt[n++] = (r >> 8) & 0xFF;
        cdt[n++] = (uint8_t)r;
    }
    inline void FunV03D(const float& r)
    {
        int32_t t = r*FYSTLCD_DOT_TEN_MUL;
        FunV037(t);
    }
    inline void FunV03E(int16_t& r)
    {
        r = cdt[n++];
        r = ((r << 8) | cdt[n++]);
    }
    inline void FunV043(float& r)
    {
        int16_t t;
        FunV03E(t);
        r = t / FYSTLCD_DOT_TEN_MUL;
    }
    inline void FunV044(int32_t& r)
    {
        r = cdt[n++];
        r = ((r << 8) | cdt[n++]);
        r = ((r << 8) | cdt[n++]);
        r = ((r << 8) | cdt[n++]);
    }
    inline void FunV047(float& r)
    {
        int32_t t;
        FunV044(t);
        r = t / FYSTLCD_DOT_TEN_MUL;
    }
    void FunV049(millis_t& pt);
    void FunV04A(const char* str);
public:
    FysTLcd():n(0){}
    ~FysTLcd(){ }
};
#endif 
