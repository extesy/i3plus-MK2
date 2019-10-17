#ifndef MARLINSERIAL_H
#define MARLINSERIAL_H
#include "MarlinConfig.h"
#ifndef SERIAL_PORT
  #define SERIAL_PORT 0
#endif
#define UART_PRESENT(port) ((port == 0 && (defined(UBRRH) || defined(UBRR0H))) || \
                            (port == 1 && defined(UBRR1H)) || (port == 2 && defined(UBRR2H)) || \
                            (port == 3 && defined(UBRR3H)))
#define SERIAL_REGNAME(registerbase,number,suffix) SERIAL_REGNAME_INTERNAL(registerbase,number,suffix)
#if SERIAL_PORT == 0 && (!defined(UBRR0H) || !defined(UDR0)) 
  #define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##suffix
#else
  #define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##number##suffix 
#endif
#ifdef mySERIAL_Nums
#define M_UCSRxA(port) SERIAL_REGNAME(UCSR,port,A) 
#define M_UCSRxB(port) SERIAL_REGNAME(UCSR,port,B)
#define M_RXENx(port) SERIAL_REGNAME(RXEN,port,)
#define M_TXENx(port) SERIAL_REGNAME(TXEN,port,)
#define M_TXCx(port) SERIAL_REGNAME(TXC,port,)
#define M_RXCIEx(port) SERIAL_REGNAME(RXCIE,port,)
#define M_UDREx(port) SERIAL_REGNAME(UDRE,port,)
#define M_UDRIEx(port) SERIAL_REGNAME(UDRIE,port,)
#define M_UDRx(port) SERIAL_REGNAME(UDR,port,)
#define M_UBRRxH(port) SERIAL_REGNAME(UBRR,port,H)
#define M_UBRRxL(port) SERIAL_REGNAME(UBRR,port,L)
#define M_RXCx(port) SERIAL_REGNAME(RXC,port,)
#define M_USARTx_RX_vect(port) SERIAL_REGNAME(USART,port,_RX_vect)
#define M_U2Xx(port) SERIAL_REGNAME(U2X,port,)
#define M_USARTx_UDRE_vect(port) SERIAL_REGNAME(USART,port,_UDRE_vect)
#else 
#define M_UCSRxA           SERIAL_REGNAME(UCSR,SERIAL_PORT,A) 
#define M_UCSRxB           SERIAL_REGNAME(UCSR,SERIAL_PORT,B)
#define M_RXENx            SERIAL_REGNAME(RXEN,SERIAL_PORT,)
#define M_TXENx            SERIAL_REGNAME(TXEN,SERIAL_PORT,)
#define M_TXCx             SERIAL_REGNAME(TXC,SERIAL_PORT,)
#define M_RXCIEx           SERIAL_REGNAME(RXCIE,SERIAL_PORT,)
#define M_UDREx            SERIAL_REGNAME(UDRE,SERIAL_PORT,)
#define M_UDRIEx           SERIAL_REGNAME(UDRIE,SERIAL_PORT,)
#define M_UDRx             SERIAL_REGNAME(UDR,SERIAL_PORT,)
#define M_UBRRxH           SERIAL_REGNAME(UBRR,SERIAL_PORT,H)
#define M_UBRRxL           SERIAL_REGNAME(UBRR,SERIAL_PORT,L)
#define M_RXCx             SERIAL_REGNAME(RXC,SERIAL_PORT,)
#define M_USARTx_RX_vect   SERIAL_REGNAME(USART,SERIAL_PORT,_RX_vect)
#define M_U2Xx             SERIAL_REGNAME(U2X,SERIAL_PORT,)
#define M_USARTx_UDRE_vect SERIAL_REGNAME(USART,SERIAL_PORT,_UDRE_vect)
#endif
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0
#ifndef USBCON
  #ifndef RX_BUFFER_SIZE
    #define RX_BUFFER_SIZE 128
  #endif
  #ifndef TX_BUFFER_SIZE
    #define TX_BUFFER_SIZE 32
  #endif
  #if !((RX_BUFFER_SIZE == 256) ||(RX_BUFFER_SIZE == 128) ||(RX_BUFFER_SIZE == 64) ||(RX_BUFFER_SIZE == 32) ||(RX_BUFFER_SIZE == 16) ||(RX_BUFFER_SIZE == 8) ||(RX_BUFFER_SIZE == 4) ||(RX_BUFFER_SIZE == 2))
    #error "RX_BUFFER_SIZE has to be a power of 2 and >= 2"
  #endif
  #if !((TX_BUFFER_SIZE == 256) ||(TX_BUFFER_SIZE == 128) ||(TX_BUFFER_SIZE == 64) ||(TX_BUFFER_SIZE == 32) ||(TX_BUFFER_SIZE == 16) ||(TX_BUFFER_SIZE == 8) ||(TX_BUFFER_SIZE == 4) ||(TX_BUFFER_SIZE == 2) ||(TX_BUFFER_SIZE == 0))
    #error TX_BUFFER_SIZE has to be a power of 2 or 0
  #endif
  struct ring_buffer_r {
    unsigned char buffer[RX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
  };
  #if TX_BUFFER_SIZE > 0
    struct ring_buffer_t {
      unsigned char buffer[TX_BUFFER_SIZE];
      volatile uint8_t head;
      volatile uint8_t tail;
    };
  #endif
#ifdef mySERIAL_Nums
    extern ring_buffer_r*rx_buffer0;
      #if TX_BUFFER_SIZE > 0
    extern ring_buffer_t*tx_buffer0;
      #endif
#else 
    #if UART_PRESENT(SERIAL_PORT)
      extern ring_buffer_r rx_buffer;
      #if TX_BUFFER_SIZE > 0
        extern ring_buffer_t tx_buffer;
      #endif
    #endif
#endif
  class MarlinSerial { 
#ifdef mySERIAL_Nums
  private:
      const static unsigned char Port[mySERIAL_Nums];
      static ring_buffer_r*activeRxBuffer;
    #if TX_BUFFER_SIZE > 0
      static ring_buffer_t*activeTxBuffer;
    #endif
#endif
    public:
      MarlinSerial(){};
#ifdef mySERIAL_Nums
      const static bool sPcomm[mySERIAL_Nums];
      const static bool sCmd[mySERIAL_Nums];
      static char activeRxSerial,activeTxSerial;
      static void begin();
      inline void uart0Send(uint8_t c) 
      {
          while (!TEST(M_UCSRxA(0), M_UDREx(0)));
          M_UDRx(0) = c;
      }
      static inline void storeCharToUart0(uint8_t c){
          if (rx_buffer0){
              const uint8_t h = rx_buffer0->head,
                  i = (uint8_t)(h + 1) & (RX_BUFFER_SIZE - 1);
              if (i != rx_buffer0->tail) {
                  rx_buffer0->buffer[h] = c;
                  rx_buffer0->head = i;
              }
          }
      }
      inline uint8_t space(){ return RX_BUFFER_SIZE - available(); }
#else
      static void begin(const long);
#endif
      static void end();
      static int peek(void);
      static int read(void);
      static void flush(void);
      static uint8_t available(void);
      static void checkRx(void);
      static void write(const uint8_t c);
      #if TX_BUFFER_SIZE > 0
        static uint8_t availableForWrite(void);
        static void flushTX(void);
      #endif
    private:
      static void printNumber(unsigned long, const uint8_t);
      static void printFloat(double, uint8_t);
    public:
      static FORCE_INLINE void write(const char* str) { while (*str) write(*str++); }
      static FORCE_INLINE void write(const uint8_t* buffer, size_t size) { while (size--) write(*buffer++); }
      static FORCE_INLINE void print(const String& s) { for (int i = 0; i < (int)s.length(); i++) write(s[i]); }
      static FORCE_INLINE void print(const char* str) { write(str); }
#ifdef mySERIAL_Nums
      static void setRxActiveSerial(int s);
      static void setTxActiveSerial(int s);
      static void storeChar(char c, int port);
#endif
      static void print(char, int = BYTE);
      static void print(unsigned char, int = BYTE);
      static void print(int, int = DEC);
      static void print(unsigned int, int = DEC);
      static void print(long, int = DEC);
      static void print(unsigned long, int = DEC);
      static void print(double, int = 2);
      static void println(const String& s);
      static void println(const char[]);
      static void println(char, int = BYTE);
      static void println(unsigned char, int = BYTE);
      static void println(int, int = DEC);
      static void println(unsigned int, int = DEC);
      static void println(long, int = DEC);
      static void println(unsigned long, int = DEC);
      static void println(double, int = 2);
      static void println(void);
  };
  extern MarlinSerial customizedSerial;
#endif 
#if defined(USBCON) && ENABLED(BLUETOOTH)
  extern HardwareSerial bluetoothSerial;
#endif
#endif 
