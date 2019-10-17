#include "MarlinSerial.h"
#include "Marlin.h"
#if !defined(USBCON) && (defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H))
#ifdef mySERIAL_Nums
    char MarlinSerial::activeRxSerial=0, MarlinSerial::activeTxSerial=99;
    const unsigned char MarlinSerial::Port[mySERIAL_Nums] = { mySERIAL_Array };
    const bool MarlinSerial::sPcomm[mySERIAL_Nums] = { mySERIAL_PublicCOM };
    const bool MarlinSerial::sCmd[mySERIAL_Nums] = { mySERIAL_gcodeCmd };
    const static uint32_t bauds[mySERIAL_Nums] = { mySERIAL_Bauderate };
    ring_buffer_r*rx_buffer0 = nullptr, *rx_buffer1 = nullptr, *rx_buffer2 = nullptr, *rx_buffer3 = nullptr;
    ring_buffer_r*MarlinSerial::activeRxBuffer = nullptr;
    #if TX_BUFFER_SIZE > 0
    ring_buffer_t*tx_buffer0 = nullptr, *tx_buffer1 = nullptr, *tx_buffer2 = nullptr, *tx_buffer3 = nullptr;
    ring_buffer_t*MarlinSerial::activeTxBuffer = nullptr;
    static bool _written0, _written1, _written2, _written3;
    #endif
#else
  #if UART_PRESENT(SERIAL_PORT)
    ring_buffer_r rx_buffer = { { 0 }, 0, 0 };
    #if TX_BUFFER_SIZE > 0
      ring_buffer_t tx_buffer = { { 0 }, 0, 0 };
      static bool _written;
    #endif
  #endif
#endif
  #if ENABLED(EMERGENCY_PARSER)
    #include "stepper.h"
    #include "language.h"
    FORCE_INLINE void emergency_parser(const unsigned char c) {
      static e_parser_state state = state_RESET;
      switch (state) {
        case state_RESET:
          switch (c) {
            case ' ': break;
            case 'N': state = state_N;      break;
            case 'M': state = state_M;      break;
            default: state = state_IGNORE;
          }
          break;
        case state_N:
          switch (c) {
            case '0': case '1': case '2':
            case '3': case '4': case '5':
            case '6': case '7': case '8':
            case '9': case '-': case ' ':   break;
            case 'M': state = state_M;      break;
            default:  state = state_IGNORE;
          }
          break;
        case state_M:
          switch (c) {
            case ' ': break;
            case '1': state = state_M1;     break;
            case '4': state = state_M4;     break;
            default: state = state_IGNORE;
          }
          break;
        case state_M1:
          switch (c) {
            case '0': state = state_M10;    break;
            case '1': state = state_M11;    break;
            default: state = state_IGNORE;
          }
          break;
        case state_M10:
          state = (c == '8') ? state_M108 : state_IGNORE;
          break;
        case state_M11:
          state = (c == '2') ? state_M112 : state_IGNORE;
          break;
        case state_M4:
          state = (c == '1') ? state_M41 : state_IGNORE;
          break;
        case state_M41:
          state = (c == '0') ? state_M410 : state_IGNORE;
          break;
        case state_IGNORE:
          if (c == '\n') state = state_RESET;
          break;
        default:
          if (c == '\n') {
            switch (state) {
              case state_M108:
                wait_for_user = wait_for_heatup = false;
                break;
              case state_M112:
                kill(PSTR(MSG_KILLED));
                break;
              case state_M410:
                quickstop_stepper();
                break;
              default:
                break;
            }
            state = state_RESET;
          }
      }
    }
#endif 
#ifdef mySERIAL_Nums
#define SERIAL_STORE_CHAR(c,port) if(rx_buffer##port){CRITICAL_SECTION_START;\
    const uint8_t h = rx_buffer##port->head, \
    i = (uint8_t)(h + 1) & (RX_BUFFER_SIZE - 1);\
    if (i != rx_buffer##port->tail) {\
    rx_buffer##port->buffer[h] = c; \
    rx_buffer##port->head = i;}\
    CRITICAL_SECTION_END;}
#else
  FORCE_INLINE void store_char(unsigned char c) {
    CRITICAL_SECTION_START;
      const uint8_t h = rx_buffer.head,
                    i = (uint8_t)(h + 1) & (RX_BUFFER_SIZE - 1);
      if (i != rx_buffer.tail) {
        rx_buffer.buffer[h] = c;
        rx_buffer.head = i;
      }
    CRITICAL_SECTION_END;
    #if ENABLED(EMERGENCY_PARSER)
      emergency_parser(c);
    #endif
  }
#endif
  #if TX_BUFFER_SIZE > 0
#ifdef mySERIAL_Nums
#define TX_UDR_EMPTY_IRQ_FUN(port)  FORCE_INLINE void _tx_udr_empty_irq##port() {\
  if (tx_buffer##port){\
  uint8_t t = tx_buffer##port->tail;\
  uint8_t c = tx_buffer##port->buffer[t];\
  tx_buffer##port->tail = (t + 1) & (TX_BUFFER_SIZE - 1);\
  M_UDRx(port) = c;\
  SBI(M_UCSRxA(port), M_TXCx(port));\
  if (tx_buffer##port->head == tx_buffer##port->tail){ CBI(M_UCSRxB(port), M_UDRIEx(port)); }\
  }}
    TX_UDR_EMPTY_IRQ_FUN(0)
    TX_UDR_EMPTY_IRQ_FUN(1)
    TX_UDR_EMPTY_IRQ_FUN(2)
    TX_UDR_EMPTY_IRQ_FUN(3)
    #ifdef M_USARTx_UDRE_vect(0)
        ISR(M_USARTx_UDRE_vect(0)) {
            _tx_udr_empty_irq0();
        }
    #endif
    #ifdef M_USARTx_UDRE_vect(1)
        ISR(M_USARTx_UDRE_vect(1)) {
            _tx_udr_empty_irq1();
        }
    #endif
    #ifdef M_USARTx_UDRE_vect(2)
        ISR(M_USARTx_UDRE_vect(2)) {
            _tx_udr_empty_irq2();
        }
    #endif
    #ifdef M_USARTx_UDRE_vect(3)
        ISR(M_USARTx_UDRE_vect(3)) {
            _tx_udr_empty_irq3();
        }
    #endif
#else 
    FORCE_INLINE void _tx_udr_empty_irq(void) {
      const uint8_t t = tx_buffer.tail,
                    c = tx_buffer.buffer[t];
      tx_buffer.tail = (t + 1) & (TX_BUFFER_SIZE - 1);
      M_UDRx = c;
      SBI(M_UCSRxA, M_TXCx);
      if (tx_buffer.head == tx_buffer.tail) {
        CBI(M_UCSRxB, M_UDRIEx);
      }
    }
    #ifdef M_USARTx_UDRE_vect
      ISR(M_USARTx_UDRE_vect) {
        _tx_udr_empty_irq();
      }
    #endif
#endif 
  #endif 
#ifdef mySERIAL_Nums
    #ifdef M_USARTx_RX_vect(0)
        #if !defined(DISABLED_SERIAL_RXISR)||DISABLED_SERIAL_RXISR!=0
        ISR(M_USARTx_RX_vect(0)) {
            unsigned char c = M_UDRx(0);
            SERIAL_STORE_CHAR(c,0);
            #if ENABLED(EMERGENCY_PARSER)
              emergency_parser(c);
            #endif
        }
        #endif
    #endif
    #ifdef M_USARTx_RX_vect(1)
        #if !defined(DISABLED_SERIAL_RXISR)||DISABLED_SERIAL_RXISR!=1
        ISR(M_USARTx_RX_vect(1)) {
            unsigned char c = M_UDRx(1);
            SERIAL_STORE_CHAR(c, 1);
            #if ENABLED(EMERGENCY_PARSER)
              emergency_parser(c);
            #endif
        }
        #endif
    #endif
#if !(defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__))
    #ifdef M_USARTx_RX_vect(2)
        #if !defined(DISABLED_SERIAL_RXISR)||DISABLED_SERIAL_RXISR!=2
        ISR(M_USARTx_RX_vect(2)) {
            unsigned char c = M_UDRx(2);
            SERIAL_STORE_CHAR(c, 2);
            #if ENABLED(EMERGENCY_PARSER)
              emergency_parser(c);
            #endif
        }
        #endif
    #endif
    #ifdef M_USARTx_RX_vect(3)
        #if !defined(DISABLED_SERIAL_RXISR)||DISABLED_SERIAL_RXISR!=3
        ISR(M_USARTx_RX_vect(3)) {
            unsigned char c = M_UDRx(3);
            SERIAL_STORE_CHAR(c, 3);
            #if ENABLED(EMERGENCY_PARSER)
              emergency_parser(c);
            #endif
        }
        #endif
    #endif
#endif
#else 
  #ifdef M_USARTx_RX_vect
    ISR(M_USARTx_RX_vect) {
      const unsigned char c = M_UDRx;
      store_char(c);
    }
  #endif
#endif 
#ifdef mySERIAL_Nums
void MarlinSerial::begin() {
    uint16_t baud_setting;
#define SERIAL_BAUD_SET(portx) M_UBRRxH(portx) = baud_setting >> 8; \
        M_UBRRxL(portx) = baud_setting; \
        SBI(M_UCSRxB(portx), M_RXENx(portx)); \
        SBI(M_UCSRxB(portx), M_TXENx(portx)); \
        SBI(M_UCSRxB(portx), M_RXCIEx(portx))
    for (char i = 0; i < mySERIAL_Nums; i++)
    {
        if (Port[i]>3)continue;
        switch (Port[i])
        {
            case 0:
                #if UART_PRESENT(0)
                if(rx_buffer0==nullptr)rx_buffer0=new ring_buffer_r();
                #if TX_BUFFER_SIZE > 0
                if(tx_buffer0==nullptr)tx_buffer0=new ring_buffer_t();
                #endif
                #if F_CPU == 16000000UL
                if(bauds[i]==57600)
                {
                    M_UCSRxA(0)=0;
                    baud_setting = (F_CPU / 8 / bauds[i] - 1) / 2;
                }
                else
                {
                    M_UCSRxA(0) = _BV(M_U2Xx(0));
                    baud_setting = (F_CPU / 4 / bauds[i] - 1) / 2;
                }
                #else
                M_UCSRxA(0) = _BV(M_U2Xx(0));
                baud_setting = (F_CPU / 4 / bauds[i] - 1) / 2;
                #endif
                SERIAL_BAUD_SET(0);
                #endif
                break;
            case 1:
                #if UART_PRESENT(1)
                if(rx_buffer1==nullptr)rx_buffer1=new ring_buffer_r();
                #if TX_BUFFER_SIZE > 0
                if(tx_buffer1==nullptr)tx_buffer1=new ring_buffer_t();
                #endif
                M_UCSRxA(1) = _BV(M_U2Xx(1));
                baud_setting = (F_CPU / 4 / bauds[i] - 1) / 2;
                SERIAL_BAUD_SET(1);
                #endif
                break;
#if !(defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__))
            case 2:
                #if UART_PRESENT(2)
                if(rx_buffer2==nullptr)rx_buffer2=new ring_buffer_r();
                #if TX_BUFFER_SIZE > 0
                if(tx_buffer2==nullptr)tx_buffer2=new ring_buffer_t();
                #endif
                M_UCSRxA(2) = _BV(M_U2Xx(2));
                baud_setting = (F_CPU / 4 / bauds[i] - 1) / 2;
                SERIAL_BAUD_SET(2);
                #endif
                break;
            case 3:
                #if UART_PRESENT(3)
                if(rx_buffer3==nullptr)rx_buffer3=new ring_buffer_r();
                #if TX_BUFFER_SIZE > 0
                if(tx_buffer3==nullptr)tx_buffer3=new ring_buffer_t();
                #endif
                M_UCSRxA(3) = _BV(M_U2Xx(3));
                baud_setting = (F_CPU / 4 / bauds[i] - 1) / 2;
                SERIAL_BAUD_SET(3);
                #endif
                break;
#endif
        }
    }
    #if TX_BUFFER_SIZE > 0
    for (char i = 0; i < mySERIAL_Nums; i++)
    {
        if (Port[i]>3)continue;
        switch (Port[i])
        {
        case 0:
            #if UART_PRESENT(0)
            CBI(M_UCSRxB(0), M_UDRIEx(0));
            _written0=false;
            #endif
            break;
        case 1:
            #if UART_PRESENT(1)
            CBI(M_UCSRxB(1), M_UDRIEx(1));
            _written1=false;
            #endif
            break;
#if !(defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__))
        case 2:
            #if UART_PRESENT(2)
            CBI(M_UCSRxB(2), M_UDRIEx(2));
            _written2=false;
            #endif
            break;
        case 3:
            #if UART_PRESENT(3)
            CBI(M_UCSRxB(3), M_UDRIEx(3));
            _written3=false;
            #endif
            break;
#endif
        }
    }
    #endif 
    }
#else 
  void MarlinSerial::begin(const long baud) {
    uint16_t baud_setting;
    bool useU2X = true;
    #if F_CPU == 16000000UL && SERIAL_PORT == 0
      if (baud == 57600) useU2X = false;
    #endif
    if (useU2X) {
      M_UCSRxA = _BV(M_U2Xx);
      baud_setting = (F_CPU / 4 / baud - 1) / 2;
    }
    else {
      M_UCSRxA = 0;
      baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }
    M_UBRRxH = baud_setting >> 8;
    M_UBRRxL = baud_setting;
    SBI(M_UCSRxB, M_RXENx);
    SBI(M_UCSRxB, M_TXENx);
    SBI(M_UCSRxB, M_RXCIEx);
    #if TX_BUFFER_SIZE > 0
      CBI(M_UCSRxB, M_UDRIEx);
      _written = false;
    #endif
  }
#endif
#ifdef mySERIAL_Nums
  void MarlinSerial::setRxActiveSerial(int s)
  {
      if (s >= mySERIAL_Nums)return;
      activeRxSerial = s;
      if (activeRxSerial<mySERIAL_Nums)
      switch (Port[s])
      {
      case 0:
          activeRxBuffer = rx_buffer0;
          break;
      case 1:
          activeRxBuffer = rx_buffer1;
          break;
    #if !(defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__))
      case 2:
          activeRxBuffer = rx_buffer2;
          break;
      case 3:
          activeRxBuffer = rx_buffer3;
          break;
    #endif
      }
  }
  void MarlinSerial::setTxActiveSerial(int s)
  {
      if (s >= mySERIAL_Nums)return;
      activeTxSerial = s;
    #if TX_BUFFER_SIZE > 0
      if(activeTxSerial<mySERIAL_Nums)
      switch (Port[s])
      {
      case 0:
          activeTxBuffer = tx_buffer0;
          break;
      case 1:
          activeTxBuffer = tx_buffer1;
          break;
        #if !(defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__))
      case 2:
          activeTxBuffer = tx_buffer2;
          break;
      case 3:
          activeTxBuffer = tx_buffer3;
          break;
        #endif
      }
    #endif
  }
#endif
#ifdef mySERIAL_Nums
  void MarlinSerial::end() {
#define _SHUT_PORT(portx) \
    CBI(M_UCSRxB(portx), M_RXENx(portx)); \
    CBI(M_UCSRxB(portx), M_TXENx(portx)); \
    CBI(M_UCSRxB(portx), M_RXCIEx(portx)); \
    CBI(M_UCSRxB(portx), M_UDRIEx(portx));
#define SHUT_PORT(portx) _SHUT_PORT(portx)
        for (char i = 0; i < mySERIAL_Nums; i++)
            switch (Port[i])
        {
            case 0:
                SHUT_PORT(0);
                if (rx_buffer0)
                {
                    delete rx_buffer0;
                    rx_buffer0 = nullptr;
                }
                #if TX_BUFFER_SIZE > 0
                if (tx_buffer0)
                {
                    delete tx_buffer0;
                    tx_buffer0 = nullptr;
                }
                #endif
                break;
            case 1:
                SHUT_PORT(1);
                if (rx_buffer1)
                {
                    delete rx_buffer1;
                    rx_buffer1 = nullptr;
                }
                #if TX_BUFFER_SIZE > 0
                if (tx_buffer1)
                {
                    delete tx_buffer1;
                    tx_buffer1 = nullptr;
                }
                #endif
                break;
            #if !(defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__))
            case 2:
                SHUT_PORT(2);
                if (rx_buffer2)
                {
                    delete rx_buffer2;
                    rx_buffer2 = nullptr;
                }
                #if TX_BUFFER_SIZE > 0
                if (tx_buffer2)
                {
                    delete tx_buffer2;
                    tx_buffer2 = nullptr;
                }
                #endif
                break;
            case 3:
                SHUT_PORT(3);
                if (rx_buffer3)
                {
                    delete rx_buffer3;
                    rx_buffer3 = nullptr;
                }
                #if TX_BUFFER_SIZE > 0
                if (tx_buffer3)
                {
                    delete tx_buffer3;
                    tx_buffer3 = nullptr;
                }
                #endif
                break;
            #endif
        }
    }
#else
  void MarlinSerial::end() {
    CBI(M_UCSRxB, M_RXENx);
    CBI(M_UCSRxB, M_TXENx);
    CBI(M_UCSRxB, M_RXCIEx);
    CBI(M_UCSRxB, M_UDRIEx);
  }
#endif
#ifdef mySERIAL_Nums
void MarlinSerial::checkRx(void) {
#define CHECKRX(portx) \
    if (TEST(M_UCSRxA(portx), M_RXCx(portx))) {\
    uint8_t c = M_UDRx(portx); \
    SERIAL_STORE_CHAR(c, portx); \
    }
    for (char i = 0; i < mySERIAL_Nums; i++)
        switch (Port[i])
    {
        case 0:
            CHECKRX(0);
            break;
        case 1:
            CHECKRX(1);
            break;
#if !(defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__))
        case 2:
            CHECKRX(2);
            break;
        case 3:
            CHECKRX(3);
            break;
#endif
    }
}
#else
  void MarlinSerial::checkRx(void) {
    if (TEST(M_UCSRxA, M_RXCx)) {
      const uint8_t c = M_UDRx;
      store_char(c);
    }
  }
#endif
#ifdef mySERIAL_Nums
  int MarlinSerial::peek() {
      CRITICAL_SECTION_START;
      int v = activeRxBuffer->head == activeRxBuffer->tail ? -1 : activeRxBuffer->buffer[activeRxBuffer->tail];
      CRITICAL_SECTION_END;
      return v;
  }
#else
  int MarlinSerial::peek(void) {
    CRITICAL_SECTION_START;
      const int v = rx_buffer.head == rx_buffer.tail ? -1 : rx_buffer.buffer[rx_buffer.tail];
    CRITICAL_SECTION_END;
    return v;
  }
#endif
#ifdef mySERIAL_Nums
  int MarlinSerial::read() {
      int v;
      CRITICAL_SECTION_START;
      const uint8_t t = activeRxBuffer->tail;
      if (activeRxBuffer->head == t)
          v = -1;
      else {
          v = activeRxBuffer->buffer[t];
          activeRxBuffer->tail = (uint8_t)(t + 1) & (RX_BUFFER_SIZE - 1);
      }
      CRITICAL_SECTION_END;
      return v;
  }
#else
  int MarlinSerial::read(void) {
    int v;
    CRITICAL_SECTION_START;
      const uint8_t t = rx_buffer.tail;
      if (rx_buffer.head == t)
        v = -1;
      else {
        v = rx_buffer.buffer[t];
        rx_buffer.tail = (uint8_t)(t + 1) & (RX_BUFFER_SIZE - 1);
      }
    CRITICAL_SECTION_END;
    return v;
  }
#endif
#ifdef mySERIAL_Nums
  uint8_t MarlinSerial::available() {
      CRITICAL_SECTION_START;
      const uint8_t h = activeRxBuffer->head,
          t = activeRxBuffer->tail;
      CRITICAL_SECTION_END;
      return (uint8_t)(RX_BUFFER_SIZE + h - t) & (RX_BUFFER_SIZE - 1);
  }
#else
  uint8_t MarlinSerial::available(void) {
    CRITICAL_SECTION_START;
      const uint8_t h = rx_buffer.head,
                    t = rx_buffer.tail;
    CRITICAL_SECTION_END;
    return (uint8_t)(RX_BUFFER_SIZE + h - t) & (RX_BUFFER_SIZE - 1);
  }
#endif
#ifdef mySERIAL_Nums
  void MarlinSerial::flush() {
      CRITICAL_SECTION_START;
      activeRxBuffer->head = activeRxBuffer->tail;
      CRITICAL_SECTION_END;
  }
#else
  void MarlinSerial::flush(void) {
    CRITICAL_SECTION_START;
      rx_buffer.head = rx_buffer.tail;
    CRITICAL_SECTION_END;
  }
#endif
#if TX_BUFFER_SIZE > 0
    #ifdef mySERIAL_Nums
      uint8_t MarlinSerial::availableForWrite() {
          CRITICAL_SECTION_START;
          uint8_t h = activeTxBuffer->head;
          uint8_t t = activeTxBuffer->tail;
          CRITICAL_SECTION_END;
          return (uint8_t)(TX_BUFFER_SIZE + h - t) & (TX_BUFFER_SIZE - 1);
      }
    #else
        uint8_t MarlinSerial::availableForWrite(void) {
          CRITICAL_SECTION_START;
            const uint8_t h = tx_buffer.head,
                          t = tx_buffer.tail;
          CRITICAL_SECTION_END;
          return (uint8_t)(TX_BUFFER_SIZE + h - t) & (TX_BUFFER_SIZE - 1);
        }
    #endif
    #ifdef mySERIAL_Nums
      void MarlinSerial::write(uint8_t c) {
          bool empty;
          char needWrite[mySERIAL_Nums] = { 0 };
        #define _sendUDR(portx)  {CRITICAL_SECTION_START;\
          empty = (tx_buffer##portx->head == tx_buffer##portx->tail); \
          CRITICAL_SECTION_END;\
          if (empty&&TEST(M_UCSRxA(portx), M_UDREx(portx))) {\
          CRITICAL_SECTION_START; \
          M_UDRx(portx) = c; \
          SBI(M_UCSRxA(portx), M_TXCx(portx)); \
          CRITICAL_SECTION_END; \
          needWrite[n]=0; }}
          for (char n = 0; n < mySERIAL_Nums; n++)
          if (sPcomm[n] || activeTxSerial == n)
          {
              needWrite[n] = 1;
              switch (Port[n])
              {
              case 0:
                  _written0 = true;
                  _sendUDR(0);
                  break;
              case 1:
                  _written1 = true;
                  _sendUDR(1);
                  break;
#if !(defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__))
              case 2:
                  _written2 = true;
                  _sendUDR(2);
                  break;
              case 3:
                  _written3 = true;
                  _sendUDR(3);
                  break;
#endif
              }
          }
          uint8_t r;
    #define fun(x) _tx_udr_empty_irq##x
    #define _BUFFER_SEND(portx) r = (tx_buffer##portx->head + 1) & (TX_BUFFER_SIZE - 1);\
          while (r == tx_buffer##portx->tail) {\
          if (!TEST(SREG, SREG_I)) {\
          if (TEST(M_UCSRxA(portx), M_UDREx(portx))) \
          fun(portx)();} \
          else {}}\
          tx_buffer##portx->buffer[tx_buffer##portx->head] = c; {\
          CRITICAL_SECTION_START; \
          tx_buffer##portx->head = r; \
          SBI(M_UCSRxB(portx), M_UDRIEx(portx)); \
          CRITICAL_SECTION_END; }
    #define BUFFER_SEND(portx)    _BUFFER_SEND(portx)
          for (char n = 0; n < mySERIAL_Nums; n++)
          if (needWrite[n]==1)
              switch (Port[n])
          {
              case 0:
                  BUFFER_SEND(0);
                  break;
              case 1:
                  BUFFER_SEND(1);
                  break;
#if !(defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__))
              case 2:
                  BUFFER_SEND(2);
                  break;
              case 3:
                  BUFFER_SEND(3);
                  break;
#endif
          }
          return;
      }
    #else
        void MarlinSerial::write(const uint8_t c) {
          _written = true;
          CRITICAL_SECTION_START;
            bool emty = (tx_buffer.head == tx_buffer.tail);
          CRITICAL_SECTION_END;
          if (emty && TEST(M_UCSRxA, M_UDREx)) {
            CRITICAL_SECTION_START;
              M_UDRx = c;
              SBI(M_UCSRxA, M_TXCx);
            CRITICAL_SECTION_END;
            return;
          }
          const uint8_t i = (tx_buffer.head + 1) & (TX_BUFFER_SIZE - 1);
          while (i == tx_buffer.tail) {
            if (!TEST(SREG, SREG_I)) {
              if (TEST(M_UCSRxA, M_UDREx))
                _tx_udr_empty_irq();
            }
            else {
            }
          }
          tx_buffer.buffer[tx_buffer.head] = c;
          { CRITICAL_SECTION_START;
              tx_buffer.head = i;
              SBI(M_UCSRxB, M_UDRIEx);
            CRITICAL_SECTION_END;
          }
          return;
        }
    #endif
    #ifdef mySERIAL_Nums
      void MarlinSerial::flushTX(void) {
        #define _FLUSH_TX(portx)    if(_written##portx){\
          while (TEST(M_UCSRxB(portx), M_UDRIEx(portx)) || !TEST(M_UCSRxA(portx), M_TXCx(portx))) {\
          if (!TEST(SREG, SREG_I) && TEST(M_UCSRxB(portx), M_UDRIEx(portx)))\
          if (TEST(M_UCSRxA(portx), M_UDREx(portx)))\
          _tx_udr_empty_irq##portx();} }
        #define FLUSH_TX(portx) _FLUSH_TX(portx)
          for (char n = 0; n < mySERIAL_Nums; n++)
              switch (Port[n])
          {
              case 0:
                  FLUSH_TX(0);
                  break;
              case 1:
                  FLUSH_TX(1);
                  break;
#if !(defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__))
              case 2:
                  FLUSH_TX(2);
                  break;
              case 3:
                  FLUSH_TX(3);
                  break;
#endif
          }
      }
    #else
        void MarlinSerial::flushTX(void) {
          if (!_written)
            return;
          while (TEST(M_UCSRxB, M_UDRIEx) || !TEST(M_UCSRxA, M_TXCx)) {
            if (!TEST(SREG, SREG_I) && TEST(M_UCSRxB, M_UDRIEx))
              if (TEST(M_UCSRxA, M_UDREx))
                _tx_udr_empty_irq();
          }
      }
    #endif
#else
    #ifdef mySERIAL_Nums
    void MarlinSerial::write(uint8_t c) {
    #define _writer(portx) \
        while (!TEST(M_UCSRxA(portx), M_UDREx(portx)));\
        M_UDRx(portx) = c
        for (char n = 0; n < mySERIAL_Nums; n++)
        if (sPcomm[n] || activeTxSerial == n)
            switch (Port[n])
        {
            case 0:
                _writer(0);
                break;
            case 1:
                _writer(1);
                break;
            #if !(defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__))
            case 2:
                _writer(2);
                break;
            case 3:
                _writer(3);
                break;
            #endif
        }
    }
    #else
        void MarlinSerial::write(uint8_t c) {
          while (!TEST(M_UCSRxA, M_UDREx))
            ;
          M_UDRx = c;
        }
    #endif
#endif
#ifdef mySERIAL_Nums
        void MarlinSerial::storeChar(char c, int port) 
        {
            switch (port)
            {
            case 0:
                if (rx_buffer0){
                       const uint8_t h = rx_buffer0->head, 
                       i = (uint8_t)(h + 1) & (RX_BUFFER_SIZE - 1); 
                       if (i != rx_buffer0->tail) {
                       rx_buffer0->buffer[h] = c; 
                       rx_buffer0->head = i;
                       }}
                break;
            case 1:
                if (rx_buffer1){
                    const uint8_t h = rx_buffer1->head,
                        i = (uint8_t)(h + 1) & (RX_BUFFER_SIZE - 1);
                    if (i != rx_buffer1->tail) {
                        rx_buffer1->buffer[h] = c;
                        rx_buffer1->head = i;
                    }
                }
                break;
            case 2:
                if (rx_buffer2){
                    const uint8_t h = rx_buffer2->head,
                        i = (uint8_t)(h + 1) & (RX_BUFFER_SIZE - 1);
                    if (i != rx_buffer2->tail) {
                        rx_buffer2->buffer[h] = c;
                        rx_buffer2->head = i;
                    }
                }
                break;
            case 3:
                if (rx_buffer3){
                    const uint8_t h = rx_buffer3->head,
                        i = (uint8_t)(h + 1) & (RX_BUFFER_SIZE - 1);
                    if (i != rx_buffer3->tail) {
                        rx_buffer3->buffer[h] = c;
                        rx_buffer3->head = i;
                    }
                }
                break;
            }
        }
#endif
  void MarlinSerial::print(char c, int base) {
    print((long)c, base);
  }
  void MarlinSerial::print(unsigned char b, int base) {
    print((unsigned long)b, base);
  }
  void MarlinSerial::print(int n, int base) {
    print((long)n, base);
  }
  void MarlinSerial::print(unsigned int n, int base) {
    print((unsigned long)n, base);
  }
  void MarlinSerial::print(long n, int base) {
    if (base == 0)
      write(n);
    else if (base == 10) {
      if (n < 0) {
        print('-');
        n = -n;
      }
      printNumber(n, 10);
    }
    else
      printNumber(n, base);
  }
  void MarlinSerial::print(unsigned long n, int base) {
    if (base == 0) write(n);
    else printNumber(n, base);
  }
  void MarlinSerial::print(double n, int digits) {
    printFloat(n, digits);
  }
  void MarlinSerial::println(void) {
    print('\r');
    print('\n');
  }
  void MarlinSerial::println(const String& s) {
    print(s);
    println();
  }
  void MarlinSerial::println(const char c[]) {
    print(c);
    println();
  }
  void MarlinSerial::println(char c, int base) {
    print(c, base);
    println();
  }
  void MarlinSerial::println(unsigned char b, int base) {
    print(b, base);
    println();
  }
  void MarlinSerial::println(int n, int base) {
    print(n, base);
    println();
  }
  void MarlinSerial::println(unsigned int n, int base) {
    print(n, base);
    println();
  }
  void MarlinSerial::println(long n, int base) {
    print(n, base);
    println();
  }
  void MarlinSerial::println(unsigned long n, int base) {
    print(n, base);
    println();
  }
  void MarlinSerial::println(double n, int digits) {
    print(n, digits);
    println();
  }
  void MarlinSerial::printNumber(unsigned long n, uint8_t base) {
    if (n) {
      unsigned char buf[8 * sizeof(long)]; 
      int8_t i = 0;
      while (n) {
        buf[i++] = n % base;
        n /= base;
      }
      while (i--)
        print((char)(buf[i] + (buf[i] < 10 ? '0' : 'A' - 10)));
    }
    else
      print('0');
  }
  void MarlinSerial::printFloat(double number, uint8_t digits) {
    if (number < 0.0) {
      print('-');
      number = -number;
    }
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i)
      rounding *= 0.1;
    number += rounding;
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    print(int_part);
    if (digits) {
      print('.');
      while (digits--) {
        remainder *= 10.0;
        int toPrint = int(remainder);
        print(toPrint);
        remainder -= toPrint;
      }
    }
  }
  MarlinSerial customizedSerial;
#endif 
#if defined(USBCON) && ENABLED(BLUETOOTH)
  HardwareSerial bluetoothSerial;
#endif
