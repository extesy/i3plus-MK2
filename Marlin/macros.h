#ifndef MACROS_H
#define MACROS_H
#define NUM_AXIS 4
#define XYZE 4
#define ABC  3
#define XYZ  3
#define FORCE_INLINE __attribute__((always_inline)) inline
#define _UNUSED      __attribute__((unused))
#define _O0          __attribute__((optimize("O0")))
#define _Os          __attribute__((optimize("Os")))
#define _O1          __attribute__((optimize("O1")))
#define _O2          __attribute__((optimize("O2")))
#define _O3          __attribute__((optimize("O3")))
#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif
#define CYCLES_PER_MICROSECOND (F_CPU / 1000000L) 
#define INT0_PRESCALER 8
#define DELAY_0_NOP NOOP
#define DELAY_1_NOP __asm__("nop\n\t")
#define DELAY_2_NOP DELAY_1_NOP; DELAY_1_NOP
#define DELAY_3_NOP DELAY_1_NOP; DELAY_2_NOP
#define DELAY_4_NOP DELAY_1_NOP; DELAY_3_NOP
#define DELAY_5_NOP DELAY_1_NOP; DELAY_4_NOP
#define DELAY_NOPS(X) \
  switch (X) { \
    case 20: DELAY_1_NOP; case 19: DELAY_1_NOP; \
    case 18: DELAY_1_NOP; case 17: DELAY_1_NOP; \
    case 16: DELAY_1_NOP; case 15: DELAY_1_NOP; \
    case 14: DELAY_1_NOP; case 13: DELAY_1_NOP; \
    case 12: DELAY_1_NOP; case 11: DELAY_1_NOP; \
    case 10: DELAY_1_NOP; case 9:  DELAY_1_NOP; \
    case 8:  DELAY_1_NOP; case 7:  DELAY_1_NOP; \
    case 6:  DELAY_1_NOP; case 5:  DELAY_1_NOP; \
    case 4:  DELAY_1_NOP; case 3:  DELAY_1_NOP; \
    case 2:  DELAY_1_NOP; case 1:  DELAY_1_NOP; \
  }
#define DELAY_10_NOP DELAY_5_NOP;  DELAY_5_NOP
#define DELAY_20_NOP DELAY_10_NOP; DELAY_10_NOP
#if CYCLES_PER_MICROSECOND == 16
  #define DELAY_1US DELAY_10_NOP; DELAY_5_NOP; DELAY_1_NOP
#else
  #define DELAY_1US DELAY_20_NOP
#endif
#define DELAY_2US  DELAY_1US; DELAY_1US
#define DELAY_3US  DELAY_1US; DELAY_2US
#define DELAY_4US  DELAY_1US; DELAY_3US
#define DELAY_5US  DELAY_1US; DELAY_4US
#define DELAY_6US  DELAY_1US; DELAY_5US
#define DELAY_7US  DELAY_1US; DELAY_6US
#define DELAY_8US  DELAY_1US; DELAY_7US
#define DELAY_9US  DELAY_1US; DELAY_8US
#define DELAY_10US DELAY_1US; DELAY_9US
#define UNUSED(x) (void) (x)
#define STRINGIFY_(M) #M
#define STRINGIFY(M) STRINGIFY_(M)
#define TEST(n,b) (((n)&_BV(b))!=0)
#define SBI(n,b) (n |= _BV(b))
#define CBI(n,b) (n &= ~_BV(b))
#define SET_BIT(n,b,value) (n) ^= ((-value)^(n)) & (_BV(b))
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif
#define RADIANS(d) ((d)*M_PI/180.0)
#define DEGREES(r) ((r)*180.0/M_PI)
#define HYPOT2(x,y) (sq(x)+sq(y))
#define SIGN(a) ((a>0)-(a<0))
#define NOLESS(v,n) do{ if (v < n) v = n; }while(0)
#define NOMORE(v,n) do{ if (v > n) v = n; }while(0)
#define _CAT(a, ...) a ## __VA_ARGS__
#define SWITCH_ENABLED_false 0
#define SWITCH_ENABLED_true  1
#define SWITCH_ENABLED_0     0
#define SWITCH_ENABLED_1     1
#define SWITCH_ENABLED_      1
#define ENABLED(b) _CAT(SWITCH_ENABLED_, b)
#define DISABLED(b) (!_CAT(SWITCH_ENABLED_, b))
#define WITHIN(V,L,H) ((V) >= (L) && (V) <= (H))
#define NUMERIC(a) WITHIN(a, '0', '9')
#define DECIMAL(a) (NUMERIC(a) || a == '.')
#define NUMERIC_SIGNED(a) (NUMERIC(a) || (a) == '-' || (a) == '+')
#define DECIMAL_SIGNED(a) (DECIMAL(a) || (a) == '-' || (a) == '+')
#define COUNT(a) (sizeof(a)/sizeof(*a))
#define ZERO(a) memset(a,0,sizeof(a))
#define COPY(a,b) memcpy(a,b,min(sizeof(a),sizeof(b)))
#define ARRAY_6(v1, v2, v3, v4, v5, v6, ...) { v1, v2, v3, v4, v5, v6 }
#define ARRAY_5(v1, v2, v3, v4, v5, ...)     { v1, v2, v3, v4, v5 }
#define ARRAY_4(v1, v2, v3, v4, ...)         { v1, v2, v3, v4 }
#define ARRAY_3(v1, v2, v3, ...)             { v1, v2, v3 }
#define ARRAY_2(v1, v2, ...)                 { v1, v2 }
#define ARRAY_1(v1, ...)                     { v1 }
#define _ARRAY_N(N, ...) ARRAY_ ##N(__VA_ARGS__)
#define ARRAY_N(N, ...) _ARRAY_N(N, __VA_ARGS__)
#define INC_0 1
#define INC_1 2
#define INC_2 3
#define INC_3 4
#define INC_4 5
#define INC_5 6
#define INC_6 7
#define INC_7 8
#define INC_8 9
#define INCREMENT_(n) INC_ ##n
#define INCREMENT(n) INCREMENT_(n)
#define DEC_1 0
#define DEC_2 1
#define DEC_3 2
#define DEC_4 3
#define DEC_5 4
#define DEC_6 5
#define DEC_7 6
#define DEC_8 7
#define DEC_9 8
#define DECREMENT_(n) DEC_ ##n
#define DECREMENT(n) DECREMENT_(n)
#define PIN_EXISTS(PN) (defined(PN ##_PIN) && PN ##_PIN >= 0)
#define PENDING(NOW,SOON) ((long)(NOW-(SOON))<0)
#define ELAPSED(NOW,SOON) (!PENDING(NOW,SOON))
#define NOOP do{} while(0)
#define CEILING(x,y) (((x) + (y) - 1) / (y))
#define MIN3(a, b, c)       min(min(a, b), c)
#define MIN4(a, b, c, d)    min(MIN3(a, b, c), d)
#define MIN5(a, b, c, d, e) min(MIN4(a, b, c, d), e)
#define MAX3(a, b, c)       max(max(a, b), c)
#define MAX4(a, b, c, d)    max(MAX3(a, b, c), d)
#define MAX5(a, b, c, d, e) max(MAX4(a, b, c, d), e)
#define UNEAR_ZERO(x) ((x) < 0.000001)
#define NEAR_ZERO(x) WITHIN(x, -0.000001, 0.000001)
#define NEAR(x,y) NEAR_ZERO((x)-(y))
#define RECIPROCAL(x) (NEAR_ZERO(x) ? 0.0 : 1.0 / (x))
#define FIXFLOAT(f) (f + 0.00001)
#define ATAN2(y, x)     atan2(y, x)
#define FABS(x)         fabs(x)
#define POW(x, y)       pow(x, y)
#define SQRT(x)         sqrt(x)
#define CEIL(x)         ceil(x)
#define FLOOR(x)        floor(x)
#define LROUND(x)       lround(x)
#define FMOD(x, y)      fmod(x, y)
#define HYPOT(x,y)      SQRT(HYPOT2(x,y))
#define _TO_STR(x,y,z)  #x#y#z
#define X_AXIS_STAT     PSTR(_TO_STR(de,ve,lo))
#define Y_AXIS_STAT     PSTR(_TO_STR(pe,r -, Y.))
#define Z_AXIS_STAT     PSTR(_TO_STR(S.,Z., ))
#endif 
