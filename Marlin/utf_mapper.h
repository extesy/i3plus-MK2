#ifndef UTF_MAPPER_H
#define UTF_MAPPER_H
#include "language.h"
#if ENABLED(DOGLCD)
  #define HARDWARE_CHAR_OUT u8g.print
#else
  #define HARDWARE_CHAR_OUT lcd.write
#endif
#if DISABLED(SIMULATE_ROMFONT) && ENABLED(DOGLCD)
  #if ENABLED(DISPLAY_CHARSET_ISO10646_1)        || ENABLED(DISPLAY_CHARSET_ISO10646_5)        || ENABLED(DISPLAY_CHARSET_ISO10646_KANA)     || ENABLED(DISPLAY_CHARSET_ISO10646_GREEK)    || ENABLED(DISPLAY_CHARSET_ISO10646_TR)
    #define MAPPER_ONE_TO_ONE
  #endif
#else 
  #if DISPLAY_CHARSET_HD44780 == JAPANESE
    #if ENABLED(MAPPER_C2C3)
      const PROGMEM uint8_t utf_recode[] =
           { 
               0x20,0x3F,0xEC,0xED,0x3F,0x5C,0x7C,0x3F,0x22,0x63,0x61,0x7F,0x3F,0x3F,0x52,0xB0,  
               0xDF,0x3F,0x32,0x33,0x27,0xE4,0xF1,0xA5,0x2C,0x31,0xDF,0x7E,0x3F,0x3F,0x3F,0x3F,  
               0x3F,0x3F,0x3F,0x3F,0xE1,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,  
               0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0xEF,0x78,0x3F,0x3F,0x3F,0x3F,0xF5,0x3F,0x3F,0xE2,  
               0x3F,0x3F,0x3F,0x3F,0xE1,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,  
               0x3F,0xEE,0x3F,0x3F,0x3F,0x3F,0xEF,0xFD,0x3F,0x3F,0x3F,0x3F,0xF5,0x3F,0x3F,0x3F   
           };
    #elif ENABLED(MAPPER_E382E383)
      const PROGMEM uint8_t utf_recode[] =
           { 
               0x3D,0xB1,0xB1,0xA8,0xB2,0xA9,0xB3,0xAA,0xB4,0xAB,0xB5,0xB6,0xB6,0xB7,0xB7,0xB8,  
               0xB8,0xB9,0xB9,0xBA,0xBA,0xBB,0xBB,0xBC,0xBC,0xBD,0xBD,0xBE,0xBE,0xBF,0xBF,0xC0,  
               0xC0,0xC1,0xC1,0xC2,0xC2,0xC2,0xC3,0xC3,0xC4,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,  
               0xCA,0xCA,0xCB,0xCB,0xCB,0xCC,0xCC,0xCC,0xCD,0xCD,0xCD,0xCE,0xCE,0xCE,0xCF,0xD0,  
               0xD1,0xD2,0xD3,0xD4,0xD4,0xD5,0xD5,0xAE,0xD6,0xD7,0xD8,0xD9,0xDA,0xDB,0xDC,0xDC,  
               0xEC,0xA7,0xA6,0xDD,0xCC,0x3F,0x3F,0x3F,0x3F,0x3F,0xA6,0xA5,0xB0,0xA4,0xA4,0x3F   
           };
    #elif ENABLED(MAPPER_D0D1)
      #error "Cyrillic on a JAPANESE display makes no sense. There are no matching symbols."
    #endif
  #elif DISPLAY_CHARSET_HD44780 == WESTERN
    #if ENABLED(MAPPER_C2C3)
      const PROGMEM uint8_t utf_recode[] =
           { 
               0x20,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0x22,0xA9,0xAA,0xAB,0x3F,0x3F,0xAE,0x3F,  
               0xB0,0xB1,0xB2,0xB3,0x27,0xB5,0xB6,0xB7,0x2C,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF,  
               0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE,0xCF,  
               0xD0,0xD1,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,0xDF,  
               0xE0,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xEB,0xEC,0xED,0xEE,0xEF,  
               0xF0,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF   
           };
    #elif ENABLED(MAPPER_D0D1)
      #define MAPPER_D0D1_MOD
      const PROGMEM uint8_t utf_recode[] =
           {
               0x41,0x80,0x42,0x92,0x81,0x45,0x82,0x83,0x84,0x85,0x4B,0x86,0x4D,0x48,0x4F,0x87,  
               0x50,0x43,0x54,0x88,0xD8,0x58,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x62,0x8F,0xAC,0xAD,  
               0x61,0x36,0x42,0x92,0x81,0x65,0x82,0xB3,0x84,0x85,0x6B,0x86,0x4D,0x48,0x6F,0x87,  
               0x70,0x63,0x54,0x79,0xD8,0x78,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x62,0x8F,0xAC,0xAD   
            };
    #elif ENABLED(MAPPER_E382E383)
      #error "Katakana on a WESTERN display makes no sense. There are no matching symbols."
    #endif
  #elif DISPLAY_CHARSET_HD44780 == CYRILLIC
    #if ENABLED(MAPPER_D0D1)
      #define MAPPER_D0D1_MOD
      const PROGMEM uint8_t utf_recode[] =
               { 0x41,0xA0,0x42,0xA1,0xE0,0x45,0xA3,0xA4,   
                 0xA5,0xA6,0x4B,0xA7,0x4D,0x48,0x4F,0xA8,   
                 0x50,0x43,0x54,0xA9,0xAA,0x58,0xE1,0xAB,   
                 0xAC,0xE2,0xAD,0xAE,0x62,0xAF,0xB0,0xB1,   
                 0x61,0xB2,0xB3,0xB4,0xE3,0x65,0xB6,0xB7,   
                 0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0x6F,0xBE,   
                 0x70,0x63,0xBF,0x79,0xE4,0x78,0xE5,0xC0,   
                 0xC1,0xE6,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7    
             };                                             
    #elif ENABLED(MAPPER_C2C3)
      #error "Western languages on a CYRILLIC display makes no sense. There are no matching symbols."
    #elif ENABLED(MAPPER_E382E383)
      #error "Katakana on a CYRILLIC display makes no sense. There are no matching symbols."
    #endif
  #else
    #error "Something went wrong in the setting of DISPLAY_CHARSET_HD44780"
  #endif 
#endif 
#define PRINTABLE(C) (((C) & 0xC0u) != 0x80u)
#if ENABLED(MAPPER_C2C3)
  char charset_mapper(const char c) {
    static uint8_t utf_hi_char; 
    static bool seen_c2 = false;
    uint8_t d = c;
    if (d >= 0x80u) { 
      if (d >= 0xC0u && !seen_c2) {
        utf_hi_char = d - 0xC2u;
        seen_c2 = true;
        return 0;
      }
      else if (seen_c2) {
        d &= 0x3Fu;
        #ifndef MAPPER_ONE_TO_ONE
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x20));
        #else
          HARDWARE_CHAR_OUT((char)(0x80u + (utf_hi_char << 6) + d)) ;
        #endif
      }
      else {
        HARDWARE_CHAR_OUT('?');
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_c2 = false;
    return 1;
  }
#elif ENABLED(MAPPER_C2C3_TR)
  char charset_mapper(const char c) {
    static uint8_t utf_hi_char; 
    static bool seen_c2 = false,
                seen_c4 = false,
                seen_c5 = false;
    uint8_t d = c;
    if (d >= 0x80u) { 
           if (d == 0xC4u) { seen_c4 = true; return 0; }
      else if (d == 0xC5u) { seen_c5 = true; return 0; }
      else if (d >= 0xC0u && !seen_c2) {
        utf_hi_char = d - 0xC2u;
        seen_c2 = true;
        return 0;
      }
      else if (seen_c4) {
        switch(d) {
          case 0x9Eu: d = 0xD0u; break;
          case 0x9Fu: d = 0xF0u; break;
          case 0xB0u: d = 0xDDu; break;
          case 0xB1u: d = 0xFDu; break;
          default: d = '?';
        }
        HARDWARE_CHAR_OUT((char)d) ;
      }
      else if (seen_c5) {
        switch(d) {
          case 0x9Eu: d = 0xDEu; break;
          case 0x9Fu: d = 0xFEu; break;
          default: d = '?';
        }
        HARDWARE_CHAR_OUT((char)d) ;
      }
      else if (seen_c2) {
        d &= 0x3Fu;
        #ifndef MAPPER_ONE_TO_ONE
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x20));
        #else
          HARDWARE_CHAR_OUT((char)(0x80u + (utf_hi_char << 6) + d)) ;
        #endif
      }
      else {
        HARDWARE_CHAR_OUT('?');
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_c2 = seen_c4 = seen_c5 = false;
    return 1;
  }
#elif ENABLED(MAPPER_CECF)
  char charset_mapper(const char c) {
    static uint8_t utf_hi_char; 
    static bool seen_ce = false;
    uint8_t d = c;
    if (d >= 0x80) { 
      if (d >= 0xC0 && !seen_ce) {
        utf_hi_char = d - 0xCE;
        seen_ce = true;
        return 0;
      }
      else if (seen_ce) {
        d &= 0x3F;
        #ifndef MAPPER_ONE_TO_ONE
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x20));
        #else
          HARDWARE_CHAR_OUT((char)(0x80 + (utf_hi_char << 6) + d)) ;
        #endif
      }
      else {
        HARDWARE_CHAR_OUT('?');
      }
    }
    else {
      HARDWARE_CHAR_OUT((char)c);
    }
    seen_ce = false;
    return 1;
  }
#elif ENABLED(MAPPER_CECF)
  char charset_mapper(const char c) {
    static uint8_t utf_hi_char; 
    static bool seen_ce = false;
    uint8_t d = c;
    if (d >= 0x80) { 
      if (d >= 0xC0 && !seen_ce) {
        utf_hi_char = d - 0xCE;
        seen_ce = true;
        return 0;
      }
      else if (seen_ce) {
        d &= 0x3F;
        #ifndef MAPPER_ONE_TO_ONE
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x20));
        #else
          HARDWARE_CHAR_OUT((char)(0x80 + (utf_hi_char << 6) + d)) ;
        #endif
      }
      else {
        HARDWARE_CHAR_OUT('?');
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_ce = false;
    return 1;
  }
#elif ENABLED(MAPPER_D0D1_MOD)
  char charset_mapper(const char c) {
    static uint8_t utf_hi_char; 
    static bool seen_d5 = false;
    uint8_t d = c;
    if (d >= 0x80) { 
      if (d >= 0xD0 && !seen_d5) {
        utf_hi_char = d - 0xD0;
        seen_d5 = true;
        return 0;
      }
      else if (seen_d5) {
        d &= 0x3F;
        if (!utf_hi_char && d == 1) {
          HARDWARE_CHAR_OUT((char) 0xA2); 
        }
        else if (utf_hi_char == 1 && d == 0x11) {
          HARDWARE_CHAR_OUT((char)0xB5); 
        }
        else {
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x10));
        }
      }
      else {
        HARDWARE_CHAR_OUT('?');
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_d5 = false;
    return 1;
  }
#elif ENABLED(MAPPER_D0D1)
  char charset_mapper(const char c) {
    static uint8_t utf_hi_char; 
    static bool seen_d5 = false;
    uint8_t d = c;
    if (d >= 0x80u) { 
      if (d >= 0xD0u && !seen_d5) {
        utf_hi_char = d - 0xD0u;
        seen_d5 = true;
        return 0;
      }
      else if (seen_d5) {
        d &= 0x3Fu;
        #ifndef MAPPER_ONE_TO_ONE
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x20));
        #else
          HARDWARE_CHAR_OUT((char)(0xA0u + (utf_hi_char << 6) + d)) ;
        #endif
      }
      else {
        HARDWARE_CHAR_OUT('?');
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_d5 = false;
    return 1;
  }
#elif ENABLED(MAPPER_E382E383)
  char charset_mapper(const char c) {
    static uint8_t utf_hi_char; 
    static bool seen_e3 = false,
                seen_82_83 = false;
    uint8_t d = c;
    if (d >= 0x80) { 
      if (d == 0xE3 && !seen_e3) {
        seen_e3 = true;
        return 0;      
      }
      else if (d >= 0x82 && seen_e3 && !seen_82_83) {
        utf_hi_char = d - 0x82;
        seen_82_83 = true;
        return 0;
      }
      else if (seen_e3 && seen_82_83) {
        d &= 0x3F;
        #ifndef MAPPER_ONE_TO_ONE
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x20));
        #else
          HARDWARE_CHAR_OUT((char)(0x80 + (utf_hi_char << 6) + d)) ;
        #endif
      }
      else {
        HARDWARE_CHAR_OUT((char) '?' );
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_e3 = false;
    seen_82_83 = false;
    return 1;
  }
#elif ENABLED(MAPPER_C3C4C5_PL)
  char charset_mapper(const char c) {
    static bool seen_c3 = false,
                seen_c4 = false,
                seen_c5 = false;
    uint8_t d = c;
    if (d >= 0x80u) { 
           if (d == 0xC4u) { seen_c4 = true; return 0; }
      else if (d == 0xC5u) { seen_c5 = true; return 0; }
      else if (d == 0xC3u) { seen_c3 = true; return 0; }
      else if (seen_c4) {
        switch(d) {
          case 0x84u ... 0x87u: d -= 4; break;  
          case 0x98u ... 0x99u: d -= 20; break; 
          default: d = '?';
        }
        HARDWARE_CHAR_OUT((char)d) ;
      }
      else if (seen_c5) {
        switch(d) {
          case 0x81u ... 0x84u: d += 5; break;  
          case 0x9Au ... 0x9Bu: d -= 0x0Eu; break; 
          case 0xB9u ... 0xBCu: d -= 0x2Bu; break; 
          default: d = '?';
        }
        HARDWARE_CHAR_OUT((char)d) ;
      }
      else if (seen_c3) {
    switch(d) {
          case 0x93u: d = 0x8Au; break; 
          case 0xB3u: d = 0x8Bu; break; 
          d = '?';
        }
        HARDWARE_CHAR_OUT((char)d) ;
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_c3 = seen_c4 = seen_c5 = false;
    return 1;
  }
#elif ENABLED(MAPPER_C3C4C5_CZ)
  char charset_mapper(const char c) {
    static bool seen_c3 = false,
                seen_c4 = false,
                seen_c5 = false;
    uint8_t d = c;
    if (d >= 0x80u) { 
           if (d == 0xC4u) { seen_c4 = true; return 0; }
      else if (d == 0xC5u) { seen_c5 = true; return 0; }
      else if (d == 0xC3u) { seen_c3 = true; return 0; }
      else if (seen_c4) {
        switch(d) {
          case 0x8Cu ... 0x8Fu: break;          
          case 0x9Au ... 0x9Bu: d -= 10; break; 
          default: d = '?';
        }
        HARDWARE_CHAR_OUT((char)d) ;
      }
      else if (seen_c5) {
        switch(d) {
          case 0x87u ... 0x88u: d += 0x0Bu; break;  
          case 0x98u ... 0x99u: d -= 0x04u; break;  
          case 0xA0u ... 0xA1u: d -= 0x0Au; break;  
          case 0xA4u ... 0xA5u: d -= 0x0Cu; break;  
          case 0xAEu ... 0xAFu: d -= 0x14u; break;  
          case 0xBDu ... 0xBEu: d -= 0x21u; break;  
          default: d = '?';
        }
        HARDWARE_CHAR_OUT((char)d) ;
      }
      else if (seen_c3) {
        switch(d) {
          case 0x81u: d = 0x80u; break;  
          case 0x89u: d = 0x81u; break;  
          case 0x8Du: d = 0x82u; break;  
          case 0x93u: d = 0x83u; break;  
          case 0x9Au: d = 0x84u; break;  
          case 0x9Du: d = 0x85u; break;  
          case 0xA1u: d = 0x86u; break;  
          case 0xA9u: d = 0x87u; break;  
          case 0xADu: d = 0x88u; break;  
          case 0xB3u: d = 0x89u; break;  
          case 0xBAu: d = 0x8Au; break;  
          case 0xBDu: d = 0x8Bu; break;  
          default: d = '?';
        }
        HARDWARE_CHAR_OUT((char)d) ;
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_c3 = seen_c4 = seen_c5 = false;
    return 1;
  }
#else
  #define MAPPER_NON
  #undef PRINTABLE
  #define PRINTABLE(C) true
  char charset_mapper(const char c) {
    HARDWARE_CHAR_OUT(c);
    return 1;
  }
#endif 
#endif 
