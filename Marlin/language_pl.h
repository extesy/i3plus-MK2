#ifndef LANGUAGE_PL_H
#define LANGUAGE_PL_H
#define DISPLAY_CHARSET_ISO10646_PL
#define MAPPER_C3C4C5_PL
#if ENABLED(DOGLCD)
  #include "language_pl-DOGM.h"
#else
  #include "language_pl-HD44780.h"
#endif
#endif 
