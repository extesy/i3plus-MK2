#ifndef __DURATION_T__
#define __DURATION_T__
struct duration_t {
  uint32_t value;
  duration_t()
    : duration_t(0) {};
  duration_t(uint32_t const &seconds) {
    this->value = seconds;
  }
  bool operator==(const uint32_t &value) const {
    return (this->value == value);
  }
  bool operator!=(const uint32_t &value) const {
    return ! this->operator==(value);
  }
  inline uint8_t year() const {
    return this->day() / 365;
  }
  inline uint16_t day() const {
    return this->hour() / 24;
  }
  inline uint32_t hour() const {
    return this->minute() / 60;
  }
  inline uint32_t minute() const {
    return this->second() / 60;
  }
  inline uint32_t second() const {
    return this->value;
  }
  void toString(char *buffer) const {
    int y = this->year(),
        d = this->day() % 365,
        h = this->hour() % 24,
        m = this->minute() % 60,
        s = this->second() % 60;
    if (y) sprintf_P(buffer, PSTR("%iy %id %ih %im %is"), y, d, h, m, s);
    else if (d) sprintf_P(buffer, PSTR("%id %ih %im %is"), d, h, m, s);
    else if (h) sprintf_P(buffer, PSTR("%ih %im %is"), h, m, s);
    else if (m) sprintf_P(buffer, PSTR("%im %is"), m, s);
    else sprintf_P(buffer, PSTR("%is"), s);
  }
  uint8_t toDigital(char *buffer, bool with_days=false) const {
    uint16_t h = uint16_t(this->hour()),
             m = uint16_t(this->minute() % 60UL);
    if (with_days) {
      uint16_t d = this->day();
      sprintf_P(buffer, PSTR("%ud %02u:%02u"), d, h % 24, m);
      return d >= 10 ? 8 : 7;
    }
    else if (h < 100) {
      sprintf_P(buffer, PSTR("%02u:%02u"), h % 24, m);
      return 5;
    }
    else {
      sprintf_P(buffer, PSTR("%u:%02u"), h, m);
      return 6;
    }
  }
};
#endif 
