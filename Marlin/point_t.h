#ifndef __POINT_T__
#define __POINT_T__
struct point_t {
  float x;
  float y;
  float z;
  float e;
  point_t(float const x, float const y)
    : point_t(x, y, NAN, NAN) {}
  point_t(float const x, float const y, float const z)
    : point_t(x, y, z, NAN) {}
  point_t(float const x, float const y, float const z, float const e) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->e = e;
  }
};
#endif 
