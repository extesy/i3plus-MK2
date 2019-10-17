#include "qr_solve.h"
#if ENABLED(AUTO_BED_LEVELING_LINEAR)
#include <stdlib.h>
#include <math.h>
int i4_min(int i1, int i2)
{
  return (i1 < i2) ? i1 : i2;
}
float r8_epsilon(void)
{
  const float value = 2.220446049250313E-016;
  return value;
}
float r8_max(float x, float y)
{
  return (y < x) ? x : y;
}
float r8_abs(float x)
{
  return (x < 0.0) ? -x : x;
}
float r8_sign(float x)
{
  return (x < 0.0) ? -1.0 : 1.0;
}
float r8mat_amax(int m, int n, float a[])
{
  float value = r8_abs(a[0 + 0 * m]);
  for (int j = 0; j < n; j++) {
    for (int i = 0; i < m; i++) {
      NOLESS(value, r8_abs(a[i + j * m]));
    }
  }
  return value;
}
void r8mat_copy(float a2[], int m, int n, float a1[])
{
  for (int j = 0; j < n; j++) {
    for (int i = 0; i < m; i++)
      a2[i + j * m] = a1[i + j * m];
  }
}
void daxpy(int n, float da, float dx[], int incx, float dy[], int incy)
{
  if (n <= 0 || da == 0.0) return;
  int i, ix, iy, m;
  if (incx != 1 || incy != 1) {
    if (0 <= incx)
      ix = 0;
    else
      ix = (- n + 1) * incx;
    if (0 <= incy)
      iy = 0;
    else
      iy = (- n + 1) * incy;
    for (i = 0; i < n; i++) {
      dy[iy] = dy[iy] + da * dx[ix];
      ix = ix + incx;
      iy = iy + incy;
    }
  }
  else {
    m = n % 4;
    for (i = 0; i < m; i++)
      dy[i] = dy[i] + da * dx[i];
    for (i = m; i < n; i = i + 4) {
      dy[i  ] = dy[i  ] + da * dx[i  ];
      dy[i + 1] = dy[i + 1] + da * dx[i + 1];
      dy[i + 2] = dy[i + 2] + da * dx[i + 2];
      dy[i + 3] = dy[i + 3] + da * dx[i + 3];
    }
  }
}
float ddot(int n, float dx[], int incx, float dy[], int incy)
{
  if (n <= 0) return 0.0;
  int i, m;
  float dtemp = 0.0;
  if (incx != 1 || incy != 1) {
    int ix = (incx >= 0) ? 0 : (-n + 1) * incx,
        iy = (incy >= 0) ? 0 : (-n + 1) * incy;
    for (i = 0; i < n; i++) {
      dtemp += dx[ix] * dy[iy];
      ix = ix + incx;
      iy = iy + incy;
    }
  }
  else {
    m = n % 5;
    for (i = 0; i < m; i++)
      dtemp += dx[i] * dy[i];
    for (i = m; i < n; i = i + 5) {
      dtemp += dx[i] * dy[i]
              + dx[i + 1] * dy[i + 1]
              + dx[i + 2] * dy[i + 2]
              + dx[i + 3] * dy[i + 3]
              + dx[i + 4] * dy[i + 4];
    }
  }
  return dtemp;
}
float dnrm2(int n, float x[], int incx)
{
  float norm;
  if (n < 1 || incx < 1)
    norm = 0.0;
  else if (n == 1)
    norm = r8_abs(x[0]);
  else {
    float scale = 0.0, ssq = 1.0;
    int ix = 0;
    for (int i = 0; i < n; i++) {
      if (x[ix] != 0.0) {
        float absxi = r8_abs(x[ix]);
        if (scale < absxi) {
          ssq = 1.0 + ssq * (scale / absxi) * (scale / absxi);
          scale = absxi;
        }
        else
          ssq = ssq + (absxi / scale) * (absxi / scale);
      }
      ix += incx;
    }
    norm = scale * SQRT(ssq);
  }
  return norm;
}
void dqrank(float a[], int lda, int m, int n, float tol, int* kr,
            int jpvt[], float qraux[])
{
  float work[n];
  for (int i = 0; i < n; i++)
    jpvt[i] = 0;
  int job = 1;
  dqrdc(a, lda, m, n, qraux, jpvt, work, job);
  *kr = 0;
  int k = i4_min(m, n);
  for (int j = 0; j < k; j++) {
    if (r8_abs(a[j + j * lda]) <= tol * r8_abs(a[0 + 0 * lda]))
      return;
    *kr = j + 1;
  }
}
void dqrdc(float a[], int lda, int n, int p, float qraux[], int jpvt[],
           float work[], int job)
{
  int jp;
  int j;
  int lup;
  int maxj;
  float maxnrm, nrmxl, t, tt;
  int pl = 1, pu = 0;
  if (job != 0) {
    for (j = 1; j <= p; j++) {
      int swapj = (0 < jpvt[j - 1]);
      jpvt[j - 1] = (jpvt[j - 1] < 0) ? -j : j;
      if (swapj) {
        if (j != pl)
          dswap(n, a + 0 + (pl - 1)*lda, 1, a + 0 + (j - 1), 1);
        jpvt[j - 1] = jpvt[pl - 1];
        jpvt[pl - 1] = j;
        pl++;
      }
    }
    pu = p;
    for (j = p; 1 <= j; j--) {
      if (jpvt[j - 1] < 0) {
        jpvt[j - 1] = -jpvt[j - 1];
        if (j != pu) {
          dswap(n, a + 0 + (pu - 1)*lda, 1, a + 0 + (j - 1)*lda, 1);
          jp = jpvt[pu - 1];
          jpvt[pu - 1] = jpvt[j - 1];
          jpvt[j - 1] = jp;
        }
        pu = pu - 1;
      }
    }
  }
  for (j = pl; j <= pu; j++)
    qraux[j - 1] = dnrm2(n, a + 0 + (j - 1) * lda, 1);
  for (j = pl; j <= pu; j++)
    work[j - 1] = qraux[j - 1];
  lup = i4_min(n, p);
  for (int l = 1; l <= lup; l++) {
    if (pl <= l && l < pu) {
      maxnrm = 0.0;
      maxj = l;
      for (j = l; j <= pu; j++) {
        if (maxnrm < qraux[j - 1]) {
          maxnrm = qraux[j - 1];
          maxj = j;
        }
      }
      if (maxj != l) {
        dswap(n, a + 0 + (l - 1)*lda, 1, a + 0 + (maxj - 1)*lda, 1);
        qraux[maxj - 1] = qraux[l - 1];
        work[maxj - 1] = work[l - 1];
        jp = jpvt[maxj - 1];
        jpvt[maxj - 1] = jpvt[l - 1];
        jpvt[l - 1] = jp;
      }
    }
    qraux[l - 1] = 0.0;
    if (l != n) {
      nrmxl = dnrm2(n - l + 1, a + l - 1 + (l - 1) * lda, 1);
      if (nrmxl != 0.0) {
        if (a[l - 1 + (l - 1)*lda] != 0.0)
          nrmxl = nrmxl * r8_sign(a[l - 1 + (l - 1) * lda]);
        dscal(n - l + 1, 1.0 / nrmxl, a + l - 1 + (l - 1)*lda, 1);
        a[l - 1 + (l - 1)*lda] = 1.0 + a[l - 1 + (l - 1) * lda];
        for (j = l + 1; j <= p; j++) {
          t = -ddot(n - l + 1, a + l - 1 + (l - 1) * lda, 1, a + l - 1 + (j - 1) * lda, 1)
              / a[l - 1 + (l - 1) * lda];
          daxpy(n - l + 1, t, a + l - 1 + (l - 1)*lda, 1, a + l - 1 + (j - 1)*lda, 1);
          if (pl <= j && j <= pu) {
            if (qraux[j - 1] != 0.0) {
              tt = 1.0 - POW(r8_abs(a[l - 1 + (j - 1) * lda]) / qraux[j - 1], 2);
              tt = r8_max(tt, 0.0);
              t = tt;
              tt = 1.0 + 0.05 * tt * POW(qraux[j - 1] / work[j - 1], 2);
              if (tt != 1.0)
                qraux[j - 1] = qraux[j - 1] * SQRT(t);
              else {
                qraux[j - 1] = dnrm2(n - l, a + l + (j - 1) * lda, 1);
                work[j - 1] = qraux[j - 1];
              }
            }
          }
        }
        qraux[l - 1] = a[l - 1 + (l - 1) * lda];
        a[l - 1 + (l - 1)*lda] = -nrmxl;
      }
    }
  }
}
int dqrls(float a[], int lda, int m, int n, float tol, int* kr, float b[],
          float x[], float rsd[], int jpvt[], float qraux[], int itask)
{
  int ind;
  if (lda < m) {
    ind = -1;
    return ind;
  }
  if (n <= 0) {
    ind = -2;
    return ind;
  }
  if (itask < 1) {
    ind = -3;
    return ind;
  }
  ind = 0;
  if (itask == 1)
    dqrank(a, lda, m, n, tol, kr, jpvt, qraux);
  dqrlss(a, lda, m, n, *kr, b, x, rsd, jpvt, qraux);
  return ind;
}
void dqrlss(float a[], int lda, int m, int n, int kr, float b[], float x[],
            float rsd[], int jpvt[], float qraux[])
{
  int i;
  int info;
  int j;
  int job;
  int k;
  float t;
  if (kr != 0) {
    job = 110;
    info = dqrsl(a, lda, m, kr, qraux, b, rsd, rsd, x, rsd, rsd, job); UNUSED(info);
  }
  for (i = 0; i < n; i++)
    jpvt[i] = - jpvt[i];
  for (i = kr; i < n; i++)
    x[i] = 0.0;
  for (j = 1; j <= n; j++) {
    if (jpvt[j - 1] <= 0) {
      k = - jpvt[j - 1];
      jpvt[j - 1] = k;
      while (k != j) {
        t = x[j - 1];
        x[j - 1] = x[k - 1];
        x[k - 1] = t;
        jpvt[k - 1] = -jpvt[k - 1];
        k = jpvt[k - 1];
      }
    }
  }
}
int dqrsl(float a[], int lda, int n, int k, float qraux[], float y[],
          float qy[], float qty[], float b[], float rsd[], float ab[], int job)
{
  int cab;
  int cb;
  int cqty;
  int cqy;
  int cr;
  int i;
  int info;
  int j;
  int jj;
  int ju;
  float t;
  float temp;
  info = 0;
  cqy  = ( job / 10000        != 0);
  cqty = ((job % 10000)       != 0);
  cb   = ((job %  1000) / 100 != 0);
  cr   = ((job %   100) /  10 != 0);
  cab  = ((job %    10)       != 0);
  ju = i4_min(k, n - 1);
  if (ju == 0) {
    if (cqy)
      qy[0] = y[0];
    if (cqty)
      qty[0] = y[0];
    if (cab)
      ab[0] = y[0];
    if (cb) {
      if (a[0 + 0 * lda] == 0.0)
        info = 1;
      else
        b[0] = y[0] / a[0 + 0 * lda];
    }
    if (cr)
      rsd[0] = 0.0;
    return info;
  }
  if (cqy) {
    for (i = 1; i <= n; i++)
      qy[i - 1] = y[i - 1];
  }
  if (cqty) {
    for (i = 1; i <= n; i++)
      qty[i - 1] = y[i - 1];
  }
  if (cqy) {
    for (jj = 1; jj <= ju; jj++) {
      j = ju - jj + 1;
      if (qraux[j - 1] != 0.0) {
        temp = a[j - 1 + (j - 1) * lda];
        a[j - 1 + (j - 1)*lda] = qraux[j - 1];
        t = -ddot(n - j + 1, a + j - 1 + (j - 1) * lda, 1, qy + j - 1, 1) / a[j - 1 + (j - 1) * lda];
        daxpy(n - j + 1, t, a + j - 1 + (j - 1)*lda, 1, qy + j - 1, 1);
        a[j - 1 + (j - 1)*lda] = temp;
      }
    }
  }
  if (cqty) {
    for (j = 1; j <= ju; j++) {
      if (qraux[j - 1] != 0.0) {
        temp = a[j - 1 + (j - 1) * lda];
        a[j - 1 + (j - 1)*lda] = qraux[j - 1];
        t = -ddot(n - j + 1, a + j - 1 + (j - 1) * lda, 1, qty + j - 1, 1) / a[j - 1 + (j - 1) * lda];
        daxpy(n - j + 1, t, a + j - 1 + (j - 1)*lda, 1, qty + j - 1, 1);
        a[j - 1 + (j - 1)*lda] = temp;
      }
    }
  }
  if (cb) {
    for (i = 1; i <= k; i++)
      b[i - 1] = qty[i - 1];
  }
  if (cab) {
    for (i = 1; i <= k; i++)
      ab[i - 1] = qty[i - 1];
  }
  if (cr && k < n) {
    for (i = k + 1; i <= n; i++)
      rsd[i - 1] = qty[i - 1];
  }
  if (cab && k + 1 <= n) {
    for (i = k + 1; i <= n; i++)
      ab[i - 1] = 0.0;
  }
  if (cr) {
    for (i = 1; i <= k; i++)
      rsd[i - 1] = 0.0;
  }
  if (cb) {
    for (jj = 1; jj <= k; jj++) {
      j = k - jj + 1;
      if (a[j - 1 + (j - 1)*lda] == 0.0) {
        info = j;
        break;
      }
      b[j - 1] = b[j - 1] / a[j - 1 + (j - 1) * lda];
      if (j != 1) {
        t = -b[j - 1];
        daxpy(j - 1, t, a + 0 + (j - 1)*lda, 1, b, 1);
      }
    }
  }
  if (cr || cab) {
    for (jj = 1; jj <= ju; jj++) {
      j = ju - jj + 1;
      if (qraux[j - 1] != 0.0) {
        temp = a[j - 1 + (j - 1) * lda];
        a[j - 1 + (j - 1)*lda] = qraux[j - 1];
        if (cr) {
          t = -ddot(n - j + 1, a + j - 1 + (j - 1) * lda, 1, rsd + j - 1, 1)
              / a[j - 1 + (j - 1) * lda];
          daxpy(n - j + 1, t, a + j - 1 + (j - 1)*lda, 1, rsd + j - 1, 1);
        }
        if (cab) {
          t = -ddot(n - j + 1, a + j - 1 + (j - 1) * lda, 1, ab + j - 1, 1)
              / a[j - 1 + (j - 1) * lda];
          daxpy(n - j + 1, t, a + j - 1 + (j - 1)*lda, 1, ab + j - 1, 1);
        }
        a[j - 1 + (j - 1)*lda] = temp;
      }
    }
  }
  return info;
}
void dscal(int n, float sa, float x[], int incx)
{
  int i;
  int ix;
  int m;
  if (n <= 0) return;
  if (incx == 1) {
    m = n % 5;
    for (i = 0; i < m; i++)
      x[i] = sa * x[i];
    for (i = m; i < n; i = i + 5) {
      x[i]   = sa * x[i];
      x[i + 1] = sa * x[i + 1];
      x[i + 2] = sa * x[i + 2];
      x[i + 3] = sa * x[i + 3];
      x[i + 4] = sa * x[i + 4];
    }
  }
  else {
    if (0 <= incx)
      ix = 0;
    else
      ix = (- n + 1) * incx;
    for (i = 0; i < n; i++) {
      x[ix] = sa * x[ix];
      ix = ix + incx;
    }
  }
}
void dswap(int n, float x[], int incx, float y[], int incy)
{
  if (n <= 0) return;
  int i, ix, iy, m;
  float temp;
  if (incx == 1 && incy == 1) {
    m = n % 3;
    for (i = 0; i < m; i++) {
      temp = x[i];
      x[i] = y[i];
      y[i] = temp;
    }
    for (i = m; i < n; i = i + 3) {
      temp = x[i];
      x[i] = y[i];
      y[i] = temp;
      temp = x[i + 1];
      x[i + 1] = y[i + 1];
      y[i + 1] = temp;
      temp = x[i + 2];
      x[i + 2] = y[i + 2];
      y[i + 2] = temp;
    }
  }
  else {
    ix = (incx >= 0) ? 0 : (-n + 1) * incx;
    iy = (incy >= 0) ? 0 : (-n + 1) * incy;
    for (i = 0; i < n; i++) {
      temp = x[ix];
      x[ix] = y[iy];
      y[iy] = temp;
      ix = ix + incx;
      iy = iy + incy;
    }
  }
}
void qr_solve(float x[], int m, int n, float a[], float b[])
{
  float a_qr[n * m], qraux[n], r[m], tol;
  int ind, itask, jpvt[n], kr, lda;
  r8mat_copy(a_qr, m, n, a);
  lda = m;
  tol = r8_epsilon() / r8mat_amax(m, n, a_qr);
  itask = 1;
  ind = dqrls(a_qr, lda, m, n, tol, &kr, b, x, r, jpvt, qraux, itask); UNUSED(ind);
}
#endif
