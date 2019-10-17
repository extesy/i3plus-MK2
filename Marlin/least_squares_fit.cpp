#include "MarlinConfig.h"
#if ENABLED(AUTO_BED_LEVELING_UBL)  
#include "macros.h"
#include <math.h>
#include "least_squares_fit.h"
int finish_incremental_LSF(struct linear_fit_data *lsf) {
  const float N = lsf->N;
  if (N == 0.0)
    return 1;
  lsf->xbar /= N;
  lsf->ybar /= N;
  lsf->zbar /= N;
  lsf->x2bar = lsf->x2bar / N - sq(lsf->xbar);
  lsf->y2bar = lsf->y2bar / N - sq(lsf->ybar);
  lsf->z2bar = lsf->z2bar / N - sq(lsf->zbar);
  lsf->xybar = lsf->xybar / N - lsf->xbar * lsf->ybar;
  lsf->yzbar = lsf->yzbar / N - lsf->ybar * lsf->zbar;
  lsf->xzbar = lsf->xzbar / N - lsf->xbar * lsf->zbar;
  const float DD = lsf->x2bar * lsf->y2bar - sq(lsf->xybar);
  if (FABS(DD) <= 1e-10 * (lsf->max_absx + lsf->max_absy))
    return 1;
  lsf->A = (lsf->yzbar * lsf->xybar - lsf->xzbar * lsf->y2bar) / DD;
  lsf->B = (lsf->xzbar * lsf->xybar - lsf->yzbar * lsf->x2bar) / DD;
  lsf->D = -(lsf->zbar + lsf->A * lsf->xbar + lsf->B * lsf->ybar);
  return 0;
}
#endif 
