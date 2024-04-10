/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * bezier.c
 *
 * Code generation for function 'bezier'
 *
 */

/* Include files */
#include "bezier.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static double rt_roundd(double u);

/* Function Definitions */
static double rt_roundd(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }
  return y;
}

void bezier(const double p[6], double l[1500])
{
  double b_u;
  int r;
  int u;
  (void)p;
  memset(&l[0], 0, 1500U * sizeof(double));
  for (u = 0; u < 498; u++) {
    b_u = (double)u * 0.002 + 0.004;
    /* l=cat(1,l,v); */
    /* l(=[l;v];%catenation  */
    r = (int)rt_roundd(500.0 * b_u) - 1;
    /* r = 2,3,4...499 */
    l[r] = (1.0 - b_u) * (1.0 - b_u);
    l[r + 500] = 2.0 * (1.0 - b_u) * b_u;
    l[r + 1000] = b_u * b_u;
  }
  l[0] = 1.0;
  l[1499] = 1.0;
}

/* End of code generation (bezier.c) */
