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

void bezier(const double p[6], double P[1000])
{
  double l[1500];
  double b_u;
  double d;
  double d1;
  int r;
  int u;
  /* {A function that returns a 500x2 long vector (P) with coordinates along a
   */
  /* bezier curve. In arguments (p) are [x1 y1; PoIx PoIy; x2 y2] PoI is Point
   */
  /* of Intercection */
  /* } */
  /* {size of the coordinate vector that gets return, same as steps in the  */
  /*  vector. */
  /* } */
  memset(&l[0], 0, 1500U * sizeof(double));
  for (u = 0; u < 498; u++) {
    b_u = (double)u * 0.002 + 0.004;
    /* the loop for calculation of the coordinates  */
    r = (int)rt_roundd(500.0 * b_u) - 1;
    /* r = 2,3,4...499 but can be scaled. */
    l[r] = (1.0 - b_u) * (1.0 - b_u);
    l[r + 500] = 2.0 * (1.0 - b_u) * b_u;
    l[r + 1000] = b_u * b_u;
  }
  l[0] = 1.0;
  l[1499] = 1.0;
  for (u = 0; u < 500; u++) {
    b_u = l[u];
    d = l[u + 500];
    d1 = l[u + 1000];
    for (r = 0; r < 2; r++) {
      P[u + 500 * r] = (b_u * p[3 * r] + d * p[3 * r + 1]) + d1 * p[3 * r + 2];
    }
  }
}

/* End of code generation (bezier.c) */
