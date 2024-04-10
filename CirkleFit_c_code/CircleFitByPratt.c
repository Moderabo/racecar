/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * CircleFitByPratt.c
 *
 * Code generation for function 'CircleFitByPratt'
 *
 */

/* Include files */
#include "CircleFitByPratt.h"
#include "CircleFitByPratt_types.h"
#include <math.h>
#include <stdio.h>

/* Function Definitions */
void CircleFitByPratt(const emxArray_real_T *XY, double Par[3])
{
  double centroid[2];
  const double *XY_data;
  double A1;
  double A2;
  double A22;
  double Cov_xy;
  double Mxx;
  double Mxy;
  double Mxz;
  double Myy;
  double Myz;
  double Mz;
  double Mzz;
  double Yi;
  double Zi;
  double bsum;
  double ynew;
  int firstBlockLength;
  int hi;
  int ib;
  int k;
  int lastBlockLength;
  int nblocks;
  int xblockoffset;
  int xi;
  int xpageoffset;
  boolean_T exitg1;
  XY_data = XY->data;
  /* --------------------------------------------------------------------------
   */
  /*    */
  /*      Circle fit by Pratt */
  /*       V. Pratt, "Direct least-squares fitting of algebraic surfaces", */
  /*       Computer Graphics, Vol. 21, pages 145-152 (1987) */
  /*  */
  /*      Input:  XY(n,2) is the array of coordinates of n points x(i)=XY(i,1),
   * y(i)=XY(i,2) */
  /*  */
  /*      Output: Par = [a b R] is the fitting circle: */
  /*                            center (a,b) and radius R */
  /*  */
  /*      Note: this fit does not use built-in matrix functions (except "mean"),
   */
  /*            so it can be easily programmed in any programming language */
  /*  */
  /* --------------------------------------------------------------------------
   */
  /*  number of data points */
  if (XY->size[0] == 0) {
    centroid[0] = 0.0;
    centroid[1] = 0.0;
  } else {
    if (XY->size[0] <= 1024) {
      firstBlockLength = XY->size[0];
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = (int)((unsigned int)XY->size[0] >> 10);
      lastBlockLength = XY->size[0] - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }
    for (xi = 0; xi < 2; xi++) {
      xpageoffset = xi * XY->size[0];
      centroid[xi] = XY_data[xpageoffset];
      for (k = 2; k <= firstBlockLength; k++) {
        centroid[xi] += XY_data[(xpageoffset + k) - 1];
      }
      for (ib = 2; ib <= nblocks; ib++) {
        xblockoffset = xpageoffset + ((ib - 1) << 10);
        bsum = XY_data[xblockoffset];
        if (ib == nblocks) {
          hi = lastBlockLength;
        } else {
          hi = 1024;
        }
        for (k = 2; k <= hi; k++) {
          bsum += XY_data[(xblockoffset + k) - 1];
        }
        centroid[xi] += bsum;
      }
    }
  }
  centroid[0] /= (double)XY->size[0];
  centroid[1] /= (double)XY->size[0];
  /*  the centroid of the data set */
  /*      computing moments (note: all moments will be normed, i.e. divided by
   * n) */
  Mxx = 0.0;
  Myy = 0.0;
  Mxy = 0.0;
  Mxz = 0.0;
  Myz = 0.0;
  Mzz = 0.0;
  firstBlockLength = XY->size[0];
  for (nblocks = 0; nblocks < firstBlockLength; nblocks++) {
    bsum = XY_data[nblocks] - centroid[0];
    /*   centering data */
    Yi = XY_data[nblocks + XY->size[0]] - centroid[1];
    /*   centering data */
    A22 = bsum * bsum;
    ynew = Yi * Yi;
    Zi = A22 + ynew;
    Mxy += bsum * Yi;
    Mxx += A22;
    Myy += ynew;
    Mxz += bsum * Zi;
    Myz += Yi * Zi;
    Mzz += Zi * Zi;
  }
  Mxx /= (double)XY->size[0];
  Myy /= (double)XY->size[0];
  Mxy /= (double)XY->size[0];
  Mxz /= (double)XY->size[0];
  Myz /= (double)XY->size[0];
  Mzz /= (double)XY->size[0];
  /*     computing the coefficients of the characteristic polynomial */
  Mz = Mxx + Myy;
  Cov_xy = Mxx * Myy - Mxy * Mxy;
  bsum = Mxz * Mxz;
  Yi = Myz * Myz;
  A2 = (4.0 * Cov_xy - 3.0 * Mz * Mz) - Mzz;
  A22 = Mz * Mz;
  A1 = (((Mzz * Mz + 4.0 * Cov_xy * Mz) - bsum) - Yi) - A22 * Mz;
  Yi = (((bsum * Myy + Yi * Mxx) - Mzz * Cov_xy) - 2.0 * Mxz * Myz * Mxy) +
       A22 * Cov_xy;
  A22 = A2 + A2;
  ynew = 1.0E+20;
  Zi = 0.0;
  /*     Newton's method starting at x=0 */
  firstBlockLength = 0;
  exitg1 = false;
  while ((!exitg1) && (firstBlockLength < 20)) {
    bsum = ynew;
    ynew = Yi + Zi * (A1 + Zi * (A2 + 4.0 * Zi * Zi));
    if (fabs(ynew) > fabs(bsum)) {
      Zi = 0.0;
      exitg1 = true;
    } else {
      bsum = Zi;
      Zi -= ynew / (A1 + Zi * (A22 + 16.0 * Zi * Zi));
      if (fabs((Zi - bsum) / Zi) < 1.0E-12) {
        exitg1 = true;
      } else {
        if (firstBlockLength + 1 >= 20) {
          Zi = 0.0;
        }
        if (Zi < 0.0) {
          printf("Newton-Pratt negative root:  x=%f\n", Zi);
          fflush(stdout);
          Zi = 0.0;
        }
        firstBlockLength++;
      }
    }
  }
  /*     computing the circle parameters */
  bsum = (Zi * Zi - Zi * Mz) + Cov_xy;
  Yi = (Mxz * (Myy - Zi) - Myz * Mxy) / bsum / 2.0;
  bsum = (Myz * (Mxx - Zi) - Mxz * Mxy) / bsum / 2.0;
  Par[0] = Yi + centroid[0];
  Par[1] = bsum + centroid[1];
  Par[2] = sqrt(((Yi * Yi + bsum * bsum) + Mz) + 2.0 * Zi);
}

/* End of code generation (CircleFitByPratt.c) */
