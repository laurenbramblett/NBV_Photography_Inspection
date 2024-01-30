/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * linspace.c
 *
 * Code generation for function 'linspace'
 *
 */

/* Include files */
#include "linspace.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void linspace(real_T d1, real_T d2, real_T y[5])
{
  y[4] = d2;
  y[0] = d1;
  if (d1 == -d2) {
    real_T delta1;
    delta1 = d2 / 4.0;
    y[1] = -2.0 * delta1;
    y[3] = 2.0 * delta1;
    y[2] = 0.0;
  } else if (((d1 < 0.0) != (d2 < 0.0)) &&
             ((muDoubleScalarAbs(d1) > 8.9884656743115785E+307) ||
              (muDoubleScalarAbs(d2) > 8.9884656743115785E+307))) {
    real_T delta1;
    real_T delta2;
    delta1 = d1 / 4.0;
    delta2 = d2 / 4.0;
    y[1] = (d1 + delta2) - delta1;
    y[2] = (d1 + delta2 * 2.0) - delta1 * 2.0;
    y[3] = (d1 + delta2 * 3.0) - delta1 * 3.0;
  } else {
    real_T delta1;
    delta1 = (d2 - d1) / 4.0;
    y[1] = d1 + delta1;
    y[2] = d1 + 2.0 * delta1;
    y[3] = d1 + 3.0 * delta1;
  }
}

/* End of code generation (linspace.c) */
