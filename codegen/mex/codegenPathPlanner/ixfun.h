/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ixfun.h
 *
 * Code generation for function 'ixfun'
 *
 */

#pragma once

/* Include files */
#include "codegenPathPlanner_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void expand_max(const emlrtStack *sp, const emxArray_real_T *a,
                const emxArray_real_T *b, emxArray_real_T *c);

void expand_min(const emlrtStack *sp, const emxArray_real_T *a,
                const emxArray_real_T *b, emxArray_real_T *c);

void expand_mod(const emlrtStack *sp, const emxArray_real_T *a,
                emxArray_real_T *c);

/* End of code generation (ixfun.h) */
