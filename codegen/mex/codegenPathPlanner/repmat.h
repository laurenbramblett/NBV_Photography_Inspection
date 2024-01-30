/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * repmat.h
 *
 * Code generation for function 'repmat'
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
void b_repmat(const emlrtStack *sp, real_T varargin_1, emxArray_real_T *b);

void c_repmat(const emlrtStack *sp, const real_T a[3], real_T varargin_1,
              emxArray_real_T *b);

void d_repmat(const emlrtStack *sp, real_T a, real_T varargin_1,
              emxArray_real_T *b);

void e_repmat(const emlrtStack *sp, const real_T a_data[], real_T varargin_2,
              emxArray_real_T *b);

void repmat(const emlrtStack *sp, const real_T a[2], real_T varargin_1,
            emxArray_real_T *b);

/* End of code generation (repmat.h) */
