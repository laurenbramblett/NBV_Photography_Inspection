/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * NameValueParser.c
 *
 * Code generation for function 'NameValueParser'
 *
 */

/* Include files */
#include "NameValueParser.h"
#include "codegenPathPlanner_internal_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
c_robotics_core_internal_NameVa *
NameValueParser_NameValueParser(c_robotics_core_internal_NameVa *obj)
{
  static const char_T b_cv[6] = {'g', 'r', 'e', 'e', 'd', 'y'};
  c_robotics_core_internal_NameVa *b_obj;
  cell_wrap_17 r;
  int32_T i;
  b_obj = obj;
  for (i = 0; i < 6; i++) {
    r.f1[i] = b_cv[i];
  }
  b_obj->Defaults[0] = r;
  return b_obj;
}

/* End of code generation (NameValueParser.c) */
