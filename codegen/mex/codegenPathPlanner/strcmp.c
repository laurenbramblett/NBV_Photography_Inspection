/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * strcmp.c
 *
 * Code generation for function 'strcmp'
 *
 */

/* Include files */
#include "strcmp.h"
#include "codegenPathPlanner_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
boolean_T b_strcmp(const char_T a_data[], const int32_T a_size[2])
{
  boolean_T b_bool;
  b_bool = false;
  if (a_size[1] == 10) {
    int32_T kstr;
    kstr = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (kstr < 10) {
        if (a_data[kstr] != cv[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return b_bool;
}

boolean_T c_strcmp(const char_T a_data[], const int32_T a_size[2])
{
  static const char_T b_cv[6] = {'g', 'r', 'e', 'e', 'd', 'y'};
  boolean_T b_bool;
  b_bool = false;
  if (a_size[1] == 6) {
    int32_T kstr;
    kstr = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (kstr < 6) {
        if (a_data[kstr] != b_cv[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return b_bool;
}

/* End of code generation (strcmp.c) */
