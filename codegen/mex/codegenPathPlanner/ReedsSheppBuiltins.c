/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ReedsSheppBuiltins.c
 *
 * Code generation for function 'ReedsSheppBuiltins'
 *
 */

/* Include files */
#include "ReedsSheppBuiltins.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include "autonomouscodegen_reeds_shepp_api.hpp"
#include "autonomouscodegen_reeds_shepp_tbb_api.hpp"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo vn_emlrtRSI = {
    89,                                                           /* lineNo */
    "ReedsSheppBuiltins/autonomousReedsSheppInterpolateSegments", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\autonomouslib\\+"
    "matlabshared\\+planning\\+internal\\ReedsSheppBu"
    "iltins.m" /* pathName */
};

static emlrtRTEInfo ig_emlrtRTEI = {
    139,                   /* lineNo */
    36,                    /* colNo */
    "ReedsSheppBuildable", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\autonomouslib\\+"
    "matlabshared\\+planning\\+internal\\+coder\\Reed"
    "sSheppBuildable.m" /* pName */
};

static emlrtRTEInfo jg_emlrtRTEI = {
    140,                   /* lineNo */
    41,                    /* colNo */
    "ReedsSheppBuildable", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\autonomouslib\\+"
    "matlabshared\\+planning\\+internal\\+coder\\Reed"
    "sSheppBuildable.m" /* pName */
};

static emlrtRTEInfo kg_emlrtRTEI = {
    77,                   /* lineNo */
    40,                   /* colNo */
    "ReedsSheppBuiltins", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\autonomouslib\\+"
    "matlabshared\\+planning\\+internal\\ReedsSheppBu"
    "iltins.m" /* pName */
};

static emlrtRTEInfo gj_emlrtRTEI = {
    89,                   /* lineNo */
    18,                   /* colNo */
    "ReedsSheppBuiltins", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\autonomouslib\\+"
    "matlabshared\\+planning\\+internal\\ReedsSheppBu"
    "iltins.m" /* pName */
};

static emlrtRTEInfo hj_emlrtRTEI = {
    89,                   /* lineNo */
    25,                   /* colNo */
    "ReedsSheppBuiltins", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\autonomouslib\\+"
    "matlabshared\\+planning\\+internal\\ReedsSheppBu"
    "iltins.m" /* pName */
};

/* Function Definitions */
real_T c_ReedsSheppBuiltins_autonomous(const real_T startPose[3],
                                       const real_T goalPose[3],
                                       real_T turningRadius, real_T forwardCost,
                                       real_T reverseCost,
                                       real_T motionLengths[5],
                                       real_T motionTypes[5])
{
  real_T cost;
  int32_T i;
  boolean_T allPathTypes[44];
  for (i = 0; i < 44; i++) {
    allPathTypes[i] = true;
  }
  autonomousReedsSheppSegmentsCodegen_tbb_real64(
      &startPose[0], 1U, &goalPose[0], 1U, turningRadius, forwardCost,
      reverseCost, &allPathTypes[0], 0U, 1U, true, 3U, &cost, &motionLengths[0],
      &motionTypes[0]);
  return cost;
}

void d_ReedsSheppBuiltins_autonomous(
    const emlrtStack *sp, const real_T startPose[3], const real_T goalPose[3],
    const emxArray_real_T *samples, real_T turningRadius,
    const real_T segmentsLengths[5], const int32_T segmentsDirections[5],
    const uint32_T segmentsTypes[5], emxArray_real_T *poses)
{
  emlrtStack st;
  emxArray_real_T *directions;
  const real_T *samples_data;
  real_T *directions_data;
  real_T *poses_data;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  samples_data = samples->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &vn_emlrtRSI;
  i = poses->size[0] * poses->size[1];
  poses->size[0] = samples->size[1];
  poses->size[1] = 3;
  emxEnsureCapacity_real_T(&st, poses, i, &ig_emlrtRTEI);
  poses_data = poses->data;
  emxInit_real_T(&st, &directions, 1, &kg_emlrtRTEI);
  i = directions->size[0];
  directions->size[0] = samples->size[1];
  emxEnsureCapacity_real_T(&st, directions, i, &jg_emlrtRTEI);
  directions_data = directions->data;
  autonomousReedsSheppInterpolateSegmentsCodegen_real64(
      &startPose[0], &goalPose[0], &samples_data[0], (uint32_T)samples->size[1],
      turningRadius, &segmentsLengths[0], &segmentsDirections[0],
      &segmentsTypes[0], &poses_data[0], &directions_data[0]);
  emxFree_real_T(&st, &directions);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void e_ReedsSheppBuiltins_autonomous(
    const emlrtStack *sp, const real_T startPose[3], const real_T goalPose[3],
    const emxArray_real_T *samples, real_T turningRadius,
    const real_T segmentsLengths[5], const int32_T segmentsDirections[5],
    const uint32_T segmentsTypes[5], emxArray_real_T *poses,
    emxArray_real_T *directions)
{
  emlrtStack st;
  emxArray_real_T *b_directions;
  emxArray_real_T *b_poses;
  const real_T *samples_data;
  real_T *b_poses_data;
  real_T *directions_data;
  real_T *poses_data;
  int32_T i;
  int32_T loop_ub;
  st.prev = sp;
  st.tls = sp->tls;
  samples_data = samples->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &vn_emlrtRSI;
  emxInit_real_T(&st, &b_poses, 2, &kg_emlrtRTEI);
  i = b_poses->size[0] * b_poses->size[1];
  b_poses->size[0] = samples->size[1];
  b_poses->size[1] = 3;
  emxEnsureCapacity_real_T(&st, b_poses, i, &ig_emlrtRTEI);
  poses_data = b_poses->data;
  emxInit_real_T(&st, &b_directions, 1, &kg_emlrtRTEI);
  i = b_directions->size[0];
  b_directions->size[0] = samples->size[1];
  emxEnsureCapacity_real_T(&st, b_directions, i, &jg_emlrtRTEI);
  directions_data = b_directions->data;
  autonomousReedsSheppInterpolateSegmentsCodegen_real64(
      &startPose[0], &goalPose[0], &samples_data[0], (uint32_T)samples->size[1],
      turningRadius, &segmentsLengths[0], &segmentsDirections[0],
      &segmentsTypes[0], &poses_data[0], &directions_data[0]);
  i = poses->size[0] * poses->size[1];
  poses->size[0] = b_poses->size[0];
  poses->size[1] = 3;
  emxEnsureCapacity_real_T(sp, poses, i, &gj_emlrtRTEI);
  b_poses_data = poses->data;
  loop_ub = b_poses->size[0] * 3;
  for (i = 0; i < loop_ub; i++) {
    b_poses_data[i] = poses_data[i];
  }
  emxFree_real_T(sp, &b_poses);
  i = directions->size[0];
  directions->size[0] = b_directions->size[0];
  emxEnsureCapacity_real_T(sp, directions, i, &hj_emlrtRTEI);
  b_poses_data = directions->data;
  loop_ub = b_directions->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_poses_data[i] = directions_data[i];
  }
  emxFree_real_T(sp, &b_directions);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (ReedsSheppBuiltins.c) */
