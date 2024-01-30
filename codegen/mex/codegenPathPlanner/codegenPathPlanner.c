/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * codegenPathPlanner.c
 *
 * Code generation for function 'codegenPathPlanner'
 *
 */

/* Include files */
#include "codegenPathPlanner.h"
#include "binaryOccupancyMap.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "plannerHybridAStar.h"
#include "rt_nonfinite.h"
#include "validatorOccupancyMap.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    5,                    /* lineNo */
    "codegenPathPlanner", /* fcnName */
    "C:\\Users\\qbr5kx\\OneDrive - University of Virginia\\Desktop\\UVA\\PhD "
    "Scratch\\NextBestView\\ExampleCode\\utils\\codegenPathPl"
    "anner.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    6,                    /* lineNo */
    "codegenPathPlanner", /* fcnName */
    "C:\\Users\\qbr5kx\\OneDrive - University of Virginia\\Desktop\\UVA\\PhD "
    "Scratch\\NextBestView\\ExampleCode\\utils\\codegenPathPl"
    "anner.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    9,                    /* lineNo */
    "codegenPathPlanner", /* fcnName */
    "C:\\Users\\qbr5kx\\OneDrive - University of Virginia\\Desktop\\UVA\\PhD "
    "Scratch\\NextBestView\\ExampleCode\\utils\\codegenPathPl"
    "anner.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    12,                   /* lineNo */
    "codegenPathPlanner", /* fcnName */
    "C:\\Users\\qbr5kx\\OneDrive - University of Virginia\\Desktop\\UVA\\PhD "
    "Scratch\\NextBestView\\ExampleCode\\utils\\codegenPathPl"
    "anner.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    15,                   /* lineNo */
    "codegenPathPlanner", /* fcnName */
    "C:\\Users\\qbr5kx\\OneDrive - University of Virginia\\Desktop\\UVA\\PhD "
    "Scratch\\NextBestView\\ExampleCode\\utils\\codegenPathPl"
    "anner.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    18,                   /* lineNo */
    "codegenPathPlanner", /* fcnName */
    "C:\\Users\\qbr5kx\\OneDrive - University of Virginia\\Desktop\\UVA\\PhD "
    "Scratch\\NextBestView\\ExampleCode\\utils\\codegenPathPl"
    "anner.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    21,                   /* lineNo */
    "codegenPathPlanner", /* fcnName */
    "C:\\Users\\qbr5kx\\OneDrive - University of Virginia\\Desktop\\UVA\\PhD "
    "Scratch\\NextBestView\\ExampleCode\\utils\\codegenPathPl"
    "anner.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    27,                   /* lineNo */
    "codegenPathPlanner", /* fcnName */
    "C:\\Users\\qbr5kx\\OneDrive - University of Virginia\\Desktop\\UVA\\PhD "
    "Scratch\\NextBestView\\ExampleCode\\utils\\codegenPathPl"
    "anner.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    24,                   /* lineNo */
    "codegenPathPlanner", /* fcnName */
    "C:\\Users\\qbr5kx\\OneDrive - University of Virginia\\Desktop\\UVA\\PhD "
    "Scratch\\NextBestView\\ExampleCode\\utils\\codegenPathPl"
    "anner.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    1,                                       /* lineNo */
    "binaryOccupancyMap/binaryOccupancyMap", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    119,                 /* lineNo */
    "MapLayer/MapLayer", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    318,                 /* lineNo */
    "MapLayer/MapLayer", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    325,                 /* lineNo */
    "MapLayer/MapLayer", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

static emlrtRSInfo n_emlrtRSI = {
    328,                 /* lineNo */
    "MapLayer/MapLayer", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

static emlrtRSInfo o_emlrtRSI = {
    329,                 /* lineNo */
    "MapLayer/MapLayer", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

static emlrtRSInfo p_emlrtRSI = {
    347,                 /* lineNo */
    "MapLayer/MapLayer", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

static emlrtRSInfo q_emlrtRSI = {
    380,                 /* lineNo */
    "MapLayer/MapLayer", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

static emlrtRSInfo r_emlrtRSI = {
    385,                 /* lineNo */
    "MapLayer/MapLayer", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

static emlrtRSInfo s_emlrtRSI = {
    1,                           /* lineNo */
    "MapInterface/MapInterface", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo t_emlrtRSI = {
    1,                               /* lineNo */
    "InternalAccess/InternalAccess", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+map\\+internal\\I"
    "nternalAccess.m" /* pathName */
};

static emlrtRSInfo u_emlrtRSI = {
    1015,                                   /* lineNo */
    "binaryOccupancyMap/parseGridVsMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

static emlrtRSInfo v_emlrtRSI = {
    166,                          /* lineNo */
    "MapUtils/parseGridVsMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\MapUtils.m" /* pathName */
};

static emlrtRSInfo w_emlrtRSI = {
    853,                                /* lineNo */
    "MapInterface/validateMatrixInput", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo bb_emlrtRSI = {
    71,                                        /* lineNo */
    "SharedMapProperties/SharedMapProperties", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\SharedM"
    "apProperties.m" /* pathName */
};

static emlrtRSInfo cb_emlrtRSI = {
    430,                                   /* lineNo */
    "MapInterface/set.LocalOriginInWorld", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo db_emlrtRSI = {
    494,                                  /* lineNo */
    "MapInterface/setLocalOriginInWorld", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo eb_emlrtRSI = {
    30,                                        /* lineNo */
    "CircularBufferIndex/CircularBufferIndex", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\Circula"
    "rBufferIndex.m" /* pathName */
};

static emlrtRSInfo fb_emlrtRSI = {
    35,                              /* lineNo */
    "CircularBuffer/CircularBuffer", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\Circula"
    "rBuffer.m" /* pathName */
};

static emlrtRSInfo gb_emlrtRSI = {
    917,                                   /* lineNo */
    "binaryOccupancyMap/postConstructSet", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

static emlrtRSInfo hb_emlrtRSI = {
    722,                               /* lineNo */
    "binaryOccupancyMap/setOccupancy", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

static emlrtRSInfo ib_emlrtRSI = {
    868,                             /* lineNo */
    "binaryOccupancyMap/setMapData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

static emlrtRSInfo sc_emlrtRSI =
    {
        102,                           /* lineNo */
        "stateSpaceSE2/stateSpaceSE2", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pathName
                                                                          */
};

static emlrtRSInfo tc_emlrtRSI =
    {
        107,                           /* lineNo */
        "stateSpaceSE2/stateSpaceSE2", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pathName
                                                                          */
};

static emlrtRSInfo uc_emlrtRSI = {
    11,                            /* lineNo */
    "HandleCodegen/HandleCodegen", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\autonomouslib\\+"
    "matlabshared\\+planning\\+internal\\HandleCodege"
    "n.m" /* pathName */
};

static emlrtRSInfo vc_emlrtRSI = {
    10,                      /* lineNo */
    "HandleBase/HandleBase", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\autonomouslib\\+"
    "matlabshared\\+planning\\+internal\\HandleBase.m" /* pathName */
};

static emlrtRSInfo xc_emlrtRSI = {
    57,                      /* lineNo */
    "StateSpace/StateSpace", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\StateSpace.m" /* pathName
                                                                         */
};

static emlrtRSInfo bd_emlrtRSI = {
    113,                          /* lineNo */
    "StateSpace/set.StateBounds", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\StateSpace.m" /* pathName
                                                                         */
};

static emlrtRSInfo cd_emlrtRSI = {
    277,                              /* lineNo */
    "StateSpace/validateStateBounds", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\StateSpace.m" /* pathName
                                                                         */
};

static emlrtRTEInfo emlrtRTEI = {
    283,                              /* lineNo */
    17,                               /* colNo */
    "StateSpace/validateStateBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\StateSpace.m" /* pName
                                                                         */
};

static emlrtBCInfo
    emlrtBCI =
        {
            -1,                   /* iFirst */
            -1,                   /* iLast */
            370,                  /* lineNo */
            40,                   /* colNo */
            "",                   /* aName */
            "navPath/get.States", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\navPath.m", /* pName
                                                                         */
            0 /* checkKind */
};

static emlrtDCInfo
    emlrtDCI =
        {
            370,                  /* lineNo */
            42,                   /* colNo */
            "navPath/get.States", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\navPath.m", /* pName
                                                                         */
            1 /* checkKind */
};

static emlrtBCInfo
    b_emlrtBCI =
        {
            -1,                   /* iFirst */
            -1,                   /* iLast */
            370,                  /* lineNo */
            42,                   /* colNo */
            "",                   /* aName */
            "navPath/get.States", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\navPath.m", /* pName
                                                                         */
            0 /* checkKind */
};

static emlrtRTEInfo dd_emlrtRTEI = {
    27,                   /* lineNo */
    5,                    /* colNo */
    "codegenPathPlanner", /* fName */
    "C:\\Users\\qbr5kx\\OneDrive - University of Virginia\\Desktop\\UVA\\PhD "
    "Scratch\\NextBestView\\ExampleCode\\utils\\codegenPathPl"
    "anner.m" /* pName */
};

static emlrtRTEInfo ed_emlrtRTEI = {
    24,                   /* lineNo */
    5,                    /* colNo */
    "codegenPathPlanner", /* fName */
    "C:\\Users\\qbr5kx\\OneDrive - University of Virginia\\Desktop\\UVA\\PhD "
    "Scratch\\NextBestView\\ExampleCode\\utils\\codegenPathPl"
    "anner.m" /* pName */
};

/* Function Definitions */
void codegenPathPlanner(codegenPathPlannerStackData *SD, const emlrtStack *sp,
                        const real_T mapData[10000], const real_T startPose[3],
                        const real_T goalPose[3], real_T infl,
                        emxArray_real_T *path)
{
  binaryOccupancyMap map;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  navPath pathObj;
  stateSpaceSE2 stateSpace;
  validatorOccupancyMap validator;
  real_T b_map;
  real_T *path_data;
  int32_T i;
  int32_T i1;
  int32_T k;
  boolean_T x_data[3];
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInitStruct_navPath(sp, &pathObj, &ed_emlrtRTEI);
  /*  Create a binary occupancy map */
  st.site = &emlrtRSI;
  b_st.site = &j_emlrtRSI;
  map.HasParent = false;
  c_st.site = &k_emlrtRSI;
  d_st.site = &s_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  c_st.site = &l_emlrtRSI;
  d_st.site = &u_emlrtRSI;
  e_st.site = &v_emlrtRSI;
  f_st.site = &w_emlrtRSI;
  g_st.site = &ab_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 10000)) {
    if (!muDoubleScalarIsNaN(mapData[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &g_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:binaryOccupancyMap:expectedNonNaN", 3, 4, 18,
        "input number 1, P,");
  }
  c_st.site = &m_emlrtRSI;
  d_st.site = &bb_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  c_st.site = &n_emlrtRSI;
  map.SharedProperties.GridOriginInLocal[0] = 0.0;
  map.SharedProperties.GridOriginInLocal[1] = 0.0;
  c_st.site = &o_emlrtRSI;
  d_st.site = &cb_emlrtRSI;
  map.SharedProperties.LocalOriginInWorld[0] = 0.0;
  map.SharedProperties.LocalOriginInWorld[1] = 0.0;
  e_st.site = &db_emlrtRSI;
  c_st.site = &p_emlrtRSI;
  d_st.site = &eb_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  map.DefaultValueInternal = false;
  c_st.site = &q_emlrtRSI;
  d_st.site = &fb_emlrtRSI;
  e_st.site = &t_emlrtRSI;
  map.Buffer.Index = &map.Index;
  c_st.site = &r_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  e_st.site = &hb_emlrtRSI;
  f_st.site = &ib_emlrtRSI;
  for (i = 0; i < 10000; i++) {
    map.Buffer.Buffer[i] = (mapData[i] != 0.0);
  }
  real_T c_map;
  map.Index.Head[0] = 1.0;
  map.Index.Head[1] = 1.0;
  st.site = &b_emlrtRSI;
  binaryOccupancyMap_inflate(&st, &map, infl);
  /*  Create a state space object */
  st.site = &c_emlrtRSI;
  stateSpace.WeightXY = 1.0;
  stateSpace.WeightTheta = 0.1;
  b_st.site = &sc_emlrtRSI;
  c_st.site = &uc_emlrtRSI;
  d_st.site = &vc_emlrtRSI;
  b_st.site = &sc_emlrtRSI;
  c_st.site = &wc_emlrtRSI;
  b_st.site = &tc_emlrtRSI;
  c_st.site = &xc_emlrtRSI;
  stateSpace.Name[0] = 'S';
  stateSpace.Name[1] = 'E';
  stateSpace.Name[2] = '2';
  stateSpace.SkipStateValidation = false;
  /*  Update state space bounds to be the same as map limits. */
  st.site = &d_emlrtRSI;
  b_st.site = &yc_emlrtRSI;
  b_map = map.SharedProperties.LocalOriginInWorld[0] +
          map.SharedProperties.GridOriginInLocal[0];
  st.site = &d_emlrtRSI;
  b_st.site = &ad_emlrtRSI;
  c_map = map.SharedProperties.LocalOriginInWorld[1] +
          map.SharedProperties.GridOriginInLocal[1];
  st.site = &d_emlrtRSI;
  stateSpace.StateBoundsInternal[0] = b_map;
  stateSpace.StateBoundsInternal[3] = b_map + 100.0;
  stateSpace.StateBoundsInternal[1] = c_map;
  stateSpace.StateBoundsInternal[4] = c_map + 100.0;
  stateSpace.StateBoundsInternal[2] = -3.1415926535897931;
  stateSpace.StateBoundsInternal[5] = 3.1415926535897931;
  b_st.site = &bd_emlrtRSI;
  c_st.site = &cd_emlrtRSI;
  d_st.site = &ab_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    if (!muDoubleScalarIsNaN(stateSpace.StateBoundsInternal[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:StateSpace:expectedNonNaN", 3, 4, 11, "StateBounds");
  }
  x_data[0] =
      (stateSpace.StateBoundsInternal[0] > stateSpace.StateBoundsInternal[3]);
  x_data[1] =
      (stateSpace.StateBoundsInternal[1] > stateSpace.StateBoundsInternal[4]);
  x_data[2] =
      (stateSpace.StateBoundsInternal[2] > stateSpace.StateBoundsInternal[5]);
  p = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 2)) {
    if (x_data[k]) {
      p = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &emlrtRTEI, "nav:navalgs:statespace:UpperBoundTooSmall",
        "nav:navalgs:statespace:UpperBoundTooSmall", 0);
  }
  /*  Construct a state validator object using the statespace and map object */
  st.site = &e_emlrtRSI;
  c_validatorOccupancyMap_validat(&st, &validator, &stateSpace, &map);
  /*  Set the validation distance for the validator */
  st.site = &f_emlrtRSI;
  validator.ValidationDistance = 0.01;
  /*  Assign the state validator object to the plannerHybridAStar object */
  st.site = &g_emlrtRSI;
  c_plannerHybridAStar_plannerHyb(&st, &SD->f9.planner, &validator);
  /*  Compute a path for the given start and goal poses */
  st.site = &i_emlrtRSI;
  plannerHybridAStar_plan(SD, &st, &SD->f9.planner, startPose, goalPose,
                          &SD->f9.lobj_5, &pathObj);
  /*  Extract the path poses from the path object */
  st.site = &h_emlrtRSI;
  b_map = pathObj.NumStates;
  if (b_map < 1.0) {
    k = 0;
  } else {
    i = pathObj.StateInternal->size[0];
    if (i < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, i, &emlrtBCI, &st);
    }
    if (b_map != (int32_T)muDoubleScalarFloor(b_map)) {
      emlrtIntegerCheckR2012b(b_map, &emlrtDCI, &st);
    }
    i = pathObj.StateInternal->size[0];
    if (((int32_T)b_map < 1) || ((int32_T)b_map > i)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)b_map, 1, i, &b_emlrtBCI, &st);
    }
    k = (int32_T)b_map;
  }
  i = path->size[0] * path->size[1];
  path->size[0] = k;
  path->size[1] = 3;
  emxEnsureCapacity_real_T(&st, path, i, &dd_emlrtRTEI);
  path_data = path->data;
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < k; i1++) {
      path_data[i1 + path->size[0] * i] =
          pathObj.StateInternal->data[i1 + pathObj.StateInternal->size[0] * i];
    }
  }
  emxFreeStruct_navPath(sp, &pathObj);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (codegenPathPlanner.c) */
