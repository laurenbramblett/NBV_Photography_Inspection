/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * plannerAStarGrid.c
 *
 * Code generation for function 'plannerAStarGrid'
 *
 */

/* Include files */
#include "plannerAStarGrid.h"
#include "MapInterface.h"
#include "MapLayer.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_mexutil.h"
#include "codegenPathPlanner_types.h"
#include "plannerAStarGrid1.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "validateAStarBuiltinCostFunction.h"
#include "warning.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo th_emlrtRSI = {
    249,                                 /* lineNo */
    "plannerAStarGrid/plannerAStarGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo uh_emlrtRSI = {
    263,                                 /* lineNo */
    "plannerAStarGrid/plannerAStarGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo vh_emlrtRSI = {
    282,                                 /* lineNo */
    "plannerAStarGrid/plannerAStarGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo wh_emlrtRSI = {
    285,                                 /* lineNo */
    "plannerAStarGrid/plannerAStarGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo xh_emlrtRSI = {
    295,                                 /* lineNo */
    "plannerAStarGrid/plannerAStarGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo yh_emlrtRSI = {
    298,                                 /* lineNo */
    "plannerAStarGrid/plannerAStarGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ai_emlrtRSI = {
    306,                                 /* lineNo */
    "plannerAStarGrid/plannerAStarGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo bi_emlrtRSI = {
    315,                                 /* lineNo */
    "plannerAStarGrid/plannerAStarGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ci_emlrtRSI = {
    320,                                 /* lineNo */
    "plannerAStarGrid/plannerAStarGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo di_emlrtRSI = {
    323,                                 /* lineNo */
    "plannerAStarGrid/plannerAStarGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ei_emlrtRSI = {
    338,                        /* lineNo */
    "plannerAStarGrid/set.Map", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo hi_emlrtRSI = {
    390,                             /* lineNo */
    "plannerAStarGrid/set.GCostFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ii_emlrtRSI = {
    391,                             /* lineNo */
    "plannerAStarGrid/set.GCostFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ji_emlrtRSI = {
    411,                          /* lineNo */
    "plannerAStarGrid/set.GCost", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ki_emlrtRSI = {
    412,                          /* lineNo */
    "plannerAStarGrid/set.GCost", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo li_emlrtRSI = {
    413,                          /* lineNo */
    "plannerAStarGrid/set.GCost", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo mi_emlrtRSI = {
    379,                             /* lineNo */
    "plannerAStarGrid/set.HCostFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ni_emlrtRSI = {
    380,                             /* lineNo */
    "plannerAStarGrid/set.HCostFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo oi_emlrtRSI = {
    400,                          /* lineNo */
    "plannerAStarGrid/set.HCost", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo pi_emlrtRSI = {
    401,                          /* lineNo */
    "plannerAStarGrid/set.HCost", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo qi_emlrtRSI = {
    402,                          /* lineNo */
    "plannerAStarGrid/set.HCost", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ri_emlrtRSI = {
    648,                          /* lineNo */
    "plannerAStarGrid/updateMap", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo kj_emlrtRSI = {
    498,                     /* lineNo */
    "plannerAStarGrid/plan", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo lj_emlrtRSI = {
    500,                     /* lineNo */
    "plannerAStarGrid/plan", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo mj_emlrtRSI = {
    511,                     /* lineNo */
    "plannerAStarGrid/plan", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo nj_emlrtRSI = {
    512,                     /* lineNo */
    "plannerAStarGrid/plan", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo oj_emlrtRSI = {
    513,                     /* lineNo */
    "plannerAStarGrid/plan", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo pj_emlrtRSI = {
    520,                     /* lineNo */
    "plannerAStarGrid/plan", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo qj_emlrtRSI = {
    848,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo rj_emlrtRSI = {
    830,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo sj_emlrtRSI = {
    828,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo tj_emlrtRSI = {
    827,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo uj_emlrtRSI = {
    820,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo vj_emlrtRSI = {
    817,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo wj_emlrtRSI = {
    811,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo xj_emlrtRSI = {
    798,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo yj_emlrtRSI = {
    795,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ak_emlrtRSI = {
    783,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo bk_emlrtRSI = {
    782,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ck_emlrtRSI = {
    773,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo dk_emlrtRSI = {
    769,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ek_emlrtRSI = {
    768,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo fk_emlrtRSI = {
    766,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo gk_emlrtRSI = {
    763,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo hk_emlrtRSI = {
    761,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ik_emlrtRSI = {
    760,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo jk_emlrtRSI = {
    757,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo kk_emlrtRSI = {
    754,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo lk_emlrtRSI = {
    749,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo mk_emlrtRSI = {
    750,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo nk_emlrtRSI = {
    746,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo ok_emlrtRSI = {
    731,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo pk_emlrtRSI = {
    726,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo qk_emlrtRSI = {
    725,                       /* lineNo */
    "plannerAStarGrid/planOM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo rk_emlrtRSI = {
    863,                                  /* lineNo */
    "plannerAStarGrid/validateStartGoal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo sk_emlrtRSI = {
    866,                                  /* lineNo */
    "plannerAStarGrid/validateStartGoal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo tk_emlrtRSI = {
    867,                                  /* lineNo */
    "plannerAStarGrid/validateStartGoal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo uk_emlrtRSI = {
    868,                                  /* lineNo */
    "plannerAStarGrid/validateStartGoal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo vk_emlrtRSI = {
    883,                                  /* lineNo */
    "plannerAStarGrid/validateStartGoal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo wk_emlrtRSI = {
    884,                                  /* lineNo */
    "plannerAStarGrid/validateStartGoal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo xk_emlrtRSI = {
    894,                                  /* lineNo */
    "plannerAStarGrid/validateStartGoal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo yk_emlrtRSI = {
    903,                                  /* lineNo */
    "plannerAStarGrid/validateStartGoal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo al_emlrtRSI = {
    872,                                /* lineNo */
    "MapInterface/validateGridIndices", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo bl_emlrtRSI = {
    1036,                                    /* lineNo */
    "plannerAStarGrid/getNodeCostOMDefault", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo il_emlrtRSI = {
    357,                                     /* lineNo */
    "plannerAStarGrid/get.TieBreakConstant", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo fm_emlrtRSI = {
    1047,                         /* lineNo */
    "plannerAStarGrid/Euclidean", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pathName
                                                                         */
};

static emlrtRSInfo dn_emlrtRSI = {
    415,                               /* lineNo */
    "plannerAStarGrid/getPathIndices", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\plannerAStarGrid.m" /* pathName */
};

static emlrtRSInfo en_emlrtRSI = {
    426,                                        /* lineNo */
    "plannerAStarGrid/getNodesExploredIndices", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\plannerAStarGrid.m" /* pathName */
};

static emlrtRTEInfo ab_emlrtRTEI = {
    706,                                           /* lineNo */
    79,                                            /* colNo */
    "plannerAStarGrid/verifyCodegenCompatibility", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo db_emlrtRTEI = {
    506,                     /* lineNo */
    68,                      /* colNo */
    "plannerAStarGrid/plan", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo eb_emlrtRTEI = {
    507,                     /* lineNo */
    66,                      /* colNo */
    "plannerAStarGrid/plan", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtECInfo k_emlrtECI = {
    -1,                        /* nDims */
    848,                       /* lineNo */
    17,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtBCInfo x_emlrtBCI = {
    1,                         /* iFirst */
    10000,                     /* iLast */
    848,                       /* lineNo */
    28,                        /* colNo */
    "",                        /* aName */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo j_emlrtDCI = {
    848,                       /* lineNo */
    28,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtECInfo l_emlrtECI = {
    -1,                        /* nDims */
    841,                       /* lineNo */
    17,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtBCInfo y_emlrtBCI = {
    1,                         /* iFirst */
    10000,                     /* iLast */
    841,                       /* lineNo */
    30,                        /* colNo */
    "",                        /* aName */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo k_emlrtDCI = {
    841,                       /* lineNo */
    30,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtECInfo m_emlrtECI = {
    -1,                        /* nDims */
    825,                       /* lineNo */
    17,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtBCInfo ab_emlrtBCI = {
    1,                         /* iFirst */
    10000,                     /* iLast */
    825,                       /* lineNo */
    32,                        /* colNo */
    "",                        /* aName */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo l_emlrtDCI = {
    825,                       /* lineNo */
    32,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtECInfo n_emlrtECI = {
    -1,                        /* nDims */
    820,                       /* lineNo */
    17,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtBCInfo bb_emlrtBCI = {
    1,                         /* iFirst */
    10000,                     /* iLast */
    820,                       /* lineNo */
    30,                        /* colNo */
    "",                        /* aName */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo m_emlrtDCI = {
    820,                       /* lineNo */
    30,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtECInfo o_emlrtECI = {
    -1,                        /* nDims */
    788,                       /* lineNo */
    17,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtBCInfo cb_emlrtBCI = {
    1,                         /* iFirst */
    10000,                     /* iLast */
    788,                       /* lineNo */
    32,                        /* colNo */
    "",                        /* aName */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo n_emlrtDCI = {
    788,                       /* lineNo */
    32,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtRTEInfo fb_emlrtRTEI = {
    785,                       /* lineNo */
    132,                       /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtECInfo p_emlrtECI = {
    -1,                        /* nDims */
    778,                       /* lineNo */
    17,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtBCInfo db_emlrtBCI = {
    1,                         /* iFirst */
    10000,                     /* iLast */
    778,                       /* lineNo */
    28,                        /* colNo */
    "",                        /* aName */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo o_emlrtDCI = {
    778,                       /* lineNo */
    28,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtBCInfo eb_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    778,                       /* lineNo */
    58,                        /* colNo */
    "",                        /* aName */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo p_emlrtDCI = {
    778,                       /* lineNo */
    58,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtBCInfo fb_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    778,                       /* lineNo */
    56,                        /* colNo */
    "",                        /* aName */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo q_emlrtDCI = {
    422,                         /* lineNo */
    30,                          /* colNo */
    "plannerAStarGrid/get.Path", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtBCInfo gb_emlrtBCI = {
    1,                           /* iFirst */
    10000,                       /* iLast */
    422,                         /* lineNo */
    30,                          /* colNo */
    "",                          /* aName */
    "plannerAStarGrid/get.Path", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo r_emlrtDCI = {
    428,                               /* lineNo */
    36,                                /* colNo */
    "plannerAStarGrid/get.PathInGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtBCInfo hb_emlrtBCI = {
    1,                                 /* iFirst */
    10000,                             /* iLast */
    428,                               /* lineNo */
    36,                                /* colNo */
    "",                                /* aName */
    "plannerAStarGrid/get.PathInGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo s_emlrtDCI = {
    353,                                /* lineNo */
    45,                                 /* colNo */
    "plannerAStarGrid/get.PathIndices", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\plannerAStarGrid.m", /* pName */
    1                              /* checkKind */
};

static emlrtBCInfo ib_emlrtBCI = {
    1,                                  /* iFirst */
    10000,                              /* iLast */
    353,                                /* lineNo */
    45,                                 /* colNo */
    "",                                 /* aName */
    "plannerAStarGrid/get.PathIndices", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\plannerAStarGrid.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo kb_emlrtBCI = {
    1,                         /* iFirst */
    10000,                     /* iLast */
    817,                       /* lineNo */
    35,                        /* colNo */
    "",                        /* aName */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo u_emlrtDCI = {
    817,                       /* lineNo */
    35,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtBCInfo lb_emlrtBCI = {
    1,                         /* iFirst */
    10000,                     /* iLast */
    834,                       /* lineNo */
    31,                        /* colNo */
    "",                        /* aName */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo v_emlrtDCI = {
    834,                       /* lineNo */
    31,                        /* colNo */
    "plannerAStarGrid/planOM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtRTEInfo gb_emlrtRTEI = {
    882,                                  /* lineNo */
    21,                                   /* colNo */
    "plannerAStarGrid/validateStartGoal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo hb_emlrtRTEI = {
    899,                                  /* lineNo */
    21,                                   /* colNo */
    "plannerAStarGrid/validateStartGoal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo ib_emlrtRTEI = {
    908,                                  /* lineNo */
    21,                                   /* colNo */
    "plannerAStarGrid/validateStartGoal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtBCInfo mb_emlrtBCI = {
    1,                                    /* iFirst */
    100,                                  /* iLast */
    890,                                  /* lineNo */
    35,                                   /* colNo */
    "",                                   /* aName */
    "plannerAStarGrid/validateStartGoal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo w_emlrtDCI = {
    890,                                  /* lineNo */
    35,                                   /* colNo */
    "plannerAStarGrid/validateStartGoal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtBCInfo nb_emlrtBCI = {
    1,                                    /* iFirst */
    100,                                  /* iLast */
    890,                                  /* lineNo */
    49,                                   /* colNo */
    "",                                   /* aName */
    "plannerAStarGrid/validateStartGoal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo x_emlrtDCI = {
    890,                                  /* lineNo */
    49,                                   /* colNo */
    "plannerAStarGrid/validateStartGoal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtBCInfo ob_emlrtBCI = {
    1,                                    /* iFirst */
    100,                                  /* iLast */
    891,                                  /* lineNo */
    36,                                   /* colNo */
    "",                                   /* aName */
    "plannerAStarGrid/validateStartGoal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo y_emlrtDCI = {
    891,                                  /* lineNo */
    36,                                   /* colNo */
    "plannerAStarGrid/validateStartGoal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtBCInfo pb_emlrtBCI = {
    1,                                    /* iFirst */
    100,                                  /* iLast */
    891,                                  /* lineNo */
    51,                                   /* colNo */
    "",                                   /* aName */
    "plannerAStarGrid/validateStartGoal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo ab_emlrtDCI = {
    891,                                  /* lineNo */
    51,                                   /* colNo */
    "plannerAStarGrid/validateStartGoal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtBCInfo qb_emlrtBCI = {
    1,                                       /* iFirst */
    10000,                                   /* iLast */
    1037,                                    /* lineNo */
    47,                                      /* colNo */
    "",                                      /* aName */
    "plannerAStarGrid/getNodeCostOMDefault", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtDCInfo bb_emlrtDCI = {
    1037,                                    /* lineNo */
    47,                                      /* colNo */
    "plannerAStarGrid/getNodeCostOMDefault", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtDCInfo pc_emlrtDCI = {
    436,                           /* lineNo */
    32,                            /* colNo */
    "plannerAStarGrid/get.PathXY", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    1 /* checkKind */
};

static emlrtBCInfo yd_emlrtBCI = {
    1,                             /* iFirst */
    10000,                         /* iLast */
    436,                           /* lineNo */
    32,                            /* colNo */
    "",                            /* aName */
    "plannerAStarGrid/get.PathXY", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m", /* pName
                                                                          */
    0 /* checkKind */
};

static emlrtRTEInfo ye_emlrtRTEI = {
    445,                /* lineNo */
    37,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo af_emlrtRTEI = {
    773,                /* lineNo */
    17,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo bf_emlrtRTEI = {
    782,                /* lineNo */
    17,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo cf_emlrtRTEI = {
    422,                /* lineNo */
    13,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo df_emlrtRTEI = {
    834,                /* lineNo */
    13,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo ef_emlrtRTEI = {
    817,                /* lineNo */
    17,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo ff_emlrtRTEI = {
    820,                /* lineNo */
    68,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo gf_emlrtRTEI = {
    848,                /* lineNo */
    69,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo hf_emlrtRTEI = {
    830,                /* lineNo */
    21,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo if_emlrtRTEI = {
    428,                /* lineNo */
    13,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo jf_emlrtRTEI = {
    432,                /* lineNo */
    17,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo kf_emlrtRTEI = {
    828,                /* lineNo */
    21,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo lf_emlrtRTEI = {
    766,                /* lineNo */
    13,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo mf_emlrtRTEI = {
    768,                /* lineNo */
    13,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo nf_emlrtRTEI = {
    828,                /* lineNo */
    31,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo cg_emlrtRTEI = {
    436,                /* lineNo */
    13,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

static emlrtRTEInfo dg_emlrtRTEI = {
    440,                /* lineNo */
    17,                 /* colNo */
    "plannerAStarGrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerAStarGrid.m" /* pName
                                                                         */
};

/* Function Declarations */
static real_T c_plannerAStarGrid_getNodeCostO(const emlrtStack *sp,
                                              plannerAStarGrid *obj,
                                              real_T currentNode);

static void c_plannerAStarGrid_validateStar(const emlrtStack *sp,
                                            plannerAStarGrid *obj,
                                            const binaryOccupancyMap *map,
                                            const real_T StartInGrid[2],
                                            const real_T GoalInGrid[2]);

static void plannerAStarGrid_get_PathXY(const emlrtStack *sp,
                                        const plannerAStarGrid *obj,
                                        emxArray_real_T *val);

static real_T
plannerAStarGrid_planOM(codegenPathPlannerStackData *SD, const emlrtStack *sp,
                        plannerAStarGrid *obj, const real_T StartInGrid[2],
                        const real_T GoalInGrid[2], emxArray_real_T *pathOut,
                        real_T debugInfo_GCostMatrix[10000],
                        real_T *debugInfo_NumNodesExplored);

/* Function Definitions */
static real_T c_plannerAStarGrid_getNodeCostO(const emlrtStack *sp,
                                              plannerAStarGrid *obj,
                                              real_T currentNode)
{
  binaryOccupancyMap *val;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T cost;
  int32_T i;
  int32_T trueCount;
  boolean_T mat[10000];
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &bl_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &ri_emlrtRSI;
  val = obj->Map;
  b_st.site = &ri_emlrtRSI;
  c_st.site = &ud_emlrtRSI;
  MapLayer_getValueAllImpl(&c_st, val, mat);
  for (i = 0; i < 10000; i++) {
    obj->OccupancyMatrix[i] = mat[i];
  }
  if (currentNode != (int32_T)muDoubleScalarFloor(currentNode)) {
    emlrtIntegerCheckR2012b(currentNode, &bb_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (((int32_T)currentNode < 1) || ((int32_T)currentNode > 10000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)currentNode, 1, 10000, &qb_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  cost = obj->OccupancyMatrix[(int32_T)currentNode - 1];
  cost = muDoubleScalarRound(cost * 10000.0) / 10000.0;
  trueCount = 0;
  if (cost > obj->OccupiedThreshold) {
    trueCount = 1;
  }
  for (i = 0; i < trueCount; i++) {
    cost = rtInf;
  }
  trueCount = 0;
  if (cost <= obj->OccupiedThreshold) {
    trueCount = 1;
  }
  for (i = 0; i < trueCount; i++) {
    cost = 0.0;
  }
  return cost;
}

static void c_plannerAStarGrid_validateStar(const emlrtStack *sp,
                                            plannerAStarGrid *obj,
                                            const binaryOccupancyMap *map,
                                            const real_T StartInGrid[2],
                                            const real_T GoalInGrid[2])
{
  static const int32_T iv[2] = {1, 7};
  static const int32_T iv1[2] = {1, 7};
  static const int32_T iv2[2] = {1, 7};
  static const int32_T iv3[2] = {1, 7};
  static const char_T cv1[11] = {'X', ' ', 'd', 'i', 'r', 'e',
                                 'c', 't', 'i', 'o', 'n'};
  static const char_T cv2[11] = {'Y', ' ', 'd', 'i', 'r', 'e',
                                 'c', 't', 'i', 'o', 'n'};
  static const char_T b_cv[7] = {'c', 'o', 'l', 'u', 'm', 'n', 's'};
  static const char_T rfmt[7] = {'%', '2', '3', '.', '1', '5', 'e'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *m;
  const mxArray *y;
  real_T a__7[4];
  real_T mapSizeX_idx_0;
  real_T mapSizeY_idx_0;
  int32_T i;
  int32_T k;
  boolean_T x_data[3];
  boolean_T exitg1;
  boolean_T isStartOccupied;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &rk_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  a__7[0] = StartInGrid[0];
  a__7[1] = GoalInGrid[0];
  a__7[2] = StartInGrid[1];
  a__7[3] = GoalInGrid[1];
  b_st.site = &al_emlrtRSI;
  c_st.site = &ab_emlrtRSI;
  isStartOccupied = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 4)) {
    if ((!muDoubleScalarIsInf(a__7[k])) && (!muDoubleScalarIsNaN(a__7[k])) &&
        (muDoubleScalarFloor(a__7[k]) == a__7[k])) {
      k++;
    } else {
      isStartOccupied = false;
      exitg1 = true;
    }
  }
  if (!isStartOccupied) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &g_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedInteger",
        "MATLAB:planOM:expectedInteger", 3, 4, 9, "startGoal");
  }
  x_data[0] = ((!(StartInGrid[0] <= 100.0)) || (!(StartInGrid[1] <= 100.0)) ||
               (!(StartInGrid[0] >= 1.0)) || (!(StartInGrid[1] >= 1.0)));
  x_data[1] = ((!(GoalInGrid[0] <= 100.0)) || (!(GoalInGrid[1] <= 100.0)) ||
               (!(GoalInGrid[0] >= 1.0)) || (!(GoalInGrid[1] >= 1.0)));
  isStartOccupied = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 1)) {
    if (x_data[k]) {
      isStartOccupied = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (isStartOccupied) {
    real_T mapSizeX_idx_1;
    real_T mapSizeY_idx_1;
    int32_T strY_size_idx_1;
    char_T b_str[23];
    char_T c_str[23];
    char_T d_str[23];
    char_T str[23];
    char_T strX_data[11];
    char_T strY_data[11];
    st.site = &sk_emlrtRSI;
    mapSizeX_idx_0 = obj->IsGrid;
    if (muDoubleScalarIsNaN(mapSizeX_idx_0)) {
      emlrtErrorWithMessageIdR2018a(&st, &t_emlrtRTEI, "MATLAB:nologicalnan",
                                    "MATLAB:nologicalnan", 0);
    }
    if (!(obj->IsGrid != 0.0)) {
      st.site = &tk_emlrtRSI;
      b_st.site = &yc_emlrtRSI;
      mapSizeX_idx_0 = map->SharedProperties.LocalOriginInWorld[0] +
                       map->SharedProperties.GridOriginInLocal[0];
      mapSizeX_idx_1 = mapSizeX_idx_0 + 100.0;
      st.site = &uk_emlrtRSI;
      b_st.site = &ad_emlrtRSI;
      mapSizeY_idx_0 = map->SharedProperties.LocalOriginInWorld[1] +
                       map->SharedProperties.GridOriginInLocal[1];
      mapSizeY_idx_1 = mapSizeY_idx_0 + 100.0;
      k = 11;
      strY_size_idx_1 = 11;
      for (i = 0; i < 11; i++) {
        strX_data[i] = cv1[i];
        strY_data[i] = cv2[i];
      }
    } else {
      mapSizeX_idx_0 = 1.0;
      mapSizeY_idx_0 = 1.0;
      mapSizeX_idx_1 = 100.0;
      mapSizeY_idx_1 = 100.0;
      k = 4;
      strX_data[0] = 'r';
      strX_data[1] = 'o';
      strX_data[2] = 'w';
      strX_data[3] = 's';
      strY_size_idx_1 = 7;
      for (i = 0; i < 7; i++) {
        strY_data[i] = b_cv[i];
      }
    }
    st.site = &vk_emlrtRSI;
    b_st.site = &ee_emlrtRSI;
    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(&b_st, 7, m, &rfmt[0]);
    emlrtAssign(&y, m);
    b_y = NULL;
    m = emlrtCreateDoubleScalar(mapSizeX_idx_0);
    emlrtAssign(&b_y, m);
    c_st.site = &rt_emlrtRSI;
    emlrt_marshallIn(&c_st, b_sprintf(&c_st, y, b_y, &emlrtMCI),
                     "<output of sprintf>", str);
    st.site = &vk_emlrtRSI;
    b_st.site = &ee_emlrtRSI;
    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(&b_st, 7, m, &rfmt[0]);
    emlrtAssign(&c_y, m);
    d_y = NULL;
    m = emlrtCreateDoubleScalar(mapSizeX_idx_1);
    emlrtAssign(&d_y, m);
    c_st.site = &rt_emlrtRSI;
    emlrt_marshallIn(&c_st, b_sprintf(&c_st, c_y, d_y, &emlrtMCI),
                     "<output of sprintf>", b_str);
    st.site = &wk_emlrtRSI;
    b_st.site = &ee_emlrtRSI;
    e_y = NULL;
    m = emlrtCreateCharArray(2, &iv2[0]);
    emlrtInitCharArrayR2013a(&b_st, 7, m, &rfmt[0]);
    emlrtAssign(&e_y, m);
    f_y = NULL;
    m = emlrtCreateDoubleScalar(mapSizeY_idx_0);
    emlrtAssign(&f_y, m);
    c_st.site = &rt_emlrtRSI;
    emlrt_marshallIn(&c_st, b_sprintf(&c_st, e_y, f_y, &emlrtMCI),
                     "<output of sprintf>", c_str);
    st.site = &wk_emlrtRSI;
    b_st.site = &ee_emlrtRSI;
    g_y = NULL;
    m = emlrtCreateCharArray(2, &iv3[0]);
    emlrtInitCharArrayR2013a(&b_st, 7, m, &rfmt[0]);
    emlrtAssign(&g_y, m);
    h_y = NULL;
    m = emlrtCreateDoubleScalar(mapSizeY_idx_1);
    emlrtAssign(&h_y, m);
    c_st.site = &rt_emlrtRSI;
    emlrt_marshallIn(&c_st, b_sprintf(&c_st, g_y, h_y, &emlrtMCI),
                     "<output of sprintf>", d_str);
    emlrtErrorWithMessageIdR2018a(
        sp, &gb_emlrtRTEI, "nav:navalgs:plannerastargrid:CoordinateOutside",
        "nav:navalgs:plannerastargrid:CoordinateOutside", 18, 4, 23, &str[0], 4,
        23, &b_str[0], 4, k, &strX_data[0], 4, 23, &c_str[0], 4, 23, &d_str[0],
        4, strY_size_idx_1, &strY_data[0]);
  }
  if (GoalInGrid[0] != (int32_T)muDoubleScalarFloor(GoalInGrid[0])) {
    emlrtIntegerCheckR2012b(GoalInGrid[0], &w_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (((int32_T)GoalInGrid[0] < 1) || ((int32_T)GoalInGrid[0] > 100)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)GoalInGrid[0], 1, 100, &mb_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (GoalInGrid[1] != (int32_T)muDoubleScalarFloor(GoalInGrid[1])) {
    emlrtIntegerCheckR2012b(GoalInGrid[1], &x_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (((int32_T)GoalInGrid[1] < 1) || ((int32_T)GoalInGrid[1] > 100)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)GoalInGrid[1], 1, 100, &nb_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  mapSizeX_idx_0 = obj->PoseId[((int32_T)GoalInGrid[0] +
                                100 * ((int32_T)GoalInGrid[1] - 1)) -
                               1];
  if (StartInGrid[0] != (int32_T)muDoubleScalarFloor(StartInGrid[0])) {
    emlrtIntegerCheckR2012b(StartInGrid[0], &y_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (((int32_T)StartInGrid[0] < 1) || ((int32_T)StartInGrid[0] > 100)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)StartInGrid[0], 1, 100, &ob_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (StartInGrid[1] != (int32_T)muDoubleScalarFloor(StartInGrid[1])) {
    emlrtIntegerCheckR2012b(StartInGrid[1], &ab_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (((int32_T)StartInGrid[1] < 1) || ((int32_T)StartInGrid[1] > 100)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)StartInGrid[1], 1, 100, &pb_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  mapSizeY_idx_0 = obj->PoseId[((int32_T)StartInGrid[0] +
                                100 * ((int32_T)StartInGrid[1] - 1)) -
                               1];
  st.site = &xk_emlrtRSI;
  isStartOccupied =
      (c_plannerAStarGrid_getNodeCostO(&st, obj, mapSizeY_idx_0) == rtInf);
  if (isStartOccupied) {
    emlrtErrorWithMessageIdR2018a(
        sp, &hb_emlrtRTEI, "nav:navalgs:plannerastargrid:OccupiedLocation",
        "nav:navalgs:plannerastargrid:OccupiedLocation", 3, 4, 5, "start");
  }
  st.site = &yk_emlrtRSI;
  isStartOccupied =
      (c_plannerAStarGrid_getNodeCostO(&st, obj, mapSizeX_idx_0) == rtInf);
  if (isStartOccupied) {
    emlrtErrorWithMessageIdR2018a(
        sp, &ib_emlrtRTEI, "nav:navalgs:plannerastargrid:OccupiedLocation",
        "nav:navalgs:plannerastargrid:OccupiedLocation", 3, 4, 4, "goal");
  }
}

static void plannerAStarGrid_get_PathXY(const emlrtStack *sp,
                                        const plannerAStarGrid *obj,
                                        emxArray_real_T *val)
{
  real_T d;
  real_T *val_data;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  d = obj->NumPathPoints;
  if (d < 1.0) {
    loop_ub = 0;
  } else {
    if (d != (int32_T)muDoubleScalarFloor(d)) {
      emlrtIntegerCheckR2012b(d, &pc_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)d < 1) || ((int32_T)d > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 10000, &yd_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    loop_ub = (int32_T)d;
  }
  i = val->size[0] * val->size[1];
  val->size[0] = loop_ub;
  val->size[1] = 2;
  emxEnsureCapacity_real_T(sp, val, i, &cg_emlrtRTEI);
  val_data = val->data;
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      val_data[i1 + val->size[0] * i] = obj->PathXY[i1 + 10000 * i];
    }
  }
  if (loop_ub == 0) {
    i = val->size[0] * val->size[1];
    val->size[0] = 1;
    val->size[1] = 2;
    emxEnsureCapacity_real_T(sp, val, i, &dg_emlrtRTEI);
    val_data = val->data;
    val_data[0] = obj->PathXY[0];
    val_data[val->size[0]] = obj->PathXY[10000];
  }
}

static real_T
plannerAStarGrid_planOM(codegenPathPlannerStackData *SD, const emlrtStack *sp,
                        plannerAStarGrid *obj, const real_T StartInGrid[2],
                        const real_T GoalInGrid[2], emxArray_real_T *pathOut,
                        real_T debugInfo_GCostMatrix[10000],
                        real_T *debugInfo_NumNodesExplored)
{
  binaryOccupancyMap *map;
  binaryOccupancyMap *val;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_real_T *b_pose;
  emxArray_real_T *b_val;
  emxArray_real_T *nodesExIn;
  emxArray_real_T *path;
  emxArray_real_T *pose;
  real_T debugInfo_PathCost;
  real_T th;
  real_T *nodesExIn_data;
  real_T *path_data;
  real_T *pose_data;
  real_T *val_data;
  int32_T c_pose[2];
  int32_T iv[2];
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  boolean_T mat[10000];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &qk_emlrtRSI;
  map = obj->Map;
  st.site = &pk_emlrtRSI;
  b_st.site = &ri_emlrtRSI;
  val = obj->Map;
  b_st.site = &ri_emlrtRSI;
  c_st.site = &ud_emlrtRSI;
  MapLayer_getValueAllImpl(&c_st, val, mat);
  for (i = 0; i < 10000; i++) {
    obj->OccupancyMatrix[i] = mat[i];
  }
  st.site = &ok_emlrtRSI;
  c_plannerAStarGrid_validateStar(&st, obj, map, StartInGrid, GoalInGrid);
  obj->StartInGrid[0] = StartInGrid[0];
  obj->StartInGrid[1] = StartInGrid[1];
  obj->GoalInGrid[0] = GoalInGrid[0];
  obj->GoalInGrid[1] = GoalInGrid[1];
  for (i = 0; i < 10000; i++) {
    SD->f4.pathTemp[i] = obj->OccupancyMatrix[i];
  }
  for (b_i = 0; b_i < 10000; b_i++) {
    SD->f4.pathTemp[b_i] = muDoubleScalarRound(SD->f4.pathTemp[b_i] * 10000.0);
  }
  th = obj->OccupiedThreshold;
  st.site = &nk_emlrtRSI;
  st.site = &mk_emlrtRSI;
  st.site = &mk_emlrtRSI;
  for (i = 0; i <= 9998; i += 2) {
    __m128d r;
    r = _mm_loadu_pd(&SD->f4.pathTemp[i]);
    _mm_storeu_pd(&SD->f4.b_pathTemp[i], _mm_div_pd(r, _mm_set1_pd(10000.0)));
  }
  st.site = &lk_emlrtRSI;
  d_plannerAStarGrid_plannerAStar(&SD->f4.astarInternal, SD->f4.b_pathTemp, th);
  if (obj->UseCustomH == 0.0) {
    st.site = &kk_emlrtRSI;
    th = obj->HCost;
    st.site = &kk_emlrtRSI;
    SD->f4.astarInternal.HCostMethod = th;
    SD->f4.astarInternal.UseCustomH = 0.0;
  }
  if (obj->UseCustomG == 0.0) {
    st.site = &jk_emlrtRSI;
    th = obj->GCost;
    st.site = &jk_emlrtRSI;
    SD->f4.astarInternal.GCostMethod = th;
    SD->f4.astarInternal.UseCustomG = 0.0;
  }
  st.site = &ik_emlrtRSI;
  b_st.site = &il_emlrtRSI;
  th = obj->TieBreaker;
  if (muDoubleScalarIsNaN(th)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &t_emlrtRTEI, "MATLAB:nologicalnan",
                                  "MATLAB:nologicalnan", 0);
  }
  if (obj->TieBreaker != 0.0) {
    th = 1.07;
  } else {
    th = 1.0;
  }
  st.site = &ik_emlrtRSI;
  SD->f4.astarInternal.TieBreaker = th;
  st.site = &hk_emlrtRSI;
  th = obj->DiagonalSearch;
  SD->f4.astarInternal.DiagonalSearchFlag = th;
  st.site = &gk_emlrtRSI;
  b_plannerAStarGrid_plan(SD, &st, &SD->f4.astarInternal, obj->StartInGrid,
                          obj->GoalInGrid);
  st.site = &fk_emlrtRSI;
  b_st.site = &dn_emlrtRSI;
  th = SD->f4.astarInternal.NumPathPoints;
  if (th < 1.0) {
    i = 0;
  } else {
    if (th != (int32_T)muDoubleScalarFloor(th)) {
      emlrtIntegerCheckR2012b(th, &s_emlrtDCI, &b_st);
    }
    if (((int32_T)th < 1) || ((int32_T)th > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, 10000, &ib_emlrtBCI, &b_st);
    }
    i = (int32_T)th;
  }
  obj->NumPathPoints = ((real_T)i - 1.0) + 1.0;
  st.site = &ek_emlrtRSI;
  b_st.site = &en_emlrtRSI;
  th = SD->f4.astarInternal.NumNodesExplored;
  if (!(th < 1.0)) {
    if (th != (int32_T)muDoubleScalarFloor(th)) {
      emlrtIntegerCheckR2012b(th, &t_emlrtDCI, &b_st);
    }
    if (((int32_T)th < 1) || ((int32_T)th > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, 10000, &jb_emlrtBCI, &b_st);
    }
  }
  st.site = &dk_emlrtRSI;
  th = SD->f4.astarInternal.NumNodesExplored;
  obj->NumNodesExplored = th;
  st.site = &ck_emlrtRSI;
  b_st.site = &dn_emlrtRSI;
  th = SD->f4.astarInternal.NumPathPoints;
  if (th < 1.0) {
    b_i = 0;
  } else {
    if (th != (int32_T)muDoubleScalarFloor(th)) {
      emlrtIntegerCheckR2012b(th, &s_emlrtDCI, &b_st);
    }
    if (((int32_T)th < 1) || ((int32_T)th > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, 10000, &ib_emlrtBCI, &b_st);
    }
    b_i = (int32_T)th;
  }
  emxInit_real_T(&b_st, &path, 1, &lf_emlrtRTEI);
  i = path->size[0];
  path->size[0] = b_i;
  emxEnsureCapacity_real_T(&b_st, path, i, &af_emlrtRTEI);
  path_data = path->data;
  for (i = 0; i < b_i; i++) {
    path_data[i] = SD->f4.astarInternal.PathIndicesInternal[i];
  }
  obj->NumPathPoints = path->size[0];
  memset(&SD->f4.pathTemp[0], 0, 10000U * sizeof(real_T));
  th = obj->NumPathPoints;
  if (th < 1.0) {
    b_i = 0;
  } else {
    if (path->size[0] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, path->size[0], &fb_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if (th != (int32_T)muDoubleScalarFloor(th)) {
      emlrtIntegerCheckR2012b(th, &p_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)th < 1) || ((int32_T)th > path->size[0])) {
      emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, path->size[0], &eb_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    b_i = (int32_T)th;
  }
  th = obj->NumPathPoints;
  if (th < 1.0) {
    i = 0;
  } else {
    if (th != (int32_T)muDoubleScalarFloor(th)) {
      emlrtIntegerCheckR2012b(th, &o_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)th < 1) || ((int32_T)th > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, 10000, &db_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    i = (int32_T)th;
  }
  emlrtSubAssignSizeCheckR2012b(&i, 1, &b_i, 1, &p_emlrtECI, (emlrtCTX)sp);
  for (i = 0; i < b_i; i++) {
    SD->f4.pathTemp[i] = path_data[i];
  }
  for (b_i = 0; b_i < 10000; b_i++) {
    obj->Path[b_i] = SD->f4.pathTemp[b_i];
  }
  st.site = &bk_emlrtRSI;
  b_st.site = &en_emlrtRSI;
  th = SD->f4.astarInternal.NumNodesExplored;
  if (th < 1.0) {
    b_i = 0;
  } else {
    if (th != (int32_T)muDoubleScalarFloor(th)) {
      emlrtIntegerCheckR2012b(th, &t_emlrtDCI, &b_st);
    }
    if (((int32_T)th < 1) || ((int32_T)th > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, 10000, &jb_emlrtBCI, &b_st);
    }
    b_i = (int32_T)th;
  }
  emxInit_real_T(&b_st, &nodesExIn, 1, &mf_emlrtRTEI);
  i = nodesExIn->size[0];
  nodesExIn->size[0] = b_i;
  emxEnsureCapacity_real_T(&b_st, nodesExIn, i, &bf_emlrtRTEI);
  nodesExIn_data = nodesExIn->data;
  for (i = 0; i < b_i; i++) {
    nodesExIn_data[i] = SD->f4.astarInternal.NodesExploredIndicesInternal[i];
  }
  st.site = &ak_emlrtRSI;
  th = SD->f4.astarInternal.NumNodesExplored;
  obj->NumNodesExplored = th;
  if (!(obj->NumNodesExplored <= 10000.0)) {
    emlrtErrorWithMessageIdR2018a(
        sp, &fb_emlrtRTEI,
        "nav:navalgs:plannerastargrid:AssertionFailedLessThan",
        "nav:navalgs:plannerastargrid:AssertionFailedLessThan", 5, 4, 16,
        "NumNodesExplored", 6, 10000.0);
  }
  th = obj->NumNodesExplored;
  if (th < 1.0) {
    i = 0;
  } else {
    if (th != (int32_T)muDoubleScalarFloor(th)) {
      emlrtIntegerCheckR2012b(th, &n_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)th < 1) || ((int32_T)th > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, 10000, &cb_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    i = (int32_T)th;
  }
  emlrtSubAssignSizeCheckR2012b(&i, 1, &nodesExIn->size[0], 1, &o_emlrtECI,
                                (emlrtCTX)sp);
  st.site = &yj_emlrtRSI;
  memcpy(&SD->f4.pathTemp[0], &SD->f4.astarInternal.GCostMatrix[0],
         10000U * sizeof(real_T));
  for (b_i = 0; b_i < 10000; b_i++) {
    if (SD->f4.pathTemp[b_i] == -1.0) {
      SD->f4.pathTemp[b_i] = rtInf;
    }
  }
  for (i = 0; i < 10000; i++) {
    obj->GCostMatrix[i] = SD->f4.pathTemp[i];
  }
  st.site = &xj_emlrtRSI;
  th = SD->f4.astarInternal.PathCost;
  obj->PathCost = th;
  debugInfo_PathCost = obj->PathCost;
  *debugInfo_NumNodesExplored = obj->NumNodesExplored;
  for (i = 0; i < 10000; i++) {
    debugInfo_GCostMatrix[i] = obj->GCostMatrix[i];
  }
  for (i = 0; i < 20000; i++) {
    obj->PathXY[i] = rtNaN;
  }
  for (i = 0; i < 20000; i++) {
    obj->PathInGrid[i] = rtNaN;
  }
  emxInit_real_T(sp, &pose, 2, &ef_emlrtRTEI);
  emxInit_real_T(sp, &b_val, 2, &nf_emlrtRTEI);
  emxInit_real_T(sp, &b_pose, 2, &ff_emlrtRTEI);
  if (path->size[0] == 0) {
    st.site = &wj_emlrtRSI;
    warning(&st);
    obj->PathCost = rtInf;
    pathOut->size[0] = 0;
    pathOut->size[1] = 0;
    debugInfo_PathCost = obj->PathCost;
  } else {
    st.site = &vj_emlrtRSI;
    th = obj->NumPathPoints;
    if (th < 1.0) {
      b_i = 0;
    } else {
      if (th != (int32_T)muDoubleScalarFloor(th)) {
        emlrtIntegerCheckR2012b(th, &q_emlrtDCI, &st);
      }
      if (((int32_T)th < 1) || ((int32_T)th > 10000)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, 10000, &gb_emlrtBCI, &st);
      }
      b_i = (int32_T)th;
    }
    i = path->size[0];
    path->size[0] = b_i;
    emxEnsureCapacity_real_T(&st, path, i, &cf_emlrtRTEI);
    path_data = path->data;
    for (i = 0; i < b_i; i++) {
      path_data[i] = obj->Path[i];
    }
    i = pose->size[0] * pose->size[1];
    pose->size[0] = path->size[0];
    pose->size[1] = 3;
    emxEnsureCapacity_real_T(sp, pose, i, &ef_emlrtRTEI);
    pose_data = pose->data;
    b_i = path->size[0];
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < b_i; i1++) {
        if (path_data[i1] != (int32_T)muDoubleScalarFloor(path_data[i1])) {
          emlrtIntegerCheckR2012b(path_data[i1], &u_emlrtDCI,
                                  (emlrtConstCTX)sp);
        }
        i2 = (int32_T)path_data[i1];
        if ((i2 < 1) || (i2 > 10000)) {
          emlrtDynamicBoundsCheckR2012b(i2, 1, 10000, &kb_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        pose_data[i1 + pose->size[0] * i] = obj->IdPose[(i2 + 10000 * i) - 1];
      }
    }
    for (i = 0; i < 20000; i++) {
      SD->f4.pathXYTemp[i] = rtNaN;
    }
    th = obj->NumPathPoints;
    if (th < 1.0) {
      i = 0;
    } else {
      if (th != (int32_T)muDoubleScalarFloor(th)) {
        emlrtIntegerCheckR2012b(th, &m_emlrtDCI, (emlrtConstCTX)sp);
      }
      if (((int32_T)th < 1) || ((int32_T)th > 10000)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, 10000, &bb_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      i = (int32_T)th;
    }
    i1 = b_pose->size[0] * b_pose->size[1];
    b_pose->size[0] = pose->size[0];
    b_pose->size[1] = 2;
    emxEnsureCapacity_real_T(sp, b_pose, i1, &ff_emlrtRTEI);
    path_data = b_pose->data;
    b_i = pose->size[0];
    for (i1 = 0; i1 < 2; i1++) {
      for (i2 = 0; i2 < b_i; i2++) {
        path_data[i2 + b_pose->size[0] * i1] =
            pose_data[i2 + pose->size[0] * i1];
      }
    }
    st.site = &uj_emlrtRSI;
    MapInterface_grid2world(&st, map, b_pose, b_val);
    val_data = b_val->data;
    iv[0] = i;
    iv[1] = 2;
    emlrtSubAssignSizeCheckR2012b(&iv[0], 2, &b_val->size[0], 2, &n_emlrtECI,
                                  (emlrtCTX)sp);
    b_i = b_val->size[0];
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < b_i; i1++) {
        SD->f4.pathXYTemp[i1 + 10000 * i] = val_data[i1 + b_val->size[0] * i];
      }
    }
    for (i = 0; i < 20000; i++) {
      obj->PathXY[i] = SD->f4.pathXYTemp[i];
    }
    for (i = 0; i < 20000; i++) {
      SD->f4.pathXYTemp[i] = rtNaN;
    }
    th = obj->NumPathPoints;
    if (th < 1.0) {
      i = 0;
    } else {
      if (th != (int32_T)muDoubleScalarFloor(th)) {
        emlrtIntegerCheckR2012b(th, &l_emlrtDCI, (emlrtConstCTX)sp);
      }
      if (((int32_T)th < 1) || ((int32_T)th > 10000)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, 10000, &ab_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      i = (int32_T)th;
    }
    iv[0] = i;
    iv[1] = 2;
    c_pose[0] = pose->size[0];
    c_pose[1] = 2;
    emlrtSubAssignSizeCheckR2012b(&iv[0], 2, &c_pose[0], 2, &m_emlrtECI,
                                  (emlrtCTX)sp);
    b_i = pose->size[0];
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < b_i; i1++) {
        SD->f4.pathXYTemp[i1 + 10000 * i] = pose_data[i1 + pose->size[0] * i];
      }
    }
    for (i = 0; i < 20000; i++) {
      obj->PathInGrid[i] = SD->f4.pathXYTemp[i];
    }
    st.site = &tj_emlrtRSI;
    th = obj->IsGrid;
    if (muDoubleScalarIsNaN(th)) {
      emlrtErrorWithMessageIdR2018a(&st, &t_emlrtRTEI, "MATLAB:nologicalnan",
                                    "MATLAB:nologicalnan", 0);
    }
    if (obj->IsGrid != 0.0) {
      st.site = &sj_emlrtRSI;
      th = obj->NumPathPoints;
      if (th < 1.0) {
        b_i = 0;
      } else {
        if (th != (int32_T)muDoubleScalarFloor(th)) {
          emlrtIntegerCheckR2012b(th, &r_emlrtDCI, &st);
        }
        if (((int32_T)th < 1) || ((int32_T)th > 10000)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, 10000, &hb_emlrtBCI,
                                        &st);
        }
        b_i = (int32_T)th;
      }
      i = b_val->size[0] * b_val->size[1];
      b_val->size[0] = b_i;
      b_val->size[1] = 2;
      emxEnsureCapacity_real_T(&st, b_val, i, &if_emlrtRTEI);
      val_data = b_val->data;
      for (i = 0; i < 2; i++) {
        for (i1 = 0; i1 < b_i; i1++) {
          val_data[i1 + b_val->size[0] * i] = obj->PathInGrid[i1 + 10000 * i];
        }
      }
      if (b_val->size[0] == 0) {
        i = b_val->size[0] * b_val->size[1];
        b_val->size[0] = 1;
        b_val->size[1] = 2;
        emxEnsureCapacity_real_T(&st, b_val, i, &jf_emlrtRTEI);
        val_data = b_val->data;
        val_data[0] = obj->PathInGrid[0];
        val_data[b_val->size[0]] = obj->PathInGrid[10000];
      }
      i = pathOut->size[0] * pathOut->size[1];
      pathOut->size[0] = b_val->size[0];
      pathOut->size[1] = 2;
      emxEnsureCapacity_real_T(sp, pathOut, i, &kf_emlrtRTEI);
      path_data = pathOut->data;
      b_i = b_val->size[0] << 1;
      for (i = 0; i < b_i; i++) {
        path_data[i] = val_data[i];
      }
    } else {
      st.site = &rj_emlrtRSI;
      plannerAStarGrid_get_PathXY(&st, obj, b_val);
      val_data = b_val->data;
      i = pathOut->size[0] * pathOut->size[1];
      pathOut->size[0] = b_val->size[0];
      pathOut->size[1] = 2;
      emxEnsureCapacity_real_T(sp, pathOut, i, &hf_emlrtRTEI);
      path_data = pathOut->data;
      b_i = b_val->size[0] << 1;
      for (i = 0; i < b_i; i++) {
        path_data[i] = val_data[i];
      }
    }
  }
  emxFree_real_T(sp, &path);
  i = pose->size[0] * pose->size[1];
  pose->size[0] = nodesExIn->size[0];
  pose->size[1] = 3;
  emxEnsureCapacity_real_T(sp, pose, i, &df_emlrtRTEI);
  pose_data = pose->data;
  b_i = nodesExIn->size[0];
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < b_i; i1++) {
      if (nodesExIn_data[i1] !=
          (int32_T)muDoubleScalarFloor(nodesExIn_data[i1])) {
        emlrtIntegerCheckR2012b(nodesExIn_data[i1], &v_emlrtDCI,
                                (emlrtConstCTX)sp);
      }
      i2 = (int32_T)nodesExIn_data[i1];
      if ((i2 < 1) || (i2 > 10000)) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, 10000, &lb_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      pose_data[i1 + pose->size[0] * i] = obj->IdPose[(i2 + 10000 * i) - 1];
    }
  }
  emxFree_real_T(sp, &nodesExIn);
  th = obj->NumNodesExplored;
  if (th < 1.0) {
    i = 0;
  } else {
    if (th != (int32_T)muDoubleScalarFloor(th)) {
      emlrtIntegerCheckR2012b(th, &k_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)th < 1) || ((int32_T)th > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, 10000, &y_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    i = (int32_T)th;
  }
  iv[0] = i;
  iv[1] = 2;
  c_pose[0] = pose->size[0];
  c_pose[1] = 2;
  emlrtSubAssignSizeCheckR2012b(&iv[0], 2, &c_pose[0], 2, &l_emlrtECI,
                                (emlrtCTX)sp);
  th = obj->NumNodesExplored;
  if (th < 1.0) {
    i = 0;
  } else {
    if (th != (int32_T)muDoubleScalarFloor(th)) {
      emlrtIntegerCheckR2012b(th, &j_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)th < 1) || ((int32_T)th > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)th, 1, 10000, &x_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    i = (int32_T)th;
  }
  i1 = b_pose->size[0] * b_pose->size[1];
  b_pose->size[0] = pose->size[0];
  b_pose->size[1] = 2;
  emxEnsureCapacity_real_T(sp, b_pose, i1, &gf_emlrtRTEI);
  path_data = b_pose->data;
  b_i = pose->size[0];
  for (i1 = 0; i1 < 2; i1++) {
    for (i2 = 0; i2 < b_i; i2++) {
      path_data[i2 + b_pose->size[0] * i1] = pose_data[i2 + pose->size[0] * i1];
    }
  }
  emxFree_real_T(sp, &pose);
  st.site = &qj_emlrtRSI;
  MapInterface_grid2world(&st, map, b_pose, b_val);
  emxFree_real_T(sp, &b_pose);
  c_pose[0] = b_val->size[0];
  emxFree_real_T(sp, &b_val);
  c_pose[1] = 2;
  iv[0] = i;
  iv[1] = 2;
  emlrtSubAssignSizeCheckR2012b(&iv[0], 2, &c_pose[0], 2, &k_emlrtECI,
                                (emlrtCTX)sp);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return debugInfo_PathCost;
}

plannerAStarGrid *
c_plannerAStarGrid_plannerAStar(const emlrtStack *sp, plannerAStarGrid *obj,
                                binaryOccupancyMap *varargin_1)
{
  static const int8_T iv[10000] = {
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100,
      1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,
      17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,
      33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,
      49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,
      65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,
      81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,
      97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,   9,  10, 11, 12,
      13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,  25, 26, 27, 28,
      29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,  41, 42, 43, 44,
      45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,  57, 58, 59, 60,
      61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,  73, 74, 75, 76,
      77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,  89, 90, 91, 92,
      93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,   5,  6,  7,  8,
      9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,  21, 22, 23, 24,
      25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,  37, 38, 39, 40,
      41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,  53, 54, 55, 56,
      57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,  69, 70, 71, 72,
      73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,  85, 86, 87, 88,
      89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100, 1,  2,  3,  4,
      5,  6,  7,  8,   9,  10, 11, 12,  13, 14, 15, 16,  17, 18, 19, 20,
      21, 22, 23, 24,  25, 26, 27, 28,  29, 30, 31, 32,  33, 34, 35, 36,
      37, 38, 39, 40,  41, 42, 43, 44,  45, 46, 47, 48,  49, 50, 51, 52,
      53, 54, 55, 56,  57, 58, 59, 60,  61, 62, 63, 64,  65, 66, 67, 68,
      69, 70, 71, 72,  73, 74, 75, 76,  77, 78, 79, 80,  81, 82, 83, 84,
      85, 86, 87, 88,  89, 90, 91, 92,  93, 94, 95, 96,  97, 98, 99, 100};
  static const int8_T iv1[10000] = {
      1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
      1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
      1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
      1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
      1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
      1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
      1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,
      2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
      2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
      2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
      2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
      2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
      2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
      2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,
      3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,
      3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,
      3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,
      3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,
      3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,
      3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,
      4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,
      4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,
      4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,
      4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,
      4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,
      4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,
      4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   5,   5,   5,   5,   5,
      5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,
      5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,
      5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,
      5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,
      5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,
      5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,
      5,   5,   5,   5,   5,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,
      6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,
      6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,
      6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,
      6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,
      6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,
      6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,
      7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,
      7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,
      7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,
      7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,
      7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,
      7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,
      7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   8,   8,   8,   8,   8,
      8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,
      8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,
      8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,
      8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,
      8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,
      8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,
      8,   8,   8,   8,   8,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,
      9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,
      9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,
      9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,
      9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,
      9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,
      9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,
      10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,
      10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,
      10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,
      10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,
      10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,
      10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,
      10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  11,  11,  11,  11,  11,
      11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,
      11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,
      11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,
      11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,
      11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,
      11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,
      11,  11,  11,  11,  11,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,
      12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,
      12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,
      12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,
      12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,
      12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,
      12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,
      13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,
      13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,
      13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,
      13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,
      13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,
      13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,
      13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  14,  14,  14,  14,  14,
      14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,
      14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,
      14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,
      14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,
      14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,
      14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,
      14,  14,  14,  14,  14,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,
      15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,
      15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,
      15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,
      15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,
      15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,
      15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,
      16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,
      16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,
      16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,
      16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,
      16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,
      16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,
      16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  17,  17,  17,  17,  17,
      17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,
      17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,
      17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,
      17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,
      17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,
      17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,
      17,  17,  17,  17,  17,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,
      18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,
      18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,
      18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,
      18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,
      18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,
      18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,
      19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,
      19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,
      19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,
      19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,
      19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,
      19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,
      19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  20,  20,  20,  20,  20,
      20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,
      20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,
      20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,
      20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,
      20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,
      20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,
      20,  20,  20,  20,  20,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,
      21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,
      21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,
      21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,
      21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,
      21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,
      21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,
      22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,
      22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,
      22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,
      22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,
      22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,
      22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,
      22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  23,  23,  23,  23,  23,
      23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,
      23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,
      23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,
      23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,
      23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,
      23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,
      23,  23,  23,  23,  23,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,
      24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,
      24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,
      24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,
      24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,
      24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,
      24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,
      25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,
      25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,
      25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,
      25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,
      25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,
      25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,
      25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  26,  26,  26,  26,  26,
      26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,
      26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,
      26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,
      26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,
      26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,
      26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,
      26,  26,  26,  26,  26,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,
      27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,
      27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,
      27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,
      27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,
      27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,
      27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,
      28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,
      28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,
      28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,
      28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,
      28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,
      28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,
      28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  29,  29,  29,  29,  29,
      29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,
      29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,
      29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,
      29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,
      29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,
      29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,
      29,  29,  29,  29,  29,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,
      30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,
      30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,
      30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,
      30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,
      30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,
      30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,
      31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,
      31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,
      31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,
      31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,
      31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,
      31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,
      31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  32,  32,  32,  32,  32,
      32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,
      32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,
      32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,
      32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,
      32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,
      32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,
      32,  32,  32,  32,  32,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,
      33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,
      33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,
      33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,
      33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,
      33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,
      33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,
      34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,
      34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,
      34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,
      34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,
      34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,
      34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,
      34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  35,  35,  35,  35,  35,
      35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,
      35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,
      35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,
      35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,
      35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,
      35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,
      35,  35,  35,  35,  35,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,
      36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,
      36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,
      36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,
      36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,
      36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,
      36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,
      37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,
      37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,
      37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,
      37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,
      37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,
      37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,
      37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  38,  38,  38,  38,  38,
      38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,
      38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,
      38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,
      38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,
      38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,
      38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,
      38,  38,  38,  38,  38,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
      39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
      39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
      39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
      39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
      39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
      39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
      40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,
      40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,
      40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,
      40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,
      40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,
      40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,
      40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  41,  41,  41,  41,  41,
      41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,
      41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,
      41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,
      41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,
      41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,
      41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,
      41,  41,  41,  41,  41,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,
      42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,
      42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,
      42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,
      42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,
      42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,
      42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,
      43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,
      43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,
      43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,
      43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,
      43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,
      43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,
      43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  44,  44,  44,  44,  44,
      44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,
      44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,
      44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,
      44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,
      44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,
      44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,
      44,  44,  44,  44,  44,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,
      45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,
      45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,
      45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,
      45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,
      45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,
      45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,
      46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,
      46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,
      46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,
      46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,
      46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,
      46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,
      46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  47,  47,  47,  47,  47,
      47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,
      47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,
      47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,
      47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,
      47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,
      47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,
      47,  47,  47,  47,  47,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,
      48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,
      48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,
      48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,
      48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,
      48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,
      48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,
      49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,
      49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,
      49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,
      49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,
      49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,
      49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,
      49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  50,  50,  50,  50,  50,
      50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,
      50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,
      50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,
      50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,
      50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,
      50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,
      50,  50,  50,  50,  50,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,
      51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,
      51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,
      51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,
      51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,
      51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,
      51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,
      52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,
      52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,
      52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,
      52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,
      52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,
      52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,
      52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  53,  53,  53,  53,  53,
      53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,
      53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,
      53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,
      53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,
      53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,
      53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,
      53,  53,  53,  53,  53,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,
      54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,
      54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,
      54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,
      54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,
      54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,
      54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,
      55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,
      55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,
      55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,
      55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,
      55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,
      55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,
      55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  56,  56,  56,  56,  56,
      56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,
      56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,
      56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,
      56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,
      56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,
      56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,
      56,  56,  56,  56,  56,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,
      57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,
      57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,
      57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,
      57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,
      57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,
      57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,
      58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,
      58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,
      58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,
      58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,
      58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,
      58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,
      58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  59,  59,  59,  59,  59,
      59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,
      59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,
      59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,
      59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,
      59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,
      59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  59,
      59,  59,  59,  59,  59,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
      60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
      60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
      60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
      60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
      60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
      60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
      61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,
      61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,
      61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,
      61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,
      61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,
      61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,
      61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  62,  62,  62,  62,  62,
      62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,
      62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,
      62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,
      62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,
      62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,
      62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,
      62,  62,  62,  62,  62,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,
      63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,
      63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,
      63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,
      63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,
      63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,
      63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  63,
      64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,
      64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,
      64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,
      64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,
      64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,
      64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,
      64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  65,  65,  65,  65,  65,
      65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,
      65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,
      65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,
      65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,
      65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,
      65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,
      65,  65,  65,  65,  65,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,
      66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,
      66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,
      66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,
      66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,
      66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,
      66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,  66,
      67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,
      67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,
      67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,
      67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,
      67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,
      67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  67,
      67,  67,  67,  67,  67,  67,  67,  67,  67,  67,  68,  68,  68,  68,  68,
      68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,
      68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,
      68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,
      68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,
      68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,
      68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68,
      68,  68,  68,  68,  68,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,
      69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,
      69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,
      69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,
      69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,
      69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,
      69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,
      70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,
      70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,
      70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,
      70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,
      70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,
      70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,
      70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  71,  71,  71,  71,  71,
      71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,
      71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,
      71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,
      71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,
      71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,
      71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,  71,
      71,  71,  71,  71,  71,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,
      72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,
      72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,
      72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,
      72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,
      72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,
      72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72,
      73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,
      73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,
      73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,
      73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,
      73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,
      73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  73,
      73,  73,  73,  73,  73,  73,  73,  73,  73,  73,  74,  74,  74,  74,  74,
      74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,
      74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,
      74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,
      74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,
      74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,
      74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,  74,
      74,  74,  74,  74,  74,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,
      75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,
      75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,
      75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,
      75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,
      75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,
      75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,  75,
      76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,
      76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,
      76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,
      76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,
      76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,
      76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  76,
      76,  76,  76,  76,  76,  76,  76,  76,  76,  76,  77,  77,  77,  77,  77,
      77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,
      77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,
      77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,
      77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,
      77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,
      77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77,
      77,  77,  77,  77,  77,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,
      78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,
      78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,
      78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,
      78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,
      78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,
      78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,  78,
      79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,
      79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,
      79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,
      79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,
      79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,
      79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  79,
      79,  79,  79,  79,  79,  79,  79,  79,  79,  79,  80,  80,  80,  80,  80,
      80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,
      80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,
      80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,
      80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,
      80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,
      80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,  80,
      80,  80,  80,  80,  80,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,
      81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,
      81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,
      81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,
      81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,
      81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,
      81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,  81,
      82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,
      82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,
      82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,
      82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,
      82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,
      82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  82,
      82,  82,  82,  82,  82,  82,  82,  82,  82,  82,  83,  83,  83,  83,  83,
      83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,
      83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,
      83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,
      83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,
      83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,
      83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,  83,
      83,  83,  83,  83,  83,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,
      84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,
      84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,
      84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,
      84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,
      84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,
      84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,  84,
      85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,
      85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,
      85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,
      85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,
      85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,
      85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,
      85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  86,  86,  86,  86,  86,
      86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,
      86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,
      86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,
      86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,
      86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,
      86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,  86,
      86,  86,  86,  86,  86,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,
      87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,
      87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,
      87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,
      87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,
      87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,
      87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,
      88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,
      88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,
      88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,
      88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,
      88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,
      88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  88,
      88,  88,  88,  88,  88,  88,  88,  88,  88,  88,  89,  89,  89,  89,  89,
      89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,
      89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,
      89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,
      89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,
      89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,
      89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,  89,
      89,  89,  89,  89,  89,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,
      90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,
      90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,
      90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,
      90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,
      90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,
      90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,  90,
      91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,
      91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,
      91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,
      91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,
      91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,
      91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,
      91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  92,  92,  92,  92,  92,
      92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,
      92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,
      92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,
      92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,
      92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,
      92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,  92,
      92,  92,  92,  92,  92,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,
      93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,
      93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,
      93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,
      93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,
      93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,
      93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,  93,
      94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,
      94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,
      94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,
      94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,
      94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,
      94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,
      94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  95,  95,  95,  95,  95,
      95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,
      95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,
      95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,
      95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,
      95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,
      95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,  95,
      95,  95,  95,  95,  95,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,
      96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,
      96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,
      96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,
      96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,
      96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,
      96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,
      97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,
      97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,
      97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,
      97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,
      97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,
      97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  97,
      97,  97,  97,  97,  97,  97,  97,  97,  97,  97,  98,  98,  98,  98,  98,
      98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,
      98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,
      98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,
      98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,
      98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,
      98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,  98,
      98,  98,  98,  98,  98,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,
      99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,
      99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,
      99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,
      99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,
      99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,
      99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,  99,
      100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
      100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
      100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
      100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
      100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
      100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
      100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
  binaryOccupancyMap *val;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  plannerAStarGrid *b_obj;
  real_T a[10000];
  real_T poseIdTemp[10000];
  real_T d;
  int32_T a__2_size[2];
  int32_T i;
  char_T a__2_data[9];
  boolean_T mat[10000];
  boolean_T b;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_obj = obj;
  b_obj->isFirstRun = 1.0;
  st.site = &th_emlrtRSI;
  b_st.site = &wc_emlrtRSI;
  st.site = &uh_emlrtRSI;
  b_obj->Map = varargin_1;
  b_st.site = &ei_emlrtRSI;
  for (i = 0; i < 10000; i++) {
    b_obj->GCostMatrix[i] = rtInf;
  }
  for (i = 0; i < 30000; i++) {
    b_obj->IdPose[i] = 0.0;
  }
  for (i = 0; i < 10000; i++) {
    b_obj->IdPose[i] = iv[i];
  }
  for (i = 0; i < 10000; i++) {
    b_obj->IdPose[i + 10000] = iv1[i];
  }
  for (i = 0; i < 10000; i++) {
    b_obj->IdPose[i + 20000] = 1.0;
  }
  for (i = 0; i < 10000; i++) {
    poseIdTemp[i] = b_obj->IdPose[i + 10000] - 1.0;
  }
  for (i = 0; i < 10000; i++) {
    a[i] = b_obj->IdPose[i + 20000] - 1.0;
  }
  for (i = 0; i < 10000; i++) {
    poseIdTemp[i] =
        (b_obj->IdPose[i] + poseIdTemp[i] * 100.0) + a[i] * 100.0 * 100.0;
  }
  for (i = 0; i < 10000; i++) {
    b_obj->PoseId[i] = poseIdTemp[i];
  }
  st.site = &vh_emlrtRSI;
  b_st.site = &hi_emlrtRSI;
  b = (b_obj->isFirstRun == 0.0);
  if (b) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &ab_emlrtRTEI,
        "nav:navalgs:plannerastargrid:PropertySetInCodeGeneration",
        "nav:navalgs:plannerastargrid:PropertySetInCodeGeneration", 3, 4, 8,
        "GCostFcn");
  }
  b_st.site = &ii_emlrtRSI;
  b_obj->UseCustomG = 1.0;
  st.site = &wh_emlrtRSI;
  b_st.site = &ji_emlrtRSI;
  b = (b_obj->isFirstRun == 0.0);
  if (b) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &ab_emlrtRTEI,
        "nav:navalgs:plannerastargrid:PropertySetInCodeGeneration",
        "nav:navalgs:plannerastargrid:PropertySetInCodeGeneration", 3, 4, 5,
        "GCost");
  }
  b_st.site = &ki_emlrtRSI;
  b_obj->UseCustomG = 0.0;
  b_st.site = &li_emlrtRSI;
  d = c_validateAStarBuiltinCostFunct(&b_st, a__2_data, a__2_size);
  b_obj->GCost = d;
  b_obj->UseCustomG = 0.0;
  st.site = &xh_emlrtRSI;
  b_st.site = &mi_emlrtRSI;
  b = (b_obj->isFirstRun == 0.0);
  if (b) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &ab_emlrtRTEI,
        "nav:navalgs:plannerastargrid:PropertySetInCodeGeneration",
        "nav:navalgs:plannerastargrid:PropertySetInCodeGeneration", 3, 4, 8,
        "HCostFcn");
  }
  b_st.site = &ni_emlrtRSI;
  b_obj->UseCustomH = 1.0;
  st.site = &yh_emlrtRSI;
  b_st.site = &oi_emlrtRSI;
  b = (b_obj->isFirstRun == 0.0);
  if (b) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &ab_emlrtRTEI,
        "nav:navalgs:plannerastargrid:PropertySetInCodeGeneration",
        "nav:navalgs:plannerastargrid:PropertySetInCodeGeneration", 3, 4, 5,
        "HCost");
  }
  b_st.site = &pi_emlrtRSI;
  b_obj->UseCustomH = 0.0;
  b_st.site = &qi_emlrtRSI;
  d = c_validateAStarBuiltinCostFunct(&b_st, a__2_data, a__2_size);
  b_obj->HCost = d;
  b_obj->UseCustomH = 0.0;
  st.site = &ai_emlrtRSI;
  b_obj->OccupiedThreshold = 0.65;
  st.site = &bi_emlrtRSI;
  b_obj->TieBreaker = 0.0;
  st.site = &ci_emlrtRSI;
  b_obj->DiagonalSearch = 1.0;
  st.site = &di_emlrtRSI;
  b_st.site = &ri_emlrtRSI;
  val = b_obj->Map;
  b_st.site = &ri_emlrtRSI;
  c_st.site = &ud_emlrtRSI;
  MapLayer_getValueAllImpl(&c_st, val, mat);
  for (i = 0; i < 10000; i++) {
    b_obj->OccupancyMatrix[i] = mat[i];
  }
  b_obj->isFirstRun = 0.0;
  return b_obj;
}

real_T plannerAStarGrid_Euclidean(const emlrtStack *sp, const real_T pose1[2],
                                  const real_T pose2[2])
{
  __m128d r;
  emlrtStack st;
  real_T y[2];
  real_T dist;
  st.prev = sp;
  st.tls = sp->tls;
  r = _mm_sub_pd(_mm_loadu_pd(&pose1[0]), _mm_loadu_pd(&pose2[0]));
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  dist = b_sumColumnB(y);
  st.site = &fm_emlrtRSI;
  if (dist < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &jb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  return muDoubleScalarSqrt(dist);
}

void plannerAStarGrid_plan(codegenPathPlannerStackData *SD,
                           const emlrtStack *sp, plannerAStarGrid *obj,
                           const real_T start[2], const real_T goal[2])
{
  binaryOccupancyMap *val;
  emlrtStack b_st;
  emlrtStack st;
  emxArray_real_T *path;
  real_T goalgrid[2];
  real_T startgrid[2];
  real_T x;
  int32_T k;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &kj_emlrtRSI;
  b_st.site = &ab_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (!muDoubleScalarIsNaN(start[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:plannerAStarGrid:expectedNonNaN", 3, 4, 5, "Start");
  }
  b_st.site = &ab_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if ((!muDoubleScalarIsInf(start[k])) && (!muDoubleScalarIsNaN(start[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &h_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:plannerAStarGrid:expectedFinite", 3, 4, 5, "Start");
  }
  st.site = &lj_emlrtRSI;
  b_st.site = &ab_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (!muDoubleScalarIsNaN(goal[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:plannerAStarGrid:expectedNonNaN", 3, 4, 4, "Goal");
  }
  b_st.site = &ab_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if ((!muDoubleScalarIsInf(goal[k])) && (!muDoubleScalarIsNaN(goal[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &h_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:plannerAStarGrid:expectedFinite", 3, 4, 4, "Goal");
  }
  obj->IsGrid = 0.0;
  obj->IsGrid = 1.0;
  startgrid[0] = muDoubleScalarFloor(start[0]);
  startgrid[1] = muDoubleScalarFloor(start[1]);
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (!(start[k] == startgrid[k])) {
      p = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        sp, &db_emlrtRTEI, "nav:navalgs:plannerastargrid:ValidateGridInput",
        "nav:navalgs:plannerastargrid:ValidateGridInput", 3, 4, 5, "Start");
  }
  startgrid[0] = muDoubleScalarFloor(goal[0]);
  startgrid[1] = muDoubleScalarFloor(goal[1]);
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (!(goal[k] == startgrid[k])) {
      p = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        sp, &eb_emlrtRTEI, "nav:navalgs:plannerastargrid:ValidateGridInput",
        "nav:navalgs:plannerastargrid:ValidateGridInput", 3, 4, 4, "Goal");
  }
  st.site = &mj_emlrtRSI;
  x = obj->IsGrid;
  if (muDoubleScalarIsNaN(x)) {
    emlrtErrorWithMessageIdR2018a(&st, &t_emlrtRTEI, "MATLAB:nologicalnan",
                                  "MATLAB:nologicalnan", 0);
  }
  if (!(obj->IsGrid != 0.0)) {
    st.site = &nj_emlrtRSI;
    val = obj->Map;
    st.site = &nj_emlrtRSI;
    MapInterface_world2gridImpl(val, start, startgrid);
    st.site = &oj_emlrtRSI;
    val = obj->Map;
    st.site = &oj_emlrtRSI;
    MapInterface_world2gridImpl(val, goal, goalgrid);
  } else {
    startgrid[0] = start[0];
    goalgrid[0] = goal[0];
    startgrid[1] = start[1];
    goalgrid[1] = goal[1];
  }
  emxInit_real_T(sp, &path, 2, &ye_emlrtRTEI);
  st.site = &pj_emlrtRSI;
  plannerAStarGrid_planOM(SD, &st, obj, startgrid, goalgrid, path,
                          SD->f5.expl_temp, &x);
  emxFree_real_T(sp, &path);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (plannerAStarGrid.c) */
