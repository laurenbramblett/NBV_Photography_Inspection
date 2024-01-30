/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * plannerHybridAStar.c
 *
 * Code generation for function 'plannerHybridAStar'
 *
 */

/* Include files */
#include "plannerHybridAStar.h"
#include "MapInterface.h"
#include "MapLayer.h"
#include "NameValueParser.h"
#include "NodeMap.h"
#include "PriorityQueue.h"
#include "ReedsSheppBuiltins.h"
#include "all.h"
#include "any.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_internal_types.h"
#include "codegenPathPlanner_mexutil.h"
#include "codegenPathPlanner_types.h"
#include "diff.h"
#include "flipud.h"
#include "handle.h"
#include "ifWhileCond.h"
#include "indexShapeCheck.h"
#include "linspace.h"
#include "navPath.h"
#include "nnz.h"
#include "nonzeros.h"
#include "nullAssignment.h"
#include "plannerAStarGrid.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "strcmp.h"
#include "sum.h"
#include "sumMatrixIncludeNaN.h"
#include "unique.h"
#include "validatestring.h"
#include "validatorOccupancyMap.h"
#include "autonomouscodegen_reeds_shepp_tbb_api.hpp"
#include "mwmathutil.h"
#include "priorityqueue_api.hpp"
#include <emmintrin.h>
#include <math.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo vd_emlrtRSI = { 1009,/* lineNo */
  "plannerHybridAStar/plannerHybridAStar",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo wd_emlrtRSI = { 1016,/* lineNo */
  "plannerHybridAStar/plannerHybridAStar",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo xd_emlrtRSI = { 1019,/* lineNo */
  "plannerHybridAStar/plannerHybridAStar",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo yd_emlrtRSI = { 1020,/* lineNo */
  "plannerHybridAStar/plannerHybridAStar",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ae_emlrtRSI = { 1816,/* lineNo */
  "plannerHybridAStar/set.StateValidator",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo be_emlrtRSI = { 1817,/* lineNo */
  "plannerHybridAStar/set.StateValidator",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ce_emlrtRSI = { 381,/* lineNo */
  "plannerHybridAStar/assigningValuesToProperties",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo de_emlrtRSI = { 17, /* lineNo */
  "local_num2str",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\private\\local_num2str.m"/* pathName */
};

static emlrtRSInfo fe_emlrtRSI = { 1824,/* lineNo */
  "plannerHybridAStar/set.MinTurningRadius",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ge_emlrtRSI = { 341,/* lineNo */
  "plannerHybridAStar/validateMinimumTurningRadius",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo he_emlrtRSI = { 22, /* lineNo */
  "validatege",                        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validatege.m"/* pathName */
};

static emlrtRSInfo ie_emlrtRSI = { 1833,/* lineNo */
  "plannerHybridAStar/set.MotionPrimitiveLength",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo je_emlrtRSI = { 319,/* lineNo */
  "plannerHybridAStar/validateMotionPrimitiveLength",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ke_emlrtRSI = { 317,/* lineNo */
  "plannerHybridAStar/validateMotionPrimitiveLength",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo le_emlrtRSI = { 22, /* lineNo */
  "validatele",                        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validatele.m"/* pathName */
};

static emlrtRSInfo me_emlrtRSI = { 1121,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ne_emlrtRSI = { 1125,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo oe_emlrtRSI = { 1136,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo pe_emlrtRSI = { 1137,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo qe_emlrtRSI = { 1140,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo re_emlrtRSI = { 1143,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo se_emlrtRSI = { 1148,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo te_emlrtRSI = { 1179,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ue_emlrtRSI = { 1183,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ve_emlrtRSI = { 1195,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo we_emlrtRSI = { 1202,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo xe_emlrtRSI = { 1211,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ye_emlrtRSI = { 1228,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo af_emlrtRSI = { 1229,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo bf_emlrtRSI = { 1241,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo cf_emlrtRSI = { 1255,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo df_emlrtRSI = { 1259,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ef_emlrtRSI = { 1261,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ff_emlrtRSI = { 1272,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo gf_emlrtRSI = { 1276,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo hf_emlrtRSI = { 1286,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo if_emlrtRSI = { 1298,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo jf_emlrtRSI = { 1299,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo kf_emlrtRSI = { 1302,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo lf_emlrtRSI = { 1306,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo mf_emlrtRSI = { 1331,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo nf_emlrtRSI = { 1335,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo of_emlrtRSI = { 1340,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo pf_emlrtRSI = { 1341,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo qf_emlrtRSI = { 1342,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo rf_emlrtRSI = { 1344,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo sf_emlrtRSI = { 1347,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo tf_emlrtRSI = { 1353,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo uf_emlrtRSI = { 1343,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo vf_emlrtRSI = { 1354,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo wf_emlrtRSI = { 1358,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo xf_emlrtRSI = { 1359,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo yf_emlrtRSI = { 1362,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ag_emlrtRSI = { 1363,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo bg_emlrtRSI = { 1375,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo cg_emlrtRSI = { 1376,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo dg_emlrtRSI = { 1381,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo eg_emlrtRSI = { 1390,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo fg_emlrtRSI = { 1393,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo gg_emlrtRSI = { 1394,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo hg_emlrtRSI = { 1411,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ig_emlrtRSI = { 1416,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo jg_emlrtRSI = { 1418,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo kg_emlrtRSI = { 1448,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo lg_emlrtRSI = { 1464,/* lineNo */
  "plannerHybridAStar/plan",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo mg_emlrtRSI = { 297,/* lineNo */
  "plannerHybridAStar/validateStartGoal",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ng_emlrtRSI = { 300,/* lineNo */
  "plannerHybridAStar/validateStartGoal",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo sh_emlrtRSI = { 380,/* lineNo */
  "MapInterface/world2grid",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+matlabshared\\+autonomous\\+internal\\MapInte"
  "rface.m"                            /* pathName */
};

static emlrtRSInfo ui_emlrtRSI = { 454,/* lineNo */
  "plannerHybridAStar/get2DHeuristic", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo vi_emlrtRSI = { 477,/* lineNo */
  "plannerHybridAStar/get2DHeuristic", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo wi_emlrtRSI = { 478,/* lineNo */
  "plannerHybridAStar/get2DHeuristic", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo xi_emlrtRSI = { 485,/* lineNo */
  "plannerHybridAStar/get2DHeuristic", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ij_emlrtRSI = { 46, /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo jj_emlrtRSI = { 92, /* lineNo */
  "minimum",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo mn_emlrtRSI = { 442,/* lineNo */
  "plannerHybridAStar/get3DHeuristic", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo nn_emlrtRSI = { 445,/* lineNo */
  "plannerHybridAStar/get3DHeuristic", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo sn_emlrtRSI = { 593,/* lineNo */
  "plannerHybridAStar/checkAnalyticExpansion",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo tn_emlrtRSI = { 596,/* lineNo */
  "plannerHybridAStar/checkAnalyticExpansion",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo un_emlrtRSI = { 610,/* lineNo */
  "plannerHybridAStar/checkAnalyticExpansion",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo hp_emlrtRSI = { 784,/* lineNo */
  "plannerHybridAStar/getInterpolatedPath",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ip_emlrtRSI = { 791,/* lineNo */
  "plannerHybridAStar/getInterpolatedPath",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo jp_emlrtRSI = { 794,/* lineNo */
  "plannerHybridAStar/getInterpolatedPath",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo kp_emlrtRSI = { 800,/* lineNo */
  "plannerHybridAStar/getInterpolatedPath",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo lp_emlrtRSI = { 750,/* lineNo */
  "plannerHybridAStar/getInterpolatedPath",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo mp_emlrtRSI = { 751,/* lineNo */
  "plannerHybridAStar/getInterpolatedPath",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo qp_emlrtRSI = { 48, /* lineNo */
  "unique",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m"/* pathName */
};

static emlrtRSInfo bq_emlrtRSI = { 22, /* lineNo */
  "nullAssignment",                    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\nullAssignment.m"/* pathName */
};

static emlrtRSInfo cq_emlrtRSI = { 26, /* lineNo */
  "nullAssignment",                    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\nullAssignment.m"/* pathName */
};

static emlrtRSInfo eq_emlrtRSI = { 13, /* lineNo */
  "nullAssignment",                    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\nullAssignment.m"/* pathName */
};

static emlrtRSInfo fq_emlrtRSI = { 17, /* lineNo */
  "nullAssignment",                    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\nullAssignment.m"/* pathName */
};

static emlrtRSInfo lq_emlrtRSI = { 11, /* lineNo */
  "sin",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elfun\\sin.m"/* pathName */
};

static emlrtRSInfo mq_emlrtRSI = { 11, /* lineNo */
  "cos",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elfun\\cos.m"/* pathName */
};

static emlrtRSInfo nq_emlrtRSI = { 561,/* lineNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo oq_emlrtRSI = { 558,/* lineNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo pq_emlrtRSI = { 548,/* lineNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo qq_emlrtRSI = { 542,/* lineNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo rq_emlrtRSI = { 539,/* lineNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo sq_emlrtRSI = { 519,/* lineNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo tq_emlrtRSI = { 516,/* lineNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo uq_emlrtRSI = { 513,/* lineNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo vq_emlrtRSI = { 508,/* lineNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo wq_emlrtRSI = { 506,/* lineNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo xq_emlrtRSI = { 499,/* lineNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo yq_emlrtRSI = { 376,/* lineNo */
  "MapInterface/world2grid",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+matlabshared\\+autonomous\\+internal\\MapInte"
  "rface.m"                            /* pathName */
};

static emlrtRSInfo br_emlrtRSI = { 977,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo cr_emlrtRSI = { 979,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo dr_emlrtRSI = { 982,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo er_emlrtRSI = { 983,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo fr_emlrtRSI = { 984,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo gr_emlrtRSI = { 985,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo hr_emlrtRSI = { 993,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ir_emlrtRSI = { 991,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo jr_emlrtRSI = { 997,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo kr_emlrtRSI = { 992,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo lr_emlrtRSI = { 998,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo mr_emlrtRSI = { 999,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo nr_emlrtRSI = { 1001,/* lineNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ur_emlrtRSI = { 51, /* lineNo */
  "reshapeSizeChecks",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pathName */
};

static emlrtRSInfo vr_emlrtRSI = { 119,/* lineNo */
  "computeDimsData",                   /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pathName */
};

static emlrtRSInfo es_emlrtRSI = { 41, /* lineNo */
  "cat",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m"/* pathName */
};

static emlrtRSInfo fs_emlrtRSI = { 617,/* lineNo */
  "plannerHybridAStar/calculateCost",  /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo gs_emlrtRSI = { 620,/* lineNo */
  "plannerHybridAStar/calculateCost",  /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo hs_emlrtRSI = { 915,/* lineNo */
  "plannerHybridAStar/calculateGScore",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo is_emlrtRSI = { 920,/* lineNo */
  "plannerHybridAStar/calculateGScore",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo js_emlrtRSI = { 924,/* lineNo */
  "plannerHybridAStar/calculateGScore",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ks_emlrtRSI = { 446,/* lineNo */
  "plannerHybridAStar/get3DHeuristic", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pathName */
};

static emlrtRSInfo ms_emlrtRSI = { 38, /* lineNo */
  "squeeze",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\squeeze.m"/* pathName */
};

static emlrtRSInfo os_emlrtRSI = { 44, /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRTEInfo l_emlrtRTEI = { 1813,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar/set.StateValidator",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo m_emlrtRTEI = { 325,/* lineNo */
  17,                                  /* colNo */
  "plannerHybridAStar/validateMotionPrimitiveLength",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo n_emlrtRTEI = { 22,/* lineNo */
  27,                                  /* colNo */
  "validatege",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validatege.m"/* pName */
};

static emlrtRTEInfo o_emlrtRTEI = { 22,/* lineNo */
  27,                                  /* colNo */
  "validatele",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+valattr\\validatele.m"/* pName */
};

static emlrtBCInfo g_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  1276,                                /* lineNo */
  48,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/plan",           /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = { 1,   /* iFirst */
  50000,                               /* iLast */
  1291,                                /* lineNo */
  44,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/plan",           /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo p_emlrtRTEI = { 1309,/* lineNo */
  33,                                  /* colNo */
  "plannerHybridAStar/plan",           /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo q_emlrtRTEI = { 1312,/* lineNo */
  33,                                  /* colNo */
  "plannerHybridAStar/plan",           /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo r_emlrtRTEI = { 1351,/* lineNo */
  29,                                  /* colNo */
  "plannerHybridAStar/plan",           /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtBCInfo i_emlrtBCI = { 1,   /* iFirst */
  50000,                               /* iLast */
  1352,                                /* lineNo */
  44,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/plan",           /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = { 1,   /* iFirst */
  50000,                               /* iLast */
  1352,                                /* lineNo */
  64,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/plan",           /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo f_emlrtECI = { -1,  /* nDims */
  1352,                                /* lineNo */
  25,                                  /* colNo */
  "plannerHybridAStar/plan",           /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtBCInfo k_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  1362,                                /* lineNo */
  51,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/plan",           /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = { 1,   /* iFirst */
  50000,                               /* iLast */
  1386,                                /* lineNo */
  39,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/plan",           /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = { 1,   /* iFirst */
  10000,                               /* iLast */
  640,                                 /* lineNo */
  38,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/closeCell",      /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  3                                    /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = { 640, /* lineNo */
  38,                                  /* colNo */
  "plannerHybridAStar/closeCell",      /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = { 1,   /* iFirst */
  10000,                               /* iLast */
  636,                                 /* lineNo */
  39,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/closeCell",      /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  3                                    /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = { 636, /* lineNo */
  39,                                  /* colNo */
  "plannerHybridAStar/closeCell",      /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  1363,                                /* lineNo */
  61,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/plan",           /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  1260,                                /* lineNo */
  47,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/plan",           /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo u_emlrtRTEI = { 298,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar/validateStartGoal",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo v_emlrtRTEI = { 301,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar/validateStartGoal",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtDCInfo h_emlrtDCI = { 460, /* lineNo */
  39,                                  /* colNo */
  "plannerHybridAStar/get2DHeuristic", /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = { 1,   /* iFirst */
  10000,                               /* iLast */
  460,                                 /* lineNo */
  39,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/get2DHeuristic", /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo j_emlrtECI = { 1,   /* nDims */
  477,                                 /* lineNo */
  41,                                  /* colNo */
  "plannerHybridAStar/get2DHeuristic", /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo cb_emlrtRTEI = { 134,/* lineNo */
  27,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

static emlrtBCInfo w_emlrtBCI = { 1,   /* iFirst */
  10000,                               /* iLast */
  477,                                 /* lineNo */
  60,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/get2DHeuristic", /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo i_emlrtDCI = { 477, /* lineNo */
  60,                                  /* colNo */
  "plannerHybridAStar/get2DHeuristic", /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtRTEInfo rb_emlrtRTEI = { 591,/* lineNo */
  21,                                  /* colNo */
  "plannerHybridAStar/checkAnalyticExpansion",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo sb_emlrtRTEI = { 587,/* lineNo */
  21,                                  /* colNo */
  "plannerHybridAStar/checkAnalyticExpansion",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo dc_emlrtRTEI = { 679,/* lineNo */
  71,                                  /* colNo */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtBCInfo ke_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  687,                                 /* lineNo */
  31,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo le_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  689,                                 /* lineNo */
  42,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo me_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  689,                                 /* lineNo */
  45,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ne_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  690,                                 /* lineNo */
  34,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo oe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  690,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo pe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  691,                                 /* lineNo */
  34,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo qe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  691,                                 /* lineNo */
  39,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo re_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  692,                                 /* lineNo */
  34,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo se_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  692,                                 /* lineNo */
  39,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo te_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  695,                                 /* lineNo */
  73,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ue_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  700,                                 /* lineNo */
  42,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ve_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  700,                                 /* lineNo */
  45,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo we_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  701,                                 /* lineNo */
  34,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo xe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  701,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ye_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  702,                                 /* lineNo */
  34,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo af_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  702,                                 /* lineNo */
  39,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo bf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  703,                                 /* lineNo */
  34,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo cf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  703,                                 /* lineNo */
  39,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo df_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  708,                                 /* lineNo */
  74,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ef_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  709,                                 /* lineNo */
  78,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ff_emlrtBCI = { 1,  /* iFirst */
  50000,                               /* iLast */
  695,                                 /* lineNo */
  35,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo gf_emlrtBCI = { 1,  /* iFirst */
  50000,                               /* iLast */
  708,                                 /* lineNo */
  35,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo hf_emlrtBCI = { 1,  /* iFirst */
  50000,                               /* iLast */
  714,                                 /* lineNo */
  45,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo if_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  687,                                 /* lineNo */
  29,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo jf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  687,                                 /* lineNo */
  46,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getFinalPathData",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo ec_emlrtRTEI = { 158,/* lineNo */
  9,                                   /* colNo */
  "onearg_null_assignment",            /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\nullAssignment.m"/* pName */
};

static emlrtRTEInfo fc_emlrtRTEI = { 85,/* lineNo */
  27,                                  /* colNo */
  "validate_inputs",                   /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\nullAssignment.m"/* pName */
};

static emlrtRTEInfo gc_emlrtRTEI = { 296,/* lineNo */
  1,                                   /* colNo */
  "delete_rows",                       /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\nullAssignment.m"/* pName */
};

static emlrtRTEInfo ic_emlrtRTEI = { 819,/* lineNo */
  17,                                  /* colNo */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo jc_emlrtRTEI = { 816,/* lineNo */
  68,                                  /* colNo */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtBCInfo kf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  809,                                 /* lineNo */
  33,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo kc_emlrtRTEI = { 782,/* lineNo */
  21,                                  /* colNo */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtBCInfo lf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  760,                                 /* lineNo */
  53,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo mf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  757,                                 /* lineNo */
  35,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo vc_emlrtDCI = { 757,/* lineNo */
  35,                                  /* colNo */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtRTEInfo lc_emlrtRTEI = { 741,/* lineNo */
  21,                                  /* colNo */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo mc_emlrtRTEI = { 734,/* lineNo */
  21,                                  /* colNo */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtBCInfo nf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  761,                                 /* lineNo */
  28,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo of_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  770,                                 /* lineNo */
  28,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo wc_emlrtDCI = { 736,/* lineNo */
  28,                                  /* colNo */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  4                                    /* checkKind */
};

static emlrtBCInfo pf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  768,                                 /* lineNo */
  55,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo qf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  760,                                 /* lineNo */
  83,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo rf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  769,                                 /* lineNo */
  64,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo sf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  762,                                 /* lineNo */
  32,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo tf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  771,                                 /* lineNo */
  32,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/getInterpolatedPath",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo qc_emlrtRTEI = { 81,/* lineNo */
  23,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pName */
};

static emlrtRTEInfo rc_emlrtRTEI = { 79,/* lineNo */
  23,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pName */
};

static emlrtRTEInfo sc_emlrtRTEI = { 74,/* lineNo */
  13,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pName */
};

static emlrtRTEInfo tc_emlrtRTEI = { 530,/* lineNo */
  21,                                  /* colNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtDCInfo xc_emlrtDCI = { 535,/* lineNo */
  40,                                  /* colNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo uf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  535,                                 /* lineNo */
  40,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo yc_emlrtDCI = { 535,/* lineNo */
  72,                                  /* colNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo vf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  535,                                 /* lineNo */
  72,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo ad_emlrtDCI = { 534,/* lineNo */
  47,                                  /* colNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo wf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  534,                                 /* lineNo */
  47,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo bd_emlrtDCI = { 534,/* lineNo */
  67,                                  /* colNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo xf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  534,                                 /* lineNo */
  67,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo gb_emlrtECI = { -1, /* nDims */
  534,                                 /* lineNo */
  21,                                  /* colNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtECInfo hb_emlrtECI = { -1, /* nDims */
  559,                                 /* lineNo */
  51,                                  /* colNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtDCInfo cd_emlrtDCI = { 531,/* lineNo */
  51,                                  /* colNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo dd_emlrtDCI = { 531,/* lineNo */
  51,                                  /* colNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  4                                    /* checkKind */
};

static emlrtDCInfo ed_emlrtDCI = { 531,/* lineNo */
  17,                                  /* colNo */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo yf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  535,                                 /* lineNo */
  54,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ag_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  535,                                 /* lineNo */
  94,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo bg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  545,                                 /* lineNo */
  74,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo cg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  545,                                 /* lineNo */
  48,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/isCircularPrimitiveValid",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo dg_emlrtBCI = { 1,  /* iFirst */
  10000,                               /* iLast */
  665,                                 /* lineNo */
  65,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/checkNodeValidity",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo gd_emlrtDCI = { 665,/* lineNo */
  65,                                  /* colNo */
  "plannerHybridAStar/checkNodeValidity",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo eg_emlrtBCI = { 1,  /* iFirst */
  10000,                               /* iLast */
  663,                                 /* lineNo */
  66,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/checkNodeValidity",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo hd_emlrtDCI = { 663,/* lineNo */
  66,                                  /* colNo */
  "plannerHybridAStar/checkNodeValidity",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo fg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  617,                                 /* lineNo */
  54,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/calculateCost",  /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo gg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  623,                                 /* lineNo */
  28,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/calculateCost",  /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo ib_emlrtECI = { 1,  /* nDims */
  624,                                 /* lineNo */
  26,                                  /* colNo */
  "plannerHybridAStar/calculateCost",  /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtECInfo jb_emlrtECI = { 1,  /* nDims */
  626,                                 /* lineNo */
  26,                                  /* colNo */
  "plannerHybridAStar/calculateCost",  /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtECInfo kb_emlrtECI = { 1,  /* nDims */
  920,                                 /* lineNo */
  26,                                  /* colNo */
  "plannerHybridAStar/calculateGScore",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtECInfo lb_emlrtECI = { 1,  /* nDims */
  924,                                 /* lineNo */
  26,                                  /* colNo */
  "plannerHybridAStar/calculateGScore",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtECInfo mb_emlrtECI = { -1, /* nDims */
  457,                                 /* lineNo */
  23,                                  /* colNo */
  "plannerHybridAStar/get2DHeuristic", /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtBCInfo hg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  470,                                 /* lineNo */
  51,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/get2DHeuristic", /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ig_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  477,                                 /* lineNo */
  97,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/get2DHeuristic", /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo jg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  485,                                 /* lineNo */
  77,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/get2DHeuristic", /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo kg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  468,                                 /* lineNo */
  57,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/get2DHeuristic", /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo lg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  483,                                 /* lineNo */
  56,                                  /* colNo */
  "",                                  /* aName */
  "plannerHybridAStar/get2DHeuristic", /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo nb_emlrtECI = { 2,  /* nDims */
  985,                                 /* lineNo */
  23,                                  /* colNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtECInfo ob_emlrtECI = { 2,  /* nDims */
  984,                                 /* lineNo */
  23,                                  /* colNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtECInfo pb_emlrtECI = { 2,  /* nDims */
  983,                                 /* lineNo */
  22,                                  /* colNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtECInfo qb_emlrtECI = { 2,  /* nDims */
  982,                                 /* lineNo */
  22,                                  /* colNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtECInfo rb_emlrtECI = { 2,  /* nDims */
  979,                                 /* lineNo */
  22,                                  /* colNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo bd_emlrtRTEI = { 975,/* lineNo */
  17,                                  /* colNo */
  "plannerHybridAStar/getPosesCircularPrimitive",/* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo ie_emlrtRTEI = { 1161,/* lineNo */
  17,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo je_emlrtRTEI = { 1298,/* lineNo */
  25,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo ke_emlrtRTEI = { 1302,/* lineNo */
  25,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo le_emlrtRTEI = { 1029,/* lineNo */
  49,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo me_emlrtRTEI = { 1353,/* lineNo */
  30,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo ne_emlrtRTEI = { 1354,/* lineNo */
  48,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo eg_emlrtRTEI = { 49,/* lineNo */
  20,                                  /* colNo */
  "linspace",                          /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\linspace.m"/* pName */
};

static emlrtRTEInfo fg_emlrtRTEI = { 593,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo gg_emlrtRTEI = { 596,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo hg_emlrtRTEI = { 610,/* lineNo */
  26,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo li_emlrtRTEI = { 714,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo mi_emlrtRTEI = { 671,/* lineNo */
  34,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo ni_emlrtRTEI = { 694,/* lineNo */
  63,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo oi_emlrtRTEI = { 705,/* lineNo */
  58,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo pi_emlrtRTEI = { 736,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo qi_emlrtRTEI = { 737,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo ri_emlrtRTEI = { 750,/* lineNo */
  21,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo si_emlrtRTEI = { 801,/* lineNo */
  51,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo ti_emlrtRTEI = { 751,/* lineNo */
  21,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo ui_emlrtRTEI = { 808,/* lineNo */
  24,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo vi_emlrtRTEI = { 794,/* lineNo */
  34,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo wi_emlrtRTEI = { 808,/* lineNo */
  17,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo xi_emlrtRTEI = { 809,/* lineNo */
  23,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo yi_emlrtRTEI = { 809,/* lineNo */
  17,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo aj_emlrtRTEI = { 784,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo ij_emlrtRTEI = { 531,/* lineNo */
  17,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo jj_emlrtRTEI = { 519,/* lineNo */
  17,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo kj_emlrtRTEI = { 499,/* lineNo */
  27,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo lj_emlrtRTEI = { 380,/* lineNo */
  19,                                  /* colNo */
  "MapInterface",                      /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+matlabshared\\+autonomous\\+internal\\MapInte"
  "rface.m"                            /* pName */
};

static emlrtRTEInfo mj_emlrtRTEI = { 539,/* lineNo */
  39,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo nj_emlrtRTEI = { 76,/* lineNo */
  9,                                   /* colNo */
  "eml_mtimes_helper",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pName */
};

static emlrtRTEInfo oj_emlrtRTEI = { 924,/* lineNo */
  35,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo pj_emlrtRTEI = { 924,/* lineNo */
  80,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo qj_emlrtRTEI = { 457,/* lineNo */
  39,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo wj_emlrtRTEI = { 978,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo xj_emlrtRTEI = { 979,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo yj_emlrtRTEI = { 982,/* lineNo */
  22,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo ak_emlrtRTEI = { 982,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo bk_emlrtRTEI = { 983,/* lineNo */
  22,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo ck_emlrtRTEI = { 983,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo dk_emlrtRTEI = { 991,/* lineNo */
  17,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo ek_emlrtRTEI = { 992,/* lineNo */
  17,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo fk_emlrtRTEI = { 997,/* lineNo */
  31,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo gk_emlrtRTEI = { 998,/* lineNo */
  31,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo hk_emlrtRTEI = { 999,/* lineNo */
  30,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo ik_emlrtRTEI = { 1001,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo jk_emlrtRTEI = { 977,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo kk_emlrtRTEI = { 985,/* lineNo */
  13,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo lk_emlrtRTEI = { 997,/* lineNo */
  23,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo mk_emlrtRTEI = { 998,/* lineNo */
  23,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo nk_emlrtRTEI = { 999,/* lineNo */
  22,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo gl_emlrtRTEI = { 985,/* lineNo */
  23,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

static emlrtRTEInfo hl_emlrtRTEI = { 984,/* lineNo */
  23,                                  /* colNo */
  "plannerHybridAStar",                /* fName */
  "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\plannerHybridAStar.m"/* pName */
};

/* Function Declarations */
static int32_T b_plus(real_T in1_data[], const real_T in2_data[], const int32_T *
                      in2_size, const real_T in3_data[], const int32_T *in3_size);
static int32_T c_plannerHybridAStar_calculateC(codegenPathPlannerStackData *SD,
  const emlrtStack *sp, plannerHybridAStar *obj, const real_T newNodeData_data[],
  const int32_T newNodeData_size[2], const real_T currentNode_data[], const
  int32_T currentNode_size[2], const real_T curvature_data[], int32_T
  curvature_size, real_T direction, real_T fScore_data[], real_T gScore_data[],
  int32_T *gScore_size, real_T hScore_data[], int32_T *hScore_size);
static int32_T c_plannerHybridAStar_calculateG(const emlrtStack *sp, const
  plannerHybridAStar *obj, real_T parentGScore, const real_T curvature_data[],
  int32_T curvature_size, real_T direction, real_T gScore_data[]);
static boolean_T c_plannerHybridAStar_checkAnaly(const emlrtStack *sp,
  plannerHybridAStar *obj, const real_T initialPose[3], const real_T finalPose[3],
  real_T stepSize);
static int32_T c_plannerHybridAStar_checkNodeV(const emlrtStack *sp, const
  plannerHybridAStar *obj, const real_T PointsGrid_data[], real_T direction,
  real_T nodeValidity_data[]);
static real_T c_plannerHybridAStar_get2DHeuri(codegenPathPlannerStackData *SD,
  const emlrtStack *sp, plannerHybridAStar *obj, const real_T point[2]);
static real_T c_plannerHybridAStar_get3DHeuri(const plannerHybridAStar *obj,
  const real_T start[3], const real_T goal[3]);
static void c_plannerHybridAStar_getCircula(real_T length, const real_T
  curvature_data[], const real_T initialNodePose[3], real_T direction, real_T
  newNodesPoses_data[], int32_T newNodesPoses_size[2], real_T ICRsData_data[],
  int32_T ICRsData_size[2]);
static void c_plannerHybridAStar_getFinalPa(codegenPathPlannerStackData *SD,
  const emlrtStack *sp, const plannerHybridAStar *obj, const emxArray_real_T
  *pathData, emxArray_real_T *finalPathData);
static void c_plannerHybridAStar_getInterpo(const emlrtStack *sp, const
  plannerHybridAStar *obj, const emxArray_real_T *pathData, emxArray_real_T
  *path, emxArray_real_T *dir);
static void c_plannerHybridAStar_getPosesCi(const emlrtStack *sp, const real_T
  initialPose_data[], const real_T finalPoses_data[], const real_T ICRData_data[],
  const real_T radius_data[], real_T length, real_T stepSize, emxArray_real_T
  *poses);
static int32_T c_plannerHybridAStar_isCircular(const emlrtStack *sp,
  plannerHybridAStar *obj, const real_T initialPose[3], const real_T
  finalPoses_data[], const real_T ICRsData_data[], const real_T radius_data[],
  real_T length, real_T stepSize, real_T direction, real_T result_data[], real_T
  finalPosesGridIndices_data[], int32_T finalPosesGridIndices_size[2]);
static void c_plannerHybridAStar_validateSt(const emlrtStack *sp,
  plannerHybridAStar *obj, const real_T start[3], const real_T goal[3]);
static void c_plus(const emlrtStack *sp, emxArray_real_T *in1, const
                   emxArray_real_T *in2);
static real_T d_plannerHybridAStar_calculateC(codegenPathPlannerStackData *SD,
  const emlrtStack *sp, plannerHybridAStar *obj, const real_T newNodeData[3],
  const real_T currentNode_data[], const int32_T currentNode_size[2], real_T
  direction, real_T *gScore, real_T *hScore);
static real_T d_plannerHybridAStar_checkNodeV(const emlrtStack *sp, const
  plannerHybridAStar *obj, const real_T PointsGrid[2], real_T direction);
static int32_T d_plannerHybridAStar_get2DHeuri(codegenPathPlannerStackData *SD,
  const emlrtStack *sp, plannerHybridAStar *obj, const real_T point_data[],
  const int32_T point_size[2], real_T cost_data[]);
static int32_T f_binary_expand_op(real_T in1_data[], const real_T in2_data[],
  const int32_T *in2_size, const real_T in3_data[], const int32_T *in3_size,
  const plannerHybridAStar *in4);
static void g_binary_expand_op(real_T in1_data[], int32_T *in1_size, const
  emxArray_real_T *in2);
static void minus(const emlrtStack *sp, emxArray_real_T *in1, const
                  emxArray_real_T *in2);
static void plannerHybridAStar_closeCell(const emlrtStack *sp,
  plannerHybridAStar *obj, real_T direction, const real_T Indice_data[], const
  int32_T Indice_size[2]);
static void plus(real_T in1_data[], int32_T *in1_size, const real_T in2_data[],
                 const int32_T *in2_size);
static void times(const emlrtStack *sp, emxArray_real_T *in1, const
                  emxArray_real_T *in2);

/* Function Definitions */
static int32_T b_plus(real_T in1_data[], const real_T in2_data[], const int32_T *
                      in2_size, const real_T in3_data[], const int32_T *in3_size)
{
  int32_T i;
  int32_T in1_size;
  int32_T stride_0_0;
  int32_T stride_1_0;
  if (*in3_size == 1) {
    in1_size = *in2_size;
  } else {
    in1_size = *in3_size;
  }

  stride_0_0 = (*in2_size != 1);
  stride_1_0 = (*in3_size != 1);
  for (i = 0; i < in1_size; i++) {
    in1_data[i] = in2_data[i * stride_0_0] + in3_data[i * stride_1_0];
  }

  return in1_size;
}

static int32_T c_plannerHybridAStar_calculateC(codegenPathPlannerStackData *SD,
  const emlrtStack *sp, plannerHybridAStar *obj, const real_T newNodeData_data[],
  const int32_T newNodeData_size[2], const real_T currentNode_data[], const
  int32_T currentNode_size[2], const real_T curvature_data[], int32_T
  curvature_size, real_T direction, real_T fScore_data[], real_T gScore_data[],
  int32_T *gScore_size, real_T hScore_data[], int32_T *hScore_size)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T motionLengths_data[20];
  real_T motionTypes_data[20];
  real_T result_data[8];
  real_T pathLength_data[4];
  real_T varargin_1_data[4];
  real_T goal[3];
  int32_T fScore_size;
  int32_T i;
  int32_T m;
  int32_T maxNumPoses;
  int32_T result_size_idx_0;
  int32_T result_size_idx_1;
  int32_T varargin_1_size;
  int8_T sizes_idx_0;
  int8_T szb_idx_0;
  boolean_T allPathTypes[44];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  if (currentNode_size[1] < 2) {
    emlrtDynamicBoundsCheckR2012b(2, 1, currentNode_size[1], &fg_emlrtBCI,
      (emlrtConstCTX)sp);
  }

  st.site = &fs_emlrtRSI;
  *gScore_size = c_plannerHybridAStar_calculateG(&st, obj, currentNode_data[1],
    curvature_data, curvature_size, direction, gScore_data);
  st.site = &gs_emlrtRSI;
  b_st.site = &gs_emlrtRSI;
  varargin_1_size = d_plannerHybridAStar_get2DHeuri(SD, &b_st, obj,
    newNodeData_data, newNodeData_size, varargin_1_data);
  b_st.site = &gs_emlrtRSI;
  goal[0] = obj->GoalPose[0];
  goal[1] = obj->GoalPose[1];
  goal[2] = obj->GoalPose[2];
  c_st.site = &mn_emlrtRSI;
  maxNumPoses = (int32_T)muDoubleScalarMax(newNodeData_size[0], 1.0);
  for (i = 0; i < 44; i++) {
    allPathTypes[i] = true;
  }

  autonomousReedsSheppSegmentsCodegen_tbb_real64(&newNodeData_data[0], (uint32_T)
    newNodeData_size[0], &goal[0], 1U, obj->MinTurningRadius, obj->ForwardCost,
    obj->ReverseCost, &allPathTypes[0], 0U, 1U, true, 3U, &pathLength_data[0],
    &motionLengths_data[0], &motionTypes_data[0]);
  c_st.site = &nn_emlrtRSI;
  i = 5 * maxNumPoses;
  for (m = 0; m < i; m++) {
    motionTypes_data[m] = muDoubleScalarAbs(motionLengths_data[m]);
  }

  c_st.site = &nn_emlrtRSI;
  for (m = 0; m < maxNumPoses; m++) {
    pathLength_data[m] = sumColumnB(motionTypes_data, m + 1);
  }

  c_st.site = &ks_emlrtRSI;
  szb_idx_0 = 1;
  if (maxNumPoses != 1) {
    szb_idx_0 = (int8_T)maxNumPoses;
  }

  d_st.site = &ms_emlrtRSI;
  m = 1;
  if (maxNumPoses > 1) {
    m = maxNumPoses;
  }

  if (szb_idx_0 > muIntScalarMax_sint32(maxNumPoses, m)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &sc_emlrtRTEI,
      "Coder:toolbox:reshape_emptyReshapeLimit",
      "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }

  if (szb_idx_0 != maxNumPoses) {
    emlrtErrorWithMessageIdR2018a(&d_st, &qc_emlrtRTEI,
      "Coder:MATLAB:getReshapeDims_notSameNumel",
      "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }

  b_st.site = &es_emlrtRSI;
  if (varargin_1_size != 0) {
    sizes_idx_0 = (int8_T)varargin_1_size;
  } else {
    sizes_idx_0 = szb_idx_0;
  }

  c_st.site = &ln_emlrtRSI;
  if ((varargin_1_size != sizes_idx_0) && (varargin_1_size != 0)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &s_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if (szb_idx_0 != sizes_idx_0) {
    emlrtErrorWithMessageIdR2018a(&c_st, &s_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  szb_idx_0 = (int8_T)(varargin_1_size != 0);
  m = (varargin_1_size != 0);
  result_size_idx_0 = sizes_idx_0;
  result_size_idx_1 = szb_idx_0 + 1;
  varargin_1_size = szb_idx_0;
  for (i = 0; i < varargin_1_size; i++) {
    maxNumPoses = sizes_idx_0;
    if (maxNumPoses - 1 >= 0) {
      memcpy(&result_data[0], &varargin_1_data[0], (uint32_T)maxNumPoses *
             sizeof(real_T));
    }
  }

  varargin_1_size = sizes_idx_0;
  for (i = 0; i < varargin_1_size; i++) {
    result_data[i + sizes_idx_0 * m] = pathLength_data[i];
  }

  st.site = &gs_emlrtRSI;
  b_st.site = &ns_emlrtRSI;
  c_st.site = &os_emlrtRSI;
  m = sizes_idx_0 - 1;
  *hScore_size = sizes_idx_0;
  memcpy(&hScore_data[0], &result_data[0], (uint32_T)(m + 1) * sizeof(real_T));
  for (varargin_1_size = 2; varargin_1_size <= result_size_idx_1;
       varargin_1_size++) {
    for (maxNumPoses = 0; maxNumPoses <= m; maxNumPoses++) {
      real_T b;
      boolean_T p;
      b = result_data[maxNumPoses + sizes_idx_0];
      if (muDoubleScalarIsNaN(b)) {
        p = false;
      } else {
        real_T d;
        d = hScore_data[maxNumPoses];
        if (muDoubleScalarIsNaN(d)) {
          p = true;
        } else {
          p = (d < b);
        }
      }

      if (p) {
        hScore_data[maxNumPoses] = b;
      }
    }
  }

  if (currentNode_size[1] < 7) {
    emlrtDynamicBoundsCheckR2012b(7, 1, currentNode_size[1], &gg_emlrtBCI,
      (emlrtConstCTX)sp);
  }

  if ((currentNode_data[6] == 0.0) || (currentNode_data[6] * direction == 1.0))
  {
    if ((*gScore_size != sizes_idx_0) && ((*gScore_size != 1) && (sizes_idx_0 !=
          1))) {
      emlrtDimSizeImpxCheckR2021b(*gScore_size, sizes_idx_0, &ib_emlrtECI,
        (emlrtConstCTX)sp);
    }

    if (*gScore_size == sizes_idx_0) {
      fScore_size = *gScore_size;
      m = (*gScore_size / 2) << 1;
      varargin_1_size = m - 2;
      for (i = 0; i <= varargin_1_size; i += 2) {
        __m128d r;
        __m128d r1;
        r = _mm_loadu_pd(&gScore_data[i]);
        r1 = _mm_loadu_pd(&hScore_data[i]);
        _mm_storeu_pd(&fScore_data[i], _mm_add_pd(r, r1));
      }

      for (i = m; i < *gScore_size; i++) {
        fScore_data[i] = gScore_data[i] + hScore_data[i];
      }
    } else {
      fScore_size = b_plus(fScore_data, gScore_data, gScore_size, hScore_data,
                           &result_size_idx_0);
    }
  } else {
    if ((*gScore_size != sizes_idx_0) && ((*gScore_size != 1) && (sizes_idx_0 !=
          1))) {
      emlrtDimSizeImpxCheckR2021b(*gScore_size, sizes_idx_0, &jb_emlrtECI,
        (emlrtConstCTX)sp);
    }

    if (*gScore_size == sizes_idx_0) {
      fScore_size = *gScore_size;
      for (i = 0; i < *gScore_size; i++) {
        fScore_data[i] = (gScore_data[i] + hScore_data[i]) +
          obj->DirectionSwitchingCost;
      }
    } else {
      fScore_size = f_binary_expand_op(fScore_data, gScore_data, gScore_size,
        hScore_data, &result_size_idx_0, obj);
    }
  }

  return fScore_size;
}

static int32_T c_plannerHybridAStar_calculateG(const emlrtStack *sp, const
  plannerHybridAStar *obj, real_T parentGScore, const real_T curvature_data[],
  int32_T curvature_size, real_T direction, real_T gScore_data[])
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_real_T *b_y;
  emxArray_real_T *y;
  real_T *b_y_data;
  real_T *y_data;
  int32_T gScore_size;
  int32_T i;
  int32_T loop_ub;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &y, 1, &pj_emlrtRTEI);
  st.site = &hs_emlrtRSI;
  d_repmat(&st, parentGScore, curvature_size, y);
  y_data = y->data;
  gScore_size = y->size[0];
  loop_ub = y->size[0];
  for (i = 0; i < loop_ub; i++) {
    gScore_data[i] = y_data[i];
  }

  emxInit_real_T(sp, &b_y, 1, &oj_emlrtRTEI);
  if (direction == 1.0) {
    __m128d r;
    real_T a;
    int32_T scalarLB;
    int32_T vectorUB;
    a = obj->ForwardCost;
    st.site = &is_emlrtRSI;
    b_st.site = &mo_emlrtRSI;
    i = y->size[0];
    y->size[0] = curvature_size;
    emxEnsureCapacity_real_T(&b_st, y, i, &eh_emlrtRTEI);
    y_data = y->data;
    for (loop_ub = 0; loop_ub < curvature_size; loop_ub++) {
      y_data[loop_ub] = muDoubleScalarAbs(curvature_data[loop_ub]);
    }

    i = b_y->size[0];
    b_y->size[0] = y->size[0];
    emxEnsureCapacity_real_T(sp, b_y, i, &nj_emlrtRTEI);
    b_y_data = b_y->data;
    loop_ub = y->size[0];
    scalarLB = (y->size[0] / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&y_data[i]);
      _mm_storeu_pd(&b_y_data[i], _mm_mul_pd(_mm_set1_pd(a), _mm_add_pd
        (_mm_set1_pd(obj->MotionPrimitiveLength), r)));
    }

    for (i = scalarLB; i < loop_ub; i++) {
      b_y_data[i] = a * (obj->MotionPrimitiveLength + y_data[i]);
    }

    if ((gScore_size != b_y->size[0]) && ((gScore_size != 1) && (b_y->size[0] !=
          1))) {
      emlrtDimSizeImpxCheckR2021b(gScore_size, b_y->size[0], &kb_emlrtECI,
        (emlrtConstCTX)sp);
    }

    if (gScore_size == b_y->size[0]) {
      scalarLB = (gScore_size / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i = 0; i <= vectorUB; i += 2) {
        __m128d r1;
        r = _mm_loadu_pd(&gScore_data[i]);
        r1 = _mm_loadu_pd(&b_y_data[i]);
        _mm_storeu_pd(&gScore_data[i], _mm_add_pd(r, r1));
      }

      for (i = scalarLB; i < gScore_size; i++) {
        gScore_data[i] += b_y_data[i];
      }
    } else {
      g_binary_expand_op(gScore_data, &gScore_size, b_y);
    }
  } else if (direction == -1.0) {
    __m128d r;
    real_T a;
    int32_T scalarLB;
    int32_T vectorUB;
    a = obj->ReverseCost;
    st.site = &js_emlrtRSI;
    b_st.site = &mo_emlrtRSI;
    i = y->size[0];
    y->size[0] = curvature_size;
    emxEnsureCapacity_real_T(&b_st, y, i, &eh_emlrtRTEI);
    y_data = y->data;
    for (loop_ub = 0; loop_ub < curvature_size; loop_ub++) {
      y_data[loop_ub] = muDoubleScalarAbs(curvature_data[loop_ub]);
    }

    i = b_y->size[0];
    b_y->size[0] = y->size[0];
    emxEnsureCapacity_real_T(sp, b_y, i, &nj_emlrtRTEI);
    b_y_data = b_y->data;
    loop_ub = y->size[0];
    scalarLB = (y->size[0] / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&y_data[i]);
      _mm_storeu_pd(&b_y_data[i], _mm_mul_pd(_mm_set1_pd(a), _mm_add_pd
        (_mm_set1_pd(obj->MotionPrimitiveLength), r)));
    }

    for (i = scalarLB; i < loop_ub; i++) {
      b_y_data[i] = a * (obj->MotionPrimitiveLength + y_data[i]);
    }

    if ((gScore_size != b_y->size[0]) && ((gScore_size != 1) && (b_y->size[0] !=
          1))) {
      emlrtDimSizeImpxCheckR2021b(gScore_size, b_y->size[0], &lb_emlrtECI,
        (emlrtConstCTX)sp);
    }

    if (gScore_size == b_y->size[0]) {
      scalarLB = (gScore_size / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i = 0; i <= vectorUB; i += 2) {
        __m128d r1;
        r = _mm_loadu_pd(&gScore_data[i]);
        r1 = _mm_loadu_pd(&b_y_data[i]);
        _mm_storeu_pd(&gScore_data[i], _mm_add_pd(r, r1));
      }

      for (i = scalarLB; i < gScore_size; i++) {
        gScore_data[i] += b_y_data[i];
      }
    } else {
      g_binary_expand_op(gScore_data, &gScore_size, b_y);
    }
  }

  emxFree_real_T(sp, &y);
  emxFree_real_T(sp, &b_y);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return gScore_size;
}

static boolean_T c_plannerHybridAStar_checkAnaly(const emlrtStack *sp,
  plannerHybridAStar *obj, const real_T initialPose[3], const real_T finalPose[3],
  real_T stepSize)
{
  emlrtStack st;
  emxArray_boolean_T *r;
  emxArray_real_T *expansionPoints;
  emxArray_real_T *samples;
  real_T y[5];
  real_T delta2;
  real_T sz;
  real_T *samples_data;
  int32_T segmentDirections[5];
  int32_T i;
  int32_T k;
  uint32_T b_obj[5];
  boolean_T result;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  obj->ExpansionPoint[0] = initialPose[0];
  obj->ExpansionPoint[1] = initialPose[1];
  obj->ExpansionPoint[2] = initialPose[2];
  c_ReedsSheppBuiltins_autonomous(initialPose, finalPose, obj->MinTurningRadius,
    obj->ForwardCost, obj->ReverseCost, obj->AnalyticPathSegments,
    obj->AnalyticPathTypes);
  for (k = 0; k < 5; k++) {
    y[k] = muDoubleScalarAbs(obj->AnalyticPathSegments[k]);
  }

  obj->AnalyticPathLength = c_sumColumnB(y);
  sz = obj->AnalyticPathLength / stepSize;
  delta2 = obj->StateValidator->ValidationDistance;
  if (!muDoubleScalarIsInf(delta2)) {
    if (!(sz <= 5.0E+7)) {
      emlrtErrorWithMessageIdR2018a(sp, &sb_emlrtRTEI,
        "nav:navalgs:hybridastar:AssertionFailedLessThan",
        "nav:navalgs:hybridastar:AssertionFailedLessThan", 6, 4, 52,
        "AnalyticPathLength/StateValidator.validationDistance", 4, 61,
        "Map.GridSize(1) * Map.GridSize(1) * NumMotionPrimitives* 1000");
    }
  } else if (!(sz <= 50000.0)) {
    emlrtErrorWithMessageIdR2018a(sp, &rb_emlrtRTEI,
      "nav:navalgs:hybridastar:AssertionFailedLessThan",
      "nav:navalgs:hybridastar:AssertionFailedLessThan", 6, 4, 52,
      "AnalyticPathLength/StateValidator.validationDistance", 4, 55,
      "Map.GridSize(1) * Map.GridSize(1) * NumMotionPrimitives");
  }

  st.site = &sn_emlrtRSI;
  delta2 = obj->AnalyticPathLength;
  emxInit_real_T(&st, &samples, 2, &fg_emlrtRTEI);
  if (!(sz >= 0.0)) {
    samples->size[0] = 1;
    samples->size[1] = 0;
  } else {
    int32_T samples_tmp;
    i = (int32_T)muDoubleScalarFloor(sz);
    samples_tmp = samples->size[0] * samples->size[1];
    samples->size[0] = 1;
    samples->size[1] = i;
    emxEnsureCapacity_real_T(&st, samples, samples_tmp, &eg_emlrtRTEI);
    samples_data = samples->data;
    if (i >= 1) {
      samples_tmp = i - 1;
      samples_data[i - 1] = delta2;
      if (samples->size[1] >= 2) {
        samples_data[0] = stepSize;
        if (samples->size[1] >= 3) {
          if (stepSize == -delta2) {
            delta2 /= (real_T)samples->size[1] - 1.0;
            for (k = 2; k <= samples_tmp; k++) {
              samples_data[k - 1] = ((real_T)((k << 1) - samples->size[1]) - 1.0)
                * delta2;
            }

            if ((samples->size[1] & 1) == 1) {
              samples_data[samples->size[1] >> 1] = 0.0;
            }
          } else if (((stepSize < 0.0) != (delta2 < 0.0)) && ((muDoubleScalarAbs
                       (stepSize) > 8.9884656743115785E+307) ||
                      (muDoubleScalarAbs(delta2) > 8.9884656743115785E+307))) {
            sz = stepSize / ((real_T)samples->size[1] - 1.0);
            delta2 /= (real_T)samples->size[1] - 1.0;
            i = samples->size[1];
            for (k = 0; k <= i - 3; k++) {
              samples_data[k + 1] = (stepSize + delta2 * ((real_T)k + 1.0)) - sz
                * ((real_T)k + 1.0);
            }
          } else {
            sz = (delta2 - stepSize) / ((real_T)samples->size[1] - 1.0);
            i = samples->size[1];
            for (k = 0; k <= i - 3; k++) {
              samples_data[k + 1] = stepSize + ((real_T)k + 1.0) * sz;
            }
          }
        }
      }
    }
  }

  for (k = 0; k < 5; k++) {
    uint32_T u;
    i = 1;
    delta2 = obj->AnalyticPathSegments[k];
    if (delta2 < 0.0) {
      i = -1;
    }

    y[k] = muDoubleScalarAbs(delta2);
    segmentDirections[k] = i;
    delta2 = muDoubleScalarRound(obj->AnalyticPathTypes[k]);
    if (delta2 < 4.294967296E+9) {
      if (delta2 >= 0.0) {
        u = (uint32_T)delta2;
      } else {
        u = 0U;
      }
    } else if (delta2 >= 4.294967296E+9) {
      u = MAX_uint32_T;
    } else {
      u = 0U;
    }

    b_obj[k] = u;
  }

  emxInit_real_T(sp, &expansionPoints, 2, &gg_emlrtRTEI);
  st.site = &tn_emlrtRSI;
  d_ReedsSheppBuiltins_autonomous(&st, initialPose, finalPose, samples,
    obj->MinTurningRadius, y, segmentDirections, b_obj, expansionPoints);
  emxFree_real_T(sp, &samples);
  if (expansionPoints->size[0] == 0) {
    result = true;
  } else {
    emxInit_boolean_T(sp, &r, 1, &hg_emlrtRTEI);
    st.site = &un_emlrtRSI;
    d_validatorOccupancyMap_isState(&st, obj->StateValidator, expansionPoints, r);
    result = c_all(r);
    emxFree_boolean_T(sp, &r);
  }

  emxFree_real_T(sp, &expansionPoints);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return result;
}

static int32_T c_plannerHybridAStar_checkNodeV(const emlrtStack *sp, const
  plannerHybridAStar *obj, const real_T PointsGrid_data[], real_T direction,
  real_T nodeValidity_data[])
{
  __m128d r;
  __m128d r1;
  real_T finalPointsGridIndices_data[4];
  int32_T i;
  int32_T nodeValidity_size;
  r = _mm_set1_pd(1.0);
  r1 = _mm_set1_pd(obj->Dimensions[0]);
  _mm_storeu_pd(&finalPointsGridIndices_data[0], _mm_add_pd(_mm_mul_pd
    (_mm_sub_pd(_mm_loadu_pd(&PointsGrid_data[4]), r), r1), _mm_loadu_pd
    (&PointsGrid_data[0])));
  _mm_storeu_pd(&finalPointsGridIndices_data[2], _mm_add_pd(_mm_mul_pd
    (_mm_sub_pd(_mm_loadu_pd(&PointsGrid_data[6]), r), r1), _mm_loadu_pd
    (&PointsGrid_data[2])));
  nodeValidity_size = 4;
  for (i = 0; i < 4; i++) {
    real_T d;
    d = PointsGrid_data[i];
    if (d < 1.0) {
      nodeValidity_data[i] = 0.0;
    } else {
      real_T d1;
      d1 = PointsGrid_data[i + 4];
      if ((d1 < 1.0) || (d > obj->Dimensions[0]) || (d1 > obj->Dimensions[1])) {
        nodeValidity_data[i] = 0.0;
      } else if (direction == 1.0) {
        d = finalPointsGridIndices_data[i];
        if (d != (int32_T)muDoubleScalarFloor(d)) {
          emlrtIntegerCheckR2012b(d, &hd_emlrtDCI, (emlrtConstCTX)sp);
        }

        if (((int32_T)d < 1) || ((int32_T)d > 10000)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 10000, &eg_emlrtBCI,
            (emlrtConstCTX)sp);
        }

        nodeValidity_data[i] = !obj->visitedCellsFront[(int32_T)d - 1];
      } else {
        d = finalPointsGridIndices_data[i];
        if (d != (int32_T)muDoubleScalarFloor(d)) {
          emlrtIntegerCheckR2012b(d, &gd_emlrtDCI, (emlrtConstCTX)sp);
        }

        if (((int32_T)d < 1) || ((int32_T)d > 10000)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 10000, &dg_emlrtBCI,
            (emlrtConstCTX)sp);
        }

        nodeValidity_data[i] = !obj->visitedCellsBack[(int32_T)d - 1];
      }
    }
  }

  return nodeValidity_size;
}

static real_T c_plannerHybridAStar_get2DHeuri(codegenPathPlannerStackData *SD,
  const emlrtStack *sp, plannerHybridAStar *obj, const real_T point[2])
{
  binaryOccupancyMap *b_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T matPoint_data[18];
  real_T neighborPoints[18];
  real_T b_tmp_data[9];
  real_T neighborCosts_data[9];
  real_T matPoint[2];
  real_T indices;
  real_T minCost;
  int32_T tmp_size[2];
  int32_T b_i;
  int32_T i;
  int32_T idx;
  int32_T trueCount;
  int32_T vectorUB;
  int8_T tmp_data[9];
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ui_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  b_obj = obj->Map;
  b_st.site = &sh_emlrtRSI;
  MapInterface_world2gridImpl(b_obj, point, matPoint);
  indices = matPoint[0] + 100.0 * (matPoint[1] - 1.0);
  if (indices != (int32_T)muDoubleScalarFloor(indices)) {
    emlrtIntegerCheckR2012b(indices, &h_emlrtDCI, (emlrtConstCTX)sp);
  }

  if (((int32_T)indices < 1) || ((int32_T)indices > 10000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)indices, 1, 10000, &v_emlrtBCI,
      (emlrtConstCTX)sp);
  }

  minCost = obj->Heuristic2DMat[(int32_T)indices - 1];
  if (muDoubleScalarIsInf(minCost)) {
    __m128d r;
    real_T d;
    for (i = 0; i < 2; i++) {
      for (vectorUB = 0; vectorUB < 9; vectorUB++) {
        idx = vectorUB + 9 * i;
        neighborPoints[idx] = matPoint[i] + obj->Neighbors[idx];
      }
    }

    trueCount = 0;
    idx = 0;
    for (b_i = 0; b_i < 9; b_i++) {
      d = neighborPoints[b_i + 9];
      minCost = neighborPoints[b_i];
      if ((minCost > 0.0) && (d > 0.0) && (minCost <= 100.0) && (d <= 100.0)) {
        trueCount++;
        tmp_data[idx] = (int8_T)b_i;
        idx++;
      }
    }

    for (i = 0; i < trueCount; i++) {
      int8_T i1;
      i1 = tmp_data[i];
      d = neighborPoints[i1] + 100.0 * (neighborPoints[i1 + 9] - 1.0);
      if (d != (int32_T)muDoubleScalarFloor(d)) {
        emlrtIntegerCheckR2012b(d, &i_emlrtDCI, (emlrtConstCTX)sp);
      }

      if (((int32_T)d < 1) || ((int32_T)d > 10000)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 10000, &w_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      neighborCosts_data[i] = obj->Heuristic2DMat[(int32_T)d - 1];
    }

    for (i = 0; i < 2; i++) {
      for (vectorUB = 0; vectorUB < trueCount; vectorUB++) {
        matPoint_data[vectorUB + trueCount * i] = matPoint[i] -
          neighborPoints[tmp_data[vectorUB] + 9 * i];
      }
    }

    tmp_size[0] = trueCount;
    tmp_size[1] = 2;
    idx = trueCount << 1;
    b_i = idx / 2 * 2;
    vectorUB = b_i - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&matPoint_data[i]);
      r = _mm_mul_pd(r, r);
      _mm_storeu_pd(&neighborPoints[i], r);
    }

    for (i = b_i; i < idx; i++) {
      minCost = matPoint_data[i];
      neighborPoints[i] = minCost * minCost;
    }

    st.site = &vi_emlrtRSI;
    idx = sum(neighborPoints, tmp_size, b_tmp_data);
    st.site = &vi_emlrtRSI;
    b_sqrt(&st, b_tmp_data, &idx);
    if ((trueCount != idx) && ((trueCount != 1) && (idx != 1))) {
      emlrtDimSizeImpxCheckR2021b(trueCount, idx, &j_emlrtECI, (emlrtConstCTX)sp);
    }

    if (trueCount == idx) {
      b_i = (trueCount / 2) << 1;
      vectorUB = b_i - 2;
      for (i = 0; i <= vectorUB; i += 2) {
        __m128d r1;
        r = _mm_loadu_pd(&neighborCosts_data[i]);
        r1 = _mm_loadu_pd(&b_tmp_data[i]);
        _mm_storeu_pd(&neighborCosts_data[i], _mm_add_pd(r, r1));
      }

      for (i = b_i; i < trueCount; i++) {
        neighborCosts_data[i] += b_tmp_data[i];
      }
    } else {
      plus(neighborCosts_data, &trueCount, b_tmp_data, &idx);
    }

    st.site = &wi_emlrtRSI;
    b_st.site = &hj_emlrtRSI;
    c_st.site = &ij_emlrtRSI;
    d_st.site = &jj_emlrtRSI;
    if (trueCount < 1) {
      emlrtErrorWithMessageIdR2018a(&d_st, &cb_emlrtRTEI,
        "Coder:toolbox:eml_min_or_max_varDimZero",
        "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }

    if (trueCount <= 2) {
      if (trueCount == 1) {
        minCost = neighborCosts_data[0];
      } else if ((neighborCosts_data[0] > neighborCosts_data[1]) ||
                 (muDoubleScalarIsNaN(neighborCosts_data[0]) &&
                  (!muDoubleScalarIsNaN(neighborCosts_data[1])))) {
        minCost = neighborCosts_data[1];
      } else {
        minCost = neighborCosts_data[0];
      }
    } else {
      if (!muDoubleScalarIsNaN(neighborCosts_data[0])) {
        idx = 1;
      } else {
        boolean_T exitg1;
        idx = 0;
        vectorUB = 2;
        exitg1 = false;
        while ((!exitg1) && (vectorUB <= trueCount)) {
          if (!muDoubleScalarIsNaN(neighborCosts_data[vectorUB - 1])) {
            idx = vectorUB;
            exitg1 = true;
          } else {
            vectorUB++;
          }
        }
      }

      if (idx == 0) {
        minCost = neighborCosts_data[0];
      } else {
        minCost = neighborCosts_data[idx - 1];
        i = idx + 1;
        for (vectorUB = i; vectorUB <= trueCount; vectorUB++) {
          d = neighborCosts_data[vectorUB - 1];
          if (minCost > d) {
            minCost = d;
          }
        }
      }
    }

    if (!muDoubleScalarIsInf(minCost)) {
      obj->Heuristic2DMat[(int32_T)indices - 1] = minCost;
    } else {
      st.site = &xi_emlrtRSI;
      plannerAStarGrid_plan(SD, &st, obj->Heuristic2DObj, obj->GoalPoint,
                            matPoint);
      for (i = 0; i < 10000; i++) {
        SD->u5.f8.varargin_1[i] = obj->Heuristic2DMat[i];
      }

      for (i = 0; i < 10000; i++) {
        SD->u5.f8.varargin_2[i] = obj->Heuristic2DObj->GCostMatrix[i];
      }

      for (vectorUB = 0; vectorUB < 10000; vectorUB++) {
        SD->u5.f8.minval[vectorUB] = muDoubleScalarMin(SD->
          u5.f8.varargin_1[vectorUB], SD->u5.f8.varargin_2[vectorUB]);
      }

      for (i = 0; i < 10000; i++) {
        obj->Heuristic2DMat[i] = SD->u5.f8.minval[i];
      }
    }
  }

  return obj->Heuristic2DMat[(int32_T)indices - 1];
}

static real_T c_plannerHybridAStar_get3DHeuri(const plannerHybridAStar *obj,
  const real_T start[3], const real_T goal[3])
{
  real_T motionLengths[5];
  real_T motionTypes[5];
  real_T cost;
  int32_T k;
  boolean_T allPathTypes[44];
  for (k = 0; k < 44; k++) {
    allPathTypes[k] = true;
  }

  autonomousReedsSheppSegmentsCodegen_tbb_real64(&start[0], 1U, &goal[0], 1U,
    obj->MinTurningRadius, obj->ForwardCost, obj->ReverseCost, &allPathTypes[0],
    0U, 1U, true, 3U, &cost, &motionLengths[0], &motionTypes[0]);
  for (k = 0; k < 5; k++) {
    motionTypes[k] = muDoubleScalarAbs(motionLengths[k]);
  }

  return c_sumColumnB(motionTypes);
}

static void c_plannerHybridAStar_getCircula(real_T length, const real_T
  curvature_data[], const real_T initialNodePose[3], real_T direction, real_T
  newNodesPoses_data[], int32_T newNodesPoses_size[2], real_T ICRsData_data[],
  int32_T ICRsData_size[2])
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  __m128d r3;
  __m128d r4;
  __m128d r5;
  __m128d r6;
  __m128d r7;
  real_T b_tmp_data[4];
  real_T centerX_data[4];
  real_T centerY_data[4];
  real_T tmp_data[4];
  real_T turningAngle_data[4];
  real_T turningRadius_data[4];
  r = _mm_loadu_pd(&curvature_data[0]);
  r2 = _mm_set1_pd(1.0);
  r1 = _mm_div_pd(r2, r);
  _mm_storeu_pd(&turningRadius_data[0], r1);
  r3 = _mm_set1_pd(length);
  _mm_storeu_pd(&turningAngle_data[0], _mm_mul_pd(r3, r));
  r4 = _mm_set1_pd(initialNodePose[0]);
  r5 = _mm_set1_pd(muDoubleScalarSin(initialNodePose[2]));
  _mm_storeu_pd(&centerX_data[0], _mm_sub_pd(r4, _mm_mul_pd(r1, r5)));
  r6 = _mm_set1_pd(initialNodePose[1]);
  r7 = _mm_set1_pd(muDoubleScalarCos(initialNodePose[2]));
  _mm_storeu_pd(&centerY_data[0], _mm_add_pd(r6, _mm_mul_pd(r1, r7)));
  r = _mm_loadu_pd(&curvature_data[2]);
  r1 = _mm_div_pd(r2, r);
  _mm_storeu_pd(&turningRadius_data[2], r1);
  _mm_storeu_pd(&turningAngle_data[2], _mm_mul_pd(r3, r));
  _mm_storeu_pd(&centerX_data[2], _mm_sub_pd(r4, _mm_mul_pd(r1, r5)));
  _mm_storeu_pd(&centerY_data[2], _mm_add_pd(r6, _mm_mul_pd(r1, r7)));
  r = _mm_loadu_pd(&turningAngle_data[0]);
  r1 = _mm_set1_pd(initialNodePose[2]);
  r2 = _mm_set1_pd(direction);
  _mm_storeu_pd(&tmp_data[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
  r = _mm_loadu_pd(&turningAngle_data[2]);
  _mm_storeu_pd(&tmp_data[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
  tmp_data[0] = muDoubleScalarSin(tmp_data[0]);
  tmp_data[1] = muDoubleScalarSin(tmp_data[1]);
  tmp_data[2] = muDoubleScalarSin(tmp_data[2]);
  tmp_data[3] = muDoubleScalarSin(tmp_data[3]);
  r = _mm_loadu_pd(&turningAngle_data[0]);
  _mm_storeu_pd(&b_tmp_data[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
  r = _mm_loadu_pd(&turningAngle_data[2]);
  _mm_storeu_pd(&b_tmp_data[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
  b_tmp_data[0] = muDoubleScalarCos(b_tmp_data[0]);
  b_tmp_data[1] = muDoubleScalarCos(b_tmp_data[1]);
  b_tmp_data[2] = muDoubleScalarCos(b_tmp_data[2]);
  b_tmp_data[3] = muDoubleScalarCos(b_tmp_data[3]);
  newNodesPoses_size[0] = 4;
  newNodesPoses_size[1] = 3;
  r = _mm_loadu_pd(&turningRadius_data[0]);
  r3 = _mm_loadu_pd(&tmp_data[0]);
  r4 = _mm_loadu_pd(&centerX_data[0]);
  _mm_storeu_pd(&newNodesPoses_data[0], _mm_add_pd(r4, _mm_mul_pd(r, r3)));
  r3 = _mm_loadu_pd(&b_tmp_data[0]);
  r4 = _mm_loadu_pd(&centerY_data[0]);
  _mm_storeu_pd(&newNodesPoses_data[4], _mm_sub_pd(r4, _mm_mul_pd(r, r3)));
  r = _mm_loadu_pd(&turningAngle_data[0]);
  _mm_storeu_pd(&newNodesPoses_data[8], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
  r = _mm_loadu_pd(&turningRadius_data[2]);
  r3 = _mm_loadu_pd(&tmp_data[2]);
  r4 = _mm_loadu_pd(&centerX_data[2]);
  _mm_storeu_pd(&newNodesPoses_data[2], _mm_add_pd(r4, _mm_mul_pd(r, r3)));
  r3 = _mm_loadu_pd(&b_tmp_data[2]);
  r4 = _mm_loadu_pd(&centerY_data[2]);
  _mm_storeu_pd(&newNodesPoses_data[6], _mm_sub_pd(r4, _mm_mul_pd(r, r3)));
  r = _mm_loadu_pd(&turningAngle_data[2]);
  _mm_storeu_pd(&newNodesPoses_data[10], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
  ICRsData_size[0] = 4;
  ICRsData_size[1] = 3;
  ICRsData_data[0] = centerX_data[0];
  ICRsData_data[4] = centerY_data[0];
  ICRsData_data[8] = turningAngle_data[0];
  ICRsData_data[1] = centerX_data[1];
  ICRsData_data[5] = centerY_data[1];
  ICRsData_data[9] = turningAngle_data[1];
  ICRsData_data[2] = centerX_data[2];
  ICRsData_data[6] = centerY_data[2];
  ICRsData_data[10] = turningAngle_data[2];
  ICRsData_data[3] = centerX_data[3];
  ICRsData_data[7] = centerY_data[3];
  ICRsData_data[11] = turningAngle_data[3];
}

static void c_plannerHybridAStar_getFinalPa(codegenPathPlannerStackData *SD,
  const emlrtStack *sp, const plannerHybridAStar *obj, const emxArray_real_T
  *pathData, emxArray_real_T *finalPathData)
{
  emxArray_uint16_T *r;
  emxArray_uint16_T *r1;
  const real_T *pathData_data;
  real_T *finalPathData_data;
  int32_T PathDataRow;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  int32_T k;
  uint16_T *r2;
  boolean_T x[2];
  boolean_T exitg1;
  boolean_T y;
  pathData_data = pathData->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  PathDataRow = 1;
  memset(&SD->u1.f0.finalPathData[0], 0, 550000U * sizeof(real_T));
  x[0] = (pathData->size[0] <= 50000);
  x[1] = true;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (!y) {
    emlrtErrorWithMessageIdR2018a(sp, &dc_emlrtRTEI,
      "nav:navalgs:hybridastar:AssertionFailedLessThan",
      "nav:navalgs:hybridastar:AssertionFailedLessThan", 6, 4, 30,
      "Number of states in final path", 4, 55,
      "Map.GridSize(1) * Map.GridSize(1) * NumMotionPrimitives");
  }

  i = pathData->size[0];
  emxInit_uint16_T(sp, &r, &ni_emlrtRTEI);
  emxInit_uint16_T(sp, &r1, &oi_emlrtRTEI);
  for (b_i = 0; b_i <= i - 2; b_i++) {
    if (pathData->size[1] < 3) {
      emlrtDynamicBoundsCheckR2012b(3, 1, pathData->size[1], &ke_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    if (b_i + 1 > pathData->size[0]) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, pathData->size[0], &if_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    if (b_i + 2 > pathData->size[0]) {
      emlrtDynamicBoundsCheckR2012b(b_i + 2, 1, pathData->size[0], &jf_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    if (pathData_data[b_i + pathData->size[0] * 2] != pathData_data[(b_i +
         pathData->size[0] * 2) + 1]) {
      real_T b_pathData;
      real_T c_pathData;
      real_T d_pathData;
      real_T e_pathData;
      boolean_T bv[50000];
      if (pathData->size[1] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, pathData->size[1], &me_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (b_i + 1 > pathData->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, pathData->size[0],
          &le_emlrtBCI, (emlrtConstCTX)sp);
      }

      if (pathData->size[1] < 2) {
        emlrtDynamicBoundsCheckR2012b(2, 1, pathData->size[1], &oe_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (b_i + 1 > pathData->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, pathData->size[0],
          &ne_emlrtBCI, (emlrtConstCTX)sp);
      }

      if (pathData->size[1] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, pathData->size[1], &qe_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (b_i + 2 > pathData->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 2, 1, pathData->size[0],
          &pe_emlrtBCI, (emlrtConstCTX)sp);
      }

      if (pathData->size[1] < 2) {
        emlrtDynamicBoundsCheckR2012b(2, 1, pathData->size[1], &se_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (b_i + 2 > pathData->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 2, 1, pathData->size[0],
          &re_emlrtBCI, (emlrtConstCTX)sp);
      }

      b_pathData = pathData_data[b_i];
      c_pathData = pathData_data[b_i + pathData->size[0]];
      d_pathData = pathData_data[b_i + 1];
      e_pathData = pathData_data[(b_i + pathData->size[0]) + 1];
      k = 0;
      for (c_i = 0; c_i < 50000; c_i++) {
        y = ((b_pathData == obj->PrimitivesData[c_i]) && (c_pathData ==
              obj->PrimitivesData[c_i + 50000]) && (d_pathData ==
              obj->PrimitivesData[c_i + 150000]) && (e_pathData ==
              obj->PrimitivesData[c_i + 200000]));
        bv[c_i] = y;
        if (y) {
          k++;
        }
      }

      c_i = r->size[0];
      r->size[0] = k;
      emxEnsureCapacity_uint16_T(sp, r, c_i, &mi_emlrtRTEI);
      r2 = r->data;
      k = 0;
      for (c_i = 0; c_i < 50000; c_i++) {
        if (bv[c_i]) {
          r2[k] = (uint16_T)c_i;
          k++;
        }
      }

      if (r->size[0] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, r->size[0], &te_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (PathDataRow > 50000) {
        emlrtDynamicBoundsCheckR2012b(50001, 1, 50000, &ff_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      for (c_i = 0; c_i < 11; c_i++) {
        SD->u1.f0.finalPathData[(PathDataRow + 50000 * c_i) - 1] =
          obj->PrimitivesData[r2[0] + 50000 * c_i];
      }
    } else {
      real_T b_pathData;
      real_T c_pathData;
      real_T d_pathData;
      real_T e_pathData;
      boolean_T bv[50000];
      if (pathData->size[1] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, pathData->size[1], &ve_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (b_i + 1 > pathData->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, pathData->size[0],
          &ue_emlrtBCI, (emlrtConstCTX)sp);
      }

      if (pathData->size[1] < 2) {
        emlrtDynamicBoundsCheckR2012b(2, 1, pathData->size[1], &xe_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (b_i + 1 > pathData->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, pathData->size[0],
          &we_emlrtBCI, (emlrtConstCTX)sp);
      }

      if (pathData->size[1] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, pathData->size[1], &af_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (b_i + 2 > pathData->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 2, 1, pathData->size[0],
          &ye_emlrtBCI, (emlrtConstCTX)sp);
      }

      if (pathData->size[1] < 2) {
        emlrtDynamicBoundsCheckR2012b(2, 1, pathData->size[1], &cf_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (b_i + 2 > pathData->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 2, 1, pathData->size[0],
          &bf_emlrtBCI, (emlrtConstCTX)sp);
      }

      b_pathData = pathData_data[b_i];
      c_pathData = pathData_data[b_i + pathData->size[0]];
      d_pathData = pathData_data[b_i + 1];
      e_pathData = pathData_data[(b_i + pathData->size[0]) + 1];
      k = 0;
      for (c_i = 0; c_i < 50000; c_i++) {
        y = ((b_pathData == obj->LinesData[c_i]) && (c_pathData ==
              obj->LinesData[c_i + 50000]) && (d_pathData == obj->LinesData[c_i
              + 150000]) && (e_pathData == obj->LinesData[c_i + 200000]));
        bv[c_i] = y;
        if (y) {
          k++;
        }
      }

      c_i = r1->size[0];
      r1->size[0] = k;
      emxEnsureCapacity_uint16_T(sp, r1, c_i, &mi_emlrtRTEI);
      r2 = r1->data;
      k = 0;
      for (c_i = 0; c_i < 50000; c_i++) {
        if (bv[c_i]) {
          r2[k] = (uint16_T)c_i;
          k++;
        }
      }

      if (r1->size[0] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, r1->size[0], &df_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (r1->size[0] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, r1->size[0], &ef_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (PathDataRow > 50000) {
        emlrtDynamicBoundsCheckR2012b(50001, 1, 50000, &gf_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      for (c_i = 0; c_i < 6; c_i++) {
        SD->u1.f0.finalPathData[(PathDataRow + 50000 * c_i) - 1] =
          obj->LinesData[r2[0] + 50000 * c_i];
      }

      SD->u1.f0.finalPathData[PathDataRow + 299999] = rtNaN;
      SD->u1.f0.finalPathData[PathDataRow + 349999] = rtNaN;
      SD->u1.f0.finalPathData[PathDataRow + 399999] = rtNaN;
      SD->u1.f0.finalPathData[PathDataRow + 449999] = rtInf;
      SD->u1.f0.finalPathData[PathDataRow + 499999] = obj->LinesData[r2[0] +
        300000];
    }

    PathDataRow++;
  }

  emxFree_uint16_T(sp, &r1);
  emxFree_uint16_T(sp, &r);
  if (PathDataRow - 1 < 1) {
    k = 0;
  } else {
    if (PathDataRow - 1 < 1) {
      emlrtDynamicBoundsCheckR2012b(0, 1, 50000, &hf_emlrtBCI, (emlrtConstCTX)sp);
    }

    k = PathDataRow - 1;
  }

  i = finalPathData->size[0] * finalPathData->size[1];
  finalPathData->size[0] = k;
  finalPathData->size[1] = 11;
  emxEnsureCapacity_real_T(sp, finalPathData, i, &li_emlrtRTEI);
  finalPathData_data = finalPathData->data;
  for (i = 0; i < 11; i++) {
    for (c_i = 0; c_i < k; c_i++) {
      finalPathData_data[c_i + finalPathData->size[0] * i] =
        SD->u1.f0.finalPathData[c_i + 50000 * i];
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void c_plannerHybridAStar_getInterpo(const emlrtStack *sp, const
  plannerHybridAStar *obj, const emxArray_real_T *pathData, emxArray_real_T
  *path, emxArray_real_T *dir)
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_real_T *b_pathData;
  emxArray_real_T *b_samples;
  emxArray_real_T *c_obj;
  emxArray_real_T *directions;
  emxArray_real_T *r;
  emxArray_real_T *samples;
  emxArray_real_T *states;
  real_T b_y[5];
  real_T switchingDistance[5];
  real_T getSwitchingMotion_data[4];
  const real_T *pathData_data;
  real_T primitivePathLength;
  real_T val;
  real_T *dir_data;
  real_T *directions_data;
  real_T *path_data;
  real_T *samples_data;
  real_T *states_data;
  int32_T b_switchingDistance[5];
  int32_T i;
  int32_T idx;
  int32_T j;
  int32_T nrows;
  uint32_T b_obj[5];
  int8_T tmp_data[4];
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  pathData_data = pathData->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  primitivePathLength = obj->MotionPrimitiveLength * (real_T)pathData->size[0];
  val = muDoubleScalarFloor(primitivePathLength);
  if (!(val <= 50000.0)) {
    emlrtErrorWithMessageIdR2018a(sp, &mc_emlrtRTEI,
      "nav:navalgs:hybridastar:AssertionFailedLessThan",
      "nav:navalgs:hybridastar:AssertionFailedLessThan", 6, 4, 41,
      "primitivePathLength/InterpolationDistance", 4, 79,
      "Map.GridSize(1) * Map.GridSize(1) * NumMotionPrimitives / InterpolationDistance");
  }

  emxInit_real_T(sp, &states, 2, &pi_emlrtRTEI);
  if ((int32_T)val < 0) {
    emlrtNonNegativeCheckR2012b((int32_T)val, &wc_emlrtDCI, (emlrtConstCTX)sp);
  }

  i = states->size[0] * states->size[1];
  states->size[0] = (int32_T)val;
  states->size[1] = 3;
  emxEnsureCapacity_real_T(sp, states, i, &pi_emlrtRTEI);
  states_data = states->data;
  nrows = (int32_T)val * 3;
  for (i = 0; i < nrows; i++) {
    states_data[i] = 0.0;
  }

  emxInit_real_T(sp, &directions, 1, &qi_emlrtRTEI);
  nrows = (int32_T)val;
  i = directions->size[0];
  directions->size[0] = (int32_T)val;
  emxEnsureCapacity_real_T(sp, directions, i, &qi_emlrtRTEI);
  directions_data = directions->data;
  for (i = 0; i < nrows; i++) {
    directions_data[i] = 0.0;
  }

  emlrtForLoopVectorCheckR2021a(1.0, 1.0, primitivePathLength, mxDOUBLE_CLASS,
    (int32_T)primitivePathLength, &lc_emlrtRTEI, (emlrtConstCTX)sp);
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i <= (int32_T)primitivePathLength - 1)) {
    real_T lengthOnPrimitive;
    real_T y;
    y = trunc(((real_T)i + 1.0) / obj->MotionPrimitiveLength);
    lengthOnPrimitive = muDoubleScalarRem((real_T)i + 1.0,
      obj->MotionPrimitiveLength);
    if ((real_T)i + 1.0 >= primitivePathLength) {
      st.site = &lp_emlrtRSI;
      idx = states->size[0];
      b_st.site = &bq_emlrtRSI;
      if (states->size[0] < 1) {
        emlrtErrorWithMessageIdR2018a(&b_st, &hc_emlrtRTEI,
          "MATLAB:subsdeldimmismatch", "MATLAB:subsdeldimmismatch", 0);
      }

      b_st.site = &cq_emlrtRSI;
      nrows = states->size[0] - 1;
      for (j = 0; j < 3; j++) {
        for (i = idx; i <= nrows; i++) {
          states_data[(i + states->size[0] * j) - 1] = states_data[i +
            states->size[0] * j];
        }
      }

      if (states->size[0] - 1 > states->size[0]) {
        emlrtErrorWithMessageIdR2018a(&b_st, &gc_emlrtRTEI,
          "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
      }

      if (states->size[0] - 1 < 1) {
        nrows = 0;
      } else {
        nrows = states->size[0] - 1;
      }

      for (i = 0; i < 3; i++) {
        for (j = 0; j < nrows; j++) {
          states_data[j + nrows * i] = states_data[j + states->size[0] * i];
        }
      }

      i = states->size[0] * states->size[1];
      states->size[0] = nrows;
      states->size[1] = 3;
      emxEnsureCapacity_real_T(&b_st, states, i, &ri_emlrtRTEI);
      states_data = states->data;
      st.site = &mp_emlrtRSI;
      idx = directions->size[0];
      b_st.site = &eq_emlrtRSI;
      if (directions->size[0] < 1) {
        emlrtErrorWithMessageIdR2018a(&b_st, &fc_emlrtRTEI,
          "MATLAB:subsdeldimmismatch", "MATLAB:subsdeldimmismatch", 0);
      }

      b_st.site = &fq_emlrtRSI;
      nrows = directions->size[0] - 1;
      for (j = idx; j <= nrows; j++) {
        directions_data[j - 1] = directions_data[j];
      }

      if (directions->size[0] - 1 > directions->size[0]) {
        emlrtErrorWithMessageIdR2018a(&b_st, &ec_emlrtRTEI,
          "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
      }

      i = directions->size[0];
      if (directions->size[0] - 1 < 1) {
        directions->size[0] = 0;
      } else {
        directions->size[0]--;
      }

      emxEnsureCapacity_real_T(&b_st, directions, i, &ti_emlrtRTEI);
      directions_data = directions->data;
      exitg1 = true;
    } else {
      if (y + 1.0 != (int32_T)(y + 1.0)) {
        emlrtIntegerCheckR2012b(y + 1.0, &vc_emlrtDCI, (emlrtConstCTX)sp);
      }

      if (((int32_T)(y + 1.0) < 1) || ((int32_T)(y + 1.0) > pathData->size[0]))
      {
        emlrtDynamicBoundsCheckR2012b((int32_T)(y + 1.0), 1, pathData->size[0],
          &mf_emlrtBCI, (emlrtConstCTX)sp);
      }

      val = pathData_data[((int32_T)(y + 1.0) + pathData->size[0] * 9) - 1];
      if (muDoubleScalarIsInf(val)) {
        real_T direction_tmp;
        real_T states_tmp;
        p = (((int32_T)(y + 1.0) < 1) || ((int32_T)(y + 1.0) > pathData->size[0]));
        if (p) {
          emlrtDynamicBoundsCheckR2012b((int32_T)(y + 1.0), 1, pathData->size[0],
            &lf_emlrtBCI, (emlrtConstCTX)sp);
        }

        if (((int32_T)(y + 1.0) < 1) || ((int32_T)(y + 1.0) > pathData->size[0]))
        {
          emlrtDynamicBoundsCheckR2012b((int32_T)(y + 1.0), 1, pathData->size[0],
            &qf_emlrtBCI, (emlrtConstCTX)sp);
        }

        direction_tmp = pathData_data[((int32_T)(y + 1.0) + pathData->size[0] *
          10) - 1];
        if ((i + 1 < 1) || (i + 1 > states->size[0])) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, states->size[0], &nf_emlrtBCI,
            (emlrtConstCTX)sp);
        }

        states_tmp = direction_tmp * lengthOnPrimitive;
        val = pathData_data[((int32_T)(y + 1.0) + pathData->size[0] * 2) - 1];
        states_data[i] = pathData_data[(int32_T)(y + 1.0) - 1] + states_tmp *
          muDoubleScalarCos(val);
        states_data[i + states->size[0]] = pathData_data[((int32_T)(y + 1.0) +
          pathData->size[0]) - 1] + states_tmp * muDoubleScalarSin(val);
        states_data[i + states->size[0] * 2] = val;
        if ((i + 1 < 1) || (i + 1 > directions->size[0])) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, directions->size[0],
            &sf_emlrtBCI, (emlrtConstCTX)sp);
        }

        directions_data[i] = direction_tmp;
      } else {
        real_T direction_tmp;
        real_T states_tmp;
        real_T turningRadius;
        p = (((int32_T)(y + 1.0) < 1) || ((int32_T)(y + 1.0) > pathData->size[0]));
        if (p) {
          emlrtDynamicBoundsCheckR2012b((int32_T)(y + 1.0), 1, pathData->size[0],
            &pf_emlrtBCI, (emlrtConstCTX)sp);
        }

        val = 1.0 / val;
        if (((int32_T)(y + 1.0) < 1) || ((int32_T)(y + 1.0) > pathData->size[0]))
        {
          emlrtDynamicBoundsCheckR2012b((int32_T)(y + 1.0), 1, pathData->size[0],
            &rf_emlrtBCI, (emlrtConstCTX)sp);
        }

        direction_tmp = pathData_data[((int32_T)(y + 1.0) + pathData->size[0] *
          10) - 1];
        turningRadius = 1.0 / val;
        if ((i + 1 < 1) || (i + 1 > states->size[0])) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, states->size[0], &of_emlrtBCI,
            (emlrtConstCTX)sp);
        }

        states_tmp = pathData_data[((int32_T)(y + 1.0) + pathData->size[0] * 2)
          - 1];
        val = states_tmp + direction_tmp * (lengthOnPrimitive * val);
        states_data[i] = (pathData_data[(int32_T)(y + 1.0) - 1] - turningRadius *
                          muDoubleScalarSin(states_tmp)) + turningRadius *
          muDoubleScalarSin(val);
        states_data[i + states->size[0]] = (pathData_data[((int32_T)(y + 1.0) +
          pathData->size[0]) - 1] + turningRadius * muDoubleScalarCos(states_tmp))
          - turningRadius * muDoubleScalarCos(val);
        states_data[i + states->size[0] * 2] = val;
        if ((i + 1 < 1) || (i + 1 > directions->size[0])) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, directions->size[0],
            &tf_emlrtBCI, (emlrtConstCTX)sp);
        }

        directions_data[i] = direction_tmp;
      }

      i++;
    }
  }

  val = obj->AnalyticPathLength;
  if (!(val <= 50000.0)) {
    emlrtErrorWithMessageIdR2018a(sp, &kc_emlrtRTEI,
      "nav:navalgs:hybridastar:AssertionFailedLessThan",
      "nav:navalgs:hybridastar:AssertionFailedLessThan", 6, 4, 37,
      "Number of States in interpolated Path", 4, 55,
      "Map.GridSize(1) * Map.GridSize(1) * NumMotionPrimitives");
  }

  emxInit_real_T(sp, &samples, 2, &aj_emlrtRTEI);
  samples_data = samples->data;
  st.site = &hp_emlrtRSI;
  if (!(val >= 0.0)) {
    samples->size[0] = 1;
    samples->size[1] = 0;
  } else {
    i = samples->size[0] * samples->size[1];
    samples->size[0] = 1;
    samples->size[1] = (int32_T)muDoubleScalarFloor(val);
    emxEnsureCapacity_real_T(&st, samples, i, &eg_emlrtRTEI);
    samples_data = samples->data;
    if (samples->size[1] >= 1) {
      samples_data[samples->size[1] - 1] = val;
      if (samples->size[1] >= 2) {
        samples_data[0] = 1.0;
        if (samples->size[1] >= 3) {
          val = (val - 1.0) / ((real_T)samples->size[1] - 1.0);
          i = samples->size[1];
          for (j = 0; j <= i - 3; j++) {
            samples_data[j + 1] = ((real_T)j + 1.0) * val + 1.0;
          }
        }
      }
    }
  }

  if (samples->size[1] != 0) {
    st.site = &ip_emlrtRSI;
    b_st.site = &ip_emlrtRSI;
    nrows = nonzeros(&b_st, obj->AnalyticPathSegments, switchingDistance);
    for (j = 0; j < nrows; j++) {
      switchingDistance[j] = muDoubleScalarSign(switchingDistance[j]);
    }

    st.site = &ip_emlrtRSI;
    idx = diff(&st, switchingDistance, nrows, getSwitchingMotion_data);
    for (j = 0; j < 5; j++) {
      switchingDistance[j] = muDoubleScalarAbs(obj->AnalyticPathSegments[j]);
    }

    switchingDistance[1] += switchingDistance[0];
    switchingDistance[2] += switchingDistance[1];
    switchingDistance[3] += switchingDistance[2];
    switchingDistance[4] += switchingDistance[3];
    st.site = &jp_emlrtRSI;
    idx--;
    j = 0;
    nrows = 0;
    for (i = 0; i <= idx; i++) {
      val = getSwitchingMotion_data[i];
      if ((val == 2.0) || (val == -2.0)) {
        j++;
        tmp_data[nrows] = (int8_T)i;
        nrows++;
      }
    }

    emxInit_real_T(&st, &b_samples, 2, &vi_emlrtRTEI);
    i = b_samples->size[0] * b_samples->size[1];
    b_samples->size[0] = 1;
    b_samples->size[1] = j + samples->size[1];
    emxEnsureCapacity_real_T(&st, b_samples, i, &vi_emlrtRTEI);
    path_data = b_samples->data;
    nrows = samples->size[1];
    for (i = 0; i < nrows; i++) {
      path_data[i] = samples_data[i];
    }

    for (i = 0; i < j; i++) {
      path_data[i + samples->size[1]] = switchingDistance[tmp_data[i]];
    }

    b_st.site = &qp_emlrtRSI;
    unique_vector(&b_st, b_samples, samples);
    samples_data = samples->data;
    emxFree_real_T(&st, &b_samples);
  }

  for (j = 0; j < 5; j++) {
    switchingDistance[j] = 1.0;
    val = obj->AnalyticPathSegments[j];
    if (val < 0.0) {
      switchingDistance[j] = -1.0;
    }

    b_y[j] = muDoubleScalarAbs(val);
  }

  emxInit_real_T(sp, &r, 2, &si_emlrtRTEI);
  i = r->size[0] * r->size[1];
  r->size[0] = 1;
  r->size[1] = samples->size[1] + 1;
  emxEnsureCapacity_real_T(sp, r, i, &si_emlrtRTEI);
  path_data = r->data;
  path_data[0] = 0.0;
  nrows = samples->size[1];
  for (i = 0; i < nrows; i++) {
    path_data[i + 1] = samples_data[i];
  }

  emxFree_real_T(sp, &samples);
  for (i = 0; i < 5; i++) {
    uint32_T u;
    b_switchingDistance[i] = (int32_T)switchingDistance[i];
    val = muDoubleScalarRound(obj->AnalyticPathTypes[i]);
    if (val < 4.294967296E+9) {
      if (val >= 0.0) {
        u = (uint32_T)val;
      } else {
        u = 0U;
      }
    } else if (val >= 4.294967296E+9) {
      u = MAX_uint32_T;
    } else {
      u = 0U;
    }

    b_obj[i] = u;
  }

  st.site = &kp_emlrtRSI;
  e_ReedsSheppBuiltins_autonomous(&st, obj->ExpansionPoint, obj->GoalPose, r,
    obj->MinTurningRadius, b_y, b_switchingDistance, b_obj, path, dir);
  dir_data = dir->data;
  path_data = path->data;
  emxFree_real_T(sp, &r);
  p = true;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 3)) {
    if (!(obj->StartPose[j] == obj->ExpansionPoint[j])) {
      p = false;
      exitg1 = true;
    } else {
      j++;
    }
  }

  if (!p) {
    emxInit_real_T(sp, &c_obj, 2, &ui_emlrtRTEI);
    i = c_obj->size[0] * c_obj->size[1];
    c_obj->size[0] = (states->size[0] + path->size[0]) + 1;
    c_obj->size[1] = 3;
    emxEnsureCapacity_real_T(sp, c_obj, i, &ui_emlrtRTEI);
    samples_data = c_obj->data;
    nrows = states->size[0];
    idx = path->size[0];
    for (i = 0; i < 3; i++) {
      samples_data[c_obj->size[0] * i] = obj->StartPose[i];
      for (j = 0; j < nrows; j++) {
        samples_data[(j + c_obj->size[0] * i) + 1] = states_data[j +
          states->size[0] * i];
      }

      for (j = 0; j < idx; j++) {
        samples_data[((j + states->size[0]) + c_obj->size[0] * i) + 1] =
          path_data[j + path->size[0] * i];
      }
    }

    i = path->size[0] * path->size[1];
    path->size[0] = c_obj->size[0];
    path->size[1] = 3;
    emxEnsureCapacity_real_T(sp, path, i, &wi_emlrtRTEI);
    path_data = path->data;
    nrows = c_obj->size[0] * 3;
    for (i = 0; i < nrows; i++) {
      path_data[i] = samples_data[i];
    }

    emxFree_real_T(sp, &c_obj);
    if (pathData->size[0] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, pathData->size[0], &kf_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    emxInit_real_T(sp, &b_pathData, 1, &xi_emlrtRTEI);
    i = b_pathData->size[0];
    b_pathData->size[0] = (directions->size[0] + dir->size[0]) + 1;
    emxEnsureCapacity_real_T(sp, b_pathData, i, &xi_emlrtRTEI);
    path_data = b_pathData->data;
    path_data[0] = pathData_data[pathData->size[0] * 10];
    nrows = directions->size[0];
    for (i = 0; i < nrows; i++) {
      path_data[i + 1] = directions_data[i];
    }

    nrows = dir->size[0];
    for (i = 0; i < nrows; i++) {
      path_data[(i + directions->size[0]) + 1] = dir_data[i];
    }

    i = dir->size[0];
    dir->size[0] = b_pathData->size[0];
    emxEnsureCapacity_real_T(sp, dir, i, &yi_emlrtRTEI);
    dir_data = dir->data;
    nrows = b_pathData->size[0];
    for (i = 0; i < nrows; i++) {
      dir_data[i] = path_data[i];
    }

    emxFree_real_T(sp, &b_pathData);
  }

  emxFree_real_T(sp, &directions);
  emxFree_real_T(sp, &states);
  if (dir->size[0] > 50000) {
    emlrtErrorWithMessageIdR2018a(sp, &jc_emlrtRTEI,
      "nav:navalgs:hybridastar:AssertionFailedLessThan",
      "nav:navalgs:hybridastar:AssertionFailedLessThan", 6, 4, 30,
      "Number of States in Final Path", 4, 55,
      "Map.GridSize(1) * Map.GridSize(1) * NumMotionPrimitives");
  }

  if (path->size[0] > 50000) {
    emlrtErrorWithMessageIdR2018a(sp, &ic_emlrtRTEI,
      "nav:navalgs:hybridastar:AssertionFailedLessThan",
      "nav:navalgs:hybridastar:AssertionFailedLessThan", 6, 4, 34,
      "Number of Directions in Final Path", 4, 55,
      "Map.GridSize(1) * Map.GridSize(1) * NumMotionPrimitives");
  }

  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void c_plannerHybridAStar_getPosesCi(const emlrtStack *sp, const real_T
  initialPose_data[], const real_T finalPoses_data[], const real_T ICRData_data[],
  const real_T radius_data[], real_T length, real_T stepSize, emxArray_real_T
  *poses)
{
  __m128d r;
  __m128d r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_real_T *angles;
  emxArray_real_T *deltaX;
  emxArray_real_T *deltaY;
  emxArray_real_T *expandedIntervals;
  emxArray_real_T *intervals;
  emxArray_real_T *r2;
  emxArray_real_T *r4;
  emxArray_real_T *r5;
  emxArray_real_T *yPoints;
  real_T b_finalPoses_data[4];
  real_T sz;
  real_T *angles_data;
  real_T *deltaY_data;
  real_T *expandedIntervals_data;
  real_T *intervals_data;
  real_T *r3;
  int32_T b_varargin_1_tmp;
  int32_T i;
  int32_T k;
  int32_T loop_ub;
  int32_T varargin_1_tmp;
  int32_T vectorUB;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  sz = length / stepSize;
  if (!(sz <= 50000.0)) {
    emlrtErrorWithMessageIdR2018a(sp, &bd_emlrtRTEI,
      "nav:navalgs:hybridastar:AssertionFailedLessThan",
      "nav:navalgs:hybridastar:AssertionFailedLessThan", 6, 4, 60,
      "length of MotionPrimitiveLength/validator.ValidationDistance", 4, 55,
      "Map.GridSize(1) * Map.GridSize(1) * NumMotionPrimitives");
  }

  st.site = &br_emlrtRSI;
  emxInit_real_T(&st, &intervals, 2, &jk_emlrtRTEI);
  intervals_data = intervals->data;
  if (!(sz + 2.0 >= 0.0)) {
    intervals->size[0] = 1;
    intervals->size[1] = 0;
  } else {
    sz = muDoubleScalarFloor(sz + 2.0);
    i = intervals->size[0] * intervals->size[1];
    intervals->size[0] = 1;
    intervals->size[1] = (int32_T)sz;
    emxEnsureCapacity_real_T(&st, intervals, i, &eg_emlrtRTEI);
    intervals_data = intervals->data;
    if ((int32_T)sz >= 1) {
      intervals_data[(int32_T)sz - 1] = 1.0;
      if (intervals->size[1] >= 2) {
        intervals_data[0] = 0.0;
        if (intervals->size[1] >= 3) {
          sz = 1.0 / ((real_T)intervals->size[1] - 1.0);
          i = intervals->size[1];
          for (k = 0; k <= i - 3; k++) {
            intervals_data[k + 1] = ((real_T)k + 1.0) * sz;
          }
        }
      }
    }
  }

  _mm_storeu_pd(&b_finalPoses_data[0], _mm_sub_pd(_mm_loadu_pd(&finalPoses_data
    [8]), _mm_loadu_pd(&initialPose_data[8])));
  _mm_storeu_pd(&b_finalPoses_data[2], _mm_sub_pd(_mm_loadu_pd(&finalPoses_data
    [10]), _mm_loadu_pd(&initialPose_data[10])));
  emxInit_real_T(sp, &expandedIntervals, 2, &wj_emlrtRTEI);
  i = expandedIntervals->size[0] * expandedIntervals->size[1];
  expandedIntervals->size[0] = 4;
  expandedIntervals->size[1] = intervals->size[1];
  emxEnsureCapacity_real_T(sp, expandedIntervals, i, &wj_emlrtRTEI);
  expandedIntervals_data = expandedIntervals->data;
  loop_ub = intervals->size[1];
  for (i = 0; i < loop_ub; i++) {
    r = _mm_loadu_pd(&b_finalPoses_data[0]);
    r1 = _mm_set1_pd(intervals_data[i]);
    _mm_storeu_pd(&expandedIntervals_data[4 * i], _mm_mul_pd(r, r1));
    r = _mm_loadu_pd(&b_finalPoses_data[2]);
    _mm_storeu_pd(&expandedIntervals_data[4 * i + 2], _mm_mul_pd(r, r1));
  }

  emxFree_real_T(sp, &intervals);
  b_finalPoses_data[0] = initialPose_data[8];
  b_finalPoses_data[1] = initialPose_data[9];
  b_finalPoses_data[2] = initialPose_data[10];
  b_finalPoses_data[3] = initialPose_data[11];
  emxInit_real_T(sp, &angles, 2, &xj_emlrtRTEI);
  st.site = &cr_emlrtRSI;
  e_repmat(&st, b_finalPoses_data, expandedIntervals->size[1], angles);
  if ((angles->size[1] != expandedIntervals->size[1]) && ((angles->size[1] != 1)
       && (expandedIntervals->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(angles->size[1], expandedIntervals->size[1],
      &rb_emlrtECI, (emlrtConstCTX)sp);
  }

  if (angles->size[1] == expandedIntervals->size[1]) {
    loop_ub = angles->size[1] << 2;
    i = angles->size[0] * angles->size[1];
    angles->size[0] = 4;
    emxEnsureCapacity_real_T(sp, angles, i, &xj_emlrtRTEI);
    angles_data = angles->data;
    k = (loop_ub / 2) << 1;
    vectorUB = k - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&angles_data[i]);
      r1 = _mm_loadu_pd(&expandedIntervals_data[i]);
      _mm_storeu_pd(&angles_data[i], _mm_add_pd(r, r1));
    }

    for (i = k; i < loop_ub; i++) {
      angles_data[i] += expandedIntervals_data[i];
    }
  } else {
    st.site = &cr_emlrtRSI;
    c_plus(&st, angles, expandedIntervals);
    angles_data = angles->data;
  }

  st.site = &dr_emlrtRSI;
  emxInit_real_T(&st, &deltaX, 2, &ak_emlrtRTEI);
  i = deltaX->size[0] * deltaX->size[1];
  deltaX->size[0] = 4;
  deltaX->size[1] = angles->size[1];
  emxEnsureCapacity_real_T(&st, deltaX, i, &yj_emlrtRTEI);
  intervals_data = deltaX->data;
  loop_ub = angles->size[1] << 2;
  for (i = 0; i < loop_ub; i++) {
    intervals_data[i] = angles_data[i];
  }

  b_st.site = &lq_emlrtRSI;
  for (k = 0; k < loop_ub; k++) {
    intervals_data[k] = muDoubleScalarSin(intervals_data[k]);
  }

  b_finalPoses_data[0] = radius_data[0];
  b_finalPoses_data[1] = radius_data[1];
  b_finalPoses_data[2] = radius_data[2];
  b_finalPoses_data[3] = radius_data[3];
  st.site = &dr_emlrtRSI;
  e_repmat(&st, b_finalPoses_data, angles->size[1], expandedIntervals);
  expandedIntervals_data = expandedIntervals->data;
  if ((deltaX->size[1] != expandedIntervals->size[1]) && ((deltaX->size[1] != 1)
       && (expandedIntervals->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(deltaX->size[1], expandedIntervals->size[1],
      &qb_emlrtECI, (emlrtConstCTX)sp);
  }

  if (deltaX->size[1] == expandedIntervals->size[1]) {
    i = deltaX->size[0] * deltaX->size[1];
    deltaX->size[0] = 4;
    emxEnsureCapacity_real_T(sp, deltaX, i, &ak_emlrtRTEI);
    intervals_data = deltaX->data;
    k = (loop_ub / 2) << 1;
    vectorUB = k - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&intervals_data[i]);
      r1 = _mm_loadu_pd(&expandedIntervals_data[i]);
      _mm_storeu_pd(&intervals_data[i], _mm_mul_pd(r, r1));
    }

    for (i = k; i < loop_ub; i++) {
      intervals_data[i] *= expandedIntervals_data[i];
    }
  } else {
    st.site = &dr_emlrtRSI;
    times(&st, deltaX, expandedIntervals);
    intervals_data = deltaX->data;
  }

  st.site = &er_emlrtRSI;
  emxInit_real_T(&st, &deltaY, 2, &ck_emlrtRTEI);
  i = deltaY->size[0] * deltaY->size[1];
  deltaY->size[0] = 4;
  deltaY->size[1] = angles->size[1];
  emxEnsureCapacity_real_T(&st, deltaY, i, &bk_emlrtRTEI);
  deltaY_data = deltaY->data;
  for (i = 0; i < loop_ub; i++) {
    deltaY_data[i] = angles_data[i];
  }

  b_st.site = &mq_emlrtRSI;
  for (k = 0; k < loop_ub; k++) {
    deltaY_data[k] = muDoubleScalarCos(deltaY_data[k]);
  }

  b_finalPoses_data[0] = radius_data[0];
  b_finalPoses_data[1] = radius_data[1];
  b_finalPoses_data[2] = radius_data[2];
  b_finalPoses_data[3] = radius_data[3];
  st.site = &er_emlrtRSI;
  e_repmat(&st, b_finalPoses_data, angles->size[1], expandedIntervals);
  expandedIntervals_data = expandedIntervals->data;
  if ((deltaY->size[1] != expandedIntervals->size[1]) && ((deltaY->size[1] != 1)
       && (expandedIntervals->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(deltaY->size[1], expandedIntervals->size[1],
      &pb_emlrtECI, (emlrtConstCTX)sp);
  }

  if (deltaY->size[1] == expandedIntervals->size[1]) {
    i = deltaY->size[0] * deltaY->size[1];
    deltaY->size[0] = 4;
    emxEnsureCapacity_real_T(sp, deltaY, i, &ck_emlrtRTEI);
    deltaY_data = deltaY->data;
    k = (loop_ub / 2) << 1;
    vectorUB = k - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&deltaY_data[i]);
      r1 = _mm_loadu_pd(&expandedIntervals_data[i]);
      _mm_storeu_pd(&deltaY_data[i], _mm_mul_pd(r, r1));
    }

    for (i = k; i < loop_ub; i++) {
      deltaY_data[i] *= expandedIntervals_data[i];
    }
  } else {
    st.site = &er_emlrtRSI;
    times(&st, deltaY, expandedIntervals);
    deltaY_data = deltaY->data;
  }

  b_finalPoses_data[0] = ICRData_data[0];
  b_finalPoses_data[1] = ICRData_data[1];
  b_finalPoses_data[2] = ICRData_data[2];
  b_finalPoses_data[3] = ICRData_data[3];
  st.site = &fr_emlrtRSI;
  e_repmat(&st, b_finalPoses_data, deltaX->size[1], expandedIntervals);
  if ((expandedIntervals->size[1] != deltaX->size[1]) &&
      ((expandedIntervals->size[1] != 1) && (deltaX->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(expandedIntervals->size[1], deltaX->size[1],
      &ob_emlrtECI, (emlrtConstCTX)sp);
  }

  b_finalPoses_data[0] = ICRData_data[4];
  b_finalPoses_data[1] = ICRData_data[5];
  b_finalPoses_data[2] = ICRData_data[6];
  b_finalPoses_data[3] = ICRData_data[7];
  emxInit_real_T(sp, &yPoints, 2, &kk_emlrtRTEI);
  st.site = &gr_emlrtRSI;
  e_repmat(&st, b_finalPoses_data, deltaY->size[1], yPoints);
  if ((yPoints->size[1] != deltaY->size[1]) && ((yPoints->size[1] != 1) &&
       (deltaY->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(yPoints->size[1], deltaY->size[1], &nb_emlrtECI,
      (emlrtConstCTX)sp);
  }

  if (expandedIntervals->size[1] == deltaX->size[1]) {
    loop_ub = expandedIntervals->size[1] << 2;
    i = expandedIntervals->size[0] * expandedIntervals->size[1];
    expandedIntervals->size[0] = 4;
    emxEnsureCapacity_real_T(sp, expandedIntervals, i, &dk_emlrtRTEI);
    expandedIntervals_data = expandedIntervals->data;
    k = (loop_ub / 2) << 1;
    vectorUB = k - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&expandedIntervals_data[i]);
      r1 = _mm_loadu_pd(&intervals_data[i]);
      _mm_storeu_pd(&expandedIntervals_data[i], _mm_add_pd(r, r1));
    }

    for (i = k; i < loop_ub; i++) {
      expandedIntervals_data[i] += intervals_data[i];
    }
  } else {
    c_plus(sp, expandedIntervals, deltaX);
  }

  emxFree_real_T(sp, &deltaX);
  st.site = &ir_emlrtRSI;
  nullAssignment(&st, expandedIntervals);
  expandedIntervals_data = expandedIntervals->data;
  if (yPoints->size[1] == deltaY->size[1]) {
    loop_ub = yPoints->size[1] << 2;
    i = yPoints->size[0] * yPoints->size[1];
    yPoints->size[0] = 4;
    emxEnsureCapacity_real_T(sp, yPoints, i, &ek_emlrtRTEI);
    intervals_data = yPoints->data;
    k = (loop_ub / 2) << 1;
    vectorUB = k - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&intervals_data[i]);
      r1 = _mm_loadu_pd(&deltaY_data[i]);
      _mm_storeu_pd(&intervals_data[i], _mm_sub_pd(r, r1));
    }

    for (i = k; i < loop_ub; i++) {
      intervals_data[i] -= deltaY_data[i];
    }
  } else {
    minus(sp, yPoints, deltaY);
  }

  emxFree_real_T(sp, &deltaY);
  st.site = &kr_emlrtRSI;
  nullAssignment(&st, yPoints);
  intervals_data = yPoints->data;
  st.site = &hr_emlrtRSI;
  nullAssignment(&st, angles);
  angles_data = angles->data;
  emxInit_real_T(sp, &r2, 2, &lk_emlrtRTEI);
  i = r2->size[0] * r2->size[1];
  r2->size[0] = expandedIntervals->size[1];
  r2->size[1] = 4;
  emxEnsureCapacity_real_T(sp, r2, i, &fk_emlrtRTEI);
  r3 = r2->data;
  loop_ub = expandedIntervals->size[1];
  for (i = 0; i < 4; i++) {
    for (vectorUB = 0; vectorUB < loop_ub; vectorUB++) {
      r3[vectorUB + r2->size[0] * i] = expandedIntervals_data[i + 4 * vectorUB];
    }
  }

  varargin_1_tmp = expandedIntervals->size[1] << 2;
  emxFree_real_T(sp, &expandedIntervals);
  st.site = &jr_emlrtRSI;
  b_st.site = &ur_emlrtRSI;
  k = r2->size[0];
  if (r2->size[0] < 4) {
    k = 4;
  }

  if (varargin_1_tmp > muIntScalarMax_sint32(varargin_1_tmp, k)) {
    emlrtErrorWithMessageIdR2018a(&st, &sc_emlrtRTEI,
      "Coder:toolbox:reshape_emptyReshapeLimit",
      "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }

  emxInit_real_T(sp, &r4, 2, &mk_emlrtRTEI);
  i = r4->size[0] * r4->size[1];
  r4->size[0] = yPoints->size[1];
  r4->size[1] = 4;
  emxEnsureCapacity_real_T(sp, r4, i, &gk_emlrtRTEI);
  expandedIntervals_data = r4->data;
  loop_ub = yPoints->size[1];
  for (i = 0; i < 4; i++) {
    for (vectorUB = 0; vectorUB < loop_ub; vectorUB++) {
      expandedIntervals_data[vectorUB + r4->size[0] * i] = intervals_data[i + 4 *
        vectorUB];
    }
  }

  b_varargin_1_tmp = yPoints->size[1] << 2;
  emxFree_real_T(sp, &yPoints);
  st.site = &lr_emlrtRSI;
  b_st.site = &ur_emlrtRSI;
  k = r4->size[0];
  if (r4->size[0] < 4) {
    k = 4;
  }

  if (b_varargin_1_tmp > muIntScalarMax_sint32(b_varargin_1_tmp, k)) {
    emlrtErrorWithMessageIdR2018a(&st, &sc_emlrtRTEI,
      "Coder:toolbox:reshape_emptyReshapeLimit",
      "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }

  emxInit_real_T(sp, &r5, 2, &nk_emlrtRTEI);
  i = r5->size[0] * r5->size[1];
  r5->size[0] = angles->size[1];
  r5->size[1] = 4;
  emxEnsureCapacity_real_T(sp, r5, i, &hk_emlrtRTEI);
  deltaY_data = r5->data;
  loop_ub = angles->size[1];
  for (i = 0; i < 4; i++) {
    for (vectorUB = 0; vectorUB < loop_ub; vectorUB++) {
      deltaY_data[vectorUB + r5->size[0] * i] = angles_data[i + 4 * vectorUB];
    }
  }

  vectorUB = angles->size[1] << 2;
  emxFree_real_T(sp, &angles);
  st.site = &mr_emlrtRSI;
  b_st.site = &ur_emlrtRSI;
  k = r5->size[0];
  if (r5->size[0] < 4) {
    k = 4;
  }

  if (vectorUB > muIntScalarMax_sint32(vectorUB, k)) {
    emlrtErrorWithMessageIdR2018a(&st, &sc_emlrtRTEI,
      "Coder:toolbox:reshape_emptyReshapeLimit",
      "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }

  st.site = &nr_emlrtRSI;
  b_st.site = &kn_emlrtRSI;
  c_st.site = &ln_emlrtRSI;
  if (b_varargin_1_tmp != varargin_1_tmp) {
    emlrtErrorWithMessageIdR2018a(&c_st, &s_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if (vectorUB != varargin_1_tmp) {
    emlrtErrorWithMessageIdR2018a(&c_st, &s_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  i = poses->size[0] * poses->size[1];
  poses->size[0] = varargin_1_tmp;
  poses->size[1] = 3;
  emxEnsureCapacity_real_T(&b_st, poses, i, &ik_emlrtRTEI);
  intervals_data = poses->data;
  for (i = 0; i < varargin_1_tmp; i++) {
    intervals_data[i] = r3[i];
  }

  emxFree_real_T(&b_st, &r2);
  for (i = 0; i < b_varargin_1_tmp; i++) {
    intervals_data[i + poses->size[0]] = expandedIntervals_data[i];
  }

  emxFree_real_T(&b_st, &r4);
  for (i = 0; i < vectorUB; i++) {
    intervals_data[i + poses->size[0] * 2] = deltaY_data[i];
  }

  emxFree_real_T(&b_st, &r5);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static int32_T c_plannerHybridAStar_isCircular(const emlrtStack *sp,
  plannerHybridAStar *obj, const real_T initialPose[3], const real_T
  finalPoses_data[], const real_T ICRsData_data[], const real_T radius_data[],
  real_T length, real_T stepSize, real_T direction, real_T result_data[], real_T
  finalPosesGridIndices_data[], int32_T finalPosesGridIndices_size[2])
{
  binaryOccupancyMap *b_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_boolean_T r3;
  emxArray_boolean_T *r2;
  emxArray_real_T c_finalPoses_data;
  emxArray_real_T e_finalPoses_data;
  emxArray_real_T *primitivePoses;
  emxArray_real_T *r;
  emxArray_real_T *r1;
  emxArray_real_T *validMotionPrimitivePoses;
  real_T b_finalPoses_data[8];
  real_T d_finalPoses_data[8];
  real_T b_validIndex_data[4];
  real_T validIndex_data[4];
  real_T *primitivePoses_data;
  real_T *validMotionPrimitivePoses_data;
  int32_T b_finalPoses_size[2];
  int32_T c_numPoints[2];
  int32_T finalPoses_size[2];
  int32_T iv[2];
  int32_T numPoints[2];
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T idx;
  int32_T result_size;
  int8_T tmp_data[4];
  boolean_T validPrimitivePoses_data[4];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &r, 2, &kj_emlrtRTEI);
  st.site = &xq_emlrtRSI;
  c_repmat(&st, initialPose, 4.0, r);
  emxInit_real_T(sp, &r1, 2, &lj_emlrtRTEI);
  st.site = &wq_emlrtRSI;
  b_obj = obj->Map;
  b_st.site = &yq_emlrtRSI;
  c_st.site = &ab_emlrtRSI;
  finalPoses_size[0] = 4;
  finalPoses_size[1] = 2;
  for (i = 0; i < 2; i++) {
    b_finalPoses_data[4 * i] = finalPoses_data[4 * i];
    idx = 4 * i + 1;
    b_finalPoses_data[idx] = finalPoses_data[idx];
    idx = 4 * i + 2;
    b_finalPoses_data[idx] = finalPoses_data[idx];
    idx = 4 * i + 3;
    b_finalPoses_data[idx] = finalPoses_data[idx];
  }

  c_finalPoses_data.data = &b_finalPoses_data[0];
  c_finalPoses_data.size = &finalPoses_size[0];
  c_finalPoses_data.allocatedSize = 8;
  c_finalPoses_data.numDimensions = 2;
  c_finalPoses_data.canFreeData = false;
  b_st.site = &sh_emlrtRSI;
  b_MapInterface_world2gridImpl(&b_st, b_obj, &c_finalPoses_data, r1);
  st.site = &vq_emlrtRSI;
  result_size = c_plannerHybridAStar_checkNodeV(&st, obj, (real_T *)r1->data,
    direction, result_data);
  finalPosesGridIndices_size[0] = 0;
  finalPosesGridIndices_size[1] = 0;
  st.site = &uq_emlrtRSI;
  if (b_any(result_data)) {
    real_T b_numPoints;
    real_T d;
    real_T d1;
    int32_T ii;
    int32_T loop_ub;
    int8_T ii_data[4];
    boolean_T exitg1;
    st.site = &tq_emlrtRSI;
    b_st.site = &ym_emlrtRSI;
    c_st.site = &an_emlrtRSI;
    idx = 0;
    ii = 0;
    exitg1 = false;
    while ((!exitg1) && (ii < 4)) {
      if (result_data[ii] != 0.0) {
        idx++;
        ii_data[idx - 1] = (int8_T)(ii + 1);
        if (idx >= 4) {
          exitg1 = true;
        } else {
          ii++;
        }
      } else {
        ii++;
      }
    }

    if (idx < 1) {
      loop_ub = 0;
    } else {
      loop_ub = idx;
    }

    numPoints[0] = 1;
    numPoints[1] = loop_ub;
    d_st.site = &bn_emlrtRSI;
    indexShapeCheck(&d_st, 4, numPoints);
    for (i = 0; i < loop_ub; i++) {
      validIndex_data[i] = ii_data[i];
    }

    emxInit_real_T(sp, &primitivePoses, 2, &jj_emlrtRTEI);
    st.site = &sq_emlrtRSI;
    c_plannerHybridAStar_getPosesCi(&st, (real_T *)r->data, finalPoses_data,
      ICRsData_data, radius_data, length, stepSize, primitivePoses);
    primitivePoses_data = primitivePoses->data;
    b_numPoints = obj->NumPointsMotionPrimitive - 1.0;
    if (!(b_numPoints <= 50000.0)) {
      emlrtErrorWithMessageIdR2018a(sp, &tc_emlrtRTEI,
        "nav:navalgs:hybridastar:AssertionFailedLessThan",
        "nav:navalgs:hybridastar:AssertionFailedLessThan", 6, 4, 9, "numPoints",
        4, 55, "Map.GridSize(1) * Map.GridSize(1) * NumMotionPrimitives");
    }

    emxInit_real_T(sp, &validMotionPrimitivePoses, 2, &ij_emlrtRTEI);
    d = b_numPoints * (real_T)loop_ub;
    if (!(d >= 0.0)) {
      emlrtNonNegativeCheckR2012b(d, &dd_emlrtDCI, (emlrtConstCTX)sp);
    }

    d1 = muDoubleScalarFloor(d);
    if (d != d1) {
      emlrtIntegerCheckR2012b(d, &cd_emlrtDCI, (emlrtConstCTX)sp);
    }

    i = validMotionPrimitivePoses->size[0] * validMotionPrimitivePoses->size[1];
    validMotionPrimitivePoses->size[0] = (int32_T)d;
    validMotionPrimitivePoses->size[1] = 3;
    emxEnsureCapacity_real_T(sp, validMotionPrimitivePoses, i, &ij_emlrtRTEI);
    validMotionPrimitivePoses_data = validMotionPrimitivePoses->data;
    if (d != d1) {
      emlrtIntegerCheckR2012b(d, &ed_emlrtDCI, (emlrtConstCTX)sp);
    }

    idx = (int32_T)d * 3;
    for (i = 0; i < idx; i++) {
      validMotionPrimitivePoses_data[i] = 0.0;
    }

    if (loop_ub - 1 >= 0) {
      numPoints[1] = 3;
      iv[1] = 3;
    }

    for (b_i = 0; b_i < loop_ub; b_i++) {
      if (b_i + 1 > loop_ub) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, loop_ub, &yf_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      i = (int32_T)validIndex_data[b_i];
      d = ((real_T)i - 1.0) * b_numPoints + 1.0;
      if (b_i + 1 > loop_ub) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, loop_ub, &ag_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      d1 = b_numPoints * (real_T)i;
      if (d > d1) {
        i = 1;
        i1 = 0;
      } else {
        if (d != (int32_T)muDoubleScalarFloor(d)) {
          emlrtIntegerCheckR2012b(d, &xc_emlrtDCI, (emlrtConstCTX)sp);
        }

        if (((int32_T)d < 1) || ((int32_T)d > primitivePoses->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, primitivePoses->size[0],
            &uf_emlrtBCI, (emlrtConstCTX)sp);
        }

        i = (int32_T)d;
        if (d1 != (int32_T)muDoubleScalarFloor(d1)) {
          emlrtIntegerCheckR2012b(d1, &yc_emlrtDCI, (emlrtConstCTX)sp);
        }

        if (((int32_T)d1 < 1) || ((int32_T)d1 > primitivePoses->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)d1, 1, primitivePoses->size[0],
            &vf_emlrtBCI, (emlrtConstCTX)sp);
        }

        i1 = (int32_T)d1;
      }

      d = (((real_T)b_i + 1.0) - 1.0) * b_numPoints + 1.0;
      d1 = b_numPoints * ((real_T)b_i + 1.0);
      if (d > d1) {
        ii = 0;
        i2 = 0;
      } else {
        if (d != (int32_T)muDoubleScalarFloor(d)) {
          emlrtIntegerCheckR2012b(d, &ad_emlrtDCI, (emlrtConstCTX)sp);
        }

        if (((int32_T)d < 1) || ((int32_T)d > validMotionPrimitivePoses->size[0]))
        {
          emlrtDynamicBoundsCheckR2012b((int32_T)d, 1,
            validMotionPrimitivePoses->size[0], &wf_emlrtBCI, (emlrtConstCTX)sp);
        }

        ii = (int32_T)d - 1;
        if (d1 != (int32_T)muDoubleScalarFloor(d1)) {
          emlrtIntegerCheckR2012b(d1, &bd_emlrtDCI, (emlrtConstCTX)sp);
        }

        if (((int32_T)d1 < 1) || ((int32_T)d1 > validMotionPrimitivePoses->size
             [0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)d1, 1,
            validMotionPrimitivePoses->size[0], &xf_emlrtBCI, (emlrtConstCTX)sp);
        }

        i2 = (int32_T)d1;
      }

      numPoints[0] = i2 - ii;
      idx = i1 - i;
      iv[0] = idx + 1;
      emlrtSubAssignSizeCheckR2012b(&numPoints[0], 2, &iv[0], 2, &gb_emlrtECI,
        (emlrtCTX)sp);
      for (i1 = 0; i1 < 3; i1++) {
        for (i2 = 0; i2 <= idx; i2++) {
          validMotionPrimitivePoses_data[(ii + i2) +
            validMotionPrimitivePoses->size[0] * i1] = primitivePoses_data[((i +
            i2) + primitivePoses->size[0] * i1) - 1];
        }
      }
    }

    emxFree_real_T(sp, &primitivePoses);
    emxInit_boolean_T(sp, &r2, 1, &mj_emlrtRTEI);
    st.site = &rq_emlrtRSI;
    d_validatorOccupancyMap_isState(&st, obj->StateValidator,
      validMotionPrimitivePoses, r2);
    emxFree_real_T(sp, &validMotionPrimitivePoses);
    st.site = &qq_emlrtRSI;
    idx = r2->size[0];
    b_st.site = &ur_emlrtRSI;
    c_st.site = &vr_emlrtRSI;
    if ((b_numPoints != muDoubleScalarFloor(b_numPoints)) || muDoubleScalarIsInf
        (b_numPoints) || (b_numPoints < -2.147483648E+9)) {
      emlrtErrorWithMessageIdR2018a(&c_st, &vb_emlrtRTEI,
        "Coder:MATLAB:NonIntegerInput", "Coder:MATLAB:NonIntegerInput", 4, 12,
        MIN_int32_T, 12, MAX_int32_T);
    }

    c_st.site = &vr_emlrtRSI;
    ii = r2->size[0];
    if (r2->size[0] < 1) {
      ii = 1;
    }

    idx = muIntScalarMax_sint32(idx, ii);
    if ((int32_T)b_numPoints > idx) {
      emlrtErrorWithMessageIdR2018a(&st, &sc_emlrtRTEI,
        "Coder:toolbox:reshape_emptyReshapeLimit",
        "Coder:toolbox:reshape_emptyReshapeLimit", 0);
    }

    if (loop_ub > idx) {
      emlrtErrorWithMessageIdR2018a(&st, &sc_emlrtRTEI,
        "Coder:toolbox:reshape_emptyReshapeLimit",
        "Coder:toolbox:reshape_emptyReshapeLimit", 0);
    }

    if ((int32_T)b_numPoints < 0) {
      emlrtErrorWithMessageIdR2018a(&st, &rc_emlrtRTEI,
        "MATLAB:checkDimCommon:nonnegativeSize",
        "MATLAB:checkDimCommon:nonnegativeSize", 0);
    }

    if ((int32_T)b_numPoints * loop_ub != r2->size[0]) {
      emlrtErrorWithMessageIdR2018a(&st, &qc_emlrtRTEI,
        "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
    }

    r3 = *r2;
    c_numPoints[0] = (int32_T)b_numPoints;
    c_numPoints[1] = loop_ub;
    r3.size = &c_numPoints[0];
    r3.numDimensions = 2;
    st.site = &qq_emlrtRSI;
    d_all(&st, &r3, validPrimitivePoses_data, numPoints);
    emxFree_boolean_T(sp, &r2);
    for (b_i = 0; b_i < loop_ub; b_i++) {
      if (b_i + 1 > numPoints[1]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, numPoints[1], &bg_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (b_i + 1 > loop_ub) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, loop_ub, &cg_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      result_data[(int32_T)validIndex_data[b_i] - 1] =
        validPrimitivePoses_data[b_i];
    }

    st.site = &pq_emlrtRSI;
    if (b_any(result_data)) {
      __m128d r4;
      ii = 0;
      if (result_data[0] == 1.0) {
        ii = 1;
      }

      if (result_data[1] == 1.0) {
        ii++;
      }

      if (result_data[2] == 1.0) {
        ii++;
      }

      if (result_data[3] == 1.0) {
        ii++;
      }

      idx = 0;
      if (result_data[0] == 1.0) {
        tmp_data[0] = 0;
        idx = 1;
      }

      if (result_data[1] == 1.0) {
        tmp_data[idx] = 1;
        idx++;
      }

      if (result_data[2] == 1.0) {
        tmp_data[idx] = 2;
        idx++;
      }

      if (result_data[3] == 1.0) {
        tmp_data[idx] = 3;
      }

      st.site = &oq_emlrtRSI;
      b_obj = obj->Map;
      b_st.site = &yq_emlrtRSI;
      c_st.site = &ab_emlrtRSI;
      if (ii == 0) {
        emlrtErrorWithMessageIdR2018a(&c_st, &pb_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedNonempty",
          "MATLAB:world2grid:expectedNonempty", 3, 4, 19, "input number 2, xy,");
      }

      b_finalPoses_size[0] = ii;
      b_finalPoses_size[1] = 2;
      for (i = 0; i < 2; i++) {
        for (i1 = 0; i1 < ii; i1++) {
          d_finalPoses_data[i1 + b_finalPoses_size[0] * i] =
            finalPoses_data[tmp_data[i1] + 4 * i];
        }
      }

      e_finalPoses_data.data = &d_finalPoses_data[0];
      e_finalPoses_data.size = &b_finalPoses_size[0];
      e_finalPoses_data.allocatedSize = 8;
      e_finalPoses_data.numDimensions = 2;
      e_finalPoses_data.canFreeData = false;
      b_st.site = &sh_emlrtRSI;
      b_MapInterface_world2gridImpl(&b_st, b_obj, &e_finalPoses_data, r1);
      primitivePoses_data = r1->data;
      b_numPoints = obj->Dimensions[0];
      loop_ub = r1->size[0];
      idx = r1->size[0];
      ii = (r1->size[0] / 2) << 1;
      b_i = ii - 2;
      for (i = 0; i <= b_i; i += 2) {
        r4 = _mm_loadu_pd(&primitivePoses_data[i + r1->size[0]]);
        _mm_storeu_pd(&validIndex_data[i], _mm_mul_pd(_mm_sub_pd(r4, _mm_set1_pd
          (1.0)), _mm_set1_pd(b_numPoints)));
      }

      for (i = ii; i < loop_ub; i++) {
        validIndex_data[i] = (primitivePoses_data[i + r1->size[0]] - 1.0) *
          b_numPoints;
      }

      if (idx != r1->size[0]) {
        emlrtSizeEqCheck1DR2012b(idx, r1->size[0], &hb_emlrtECI, (emlrtConstCTX)
          sp);
      }

      st.site = &nq_emlrtRSI;
      ii = (idx / 2) << 1;
      b_i = ii - 2;
      for (i = 0; i <= b_i; i += 2) {
        __m128d r5;
        r4 = _mm_loadu_pd(&validIndex_data[i]);
        r5 = _mm_loadu_pd(&primitivePoses_data[i]);
        _mm_storeu_pd(&b_validIndex_data[i], _mm_add_pd(r4, r5));
      }

      for (i = ii; i < idx; i++) {
        b_validIndex_data[i] = validIndex_data[i] + primitivePoses_data[i];
      }

      b_st.site = &qp_emlrtRSI;
      idx = b_unique_vector(&b_st, b_validIndex_data, idx, validIndex_data);
      finalPosesGridIndices_size[0] = idx;
      finalPosesGridIndices_size[1] = 1;
      if (idx - 1 >= 0) {
        memcpy(&finalPosesGridIndices_data[0], &validIndex_data[0], (uint32_T)
               idx * sizeof(real_T));
      }
    }
  }

  emxFree_real_T(sp, &r1);
  emxFree_real_T(sp, &r);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return result_size;
}

static void c_plannerHybridAStar_validateSt(const emlrtStack *sp,
  plannerHybridAStar *obj, const real_T start[3], const real_T goal[3])
{
  emlrtStack st;
  boolean_T validity;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &mg_emlrtRSI;
  validity = c_validatorOccupancyMap_isState(&st, obj->StateValidator, start);
  if (!validity) {
    emlrtErrorWithMessageIdR2018a(sp, &u_emlrtRTEI,
      "nav:navalgs:hybridastar:StartError", "nav:navalgs:hybridastar:StartError",
      0);
  }

  st.site = &ng_emlrtRSI;
  validity = c_validatorOccupancyMap_isState(&st, obj->StateValidator, goal);
  if (!validity) {
    emlrtErrorWithMessageIdR2018a(sp, &v_emlrtRTEI,
      "nav:navalgs:hybridastar:GoalError", "nav:navalgs:hybridastar:GoalError",
      0);
  }

  obj->StartPose[0] = start[0];
  obj->GoalPose[0] = goal[0];
  obj->StartPose[1] = start[1];
  obj->GoalPose[1] = goal[1];
  obj->StartPose[2] = start[2];
  obj->GoalPose[2] = goal[2];
}

static void c_plus(const emlrtStack *sp, emxArray_real_T *in1, const
                   emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, 2, &hl_emlrtRTEI);
  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[0] = 4;
  if (in2->size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in2->size[1];
  }

  b_in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, b_in1, i, &hl_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_1 = (in1->size[1] != 1);
  stride_1_1 = (in2->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    __m128d r1;
    r = _mm_loadu_pd(&in1_data[4 * aux_0_1]);
    r1 = _mm_loadu_pd(&in2_data[4 * aux_1_1]);
    _mm_storeu_pd(&b_in1_data[4 * i], _mm_add_pd(r, r1));
    r = _mm_loadu_pd(&in1_data[4 * aux_0_1 + 2]);
    r1 = _mm_loadu_pd(&in2_data[4 * aux_1_1 + 2]);
    _mm_storeu_pd(&b_in1_data[4 * i + 2], _mm_add_pd(r, r1));
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  i = in1->size[0] * in1->size[1];
  in1->size[0] = 4;
  in1->size[1] = b_in1->size[1];
  emxEnsureCapacity_real_T(sp, in1, i, &hl_emlrtRTEI);
  in1_data = in1->data;
  loop_ub = b_in1->size[1];
  for (i = 0; i < loop_ub; i++) {
    in1_data[4 * i] = b_in1_data[4 * i];
    stride_0_1 = 4 * i + 1;
    in1_data[stride_0_1] = b_in1_data[stride_0_1];
    stride_0_1 = 4 * i + 2;
    in1_data[stride_0_1] = b_in1_data[stride_0_1];
    stride_0_1 = 4 * i + 3;
    in1_data[stride_0_1] = b_in1_data[stride_0_1];
  }

  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static real_T d_plannerHybridAStar_calculateC(codegenPathPlannerStackData *SD,
  const emlrtStack *sp, plannerHybridAStar *obj, const real_T newNodeData[3],
  const real_T currentNode_data[], const int32_T currentNode_size[2], real_T
  direction, real_T *gScore, real_T *hScore)
{
  binaryOccupancyMap *b_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T matPoint_data[18];
  real_T neighborPoints[18];
  real_T b_tmp_data[9];
  real_T neighborCosts_data[9];
  real_T matPoint[2];
  real_T d;
  real_T fScore;
  real_T indices;
  real_T minCost;
  int32_T tmp_size[2];
  int32_T b_i;
  int32_T i;
  int32_T idx;
  int32_T trueCount;
  int32_T vectorUB;
  int8_T tmp_data[9];
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
  if (currentNode_size[1] < 2) {
    emlrtDynamicBoundsCheckR2012b(2, 1, currentNode_size[1], &fg_emlrtBCI,
      (emlrtConstCTX)sp);
  }

  st.site = &fs_emlrtRSI;
  *gScore = currentNode_data[1];
  if (direction == 1.0) {
    *gScore = currentNode_data[1] + obj->ForwardCost *
      obj->MotionPrimitiveLength;
  } else if (direction == -1.0) {
    *gScore = currentNode_data[1] + obj->ReverseCost *
      obj->MotionPrimitiveLength;
  }

  st.site = &gs_emlrtRSI;
  b_st.site = &ui_emlrtRSI;
  b_obj = obj->Map;
  c_st.site = &sh_emlrtRSI;
  MapInterface_world2gridImpl(b_obj, &newNodeData[0], matPoint);
  indices = matPoint[0] + 100.0 * (matPoint[1] - 1.0);
  if (indices != (int32_T)muDoubleScalarFloor(indices)) {
    emlrtIntegerCheckR2012b(indices, &h_emlrtDCI, &st);
  }

  if (((int32_T)indices < 1) || ((int32_T)indices > 10000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)indices, 1, 10000, &v_emlrtBCI, &st);
  }

  minCost = obj->Heuristic2DMat[(int32_T)indices - 1];
  if (muDoubleScalarIsInf(minCost)) {
    __m128d r1;
    for (i = 0; i < 2; i++) {
      for (vectorUB = 0; vectorUB < 9; vectorUB++) {
        idx = vectorUB + 9 * i;
        neighborPoints[idx] = matPoint[i] + obj->Neighbors[idx];
      }
    }

    trueCount = 0;
    idx = 0;
    for (b_i = 0; b_i < 9; b_i++) {
      d = neighborPoints[b_i + 9];
      minCost = neighborPoints[b_i];
      if ((minCost > 0.0) && (d > 0.0) && (minCost <= 100.0) && (d <= 100.0)) {
        trueCount++;
        tmp_data[idx] = (int8_T)b_i;
        idx++;
      }
    }

    for (i = 0; i < trueCount; i++) {
      int8_T i1;
      i1 = tmp_data[i];
      d = neighborPoints[i1] + 100.0 * (neighborPoints[i1 + 9] - 1.0);
      if (d != (int32_T)muDoubleScalarFloor(d)) {
        emlrtIntegerCheckR2012b(d, &i_emlrtDCI, &st);
      }

      if (((int32_T)d < 1) || ((int32_T)d > 10000)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 10000, &w_emlrtBCI, &st);
      }

      neighborCosts_data[i] = obj->Heuristic2DMat[(int32_T)d - 1];
    }

    for (i = 0; i < 2; i++) {
      for (vectorUB = 0; vectorUB < trueCount; vectorUB++) {
        matPoint_data[vectorUB + trueCount * i] = matPoint[i] -
          neighborPoints[tmp_data[vectorUB] + 9 * i];
      }
    }

    tmp_size[0] = trueCount;
    tmp_size[1] = 2;
    idx = trueCount << 1;
    b_i = idx / 2 * 2;
    vectorUB = b_i - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r1 = _mm_loadu_pd(&matPoint_data[i]);
      r1 = _mm_mul_pd(r1, r1);
      _mm_storeu_pd(&neighborPoints[i], r1);
    }

    for (i = b_i; i < idx; i++) {
      minCost = matPoint_data[i];
      neighborPoints[i] = minCost * minCost;
    }

    b_st.site = &vi_emlrtRSI;
    idx = sum(neighborPoints, tmp_size, b_tmp_data);
    b_st.site = &vi_emlrtRSI;
    b_sqrt(&b_st, b_tmp_data, &idx);
    if ((trueCount != idx) && ((trueCount != 1) && (idx != 1))) {
      emlrtDimSizeImpxCheckR2021b(trueCount, idx, &j_emlrtECI, &st);
    }

    if (trueCount == idx) {
      b_i = (trueCount / 2) << 1;
      vectorUB = b_i - 2;
      for (i = 0; i <= vectorUB; i += 2) {
        __m128d r2;
        r1 = _mm_loadu_pd(&neighborCosts_data[i]);
        r2 = _mm_loadu_pd(&b_tmp_data[i]);
        _mm_storeu_pd(&neighborCosts_data[i], _mm_add_pd(r1, r2));
      }

      for (i = b_i; i < trueCount; i++) {
        neighborCosts_data[i] += b_tmp_data[i];
      }
    } else {
      plus(neighborCosts_data, &trueCount, b_tmp_data, &idx);
    }

    b_st.site = &wi_emlrtRSI;
    c_st.site = &hj_emlrtRSI;
    d_st.site = &ij_emlrtRSI;
    e_st.site = &jj_emlrtRSI;
    if (trueCount < 1) {
      emlrtErrorWithMessageIdR2018a(&e_st, &cb_emlrtRTEI,
        "Coder:toolbox:eml_min_or_max_varDimZero",
        "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }

    if (trueCount <= 2) {
      if (trueCount == 1) {
        minCost = neighborCosts_data[0];
      } else if ((neighborCosts_data[0] > neighborCosts_data[1]) ||
                 (muDoubleScalarIsNaN(neighborCosts_data[0]) &&
                  (!muDoubleScalarIsNaN(neighborCosts_data[1])))) {
        minCost = neighborCosts_data[1];
      } else {
        minCost = neighborCosts_data[0];
      }
    } else {
      if (!muDoubleScalarIsNaN(neighborCosts_data[0])) {
        idx = 1;
      } else {
        boolean_T exitg1;
        idx = 0;
        vectorUB = 2;
        exitg1 = false;
        while ((!exitg1) && (vectorUB <= trueCount)) {
          if (!muDoubleScalarIsNaN(neighborCosts_data[vectorUB - 1])) {
            idx = vectorUB;
            exitg1 = true;
          } else {
            vectorUB++;
          }
        }
      }

      if (idx == 0) {
        minCost = neighborCosts_data[0];
      } else {
        minCost = neighborCosts_data[idx - 1];
        i = idx + 1;
        for (vectorUB = i; vectorUB <= trueCount; vectorUB++) {
          d = neighborCosts_data[vectorUB - 1];
          if (minCost > d) {
            minCost = d;
          }
        }
      }
    }

    if (!muDoubleScalarIsInf(minCost)) {
      obj->Heuristic2DMat[(int32_T)indices - 1] = minCost;
    } else {
      b_st.site = &xi_emlrtRSI;
      plannerAStarGrid_plan(SD, &b_st, obj->Heuristic2DObj, obj->GoalPoint,
                            matPoint);
      for (i = 0; i < 10000; i++) {
        SD->u5.f6.varargin_1[i] = obj->Heuristic2DMat[i];
      }

      for (i = 0; i < 10000; i++) {
        SD->u5.f6.varargin_2[i] = obj->Heuristic2DObj->GCostMatrix[i];
      }

      for (vectorUB = 0; vectorUB < 10000; vectorUB++) {
        SD->u5.f6.minval[vectorUB] = muDoubleScalarMin(SD->
          u5.f6.varargin_1[vectorUB], SD->u5.f6.varargin_2[vectorUB]);
      }

      for (i = 0; i < 10000; i++) {
        obj->Heuristic2DMat[i] = SD->u5.f6.minval[i];
      }
    }
  }

  *hScore = obj->Heuristic2DMat[(int32_T)indices - 1];
  st.site = &gs_emlrtRSI;
  d = c_plannerHybridAStar_get3DHeuri(obj, newNodeData, obj->GoalPose);
  if ((*hScore < d) || (muDoubleScalarIsNaN(*hScore) && (!muDoubleScalarIsNaN(d))))
  {
    *hScore = d;
  }

  if (currentNode_size[1] < 7) {
    emlrtDynamicBoundsCheckR2012b(7, 1, currentNode_size[1], &gg_emlrtBCI, &st);
  }

  if ((currentNode_data[6] == 0.0) || (currentNode_data[6] * direction == 1.0))
  {
    fScore = *gScore + *hScore;
  } else {
    fScore = (*gScore + *hScore) + obj->DirectionSwitchingCost;
  }

  return fScore;
}

static real_T d_plannerHybridAStar_checkNodeV(const emlrtStack *sp, const
  plannerHybridAStar *obj, const real_T PointsGrid[2], real_T direction)
{
  real_T finalPointsGridIndices;
  real_T nodeValidity;
  finalPointsGridIndices = (PointsGrid[1] - 1.0) * obj->Dimensions[0] +
    PointsGrid[0];
  if ((PointsGrid[0] < 1.0) || (PointsGrid[1] < 1.0) || (PointsGrid[0] >
       obj->Dimensions[0]) || (PointsGrid[1] > obj->Dimensions[1])) {
    nodeValidity = 0.0;
  } else if (direction == 1.0) {
    if (finalPointsGridIndices != (int32_T)muDoubleScalarFloor
        (finalPointsGridIndices)) {
      emlrtIntegerCheckR2012b(finalPointsGridIndices, &hd_emlrtDCI,
        (emlrtConstCTX)sp);
    }

    if (((int32_T)finalPointsGridIndices < 1) || ((int32_T)
         finalPointsGridIndices > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)finalPointsGridIndices, 1, 10000,
        &eg_emlrtBCI, (emlrtConstCTX)sp);
    }

    nodeValidity = !obj->visitedCellsFront[(int32_T)finalPointsGridIndices - 1];
  } else {
    if (finalPointsGridIndices != (int32_T)muDoubleScalarFloor
        (finalPointsGridIndices)) {
      emlrtIntegerCheckR2012b(finalPointsGridIndices, &gd_emlrtDCI,
        (emlrtConstCTX)sp);
    }

    if (((int32_T)finalPointsGridIndices < 1) || ((int32_T)
         finalPointsGridIndices > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)finalPointsGridIndices, 1, 10000,
        &dg_emlrtBCI, (emlrtConstCTX)sp);
    }

    nodeValidity = !obj->visitedCellsBack[(int32_T)finalPointsGridIndices - 1];
  }

  return nodeValidity;
}

static int32_T d_plannerHybridAStar_get2DHeuri(codegenPathPlannerStackData *SD,
  const emlrtStack *sp, plannerHybridAStar *obj, const real_T point_data[],
  const int32_T point_size[2], real_T cost_data[])
{
  __m128d r2;
  __m128d r3;
  binaryOccupancyMap *b_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_real_T c_point_data;
  emxArray_real_T *r;
  emxArray_real_T *y;
  real_T b_tmp_data[18];
  real_T neighborPoints[18];
  real_T c_tmp_data[9];
  real_T neighborCosts_data[9];
  real_T b_point_data[8];
  real_T indices_data[4];
  real_T d;
  real_T *r1;
  real_T *y_data;
  int32_T b_point_size[2];
  int32_T tmp_size[2];
  int32_T b_i;
  int32_T cost_size;
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T loop_ub;
  int32_T trueCount;
  int32_T vectorUB_tmp;
  int8_T tmp_data[9];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &ui_emlrtRSI;
  b_obj = obj->Map;
  b_st.site = &yq_emlrtRSI;
  c_st.site = &ab_emlrtRSI;
  if (point_size[0] == 0) {
    emlrtErrorWithMessageIdR2018a(&c_st, &pb_emlrtRTEI,
      "Coder:toolbox:ValidateattributesexpectedNonempty",
      "MATLAB:world2grid:expectedNonempty", 3, 4, 19, "input number 2, xy,");
  }

  b_point_size[0] = point_size[0];
  b_point_size[1] = 2;
  loop_ub = point_size[0];
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_point_data[i1 + b_point_size[0] * i] = point_data[i1 + point_size[0] * i];
    }
  }

  c_point_data.data = &b_point_data[0];
  c_point_data.size = &b_point_size[0];
  c_point_data.allocatedSize = 8;
  c_point_data.numDimensions = 2;
  c_point_data.canFreeData = false;
  emxInit_real_T(&st, &r, 2, &lj_emlrtRTEI);
  b_st.site = &sh_emlrtRSI;
  b_MapInterface_world2gridImpl(&b_st, b_obj, &c_point_data, r);
  r1 = r->data;
  loop_ub = r->size[0];
  emxInit_real_T(sp, &y, 1, &qj_emlrtRTEI);
  i = y->size[0];
  y->size[0] = r->size[0];
  emxEnsureCapacity_real_T(sp, y, i, &nj_emlrtRTEI);
  y_data = y->data;
  idx = (r->size[0] / 2) << 1;
  vectorUB_tmp = idx - 2;
  for (i = 0; i <= vectorUB_tmp; i += 2) {
    r2 = _mm_loadu_pd(&r1[i + r->size[0]]);
    _mm_storeu_pd(&y_data[i], _mm_mul_pd(_mm_set1_pd(100.0), _mm_sub_pd(r2,
      _mm_set1_pd(1.0))));
  }

  for (i = idx; i < loop_ub; i++) {
    y_data[i] = 100.0 * (r1[i + r->size[0]] - 1.0);
  }

  if (r->size[0] != y->size[0]) {
    emlrtSizeEqCheck1DR2012b(r->size[0], y->size[0], &mb_emlrtECI,
      (emlrtConstCTX)sp);
  }

  loop_ub = r->size[0];
  cost_size = r->size[0];
  for (i = 0; i <= vectorUB_tmp; i += 2) {
    r2 = _mm_loadu_pd(&r1[i]);
    r3 = _mm_loadu_pd(&y_data[i]);
    _mm_storeu_pd(&indices_data[i], _mm_add_pd(r2, r3));
  }

  for (i = idx; i < loop_ub; i++) {
    indices_data[i] = r1[i] + y_data[i];
  }

  emxFree_real_T(sp, &y);
  for (i = 0; i < cost_size; i++) {
    d = indices_data[i];
    if (d != (int32_T)muDoubleScalarFloor(d)) {
      emlrtIntegerCheckR2012b(d, &h_emlrtDCI, (emlrtConstCTX)sp);
    }

    if (((int32_T)d < 1) || ((int32_T)d > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 10000, &v_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    cost_data[i] = obj->Heuristic2DMat[(int32_T)d - 1];
  }

  i = r->size[0];
  for (b_i = 0; b_i < i; b_i++) {
    real_T minCost;
    int32_T x_tmp;
    if (b_i + 1 > cost_size) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, cost_size, &kg_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    x_tmp = (int32_T)indices_data[b_i] - 1;
    minCost = obj->Heuristic2DMat[x_tmp];
    if (muDoubleScalarIsInf(minCost)) {
      if (b_i + 1 > r->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, r->size[0], &hg_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      for (i1 = 0; i1 < 2; i1++) {
        for (vectorUB_tmp = 0; vectorUB_tmp < 9; vectorUB_tmp++) {
          idx = vectorUB_tmp + 9 * i1;
          neighborPoints[idx] = r1[b_i + r->size[0] * i1] + obj->Neighbors[idx];
        }
      }

      trueCount = 0;
      idx = 0;
      for (vectorUB_tmp = 0; vectorUB_tmp < 9; vectorUB_tmp++) {
        boolean_T b;
        d = neighborPoints[vectorUB_tmp + 9];
        minCost = neighborPoints[vectorUB_tmp];
        b = ((minCost > 0.0) && (d > 0.0) && (minCost <= 100.0) && (d <= 100.0));
        if (b) {
          trueCount++;
          tmp_data[idx] = (int8_T)vectorUB_tmp;
          idx++;
        }
      }

      for (i1 = 0; i1 < trueCount; i1++) {
        int8_T i2;
        i2 = tmp_data[i1];
        d = neighborPoints[i2] + 100.0 * (neighborPoints[i2 + 9] - 1.0);
        if (d != (int32_T)muDoubleScalarFloor(d)) {
          emlrtIntegerCheckR2012b(d, &i_emlrtDCI, (emlrtConstCTX)sp);
        }

        if (((int32_T)d < 1) || ((int32_T)d > 10000)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 10000, &w_emlrtBCI,
            (emlrtConstCTX)sp);
        }

        neighborCosts_data[i1] = obj->Heuristic2DMat[(int32_T)d - 1];
      }

      if (b_i + 1 > r->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, r->size[0], &ig_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      for (i1 = 0; i1 < 2; i1++) {
        for (vectorUB_tmp = 0; vectorUB_tmp < trueCount; vectorUB_tmp++) {
          b_tmp_data[vectorUB_tmp + trueCount * i1] = r1[b_i + r->size[0] * i1]
            - neighborPoints[tmp_data[vectorUB_tmp] + 9 * i1];
        }
      }

      tmp_size[0] = trueCount;
      tmp_size[1] = 2;
      loop_ub = trueCount << 1;
      vectorUB_tmp = loop_ub / 2 * 2;
      idx = vectorUB_tmp - 2;
      for (i1 = 0; i1 <= idx; i1 += 2) {
        r2 = _mm_loadu_pd(&b_tmp_data[i1]);
        r2 = _mm_mul_pd(r2, r2);
        _mm_storeu_pd(&neighborPoints[i1], r2);
      }

      for (i1 = vectorUB_tmp; i1 < loop_ub; i1++) {
        minCost = b_tmp_data[i1];
        neighborPoints[i1] = minCost * minCost;
      }

      st.site = &vi_emlrtRSI;
      idx = sum(neighborPoints, tmp_size, c_tmp_data);
      st.site = &vi_emlrtRSI;
      b_sqrt(&st, c_tmp_data, &idx);
      if ((trueCount != idx) && ((trueCount != 1) && (idx != 1))) {
        emlrtDimSizeImpxCheckR2021b(trueCount, idx, &j_emlrtECI, (emlrtConstCTX)
          sp);
      }

      if (trueCount == idx) {
        vectorUB_tmp = (trueCount / 2) << 1;
        idx = vectorUB_tmp - 2;
        for (i1 = 0; i1 <= idx; i1 += 2) {
          r2 = _mm_loadu_pd(&neighborCosts_data[i1]);
          r3 = _mm_loadu_pd(&c_tmp_data[i1]);
          _mm_storeu_pd(&neighborCosts_data[i1], _mm_add_pd(r2, r3));
        }

        for (i1 = vectorUB_tmp; i1 < trueCount; i1++) {
          neighborCosts_data[i1] += c_tmp_data[i1];
        }
      } else {
        plus(neighborCosts_data, &trueCount, c_tmp_data, &idx);
      }

      st.site = &wi_emlrtRSI;
      b_st.site = &hj_emlrtRSI;
      c_st.site = &ij_emlrtRSI;
      d_st.site = &jj_emlrtRSI;
      if (trueCount < 1) {
        emlrtErrorWithMessageIdR2018a(&d_st, &cb_emlrtRTEI,
          "Coder:toolbox:eml_min_or_max_varDimZero",
          "Coder:toolbox:eml_min_or_max_varDimZero", 0);
      }

      if (trueCount <= 2) {
        if (trueCount == 1) {
          minCost = neighborCosts_data[0];
        } else if ((neighborCosts_data[0] > neighborCosts_data[1]) ||
                   (muDoubleScalarIsNaN(neighborCosts_data[0]) &&
                    (!muDoubleScalarIsNaN(neighborCosts_data[1])))) {
          minCost = neighborCosts_data[1];
        } else {
          minCost = neighborCosts_data[0];
        }
      } else {
        if (!muDoubleScalarIsNaN(neighborCosts_data[0])) {
          idx = 1;
        } else {
          boolean_T exitg1;
          idx = 0;
          vectorUB_tmp = 2;
          exitg1 = false;
          while ((!exitg1) && (vectorUB_tmp <= trueCount)) {
            if (!muDoubleScalarIsNaN(neighborCosts_data[vectorUB_tmp - 1])) {
              idx = vectorUB_tmp;
              exitg1 = true;
            } else {
              vectorUB_tmp++;
            }
          }
        }

        if (idx == 0) {
          minCost = neighborCosts_data[0];
        } else {
          minCost = neighborCosts_data[idx - 1];
          i1 = idx + 1;
          for (vectorUB_tmp = i1; vectorUB_tmp <= trueCount; vectorUB_tmp++) {
            d = neighborCosts_data[vectorUB_tmp - 1];
            if (minCost > d) {
              minCost = d;
            }
          }
        }
      }

      if (!muDoubleScalarIsInf(minCost)) {
        if (b_i + 1 > cost_size) {
          emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, cost_size, &lg_emlrtBCI,
            (emlrtConstCTX)sp);
        }

        obj->Heuristic2DMat[x_tmp] = minCost;
      } else {
        real_T dv[2];
        if (b_i + 1 > r->size[0]) {
          emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, r->size[0], &jg_emlrtBCI,
            (emlrtConstCTX)sp);
        }

        dv[0] = r1[b_i];
        dv[1] = r1[b_i + r->size[0]];
        st.site = &xi_emlrtRSI;
        plannerAStarGrid_plan(SD, &st, obj->Heuristic2DObj, obj->GoalPoint, dv);
        for (i1 = 0; i1 < 10000; i1++) {
          SD->u5.f7.varargin_1[i1] = obj->Heuristic2DMat[i1];
        }

        for (i1 = 0; i1 < 10000; i1++) {
          SD->u5.f7.varargin_2[i1] = obj->Heuristic2DObj->GCostMatrix[i1];
        }

        for (vectorUB_tmp = 0; vectorUB_tmp < 10000; vectorUB_tmp++) {
          SD->u5.f7.minval[vectorUB_tmp] = muDoubleScalarMin
            (SD->u5.f7.varargin_1[vectorUB_tmp], SD->
             u5.f7.varargin_2[vectorUB_tmp]);
        }

        for (i1 = 0; i1 < 10000; i1++) {
          obj->Heuristic2DMat[i1] = SD->u5.f7.minval[i1];
        }
      }
    }

    for (i1 = 0; i1 < cost_size; i1++) {
      cost_data[i1] = obj->Heuristic2DMat[(int32_T)indices_data[i1] - 1];
    }
  }

  emxFree_real_T(sp, &r);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return cost_size;
}

static int32_T f_binary_expand_op(real_T in1_data[], const real_T in2_data[],
  const int32_T *in2_size, const real_T in3_data[], const int32_T *in3_size,
  const plannerHybridAStar *in4)
{
  int32_T i;
  int32_T in1_size;
  int32_T stride_0_0;
  int32_T stride_1_0;
  if (*in3_size == 1) {
    in1_size = *in2_size;
  } else {
    in1_size = *in3_size;
  }

  stride_0_0 = (*in2_size != 1);
  stride_1_0 = (*in3_size != 1);
  for (i = 0; i < in1_size; i++) {
    in1_data[i] = (in2_data[i * stride_0_0] + in3_data[i * stride_1_0]) +
      in4->DirectionSwitchingCost;
  }

  return in1_size;
}

static void g_binary_expand_op(real_T in1_data[], int32_T *in1_size, const
  emxArray_real_T *in2)
{
  real_T b_in1_data[4];
  const real_T *in2_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  in2_data = in2->data;
  if (in2->size[0] == 1) {
    loop_ub = *in1_size;
  } else {
    loop_ub = in2->size[0];
  }

  stride_0_0 = (*in1_size != 1);
  stride_1_0 = (in2->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in1_data[i] = in1_data[i * stride_0_0] + in2_data[i * stride_1_0];
  }

  *in1_size = loop_ub;
  if (loop_ub - 1 >= 0) {
    memcpy(&in1_data[0], &b_in1_data[0], (uint32_T)loop_ub * sizeof(real_T));
  }
}

static void minus(const emlrtStack *sp, emxArray_real_T *in1, const
                  emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, 2, &gl_emlrtRTEI);
  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[0] = 4;
  if (in2->size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in2->size[1];
  }

  b_in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, b_in1, i, &gl_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_1 = (in1->size[1] != 1);
  stride_1_1 = (in2->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    __m128d r1;
    r = _mm_loadu_pd(&in1_data[4 * aux_0_1]);
    r1 = _mm_loadu_pd(&in2_data[4 * aux_1_1]);
    _mm_storeu_pd(&b_in1_data[4 * i], _mm_sub_pd(r, r1));
    r = _mm_loadu_pd(&in1_data[4 * aux_0_1 + 2]);
    r1 = _mm_loadu_pd(&in2_data[4 * aux_1_1 + 2]);
    _mm_storeu_pd(&b_in1_data[4 * i + 2], _mm_sub_pd(r, r1));
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  i = in1->size[0] * in1->size[1];
  in1->size[0] = 4;
  in1->size[1] = b_in1->size[1];
  emxEnsureCapacity_real_T(sp, in1, i, &gl_emlrtRTEI);
  in1_data = in1->data;
  loop_ub = b_in1->size[1];
  for (i = 0; i < loop_ub; i++) {
    in1_data[4 * i] = b_in1_data[4 * i];
    stride_0_1 = 4 * i + 1;
    in1_data[stride_0_1] = b_in1_data[stride_0_1];
    stride_0_1 = 4 * i + 2;
    in1_data[stride_0_1] = b_in1_data[stride_0_1];
    stride_0_1 = 4 * i + 3;
    in1_data[stride_0_1] = b_in1_data[stride_0_1];
  }

  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void plannerHybridAStar_closeCell(const emlrtStack *sp,
  plannerHybridAStar *obj, real_T direction, const real_T Indice_data[], const
  int32_T Indice_size[2])
{
  int32_T i;
  if (direction == 1.0) {
    int32_T loop_ub;
    int16_T tmp_data[4];
    loop_ub = Indice_size[0] * Indice_size[1];
    for (i = 0; i < loop_ub; i++) {
      real_T d;
      d = Indice_data[i];
      if (d != (int32_T)muDoubleScalarFloor(d)) {
        emlrtIntegerCheckR2012b(d, &c_emlrtDCI, (emlrtConstCTX)sp);
      }

      if (((int16_T)d < 1) || ((int16_T)d > 10000)) {
        emlrtDynamicBoundsCheckR2012b((int16_T)d, 1, 10000, &n_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      tmp_data[i] = (int16_T)d;
    }

    loop_ub = Indice_size[0] * Indice_size[1];
    for (i = 0; i < loop_ub; i++) {
      obj->visitedCellsFront[tmp_data[i] - 1] = true;
    }
  } else {
    int32_T loop_ub;
    int16_T tmp_data[4];
    loop_ub = Indice_size[0] * Indice_size[1];
    for (i = 0; i < loop_ub; i++) {
      real_T d;
      d = Indice_data[i];
      if (d != (int32_T)muDoubleScalarFloor(d)) {
        emlrtIntegerCheckR2012b(d, &b_emlrtDCI, (emlrtConstCTX)sp);
      }

      if (((int16_T)d < 1) || ((int16_T)d > 10000)) {
        emlrtDynamicBoundsCheckR2012b((int16_T)d, 1, 10000, &m_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      tmp_data[i] = (int16_T)d;
    }

    loop_ub = Indice_size[0] * Indice_size[1];
    for (i = 0; i < loop_ub; i++) {
      obj->visitedCellsBack[tmp_data[i] - 1] = true;
    }
  }
}

static void plus(real_T in1_data[], int32_T *in1_size, const real_T in2_data[],
                 const int32_T *in2_size)
{
  real_T b_in1_data[9];
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  if (*in2_size == 1) {
    loop_ub = *in1_size;
  } else {
    loop_ub = *in2_size;
  }

  stride_0_0 = (*in1_size != 1);
  stride_1_0 = (*in2_size != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in1_data[i] = in1_data[i * stride_0_0] + in2_data[i * stride_1_0];
  }

  *in1_size = loop_ub;
  if (loop_ub - 1 >= 0) {
    memcpy(&in1_data[0], &b_in1_data[0], (uint32_T)loop_ub * sizeof(real_T));
  }
}

static void times(const emlrtStack *sp, emxArray_real_T *in1, const
                  emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, 2, &bk_emlrtRTEI);
  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[0] = 4;
  if (in2->size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in2->size[1];
  }

  b_in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, b_in1, i, &bk_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_1 = (in1->size[1] != 1);
  stride_1_1 = (in2->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    __m128d r1;
    r = _mm_loadu_pd(&in1_data[4 * aux_0_1]);
    r1 = _mm_loadu_pd(&in2_data[4 * aux_1_1]);
    _mm_storeu_pd(&b_in1_data[4 * i], _mm_mul_pd(r, r1));
    r = _mm_loadu_pd(&in1_data[4 * aux_0_1 + 2]);
    r1 = _mm_loadu_pd(&in2_data[4 * aux_1_1 + 2]);
    _mm_storeu_pd(&b_in1_data[4 * i + 2], _mm_mul_pd(r, r1));
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  i = in1->size[0] * in1->size[1];
  in1->size[0] = 4;
  in1->size[1] = b_in1->size[1];
  emxEnsureCapacity_real_T(sp, in1, i, &bk_emlrtRTEI);
  in1_data = in1->data;
  loop_ub = b_in1->size[1];
  for (i = 0; i < loop_ub; i++) {
    in1_data[4 * i] = b_in1_data[4 * i];
    stride_0_1 = 4 * i + 1;
    in1_data[stride_0_1] = b_in1_data[stride_0_1];
    stride_0_1 = 4 * i + 2;
    in1_data[stride_0_1] = b_in1_data[stride_0_1];
    stride_0_1 = 4 * i + 3;
    in1_data[stride_0_1] = b_in1_data[stride_0_1];
  }

  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

plannerHybridAStar *c_plannerHybridAStar_plannerHyb(const emlrtStack *sp,
  plannerHybridAStar *obj, validatorOccupancyMap *validator)
{
  static const int32_T iv[2] = { 1, 7 };

  static const int32_T iv1[2] = { 1, 7 };

  static const char_T rfmt[7] = { '%', '2', '3', '.', '1', '5', 'e' };

  static const char_T b[3] = { 'S', 'E', '2' };

  binaryOccupancyMap *c_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *m;
  const mxArray *y;
  plannerHybridAStar *b_obj;
  real_T t0_f7;
  int32_T j;
  int32_T ret;
  char_T numstr[23];
  char_T a[3];
  int8_T neighborX[9];
  int8_T neighborY[9];
  boolean_T unusedExpr[10000];
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
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  b_obj = obj;
  b_obj->MotionPrimitiveLength = 0.0;
  st.site = &vd_emlrtRSI;
  b_st.site = &wc_emlrtRSI;
  st.site = &wd_emlrtRSI;
  a[0] = validator->StateSpace->Name[0];
  a[1] = validator->StateSpace->Name[1];
  a[2] = validator->StateSpace->Name[2];
  ret = memcmp(&a[0], (char_T *)&b[0], 3);
  if (ret != 0) {
    emlrtErrorWithMessageIdR2018a(&st, &l_emlrtRTEI,
      "nav:navalgs:hybridastar:StateSpaceError",
      "nav:navalgs:hybridastar:StateSpaceError", 0);
  }

  b_obj->StateValidator = validator;
  b_st.site = &ae_emlrtRSI;
  b_obj->Map = b_obj->StateValidator->Map;
  b_obj->Dimensions[0] = 100.0;
  b_obj->Dimensions[1] = 100.0;
  b_st.site = &be_emlrtRSI;
  b_obj->PathFound = false;
  b_obj->StartPose[0] = rtNaN;
  b_obj->StartPose[1] = rtNaN;
  b_obj->StartPose[2] = rtNaN;
  b_obj->GoalPose[0] = rtNaN;
  b_obj->GoalPose[1] = rtNaN;
  b_obj->GoalPose[2] = rtNaN;
  st.site = &xd_emlrtRSI;
  b_st.site = &ce_emlrtRSI;
  c_st.site = &fe_emlrtRSI;
  t0_f7 = 2.0 * b_obj->MotionPrimitiveLength / 3.1415926535897931;
  d_st.site = &ge_emlrtRSI;
  e_st.site = &ab_emlrtRSI;
  if (!(t0_f7 <= 2.0)) {
    f_st.site = &he_emlrtRSI;
    g_st.site = &de_emlrtRSI;
    h_st.site = &ee_emlrtRSI;
    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(&h_st, 7, m, &rfmt[0]);
    emlrtAssign(&y, m);
    b_y = NULL;
    m = emlrtCreateDoubleScalar(t0_f7);
    emlrtAssign(&b_y, m);
    i_st.site = &rt_emlrtRSI;
    emlrt_marshallIn(&i_st, b_sprintf(&i_st, y, b_y, &emlrtMCI),
                     "<output of sprintf>", numstr);
    emlrtErrorWithMessageIdR2018a(&e_st, &n_emlrtRTEI,
      "MATLAB:validateattributes:expectedScalar",
      "MATLAB:plannerHybridAStar:notGreaterEqual", 9, 4, 16, "MinTurningRadius",
      4, 2, ">=", 4, 23, &numstr[0]);
  }

  b_obj->MinTurningRadius = 2.0;
  b_st.site = &ce_emlrtRSI;
  c_st.site = &ie_emlrtRSI;
  t0_f7 = 3.1415926535897931 * b_obj->MinTurningRadius / 2.0;
  d_st.site = &je_emlrtRSI;
  d_st.site = &ke_emlrtRSI;
  e_st.site = &ab_emlrtRSI;
  if (!(t0_f7 >= 2.0)) {
    f_st.site = &le_emlrtRSI;
    g_st.site = &de_emlrtRSI;
    h_st.site = &ee_emlrtRSI;
    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(&h_st, 7, m, &rfmt[0]);
    emlrtAssign(&c_y, m);
    d_y = NULL;
    m = emlrtCreateDoubleScalar(t0_f7);
    emlrtAssign(&d_y, m);
    i_st.site = &rt_emlrtRSI;
    emlrt_marshallIn(&i_st, b_sprintf(&i_st, c_y, d_y, &emlrtMCI),
                     "<output of sprintf>", numstr);
    emlrtErrorWithMessageIdR2018a(&e_st, &o_emlrtRTEI,
      "MATLAB:validateattributes:expectedScalar",
      "MATLAB:plannerHybridAStar:notLessEqual", 9, 4, 21,
      "MotionPrimitiveLength", 4, 2, "<=", 4, 23, &numstr[0]);
  }

  t0_f7 = b_obj->StateValidator->ValidationDistance;
  if ((!muDoubleScalarIsInf(t0_f7)) && (b_obj->
       StateValidator->ValidationDistance > 2.0)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &m_emlrtRTEI,
      "nav:navalgs:hybridastar:MotionPrimitiveLengthError",
      "nav:navalgs:hybridastar:MotionPrimitiveLengthError", 0);
  }

  b_obj->MotionPrimitiveLength = 2.0;
  b_st.site = &ce_emlrtRSI;
  b_obj->ForwardCost = 1.0;
  b_st.site = &ce_emlrtRSI;
  b_obj->ReverseCost = 3.0;
  b_st.site = &ce_emlrtRSI;
  b_obj->DirectionSwitchingCost = 0.0;
  b_st.site = &ce_emlrtRSI;
  b_obj->AnalyticExpansionInterval = 5.0;
  st.site = &yd_emlrtRSI;
  c_obj = b_obj->Map;
  b_st.site = &ud_emlrtRSI;
  MapLayer_getValueAllImpl(&b_st, c_obj, unusedExpr);
  for (j = 0; j < 3; j++) {
    neighborX[3 * j] = (int8_T)(j - 1);
    neighborY[3 * j] = -1;
    ret = 3 * j + 1;
    neighborX[ret] = (int8_T)(j - 1);
    neighborY[ret] = 0;
    ret = 3 * j + 2;
    neighborX[ret] = (int8_T)(j - 1);
    neighborY[ret] = 1;
  }

  for (ret = 0; ret < 9; ret++) {
    b_obj->Neighbors[ret] = neighborX[ret];
  }

  for (ret = 0; ret < 9; ret++) {
    b_obj->Neighbors[ret + 9] = neighborY[ret];
  }

  return b_obj;
}

navPath *plannerHybridAStar_plan(codegenPathPlannerStackData *SD, const
  emlrtStack *sp, plannerHybridAStar *obj, const real_T start[3], const real_T
  goal[3], plannerAStarGrid *iobj_0, navPath *iobj_1)
{
  binaryOccupancyMap *b_obj;
  c_robotics_core_internal_NameVa showParser;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emlrtStack *r;
  emxArray_real_T b_validPrimitives_data;
  emxArray_real_T *directions;
  emxArray_real_T *path;
  emxArray_real_T *pathData;
  emxArray_real_T *pathPoses1;
  emxArray_real_T *varargin_1;
  emxArray_real_T *varargin_5;
  navPath *pathObj;
  nav_algs_internal_NodeMap nodeMap;
  nav_algs_internal_PriorityQueue openSet;
  validatorOccupancyMap *c_obj;
  real_T result_data[44];
  real_T ICRsData_data[12];
  real_T newNodesPoses_data[12];
  real_T currentNode_data[7];
  real_T curvature_data[5];
  real_T gScore_data[4];
  real_T hScore_data[4];
  real_T newNodesPosesGridIndices_data[4];
  real_T validCurvature_data[4];
  real_T validPrimitives_data[4];
  real_T gScore;
  real_T hScore;
  real_T *varargin_1_data;
  real_T *varargin_5_data;
  int32_T ICRsData_size[2];
  int32_T result_size[2];
  int32_T searchMode_size[2];
  int32_T direction;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i7;
  int32_T loop_ub;
  int32_T validPrimitive;
  int32_T vectorUB;
  char_T searchMode_data[10];
  boolean_T b_start[3];
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
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &pathPoses1, 2, &je_emlrtRTEI);
  emxInit_real_T(sp, &pathData, 2, &ke_emlrtRTEI);
  emxInit_real_T(sp, &path, 2, &le_emlrtRTEI);
  emxInit_real_T(sp, &directions, 1, &le_emlrtRTEI);
  emxInit_real_T(sp, &varargin_1, 2, &me_emlrtRTEI);
  emxInit_real_T(sp, &varargin_5, 1, &ne_emlrtRTEI);
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &openSet, (void *)
    &b_handle_matlabCodegenDestructo, NULL, NULL, NULL);
  openSet.matlabCodegenIsDeleted = true;
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &nodeMap, (void *)
    &handle_matlabCodegenDestructor, NULL, NULL, NULL);
  nodeMap.matlabCodegenIsDeleted = true;
  st.site = &me_emlrtRSI;
  c_plannerHybridAStar_validateSt(&st, obj, start, goal);
  st.site = &ne_emlrtRSI;
  for (i = 0; i < 550000; i++) {
    obj->PrimitivesData[i] = rtNaN;
  }

  for (i = 0; i < 350000; i++) {
    obj->LinesData[i] = rtNaN;
  }

  st.site = &oe_emlrtRSI;
  NameValueParser_NameValueParser(&showParser);
  st.site = &pe_emlrtRSI;
  showParser.ParsedResults[0] = showParser.Defaults[0];
  st.site = &qe_emlrtRSI;
  st.site = &qe_emlrtRSI;
  validatestring(&st, showParser.ParsedResults[0].f1, searchMode_data,
                 searchMode_size);
  st.site = &re_emlrtRSI;
  obj->Map = obj->StateValidator->Map;
  obj->Dimensions[0] = 100.0;
  obj->Dimensions[1] = 100.0;
  obj->StateValidator->SkipStateValidation = true;
  st.site = &se_emlrtRSI;
  c_validatorOccupancyMap_configu(&st, obj->StateValidator);
  obj->PathFound = false;
  path->size[0] = 0;
  path->size[1] = 3;
  b_start[0] = (start[0] == goal[0]);
  b_start[1] = (start[1] == goal[1]);
  b_start[2] = (start[2] == goal[2]);
  if (ifWhileCond(b_start)) {
    i = path->size[0] * path->size[1];
    path->size[0] = 1;
    path->size[1] = 3;
    emxEnsureCapacity_real_T(sp, path, i, &ie_emlrtRTEI);
    varargin_1_data = path->data;
    varargin_1_data[0] = start[0];
    varargin_1_data[1] = start[1];
    varargin_1_data[2] = start[2];
  } else {
    real_T currentNodeGrid[2];
    real_T d;
    real_T primitivesDataRow;
    int32_T linesDataRow;
    primitivesDataRow = 1.0;
    linesDataRow = 1;
    for (i = 0; i < 10000; i++) {
      obj->visitedCellsFront[i] = false;
    }

    for (i = 0; i < 10000; i++) {
      obj->visitedCellsBack[i] = false;
    }

    real_T pos[2];
    st.site = &te_emlrtRSI;
    b_obj = obj->Map;
    pos[0] = obj->GoalPose[0];
    pos[1] = obj->GoalPose[1];
    b_st.site = &sh_emlrtRSI;
    MapInterface_world2gridImpl(b_obj, pos, currentNodeGrid);
    obj->GoalPoint[0] = currentNodeGrid[0];
    obj->GoalPoint[1] = currentNodeGrid[1];
    st.site = &ue_emlrtRSI;
    obj->Heuristic2DObj = c_plannerAStarGrid_plannerAStar(&st, iobj_0, obj->Map);
    for (i = 0; i < 10000; i++) {
      obj->Heuristic2DMat[i] = rtInf;
    }

    st.site = &ve_emlrtRSI;
    PriorityQueue_PriorityQueue(&openSet);
    st.site = &we_emlrtRSI;
    NodeMap_NodeMap(&nodeMap);
    st.site = &xe_emlrtRSI;
    r = &st;
    currentNodeGrid[0] = obj->StartPose[0];
    currentNodeGrid[1] = obj->StartPose[1];
    hScore = c_plannerHybridAStar_get2DHeuri(SD, r, obj, currentNodeGrid);
    b_st.site = &xe_emlrtRSI;
    d = c_plannerHybridAStar_get3DHeuri(obj, obj->StartPose, obj->GoalPose);
    if ((hScore < d) || (muDoubleScalarIsNaN(hScore) && (!muDoubleScalarIsNaN(d))))
    {
      hScore = d;
    }

    if (!(hScore == rtInf)) {
      real_T initNode[7];
      real_T dv[5];
      real_T numIterations;
      real_T stepSize;
      initNode[0] = hScore;
      initNode[1] = 0.0;
      initNode[2] = hScore;
      initNode[3] = obj->StartPose[0];
      initNode[4] = obj->StartPose[1];
      initNode[5] = obj->StartPose[2];
      initNode[6] = 0.0;
      c_st.site = &ye_emlrtRSI;
      priorityqueuecodegen_push(openSet.PQInternal, &initNode[0]);
      c_st.site = &af_emlrtRSI;
      NodeMap_insertNode(&c_st, &nodeMap, obj->StartPose, 0.0);
      linspace(-1.0 / obj->MinTurningRadius, 1.0 / obj->MinTurningRadius, dv);
      for (i = 0; i < 5; i++) {
        curvature_data[i] = dv[i];
      }

      curvature_data[2] = curvature_data[3];
      curvature_data[3] = curvature_data[4];
      hScore = obj->StateValidator->ValidationDistance;
      if (muDoubleScalarIsInf(hScore)) {
        stepSize = 1.0;
        c_st.site = &bf_emlrtRSI;
        c_obj = obj->StateValidator;
        c_obj->ValidationDistance = 1.0;
      } else {
        stepSize = obj->StateValidator->ValidationDistance;
      }

      hScore = obj->MotionPrimitiveLength / stepSize;
      hScore = muDoubleScalarFloor(hScore);
      obj->NumPointsMotionPrimitive = hScore + 2.0;
      numIterations = 0.0;
      int32_T exitg1;
      do {
        exitg1 = 0;
        c_st.site = &cf_emlrtRSI;
        if (PriorityQueue_isEmpty(&c_st, &openSet) || obj->PathFound) {
          exitg1 = 1;
        } else {
          real_T currentNodePose[3];
          real_T newNodePose[3];
          real_T currentNodeId;
          int32_T currentNode_size[2];
          boolean_T guard1;
          boolean_T result;
          c_st.site = &df_emlrtRSI;
          currentNodeId = b_PriorityQueue_top(&c_st, &openSet, currentNode_data,
            currentNode_size);
          if (currentNode_size[1] < 4) {
            emlrtDynamicBoundsCheckR2012b(4, 1, currentNode_size[1], &p_emlrtBCI,
              &b_st);
          }

          currentNodePose[0] = currentNode_data[3];
          if (currentNode_size[1] < 5) {
            emlrtDynamicBoundsCheckR2012b(5, 1, 4, &p_emlrtBCI, &b_st);
          }

          currentNodePose[1] = currentNode_data[4];
          if (currentNode_size[1] < 6) {
            emlrtDynamicBoundsCheckR2012b(6, 1, 5, &p_emlrtBCI, &b_st);
          }

          currentNodePose[2] = currentNode_data[5];
          c_st.site = &ef_emlrtRSI;
          priorityqueuecodegen_pop(openSet.PQInternal);
          numIterations++;
          if (b_strcmp(searchMode_data, searchMode_size)) {
            c_st.site = &ff_emlrtRSI;
            b_obj = obj->Map;
            d_st.site = &sh_emlrtRSI;
            MapInterface_world2gridImpl(b_obj, &currentNode_data[3],
              currentNodeGrid);
            hScore = (currentNodeGrid[1] - 1.0) * obj->Dimensions[0] +
              currentNodeGrid[0];
            if (currentNode_size[1] < 7) {
              emlrtDynamicBoundsCheckR2012b(7, 1, 6, &g_emlrtBCI, &b_st);
            }

            c_st.site = &gf_emlrtRSI;
            if (currentNode_data[6] == 1.0) {
              if (hScore != (int32_T)muDoubleScalarFloor(hScore)) {
                emlrtIntegerCheckR2012b(hScore, &c_emlrtDCI, &c_st);
              }

              if (((int32_T)hScore < 1) || ((int32_T)hScore > 10000)) {
                emlrtDynamicBoundsCheckR2012b((int32_T)hScore, 1, 10000,
                  &n_emlrtBCI, &c_st);
              }

              obj->visitedCellsFront[(int32_T)hScore - 1] = true;
            } else {
              if (hScore != (int32_T)muDoubleScalarFloor(hScore)) {
                emlrtIntegerCheckR2012b(hScore, &b_emlrtDCI, &c_st);
              }

              if (((int32_T)hScore < 1) || ((int32_T)hScore > 10000)) {
                emlrtDynamicBoundsCheckR2012b((int32_T)hScore, 1, 10000,
                  &m_emlrtBCI, &c_st);
              }

              obj->visitedCellsBack[(int32_T)hScore - 1] = true;
            }
          }

          hScore = obj->AnalyticExpansionInterval;
          guard1 = false;
          if (muDoubleScalarRem(numIterations, hScore) == 0.0) {
            newNodePose[0] = obj->GoalPose[0];
            newNodePose[1] = obj->GoalPose[1];
            newNodePose[2] = obj->GoalPose[2];
            c_st.site = &hf_emlrtRSI;
            result = c_plannerHybridAStar_checkAnaly(&c_st, obj, currentNodePose,
              newNodePose, stepSize);
            if (result) {
              boolean_T b_path[2];
              if (primitivesDataRow > 50000.0) {
                i = -1;
                i1 = -1;
              } else {
                if (((int32_T)primitivesDataRow < 1) || ((int32_T)
                     primitivesDataRow > 50000)) {
                  emlrtDynamicBoundsCheckR2012b((int32_T)primitivesDataRow, 1,
                    50000, &h_emlrtBCI, &b_st);
                }

                i = (int32_T)primitivesDataRow - 2;
                i1 = 49999;
              }

              loop_ub = i1 - i;
              for (i1 = 0; i1 < 11; i1++) {
                for (i2 = 0; i2 < loop_ub; i2++) {
                  obj->PrimitivesData[((i + i2) + 50000 * i1) + 1] = rtNaN;
                }
              }

              if (linesDataRow > 50000) {
                i = -1;
                i1 = -1;
              } else {
                i = linesDataRow - 2;
                i1 = 49999;
              }

              loop_ub = i1 - i;
              for (i1 = 0; i1 < 7; i1++) {
                for (i2 = 0; i2 < loop_ub; i2++) {
                  obj->LinesData[((i + i2) + 50000 * i1) + 1] = rtNaN;
                }
              }

              c_st.site = &if_emlrtRSI;
              NodeMap_traceBack(&c_st, &nodeMap, currentNodeId, pathPoses1);
              c_st.site = &jf_emlrtRSI;
              flipud(pathPoses1);
              c_st.site = &kf_emlrtRSI;
              c_plannerHybridAStar_getFinalPa(SD, &c_st, obj, pathPoses1,
                pathData);
              c_st.site = &lf_emlrtRSI;
              c_plannerHybridAStar_getInterpo(&c_st, obj, pathData, path,
                directions);
              b_path[0] = (path->size[0] <= 50000);
              b_path[1] = true;
              if (!all(b_path)) {
                emlrtErrorWithMessageIdR2018a(&b_st, &p_emlrtRTEI,
                  "nav:navalgs:hybridastar:AssertionFailedLessThan",
                  "nav:navalgs:hybridastar:AssertionFailedLessThan", 6, 4, 30,
                  "Number of states in final path", 4, 55,
                  "Map.GridSize(1) * Map.GridSize(1) * NumMotionPrimitives");
              }

              b_path[0] = (directions->size[0] <= 50000);
              b_path[1] = true;
              if (!all(b_path)) {
                emlrtErrorWithMessageIdR2018a(&b_st, &q_emlrtRTEI,
                  "nav:navalgs:hybridastar:AssertionFailedLessThan",
                  "nav:navalgs:hybridastar:AssertionFailedLessThan", 6, 4, 33,
                  "Number of direction in final path", 4, 55,
                  "Map.GridSize(1) * Map.GridSize(1) * NumMotionPrimitives");
              }

              obj->PathFound = true;
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }

          if (guard1) {
            real_T d1;
            d = muDoubleScalarCos(currentNode_data[5]);
            d1 = muDoubleScalarSin(currentNode_data[5]);
            for (direction = 0; direction < 2; direction++) {
              __m128d r2;
              int32_T newNodesPosesGridIndices_size[2];
              int32_T newNodesPoses_size[2];
              int32_T b_direction;
              int32_T numValidPrimitives;
              int32_T validCurvature_size;
              boolean_T invalidPrimitives_data[4];
              b_direction = direction * -2 + 1;
              c_st.site = &mf_emlrtRSI;
              c_plannerHybridAStar_getCircula(obj->MotionPrimitiveLength,
                curvature_data, currentNodePose, b_direction, newNodesPoses_data,
                newNodesPoses_size, ICRsData_data, ICRsData_size);
              for (i = 0; i <= 2; i += 2) {
                r2 = _mm_loadu_pd(&curvature_data[i]);
                _mm_storeu_pd(&validCurvature_data[i], _mm_div_pd(_mm_set1_pd
                  (1.0), r2));
              }

              c_st.site = &nf_emlrtRSI;
              validPrimitive = c_plannerHybridAStar_isCircular(&c_st, obj,
                currentNodePose, newNodesPoses_data, ICRsData_data,
                validCurvature_data, obj->MotionPrimitiveLength, stepSize,
                b_direction, validPrimitives_data, newNodesPosesGridIndices_data,
                newNodesPosesGridIndices_size);
              b_validPrimitives_data.data = &validPrimitives_data[0];
              c_st.site = &of_emlrtRSI;
              i = (&validPrimitive)[0];
              for (validPrimitive = 0; validPrimitive < i; validPrimitive++) {
                if (muDoubleScalarIsNaN
                    (b_validPrimitives_data.data[validPrimitive])) {
                  emlrtErrorWithMessageIdR2018a(&c_st, &t_emlrtRTEI,
                    "MATLAB:nologicalnan", "MATLAB:nologicalnan", 0);
                }
              }

              for (i = 0; i < 4; i++) {
                invalidPrimitives_data[i] = !(validPrimitives_data[i] != 0.0);
              }

              c_st.site = &pf_emlrtRSI;
              b_nullAssignment(newNodesPoses_data, newNodesPoses_size,
                               invalidPrimitives_data);
              c_st.site = &qf_emlrtRSI;
              b_nullAssignment(ICRsData_data, ICRsData_size,
                               invalidPrimitives_data);
              validCurvature_data[0] = curvature_data[0];
              validCurvature_data[1] = curvature_data[1];
              validCurvature_data[2] = curvature_data[2];
              validCurvature_data[3] = curvature_data[3];
              c_st.site = &uf_emlrtRSI;
              validCurvature_size = c_nullAssignment(validCurvature_data,
                invalidPrimitives_data);
              c_st.site = &rf_emlrtRSI;
              validPrimitive = d_nullAssignment(validPrimitives_data,
                invalidPrimitives_data);
              numValidPrimitives = intnnz(validPrimitives_data, validPrimitive);
              c_st.site = &sf_emlrtRSI;
              if (numValidPrimitives != 0) {
                int32_T b_result;
                int8_T i3;
                int8_T i4;
                int8_T i5;
                int8_T i6;
                int8_T sizes_idx_1;
                if (numValidPrimitives > 50000) {
                  emlrtErrorWithMessageIdR2018a(&b_st, &r_emlrtRTEI,
                    "nav:navalgs:hybridastar:AssertionFailedLessThan",
                    "nav:navalgs:hybridastar:AssertionFailedLessThan", 6, 4, 18,
                    "NumValidPrimitives", 4, 55,
                    "Map.GridSize(1) * Map.GridSize(1) * NumMotionPrimitives");
                }

                hScore = primitivesDataRow + (real_T)numValidPrimitives;
                if (primitivesDataRow > hScore - 1.0) {
                  i = 0;
                  i1 = 0;
                } else {
                  if (((int32_T)primitivesDataRow < 1) || ((int32_T)
                       primitivesDataRow > 50000)) {
                    emlrtDynamicBoundsCheckR2012b((int32_T)primitivesDataRow, 1,
                      50000, &i_emlrtBCI, &b_st);
                  }

                  i = (int32_T)primitivesDataRow - 1;
                  if (((int32_T)(hScore - 1.0) < 1) || ((int32_T)(hScore - 1.0) >
                       50000)) {
                    emlrtDynamicBoundsCheckR2012b((int32_T)(hScore - 1.0), 1,
                      50000, &j_emlrtBCI, &b_st);
                  }

                  i1 = (int32_T)(hScore - 1.0);
                }

                validPrimitive = (validCurvature_size / 2) << 1;
                vectorUB = validPrimitive - 2;
                for (i2 = 0; i2 <= vectorUB; i2 += 2) {
                  r2 = _mm_loadu_pd(&validCurvature_data[i2]);
                  _mm_storeu_pd(&validPrimitives_data[i2], _mm_div_pd
                                (_mm_set1_pd(1.0), r2));
                }

                for (i2 = validPrimitive; i2 < validCurvature_size; i2++) {
                  validPrimitives_data[i2] = 1.0 / validCurvature_data[i2];
                }

                c_st.site = &tf_emlrtRSI;
                d_st.site = &tf_emlrtRSI;
                c_repmat(&d_st, currentNodePose, numValidPrimitives, varargin_1);
                varargin_1_data = varargin_1->data;
                d_st.site = &vf_emlrtRSI;
                d_repmat(&d_st, b_direction, numValidPrimitives, varargin_5);
                varargin_5_data = varargin_5->data;
                d_st.site = &es_emlrtRSI;
                if (varargin_1->size[0] != 0) {
                  b_result = varargin_1->size[0];
                } else if (newNodesPoses_size[0] != 0) {
                  b_result = newNodesPoses_size[0];
                } else if (ICRsData_size[0] != 0) {
                  b_result = ICRsData_size[0];
                } else if (validCurvature_size != 0) {
                  b_result = validCurvature_size;
                } else if (varargin_5->size[0] != 0) {
                  b_result = varargin_5->size[0];
                } else {
                  b_result = 0;
                }

                e_st.site = &ln_emlrtRSI;
                if ((varargin_1->size[0] != b_result) && (varargin_1->size[0] !=
                     0)) {
                  emlrtErrorWithMessageIdR2018a(&e_st, &s_emlrtRTEI,
                    "MATLAB:catenate:matrixDimensionMismatch",
                    "MATLAB:catenate:matrixDimensionMismatch", 0);
                }

                if ((newNodesPoses_size[0] != b_result) && (newNodesPoses_size[0]
                     != 0)) {
                  emlrtErrorWithMessageIdR2018a(&e_st, &s_emlrtRTEI,
                    "MATLAB:catenate:matrixDimensionMismatch",
                    "MATLAB:catenate:matrixDimensionMismatch", 0);
                }

                if ((ICRsData_size[0] != b_result) && (ICRsData_size[0] != 0)) {
                  emlrtErrorWithMessageIdR2018a(&e_st, &s_emlrtRTEI,
                    "MATLAB:catenate:matrixDimensionMismatch",
                    "MATLAB:catenate:matrixDimensionMismatch", 0);
                }

                if ((validCurvature_size != b_result) && (validCurvature_size !=
                     0)) {
                  emlrtErrorWithMessageIdR2018a(&e_st, &s_emlrtRTEI,
                    "MATLAB:catenate:matrixDimensionMismatch",
                    "MATLAB:catenate:matrixDimensionMismatch", 0);
                }

                if ((varargin_5->size[0] != b_result) && (varargin_5->size[0] !=
                     0)) {
                  emlrtErrorWithMessageIdR2018a(&e_st, &s_emlrtRTEI,
                    "MATLAB:catenate:matrixDimensionMismatch",
                    "MATLAB:catenate:matrixDimensionMismatch", 0);
                }

                result = (b_result == 0);
                if (result || (varargin_1->size[0] != 0)) {
                  i3 = 3;
                } else {
                  i3 = 0;
                }

                if (result || (newNodesPoses_size[0] != 0)) {
                  i4 = 3;
                } else {
                  i4 = 0;
                }

                if (result || (ICRsData_size[0] != 0)) {
                  i5 = 3;
                } else {
                  i5 = 0;
                }

                if (result || (validCurvature_size != 0)) {
                  i6 = 1;
                } else {
                  i6 = 0;
                }

                if (result || (varargin_5->size[0] != 0)) {
                  sizes_idx_1 = 1;
                } else {
                  sizes_idx_1 = 0;
                }

                result_size[0] = b_result;
                i2 = (i3 + i4) + i5;
                validPrimitive = i2 + i6;
                result_size[1] = validPrimitive + sizes_idx_1;
                loop_ub = i3;
                for (vectorUB = 0; vectorUB < loop_ub; vectorUB++) {
                  for (i7 = 0; i7 < b_result; i7++) {
                    result_data[i7 + b_result * vectorUB] = varargin_1_data[i7 +
                      b_result * vectorUB];
                  }
                }

                loop_ub = i4;
                for (vectorUB = 0; vectorUB < loop_ub; vectorUB++) {
                  for (i7 = 0; i7 < b_result; i7++) {
                    result_data[i7 + b_result * (vectorUB + i3)] =
                      newNodesPoses_data[i7 + b_result * vectorUB];
                  }
                }

                loop_ub = i5;
                for (vectorUB = 0; vectorUB < loop_ub; vectorUB++) {
                  for (i7 = 0; i7 < b_result; i7++) {
                    result_data[i7 + b_result * ((vectorUB + i3) + i4)] =
                      ICRsData_data[i7 + b_result * vectorUB];
                  }
                }

                loop_ub = i6;
                for (vectorUB = 0; vectorUB < loop_ub; vectorUB++) {
                  for (i7 = 0; i7 < b_result; i7++) {
                    result_data[i7 + b_result * i2] = validPrimitives_data[i7];
                  }
                }

                loop_ub = sizes_idx_1;
                for (i2 = 0; i2 < loop_ub; i2++) {
                  for (vectorUB = 0; vectorUB < b_result; vectorUB++) {
                    result_data[vectorUB + b_result * validPrimitive] =
                      varargin_5_data[vectorUB];
                  }
                }

                ICRsData_size[0] = i1 - i;
                ICRsData_size[1] = 11;
                emlrtSubAssignSizeCheckR2012b(&ICRsData_size[0], 2,
                  &result_size[0], 2, &f_emlrtECI, &b_st);
                loop_ub = result_size[1];
                for (i1 = 0; i1 < loop_ub; i1++) {
                  for (i2 = 0; i2 < b_result; i2++) {
                    obj->PrimitivesData[(i + i2) + 50000 * i1] = result_data[i2
                      + b_result * i1];
                  }
                }

                primitivesDataRow = hScore;
                c_st.site = &wf_emlrtRSI;
                validPrimitive = c_plannerHybridAStar_calculateC(SD, &c_st, obj,
                  newNodesPoses_data, newNodesPoses_size, currentNode_data,
                  currentNode_size, validCurvature_data, validCurvature_size,
                  b_direction, validPrimitives_data, gScore_data, &vectorUB,
                  hScore_data, &loop_ub);
                c_st.site = &xf_emlrtRSI;
                d_st.site = &xf_emlrtRSI;
                d_repmat(&d_st, b_direction, numValidPrimitives, varargin_5);
                varargin_5_data = varargin_5->data;
                d_st.site = &es_emlrtRSI;
                if (validPrimitive != 0) {
                  b_result = validPrimitive;
                } else if (vectorUB != 0) {
                  b_result = vectorUB;
                } else if (loop_ub != 0) {
                  b_result = loop_ub;
                } else if (newNodesPoses_size[0] != 0) {
                  b_result = newNodesPoses_size[0];
                } else if (varargin_5->size[0] != 0) {
                  b_result = varargin_5->size[0];
                } else {
                  b_result = 0;
                }

                e_st.site = &ln_emlrtRSI;
                if ((validPrimitive != b_result) && (validPrimitive != 0)) {
                  emlrtErrorWithMessageIdR2018a(&e_st, &s_emlrtRTEI,
                    "MATLAB:catenate:matrixDimensionMismatch",
                    "MATLAB:catenate:matrixDimensionMismatch", 0);
                }

                if ((vectorUB != b_result) && (vectorUB != 0)) {
                  emlrtErrorWithMessageIdR2018a(&e_st, &s_emlrtRTEI,
                    "MATLAB:catenate:matrixDimensionMismatch",
                    "MATLAB:catenate:matrixDimensionMismatch", 0);
                }

                if ((loop_ub != b_result) && (loop_ub != 0)) {
                  emlrtErrorWithMessageIdR2018a(&e_st, &s_emlrtRTEI,
                    "MATLAB:catenate:matrixDimensionMismatch",
                    "MATLAB:catenate:matrixDimensionMismatch", 0);
                }

                if ((newNodesPoses_size[0] != b_result) && (newNodesPoses_size[0]
                     != 0)) {
                  emlrtErrorWithMessageIdR2018a(&e_st, &s_emlrtRTEI,
                    "MATLAB:catenate:matrixDimensionMismatch",
                    "MATLAB:catenate:matrixDimensionMismatch", 0);
                }

                if ((varargin_5->size[0] != b_result) && (varargin_5->size[0] !=
                     0)) {
                  emlrtErrorWithMessageIdR2018a(&e_st, &s_emlrtRTEI,
                    "MATLAB:catenate:matrixDimensionMismatch",
                    "MATLAB:catenate:matrixDimensionMismatch", 0);
                }

                result = (b_result == 0);
                if (result || (validPrimitive != 0)) {
                  i3 = 1;
                } else {
                  i3 = 0;
                }

                if (result || (vectorUB != 0)) {
                  i4 = 1;
                } else {
                  i4 = 0;
                }

                if (result || (loop_ub != 0)) {
                  i5 = 1;
                } else {
                  i5 = 0;
                }

                if (result || (newNodesPoses_size[0] != 0)) {
                  i6 = 3;
                } else {
                  i6 = 0;
                }

                if (result || (varargin_5->size[0] != 0)) {
                  sizes_idx_1 = 1;
                } else {
                  sizes_idx_1 = 0;
                }

                i = i3 + i4;
                i1 = (i + i5) + i6;
                result_size[1] = i1 + sizes_idx_1;
                loop_ub = i3;
                for (i2 = 0; i2 < loop_ub; i2++) {
                  if (b_result - 1 >= 0) {
                    memcpy(&result_data[0], &validPrimitives_data[0], (uint32_T)
                           b_result * sizeof(real_T));
                  }
                }

                loop_ub = i4;
                if (loop_ub - 1 >= 0) {
                  for (i2 = 0; i2 < b_result; i2++) {
                    result_data[i2 + b_result * i3] = gScore_data[i2];
                  }
                }

                loop_ub = i5;
                if (loop_ub - 1 >= 0) {
                  for (i2 = 0; i2 < b_result; i2++) {
                    result_data[i2 + b_result * i] = hScore_data[i2];
                  }
                }

                loop_ub = i6;
                for (i = 0; i < loop_ub; i++) {
                  for (i2 = 0; i2 < b_result; i2++) {
                    result_data[i2 + b_result * (((i + i3) + i4) + i5)] =
                      newNodesPoses_data[i2 + b_result * i];
                  }
                }

                loop_ub = sizes_idx_1;
                for (i = 0; i < loop_ub; i++) {
                  for (i2 = 0; i2 < b_result; i2++) {
                    result_data[i2 + b_result * i1] = varargin_5_data[i2];
                  }
                }

                for (validPrimitive = 0; validPrimitive < numValidPrimitives;
                     validPrimitive++) {
                  c_st.site = &yf_emlrtRSI;
                  if (((int32_T)((uint32_T)validPrimitive + 1U) < 1) ||
                      ((int32_T)((uint32_T)validPrimitive + 1U) > b_result)) {
                    emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)
                      validPrimitive + 1U), 1, b_result, &k_emlrtBCI, &c_st);
                  }

                  loop_ub = result_size[1];
                  for (i = 0; i < loop_ub; i++) {
                    initNode[i] = result_data[validPrimitive + b_result * i];
                  }

                  priorityqueuecodegen_push(openSet.PQInternal, &initNode[0]);
                  if (result_size[1] < 4) {
                    emlrtDynamicBoundsCheckR2012b(4, 1, result_size[1],
                      &o_emlrtBCI, &b_st);
                  }

                  newNodePose[0] = result_data[validPrimitive + b_result * 3];
                  if (result_size[1] < 5) {
                    emlrtDynamicBoundsCheckR2012b(5, 1, 4, &o_emlrtBCI, &b_st);
                  }

                  newNodePose[1] = result_data[validPrimitive + b_result * 4];
                  if (result_size[1] < 6) {
                    emlrtDynamicBoundsCheckR2012b(6, 1, 5, &o_emlrtBCI, &b_st);
                  }

                  newNodePose[2] = result_data[validPrimitive + b_result * 5];
                  c_st.site = &ag_emlrtRSI;
                  NodeMap_insertNode(&c_st, &nodeMap, newNodePose, currentNodeId);
                }
              }

              hScore = obj->MotionPrimitiveLength;
              hScore *= (real_T)b_direction;
              newNodePose[0] = currentNode_data[3] + hScore * d;
              newNodePose[1] = currentNode_data[4] + hScore * d1;
              newNodePose[2] = currentNode_data[5];
              c_st.site = &bg_emlrtRSI;
              b_obj = obj->Map;
              d_st.site = &sh_emlrtRSI;
              MapInterface_world2gridImpl(b_obj, &newNodePose[0],
                currentNodeGrid);
              c_st.site = &cg_emlrtRSI;
              hScore = d_plannerHybridAStar_checkNodeV(&c_st, obj,
                currentNodeGrid, b_direction);
              validPrimitive = 0;
              c_st.site = &dg_emlrtRSI;
              if (muDoubleScalarIsNaN(hScore)) {
                emlrtErrorWithMessageIdR2018a(&c_st, &t_emlrtRTEI,
                  "MATLAB:nologicalnan", "MATLAB:nologicalnan", 0);
              }

              c_st.site = &dg_emlrtRSI;
              if ((hScore != 0.0) && c_validatorOccupancyMap_isMotio(&c_st,
                   obj->StateValidator, currentNodePose, newNodePose)) {
                real_T fScore;
                validPrimitive = 1;
                if (linesDataRow > 50000) {
                  emlrtDynamicBoundsCheckR2012b(50001, 1, 50000, &l_emlrtBCI,
                    &b_st);
                }

                obj->LinesData[linesDataRow - 1] = currentNodePose[0];
                obj->LinesData[linesDataRow + 49999] = currentNodePose[1];
                obj->LinesData[linesDataRow + 99999] = currentNodePose[2];
                obj->LinesData[linesDataRow + 149999] = newNodePose[0];
                obj->LinesData[linesDataRow + 199999] = newNodePose[1];
                obj->LinesData[linesDataRow + 249999] = currentNode_data[5];
                obj->LinesData[linesDataRow + 299999] = b_direction;
                linesDataRow++;
                c_st.site = &eg_emlrtRSI;
                fScore = d_plannerHybridAStar_calculateC(SD, &c_st, obj,
                  newNodePose, currentNode_data, currentNode_size, b_direction,
                  &gScore, &hScore);
                initNode[0] = fScore;
                initNode[1] = gScore;
                initNode[2] = hScore;
                initNode[3] = newNodePose[0];
                initNode[4] = newNodePose[1];
                initNode[5] = currentNode_data[5];
                initNode[6] = b_direction;
                c_st.site = &fg_emlrtRSI;
                priorityqueuecodegen_push(openSet.PQInternal, &initNode[0]);
                c_st.site = &gg_emlrtRSI;
                NodeMap_insertNode(&c_st, &nodeMap, &initNode[3], currentNodeId);
              }

              if (c_strcmp(searchMode_data, searchMode_size)) {
                if (numValidPrimitives != 0) {
                  c_st.site = &hg_emlrtRSI;
                  plannerHybridAStar_closeCell(&c_st, obj, b_direction,
                    newNodesPosesGridIndices_data, newNodesPosesGridIndices_size);
                }

                c_st.site = &ig_emlrtRSI;
                if (validPrimitive != 0) {
                  hScore = (currentNodeGrid[1] - 1.0) * obj->Dimensions[0] +
                    currentNodeGrid[0];
                  c_st.site = &jg_emlrtRSI;
                  if (b_direction == 1) {
                    if (hScore != (int32_T)muDoubleScalarFloor(hScore)) {
                      emlrtIntegerCheckR2012b(hScore, &c_emlrtDCI, &c_st);
                    }

                    if (((int32_T)hScore < 1) || ((int32_T)hScore > 10000)) {
                      emlrtDynamicBoundsCheckR2012b((int32_T)hScore, 1, 10000,
                        &n_emlrtBCI, &c_st);
                    }

                    obj->visitedCellsFront[(int32_T)hScore - 1] = true;
                  } else {
                    if (hScore != (int32_T)muDoubleScalarFloor(hScore)) {
                      emlrtIntegerCheckR2012b(hScore, &b_emlrtDCI, &c_st);
                    }

                    if (((int32_T)hScore < 1) || ((int32_T)hScore > 10000)) {
                      emlrtDynamicBoundsCheckR2012b((int32_T)hScore, 1, 10000,
                        &m_emlrtBCI, &c_st);
                    }

                    obj->visitedCellsBack[(int32_T)hScore - 1] = true;
                  }
                }
              }
            }
          }
        }
      } while (exitg1 == 0);
    }
  }

  st.site = &kg_emlrtRSI;
  pathObj = navPath_navPath(&st, iobj_1, obj->StateValidator->StateSpace, path);
  st.site = &lg_emlrtRSI;
  handle_matlabCodegenDestructor(&st, &nodeMap);
  st.site = &lg_emlrtRSI;
  b_handle_matlabCodegenDestructo(&st, &openSet);
  emxFree_real_T(sp, &varargin_5);
  emxFree_real_T(sp, &varargin_1);
  emxFree_real_T(sp, &directions);
  emxFree_real_T(sp, &path);
  emxFree_real_T(sp, &pathData);
  emxFree_real_T(sp, &pathPoses1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return pathObj;
}

/* End of code generation (plannerHybridAStar.c) */
