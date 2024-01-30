/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * codegenPathPlanner_data.c
 *
 * Code generation for function 'codegenPathPlanner_data'
 *
 */

/* Include files */
#include "codegenPathPlanner_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131642U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "codegenPathPlanner",                                 /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2296190603U, 1708147532U, 3288858490U, 1834519425U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

emlrtRSInfo ab_emlrtRSI = {
    93,                   /* lineNo */
    "validateattributes", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\lang\\validateattributes"
    ".m" /* pathName */
};

emlrtRSInfo wb_emlrtRSI = {
    20,                               /* lineNo */
    "eml_int_forloop_overflow_check", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_"
    "overflow_check.m" /* pathName */
};

emlrtRSInfo wc_emlrtRSI = {
    1,                               /* lineNo */
    "InternalAccess/InternalAccess", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\InternalAccess.m" /* pathName */
};

emlrtRSInfo yc_emlrtRSI = {
    447,                             /* lineNo */
    "MapInterface/get.XWorldLimits", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

emlrtRSInfo ad_emlrtRSI = {
    452,                             /* lineNo */
    "MapInterface/get.YWorldLimits", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

emlrtRSInfo ud_emlrtRSI = {
    451,                                  /* lineNo */
    "binaryOccupancyMap/occupancyMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

emlrtRSInfo
    ee_emlrtRSI =
        {
            15,        /* lineNo */
            "num2str", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\num2str.m" /* pathName */
};

emlrtRSInfo ug_emlrtRSI = {
    727,                      /* lineNo */
    "MapInterface/getParser", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

emlrtRSInfo wg_emlrtRSI = {
    949,                                     /* lineNo */
    "binaryOccupancyMap/checkOccupancyImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

emlrtRSInfo xg_emlrtRSI = {
    1512,                                  /* lineNo */
    "MapLayer/getValueWorldAtIndicesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

emlrtRSInfo yg_emlrtRSI = {
    1513,                                  /* lineNo */
    "MapLayer/getValueWorldAtIndicesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

emlrtRSInfo ch_emlrtRSI = {
    1460,                                 /* lineNo */
    "MapLayer/getValueAtIndicesInternal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

emlrtRSInfo hj_emlrtRSI = {
    15,    /* lineNo */
    "min", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\datafun\\min.m" /* pathName
                                                                        */
};

emlrtRSInfo lm_emlrtRSI = {
    71,                      /* lineNo */
    "PriorityQueue/isEmpty", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\PriorityQueue.m" /* pathName */
};

emlrtRSInfo ym_emlrtRSI = {
    39,     /* lineNo */
    "find", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m" /* pathName
                                                                       */
};

emlrtRSInfo an_emlrtRSI = {
    144,        /* lineNo */
    "eml_find", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m" /* pathName
                                                                       */
};

emlrtRSInfo bn_emlrtRSI = {
    402,                  /* lineNo */
    "find_first_indices", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m" /* pathName
                                                                       */
};

emlrtRSInfo kn_emlrtRSI = {
    39,    /* lineNo */
    "cat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                          */
};

emlrtRSInfo ln_emlrtRSI = {
    113,        /* lineNo */
    "cat_impl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                          */
};

emlrtRSInfo bo_emlrtRSI = {
    11,                    /* lineNo */
    "validateStateMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+internal\\+"
    "validation\\validateStateMatrix.m" /* pathName */
};

emlrtRSInfo lo_emlrtRSI = {
    16,     /* lineNo */
    "ceil", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elfun\\ceil.m" /* pathName
                                                                       */
};

emlrtRSInfo mo_emlrtRSI =
    {
        18,    /* lineNo */
        "abs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elfun\\abs.m" /* pathName
                                                                          */
};

emlrtRSInfo oo_emlrtRSI =
    {
        16,    /* lineNo */
        "any", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\any.m" /* pathName
                                                                        */
};

emlrtRSInfo ns_emlrtRSI = {
    15,    /* lineNo */
    "max", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\datafun\\max.m" /* pathName
                                                                        */
};

emlrtRSInfo nt_emlrtRSI = {
    102,                          /* lineNo */
    "binaryImplicitExpansionFun", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\ixfun.m" /* pathName
                                                                            */
};

emlrtMCInfo
    emlrtMCI =
        {
            53,        /* lineNo */
            19,        /* colNo */
            "flt2str", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\flt2str.m" /* pName */
};

emlrtRTEInfo b_emlrtRTEI = {
    14,               /* lineNo */
    37,               /* colNo */
    "validatenonnan", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatenonnan.m" /* pName */
};

emlrtRTEInfo f_emlrtRTEI = {
    28,           /* lineNo */
    27,           /* colNo */
    "validatele", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatele.m" /* pName */
};

emlrtRTEInfo g_emlrtRTEI = {
    13,                /* lineNo */
    37,                /* colNo */
    "validateinteger", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validateinteger.m" /* pName */
};

emlrtRTEInfo h_emlrtRTEI = {
    14,               /* lineNo */
    37,               /* colNo */
    "validatefinite", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatefinite.m" /* pName */
};

emlrtRTEInfo s_emlrtRTEI = {
    288,                   /* lineNo */
    27,                    /* colNo */
    "check_non_axis_size", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pName
                                                                          */
};

emlrtRTEInfo t_emlrtRTEI = {
    13,               /* lineNo */
    13,               /* colNo */
    "toLogicalCheck", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\toLogicalCheck.m" /* pName */
};

emlrtECInfo h_emlrtECI = {
    -1,                                   /* nDims */
    1460,                                 /* lineNo */
    17,                                   /* colNo */
    "MapLayer/getValueAtIndicesInternal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

emlrtDCInfo t_emlrtDCI = {
    360,                                         /* lineNo */
    54,                                          /* colNo */
    "plannerAStarGrid/get.NodesExploredIndices", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\plannerAStarGrid.m", /* pName */
    1                              /* checkKind */
};

emlrtBCInfo jb_emlrtBCI = {
    1,                                           /* iFirst */
    10000,                                       /* iLast */
    360,                                         /* lineNo */
    54,                                          /* colNo */
    "",                                          /* aName */
    "plannerAStarGrid/get.NodesExploredIndices", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\plannerAStarGrid.m", /* pName */
    0                              /* checkKind */
};

emlrtRTEInfo jb_emlrtRTEI = {
    13,     /* lineNo */
    9,      /* colNo */
    "sqrt", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m" /* pName
                                                                       */
};

emlrtRTEInfo pb_emlrtRTEI = {
    13,                 /* lineNo */
    37,                 /* colNo */
    "validatenonempty", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatenonempty.m" /* pName */
};

emlrtRTEInfo tb_emlrtRTEI = {
    15,             /* lineNo */
    19,             /* colNo */
    "validatesize", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatesize.m" /* pName */
};

emlrtRTEInfo vb_emlrtRTEI = {
    58,                   /* lineNo */
    23,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

emlrtRTEInfo hc_emlrtRTEI = {
    81,                /* lineNo */
    27,                /* colNo */
    "validate_inputs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pName */
};

emlrtRTEInfo fd_emlrtRTEI =
    {
        28,      /* lineNo */
        9,       /* colNo */
        "colon", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m" /* pName
                                                                          */
};

emlrtRTEInfo rd_emlrtRTEI = {
    41,        /* lineNo */
    20,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

emlrtRTEInfo ud_emlrtRTEI = {
    56,        /* lineNo */
    38,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

emlrtRTEInfo eh_emlrtRTEI = {
    30,                    /* lineNo */
    21,                    /* colNo */
    "applyScalarFunction", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\applyScalarFunction.m" /* pName */
};

const char_T cv[10] = {'e', 'x', 'h', 'a', 'u', 's', 't', 'i', 'v', 'e'};

emlrtRSInfo
    rt_emlrtRSI =
        {
            53,        /* lineNo */
            "flt2str", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\flt2str.m" /* pathName */
};

/* End of code generation (codegenPathPlanner_data.c) */
