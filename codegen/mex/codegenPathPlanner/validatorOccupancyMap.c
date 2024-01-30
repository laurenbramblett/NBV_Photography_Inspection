/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * validatorOccupancyMap.c
 *
 * Code generation for function 'validatorOccupancyMap'
 *
 */

/* Include files */
#include "validatorOccupancyMap.h"
#include "CircularBuffer.h"
#include "MapInterface.h"
#include "MapLayer.h"
#include "all.h"
#include "allOrAny.h"
#include "binaryOccupancyMap.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "wrapToPi.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo
    dd_emlrtRSI =
        {
            142,                                           /* lineNo */
            "validatorOccupancyMap/validatorOccupancyMap", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    ed_emlrtRSI =
        {
            137,                                           /* lineNo */
            "validatorOccupancyMap/validatorOccupancyMap", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    fd_emlrtRSI =
        {
            145,                                           /* lineNo */
            "validatorOccupancyMap/validatorOccupancyMap", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    gd_emlrtRSI =
        {
            148,                                           /* lineNo */
            "validatorOccupancyMap/validatorOccupancyMap", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    hd_emlrtRSI =
        {
            149,                                           /* lineNo */
            "validatorOccupancyMap/validatorOccupancyMap", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    id_emlrtRSI =
        {
            150,                                           /* lineNo */
            "validatorOccupancyMap/validatorOccupancyMap", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    jd_emlrtRSI =
        {
            156,                                           /* lineNo */
            "validatorOccupancyMap/validatorOccupancyMap", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    og_emlrtRSI =
        {
            205,                                  /* lineNo */
            "validatorOccupancyMap/isStateValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    pg_emlrtRSI =
        {
            213,                                  /* lineNo */
            "validatorOccupancyMap/isStateValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    qg_emlrtRSI =
        {
            215,                                  /* lineNo */
            "validatorOccupancyMap/isStateValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    rg_emlrtRSI =
        {
            218,                                  /* lineNo */
            "validatorOccupancyMap/isStateValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo sg_emlrtRSI = {
    203,                                 /* lineNo */
    "binaryOccupancyMap/checkOccupancy", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

static emlrtRSInfo tg_emlrtRSI = {
    205,                                 /* lineNo */
    "binaryOccupancyMap/checkOccupancy", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

static emlrtRSInfo vg_emlrtRSI = {
    803,                         /* lineNo */
    "MapInterface/getLocations", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo
    ph_emlrtRSI =
        {
            419, /* lineNo */
            "validatorOccupancyMap/configureValidatorForFastOccupancyCheck", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    qh_emlrtRSI =
        {
            420, /* lineNo */
            "validatorOccupancyMap/configureValidatorForFastOccupancyCheck", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo rh_emlrtRSI = {
    201,                                 /* lineNo */
    "binaryOccupancyMap/checkOccupancy", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

static emlrtRSInfo
    wn_emlrtRSI =
        {
            195,                                  /* lineNo */
            "validatorOccupancyMap/isStateValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    xn_emlrtRSI =
        {
            206,                                  /* lineNo */
            "validatorOccupancyMap/isStateValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    yn_emlrtRSI =
        {
            207,                                  /* lineNo */
            "validatorOccupancyMap/isStateValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    ao_emlrtRSI =
        {
            209,                                  /* lineNo */
            "validatorOccupancyMap/isStateValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    ho_emlrtRSI =
        {
            430,                                       /* lineNo */
            "validatorOccupancyMap/checkMapOccupancy", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    io_emlrtRSI =
        {
            433,                                       /* lineNo */
            "validatorOccupancyMap/checkMapOccupancy", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    jo_emlrtRSI =
        {
            434,                                       /* lineNo */
            "validatorOccupancyMap/checkMapOccupancy", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    ko_emlrtRSI =
        {
            443,                                       /* lineNo */
            "validatorOccupancyMap/checkMapOccupancy", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    qs_emlrtRSI =
        {
            283,                                   /* lineNo */
            "validatorOccupancyMap/isMotionValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    rs_emlrtRSI =
        {
            289,                                   /* lineNo */
            "validatorOccupancyMap/isMotionValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    ss_emlrtRSI =
        {
            334,                                   /* lineNo */
            "validatorOccupancyMap/isMotionValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    ts_emlrtRSI =
        {
            335,                                   /* lineNo */
            "validatorOccupancyMap/isMotionValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo
    us_emlrtRSI =
        {
            343,                                   /* lineNo */
            "validatorOccupancyMap/isMotionValid", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

static emlrtRSInfo vs_emlrtRSI =
    {
        183,                      /* lineNo */
        "stateSpaceSE2/distance", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pathName
                                                                          */
};

static emlrtRSInfo ws_emlrtRSI =
    {
        206,                         /* lineNo */
        "stateSpaceSE2/interpolate", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pathName
                                                                          */
};

static emlrtRSInfo xs_emlrtRSI = {
    148,                                   /* lineNo */
    "StateSpace/validateInterpolateInput", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\StateSpace.m" /* pathName
                                                                         */
};

static emlrtRTEInfo
    w_emlrtRTEI =
        {
            201,                                  /* lineNo */
            17,                                   /* colNo */
            "validatorOccupancyMap/isStateValid", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtECInfo
    g_emlrtECI =
        {
            -1,                                        /* nDims */
            448,                                       /* lineNo */
            17,                                        /* colNo */
            "validatorOccupancyMap/checkMapOccupancy", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtBCInfo q_emlrtBCI =
    {
        1,                                         /* iFirst */
        10000,                                     /* iLast */
        445,                                       /* lineNo */
        45,                                        /* colNo */
        "",                                        /* aName */
        "validatorOccupancyMap/checkMapOccupancy", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyMap."
        "m", /* pName */
        0    /* checkKind */
};

static emlrtDCInfo d_emlrtDCI =
    {
        445,                                       /* lineNo */
        45,                                        /* colNo */
        "validatorOccupancyMap/checkMapOccupancy", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyMap."
        "m", /* pName */
        1    /* checkKind */
};

static emlrtBCInfo r_emlrtBCI =
    {
        1,                                         /* iFirst */
        10000,                                     /* iLast */
        448,                                       /* lineNo */
        55,                                        /* colNo */
        "",                                        /* aName */
        "validatorOccupancyMap/checkMapOccupancy", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyMap."
        "m", /* pName */
        0    /* checkKind */
};

static emlrtDCInfo e_emlrtDCI =
    {
        448,                                       /* lineNo */
        55,                                        /* colNo */
        "validatorOccupancyMap/checkMapOccupancy", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyMap."
        "m", /* pName */
        1    /* checkKind */
};

static emlrtBCInfo s_emlrtBCI =
    {
        1,                                    /* iFirst */
        3,                                    /* iLast */
        209,                                  /* lineNo */
        38,                                   /* colNo */
        "",                                   /* aName */
        "validatorOccupancyMap/isStateValid", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyMap."
        "m", /* pName */
        0    /* checkKind */
};

static emlrtDCInfo f_emlrtDCI =
    {
        209,                                  /* lineNo */
        38,                                   /* colNo */
        "validatorOccupancyMap/isStateValid", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyMap."
        "m", /* pName */
        1    /* checkKind */
};

static emlrtECInfo
    u_emlrtECI =
        {
            1,                                    /* nDims */
            209,                                  /* lineNo */
            30,                                   /* colNo */
            "validatorOccupancyMap/isStateValid", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtECInfo
    v_emlrtECI =
        {
            1,                                    /* nDims */
            209,                                  /* lineNo */
            64,                                   /* colNo */
            "validatorOccupancyMap/isStateValid", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtECInfo
    w_emlrtECI =
        {
            1,                                    /* nDims */
            213,                                  /* lineNo */
            27,                                   /* colNo */
            "validatorOccupancyMap/isStateValid", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtECInfo
    x_emlrtECI =
        {
            1,                                    /* nDims */
            218,                                  /* lineNo */
            27,                                   /* colNo */
            "validatorOccupancyMap/isStateValid", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtECInfo
    y_emlrtECI =
        {
            -1,                                        /* nDims */
            425,                                       /* lineNo */
            24,                                        /* colNo */
            "validatorOccupancyMap/checkMapOccupancy", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtECInfo
    ab_emlrtECI =
        {
            1,                                         /* nDims */
            425,                                       /* lineNo */
            24,                                        /* colNo */
            "validatorOccupancyMap/checkMapOccupancy", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtECInfo
    bb_emlrtECI =
        {
            -1,                                        /* nDims */
            439,                                       /* lineNo */
            13,                                        /* colNo */
            "validatorOccupancyMap/checkMapOccupancy", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtECInfo
    cb_emlrtECI =
        {
            -1,                                        /* nDims */
            441,                                       /* lineNo */
            22,                                        /* colNo */
            "validatorOccupancyMap/checkMapOccupancy", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtBCInfo ae_emlrtBCI =
    {
        -1,                                        /* iFirst */
        -1,                                        /* iLast */
        435,                                       /* lineNo */
        25,                                        /* colNo */
        "",                                        /* aName */
        "validatorOccupancyMap/checkMapOccupancy", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyMap."
        "m", /* pName */
        0    /* checkKind */
};

static emlrtBCInfo be_emlrtBCI =
    {
        -1,                                        /* iFirst */
        -1,                                        /* iLast */
        448,                                       /* lineNo */
        62,                                        /* colNo */
        "",                                        /* aName */
        "validatorOccupancyMap/checkMapOccupancy", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyMap."
        "m", /* pName */
        0    /* checkKind */
};

static emlrtBCInfo ce_emlrtBCI =
    {
        -1,                                        /* iFirst */
        -1,                                        /* iLast */
        448,                                       /* lineNo */
        17,                                        /* colNo */
        "",                                        /* aName */
        "validatorOccupancyMap/checkMapOccupancy", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyMap."
        "m", /* pName */
        0    /* checkKind */
};

static emlrtRTEInfo vc_emlrtRTEI = {
    28,           /* lineNo */
    27,           /* colNo */
    "validatege", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatege.m" /* pName */
};

static emlrtRTEInfo qe_emlrtRTEI = {
    1460,       /* lineNo */
    38,         /* colNo */
    "MapLayer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtRTEInfo
    lg_emlrtRTEI =
        {
            209,                     /* lineNo */
            30,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    mg_emlrtRTEI =
        {
            209,                     /* lineNo */
            64,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    ng_emlrtRTEI =
        {
            213,                     /* lineNo */
            62,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    og_emlrtRTEI =
        {
            215,                     /* lineNo */
            68,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    pg_emlrtRTEI =
        {
            218,                     /* lineNo */
            54,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    qg_emlrtRTEI =
        {
            206,                     /* lineNo */
            13,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    rg_emlrtRTEI =
        {
            207,                     /* lineNo */
            13,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    sg_emlrtRTEI =
        {
            164,                     /* lineNo */
            28,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    tg_emlrtRTEI =
        {
            218,                     /* lineNo */
            27,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    wg_emlrtRTEI =
        {
            425,                     /* lineNo */
            24,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    xg_emlrtRTEI =
        {
            425,                     /* lineNo */
            59,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    yg_emlrtRTEI =
        {
            426,                     /* lineNo */
            17,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    ah_emlrtRTEI =
        {
            426,                     /* lineNo */
            52,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    bh_emlrtRTEI =
        {
            430,                     /* lineNo */
            29,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    ch_emlrtRTEI =
        {
            430,                     /* lineNo */
            62,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo dh_emlrtRTEI = {
    310,   /* lineNo */
    14,    /* colNo */
    "cat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pName
                                                                          */
};

static emlrtRTEInfo
    fh_emlrtRTEI =
        {
            433,                     /* lineNo */
            13,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    gh_emlrtRTEI =
        {
            439,                     /* lineNo */
            28,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    hh_emlrtRTEI =
        {
            441,                     /* lineNo */
            22,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    ih_emlrtRTEI =
        {
            445,                     /* lineNo */
            17,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    jh_emlrtRTEI =
        {
            423,                     /* lineNo */
            29,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    kh_emlrtRTEI =
        {
            430,                     /* lineNo */
            13,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    lh_emlrtRTEI =
        {
            441,                     /* lineNo */
            13,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    mh_emlrtRTEI =
        {
            448,                     /* lineNo */
            17,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    nh_emlrtRTEI =
        {
            433,                     /* lineNo */
            25,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRTEInfo
    fl_emlrtRTEI =
        {
            213,                     /* lineNo */
            27,                      /* colNo */
            "validatorOccupancyMap", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pName */
};

static emlrtRSInfo
    wt_emlrtRSI =
        {
            425,                                       /* lineNo */
            "validatorOccupancyMap/checkMapOccupancy", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\validatorOccupancyM"
            "ap.m" /* pathName */
};

/* Function Declarations */
static void c_binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                               const emlrtRSInfo in2,
                               const emxArray_boolean_T *in3,
                               const emxArray_boolean_T *in4);

static void c_validatorOccupancyMap_checkMa(const emlrtStack *sp,
                                            const validatorOccupancyMap *obj,
                                            const emxArray_real_T *stateXY,
                                            emxArray_boolean_T *validPos);

static void d_binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                               const emxArray_real_T *in2, const real_T in3[2],
                               const emxArray_real_T *in4);

static void e_binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                               const emxArray_real_T *in2, const real_T in3[2],
                               const emxArray_real_T *in4);

/* Function Definitions */
static void c_binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                               const emlrtRSInfo in2,
                               const emxArray_boolean_T *in3,
                               const emxArray_boolean_T *in4)
{
  emlrtStack st;
  emxArray_boolean_T *b_in3;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  const boolean_T *in3_data;
  const boolean_T *in4_data;
  boolean_T *b_in3_data;
  st.prev = sp;
  st.tls = sp->tls;
  in4_data = in4->data;
  in3_data = in3->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_boolean_T(sp, &b_in3, 2, &lg_emlrtRTEI);
  if (in4->size[0] == 1) {
    loop_ub = in3->size[0];
  } else {
    loop_ub = in4->size[0];
  }
  i = b_in3->size[0] * b_in3->size[1];
  b_in3->size[0] = loop_ub;
  b_in3->size[1] = 2;
  emxEnsureCapacity_boolean_T(sp, b_in3, i, &lg_emlrtRTEI);
  b_in3_data = b_in3->data;
  stride_0_0 = (in3->size[0] != 1);
  stride_1_0 = (in4->size[0] != 1);
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_in3_data[i1 + b_in3->size[0] * i] =
          (in3_data[i1 * stride_0_0 + in3->size[0] * i] &&
           in4_data[i1 * stride_1_0 + in4->size[0] * i]);
    }
  }
  st.site = (emlrtRSInfo *)&in2;
  b_all(&st, b_in3, in1);
  emxFree_boolean_T(sp, &b_in3);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void c_validatorOccupancyMap_checkMa(const emlrtStack *sp,
                                            const validatorOccupancyMap *obj,
                                            const emxArray_real_T *stateXY,
                                            emxArray_boolean_T *validPos)
{
  __m128d r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_boolean_T b_originIdx;
  emxArray_boolean_T *originIdx;
  emxArray_boolean_T *r;
  emxArray_int32_T *r3;
  emxArray_real_T *gridInd;
  emxArray_real_T *linIdx;
  emxArray_real_T *varargin_2;
  emxArray_real_T *y;
  const real_T *stateXY_data;
  real_T b_obj;
  real_T c_obj;
  real_T d;
  real_T *gridInd_data;
  real_T *linIdx_data;
  real_T *varargin_2_data;
  int32_T c_originIdx;
  int32_T i;
  int32_T loop_ub;
  int32_T scalarLB;
  int32_T vectorUB;
  int32_T *r4;
  boolean_T b_y;
  boolean_T *originIdx_data;
  boolean_T *validPos_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  stateXY_data = stateXY->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj->MapBounds[0];
  i = validPos->size[0];
  validPos->size[0] = stateXY->size[0];
  emxEnsureCapacity_boolean_T(sp, validPos, i, &wg_emlrtRTEI);
  validPos_data = validPos->data;
  loop_ub = stateXY->size[0];
  c_obj = obj->MapBounds[2];
  emxInit_boolean_T(sp, &r, 1, &wg_emlrtRTEI);
  i = r->size[0];
  r->size[0] = stateXY->size[0];
  emxEnsureCapacity_boolean_T(sp, r, i, &xg_emlrtRTEI);
  originIdx_data = r->data;
  for (i = 0; i < loop_ub; i++) {
    d = stateXY_data[i];
    validPos_data[i] = (d >= b_obj);
    originIdx_data[i] = (d <= c_obj);
  }
  if (validPos->size[0] != r->size[0]) {
    emlrtSizeEqCheck1DR2012b(validPos->size[0], r->size[0], &y_emlrtECI,
                             (emlrtConstCTX)sp);
  }
  loop_ub = validPos->size[0];
  for (i = 0; i < loop_ub; i++) {
    validPos_data[i] = (validPos_data[i] && originIdx_data[i]);
  }
  b_obj = obj->MapBounds[1];
  i = r->size[0];
  r->size[0] = stateXY->size[0];
  emxEnsureCapacity_boolean_T(sp, r, i, &yg_emlrtRTEI);
  originIdx_data = r->data;
  loop_ub = stateXY->size[0];
  for (i = 0; i < loop_ub; i++) {
    originIdx_data[i] = (stateXY_data[i + stateXY->size[0]] >= b_obj);
  }
  if ((validPos->size[0] != r->size[0]) &&
      ((validPos->size[0] != 1) && (r->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(validPos->size[0], r->size[0], &ab_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  if (validPos->size[0] == r->size[0]) {
    loop_ub = validPos->size[0];
    for (i = 0; i < loop_ub; i++) {
      validPos_data[i] = (validPos_data[i] && originIdx_data[i]);
    }
  } else {
    st.site = &wt_emlrtRSI;
    d_and(&st, validPos, r);
    validPos_data = validPos->data;
  }
  b_obj = obj->MapBounds[3];
  i = r->size[0];
  r->size[0] = stateXY->size[0];
  emxEnsureCapacity_boolean_T(sp, r, i, &ah_emlrtRTEI);
  originIdx_data = r->data;
  loop_ub = stateXY->size[0];
  for (i = 0; i < loop_ub; i++) {
    originIdx_data[i] = (stateXY_data[i + stateXY->size[0]] <= b_obj);
  }
  if ((validPos->size[0] != r->size[0]) &&
      ((validPos->size[0] != 1) && (r->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(validPos->size[0], r->size[0], &ab_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  if (validPos->size[0] == r->size[0]) {
    loop_ub = validPos->size[0];
    for (i = 0; i < loop_ub; i++) {
      validPos_data[i] = (validPos_data[i] && originIdx_data[i]);
    }
  } else {
    st.site = &wt_emlrtRSI;
    d_and(&st, validPos, r);
    validPos_data = validPos->data;
  }
  st.site = &ho_emlrtRSI;
  b_obj = -obj->MapBounds[1];
  emxInit_real_T(&st, &linIdx, 1, &lh_emlrtRTEI);
  i = linIdx->size[0];
  linIdx->size[0] = stateXY->size[0];
  emxEnsureCapacity_real_T(&st, linIdx, i, &bh_emlrtRTEI);
  linIdx_data = linIdx->data;
  loop_ub = stateXY->size[0];
  c_obj = -obj->MapBounds[0];
  emxInit_real_T(&st, &varargin_2, 1, &ch_emlrtRTEI);
  i = varargin_2->size[0];
  varargin_2->size[0] = stateXY->size[0];
  emxEnsureCapacity_real_T(&st, varargin_2, i, &ch_emlrtRTEI);
  varargin_2_data = varargin_2->data;
  scalarLB = (stateXY->size[0] / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    _mm_storeu_pd(
        &linIdx_data[i],
        _mm_add_pd(_mm_set1_pd(b_obj),
                   _mm_loadu_pd(&stateXY_data[i + stateXY->size[0]])));
    _mm_storeu_pd(
        &varargin_2_data[i],
        _mm_add_pd(_mm_set1_pd(c_obj), _mm_loadu_pd(&stateXY_data[i])));
  }
  for (i = scalarLB; i < loop_ub; i++) {
    linIdx_data[i] = b_obj + stateXY_data[i + stateXY->size[0]];
    varargin_2_data[i] = c_obj + stateXY_data[i];
  }
  b_st.site = &kn_emlrtRSI;
  c_st.site = &ln_emlrtRSI;
  if (varargin_2->size[0] != linIdx->size[0]) {
    emlrtErrorWithMessageIdR2018a(&c_st, &s_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  st.site = &ho_emlrtRSI;
  emxInit_real_T(&st, &gridInd, 2, &kh_emlrtRTEI);
  i = gridInd->size[0] * gridInd->size[1];
  gridInd->size[0] = linIdx->size[0];
  gridInd->size[1] = 2;
  emxEnsureCapacity_real_T(&st, gridInd, i, &dh_emlrtRTEI);
  gridInd_data = gridInd->data;
  loop_ub = linIdx->size[0];
  for (i = 0; i < loop_ub; i++) {
    gridInd_data[i] = linIdx_data[i];
    gridInd_data[i + gridInd->size[0]] = varargin_2_data[i];
  }
  emxFree_real_T(&st, &varargin_2);
  b_st.site = &lo_emlrtRSI;
  scalarLB = gridInd->size[0] << 1;
  for (vectorUB = 0; vectorUB < scalarLB; vectorUB++) {
    gridInd_data[vectorUB] = muDoubleScalarCeil(gridInd_data[vectorUB]);
  }
  emxInit_real_T(sp, &y, 2, &nh_emlrtRTEI);
  st.site = &io_emlrtRSI;
  b_st.site = &mo_emlrtRSI;
  i = y->size[0] * y->size[1];
  y->size[0] = gridInd->size[0];
  y->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, y, i, &eh_emlrtRTEI);
  linIdx_data = y->data;
  for (vectorUB = 0; vectorUB < scalarLB; vectorUB++) {
    linIdx_data[vectorUB] = muDoubleScalarAbs(gridInd_data[vectorUB]);
  }
  emxInit_boolean_T(sp, &originIdx, 2, &fh_emlrtRTEI);
  i = originIdx->size[0] * originIdx->size[1];
  originIdx->size[0] = y->size[0];
  originIdx->size[1] = 2;
  emxEnsureCapacity_boolean_T(sp, originIdx, i, &fh_emlrtRTEI);
  originIdx_data = originIdx->data;
  for (i = 0; i < scalarLB; i++) {
    originIdx_data[i] = (linIdx_data[i] < 2.2204460492503131E-16);
  }
  emxFree_real_T(sp, &y);
  st.site = &jo_emlrtRSI;
  b_st.site = &oo_emlrtRSI;
  b_originIdx = *originIdx;
  c_originIdx = scalarLB;
  b_originIdx.size = &c_originIdx;
  b_originIdx.numDimensions = 1;
  b_y = allOrAny_anonFcn1(&b_originIdx);
  if (b_y) {
    scalarLB--;
    for (loop_ub = 0; loop_ub <= scalarLB; loop_ub++) {
      if (originIdx_data[loop_ub]) {
        i = (gridInd->size[0] << 1) - 1;
        if (loop_ub > i) {
          emlrtDynamicBoundsCheckR2012b(loop_ub, 0, i, &ae_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        gridInd_data[loop_ub] = 1.0;
      }
    }
  }
  emxFree_boolean_T(sp, &originIdx);
  emlrtSubAssignSizeCheckR2012b(&gridInd->size[0], 1, &gridInd->size[0], 1,
                                &bb_emlrtECI, (emlrtCTX)sp);
  i = linIdx->size[0];
  linIdx->size[0] = gridInd->size[0];
  emxEnsureCapacity_real_T(sp, linIdx, i, &gh_emlrtRTEI);
  linIdx_data = linIdx->data;
  loop_ub = gridInd->size[0];
  scalarLB = (gridInd->size[0] / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    r1 = _mm_loadu_pd(&gridInd_data[i]);
    _mm_storeu_pd(&linIdx_data[i], _mm_sub_pd(_mm_set1_pd(101.0), r1));
  }
  for (i = scalarLB; i < loop_ub; i++) {
    linIdx_data[i] = 101.0 - gridInd_data[i];
  }
  loop_ub = linIdx->size[0];
  for (i = 0; i < loop_ub; i++) {
    gridInd_data[i] = linIdx_data[i];
  }
  i = linIdx->size[0];
  linIdx->size[0] = gridInd->size[0];
  emxEnsureCapacity_real_T(sp, linIdx, i, &hh_emlrtRTEI);
  linIdx_data = linIdx->data;
  loop_ub = gridInd->size[0];
  for (i = 0; i <= vectorUB; i += 2) {
    r1 = _mm_loadu_pd(&gridInd_data[i + gridInd->size[0]]);
    _mm_storeu_pd(
        &linIdx_data[i],
        _mm_mul_pd(_mm_set1_pd(100.0), _mm_sub_pd(r1, _mm_set1_pd(1.0))));
  }
  for (i = scalarLB; i < loop_ub; i++) {
    linIdx_data[i] = 100.0 * (gridInd_data[i + gridInd->size[0]] - 1.0);
  }
  if (linIdx->size[0] != gridInd->size[0]) {
    emlrtSizeEqCheck1DR2012b(linIdx->size[0], gridInd->size[0], &cb_emlrtECI,
                             (emlrtConstCTX)sp);
  }
  loop_ub = linIdx->size[0];
  scalarLB = (linIdx->size[0] / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    __m128d r2;
    r1 = _mm_loadu_pd(&linIdx_data[i]);
    r2 = _mm_loadu_pd(&gridInd_data[i]);
    _mm_storeu_pd(&linIdx_data[i], _mm_add_pd(r1, r2));
  }
  for (i = scalarLB; i < loop_ub; i++) {
    linIdx_data[i] += gridInd_data[i];
  }
  emxFree_real_T(sp, &gridInd);
  st.site = &ko_emlrtRSI;
  if (c_all(validPos)) {
    i = validPos->size[0];
    validPos->size[0] = linIdx->size[0];
    emxEnsureCapacity_boolean_T(sp, validPos, i, &ih_emlrtRTEI);
    validPos_data = validPos->data;
    loop_ub = linIdx->size[0];
    for (i = 0; i < loop_ub; i++) {
      if (linIdx_data[i] != (int32_T)muDoubleScalarFloor(linIdx_data[i])) {
        emlrtIntegerCheckR2012b(linIdx_data[i], &d_emlrtDCI, (emlrtConstCTX)sp);
      }
      scalarLB = (int32_T)linIdx_data[i];
      if ((scalarLB < 1) || (scalarLB > 10000)) {
        emlrtDynamicBoundsCheckR2012b(scalarLB, 1, 10000, &q_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      validPos_data[i] = !obj->ValidMatrix[scalarLB - 1];
    }
  } else {
    vectorUB = validPos->size[0] - 1;
    scalarLB = 0;
    for (loop_ub = 0; loop_ub <= vectorUB; loop_ub++) {
      if (validPos_data[loop_ub]) {
        scalarLB++;
      }
    }
    emxInit_int32_T(sp, &r3, 1, &mh_emlrtRTEI);
    i = r3->size[0];
    r3->size[0] = scalarLB;
    emxEnsureCapacity_int32_T(sp, r3, i, &jh_emlrtRTEI);
    r4 = r3->data;
    scalarLB = 0;
    for (loop_ub = 0; loop_ub <= vectorUB; loop_ub++) {
      if (validPos_data[loop_ub]) {
        r4[scalarLB] = loop_ub;
        scalarLB++;
      }
    }
    scalarLB = 0;
    for (loop_ub = 0; loop_ub <= vectorUB; loop_ub++) {
      if (validPos_data[loop_ub]) {
        scalarLB++;
      }
    }
    i = r->size[0];
    r->size[0] = scalarLB;
    emxEnsureCapacity_boolean_T(sp, r, i, &jh_emlrtRTEI);
    originIdx_data = r->data;
    scalarLB = 0;
    for (loop_ub = 0; loop_ub <= vectorUB; loop_ub++) {
      if (validPos_data[loop_ub]) {
        if (loop_ub > linIdx->size[0] - 1) {
          emlrtDynamicBoundsCheckR2012b(loop_ub, 0, linIdx->size[0] - 1,
                                        &be_emlrtBCI, (emlrtConstCTX)sp);
        }
        d = linIdx_data[loop_ub];
        if (d != (int32_T)muDoubleScalarFloor(d)) {
          emlrtIntegerCheckR2012b(d, &e_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)d < 1) || ((int32_T)d > 10000)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 10000, &r_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        originIdx_data[scalarLB] = !obj->ValidMatrix[(int32_T)d - 1];
        scalarLB++;
      }
    }
    if (r3->size[0] != r->size[0]) {
      emlrtSubAssignSizeCheck1dR2017a(r3->size[0], r->size[0], &g_emlrtECI,
                                      (emlrtConstCTX)sp);
    }
    loop_ub = r->size[0];
    for (i = 0; i < loop_ub; i++) {
      if (r4[i] > validPos->size[0] - 1) {
        emlrtDynamicBoundsCheckR2012b(r4[i], 0, validPos->size[0] - 1,
                                      &ce_emlrtBCI, (emlrtConstCTX)sp);
      }
      validPos_data[r4[i]] = originIdx_data[i];
    }
    emxFree_int32_T(sp, &r3);
  }
  emxFree_boolean_T(sp, &r);
  emxFree_real_T(sp, &linIdx);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void d_binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                               const emxArray_real_T *in2, const real_T in3[2],
                               const emxArray_real_T *in4)
{
  const real_T *in2_data;
  const real_T *in4_data;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  boolean_T *in1_data;
  in4_data = in4->data;
  in2_data = in2->data;
  if (in4->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in4->size[0];
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = loop_ub;
  in1->size[1] = 2;
  emxEnsureCapacity_boolean_T(sp, in1, i, &mg_emlrtRTEI);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in4->size[0] != 1);
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      in1_data[i1 + in1->size[0] * i] =
          (in2_data[i1 * stride_0_0 + in2->size[0] * ((int32_T)in3[i] - 1)] <=
           in4_data[i1 * stride_1_0 + in4->size[0] * i]);
    }
  }
}

static void e_binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                               const emxArray_real_T *in2, const real_T in3[2],
                               const emxArray_real_T *in4)
{
  const real_T *in2_data;
  const real_T *in4_data;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  boolean_T *in1_data;
  in4_data = in4->data;
  in2_data = in2->data;
  if (in4->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in4->size[0];
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = loop_ub;
  in1->size[1] = 2;
  emxEnsureCapacity_boolean_T(sp, in1, i, &lg_emlrtRTEI);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in4->size[0] != 1);
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      in1_data[i1 + in1->size[0] * i] =
          (in2_data[i1 * stride_0_0 + in2->size[0] * ((int32_T)in3[i] - 1)] >=
           in4_data[i1 * stride_1_0 + in4->size[0] * i]);
    }
  }
}

void c_validatorOccupancyMap_configu(const emlrtStack *sp,
                                     validatorOccupancyMap *obj)
{
  binaryOccupancyMap *b_obj;
  emlrtStack b_st;
  emlrtStack st;
  int32_T i;
  boolean_T occupied[10000];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  st.site = &ph_emlrtRSI;
  b_obj = obj->Map;
  b_st.site = &rh_emlrtRSI;
  MapLayer_getValueAllImpl(&b_st, b_obj, occupied);
  for (i = 0; i < 10000; i++) {
    obj->ValidMatrix[i] = occupied[i];
  }
  real_T c_obj;
  real_T d_obj;
  st.site = &qh_emlrtRSI;
  b_obj = obj->Map;
  c_obj = b_obj->SharedProperties.LocalOriginInWorld[0] +
          b_obj->SharedProperties.GridOriginInLocal[0];
  st.site = &qh_emlrtRSI;
  b_obj = obj->Map;
  d_obj = b_obj->SharedProperties.LocalOriginInWorld[1] +
          b_obj->SharedProperties.GridOriginInLocal[1];
  obj->MapBounds[0] = c_obj;
  obj->MapBounds[2] = c_obj + 100.0;
  obj->MapBounds[1] = d_obj;
  obj->MapBounds[3] = d_obj + 100.0;
}

boolean_T c_validatorOccupancyMap_isMotio(const emlrtStack *sp,
                                          validatorOccupancyMap *obj,
                                          const real_T state1[3],
                                          const real_T state2[3])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  stateSpaceSE2 *b_obj;
  real_T iState[3];
  real_T stateDiff[3];
  real_T dtheta;
  boolean_T isValid;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &qs_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  if (!c_validatorOccupancyMap_isState(&st, obj, state1)) {
    isValid = false;
  } else {
    real_T dist;
    st.site = &rs_emlrtRSI;
    b_obj = obj->StateSpace;
    stateDiff[0] = muDoubleScalarAbs(state1[0] - state2[0]);
    stateDiff[1] = muDoubleScalarAbs(state1[1] - state2[1]);
    dtheta = muDoubleScalarAbs(state1[2] - state2[2]);
    wrapToPi(&dtheta);
    b_st.site = &vs_emlrtRSI;
    dist = b_obj->WeightXY *
               (stateDiff[0] * stateDiff[0] + stateDiff[1] * stateDiff[1]) +
           b_obj->WeightTheta * (dtheta * dtheta);
    if (dist < 0.0) {
      emlrtErrorWithMessageIdR2018a(
          &b_st, &jb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
          "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
    }
    dist = muDoubleScalarSqrt(dist);
    if (muDoubleScalarIsNaN(dist)) {
      isValid = false;
    } else {
      real_T delta;
      real_T n;
      boolean_T exitg1;
      delta = obj->ValidationDistance;
      dtheta = delta;
      n = 1.0;
      isValid = true;
      exitg1 = false;
      while ((!exitg1) && (dtheta < dist)) {
        __m128d r;
        st.site = &ss_emlrtRSI;
        b_obj = obj->StateSpace;
        dtheta /= dist;
        if (!b_obj->SkipStateValidation) {
          b_st.site = &ws_emlrtRSI;
          c_st.site = &xs_emlrtRSI;
          d_st.site = &ab_emlrtRSI;
          if (!(dtheta >= 0.0)) {
            emlrtErrorWithMessageIdR2018a(
                &d_st, &vc_emlrtRTEI, "MATLAB:validateattributes:expectedArray",
                "MATLAB:interpolate:notGreaterEqual", 9, 4, 6, "ratios", 4, 2,
                ">=", 4, 1, "0");
          }
          d_st.site = &ab_emlrtRSI;
          if (!(dtheta <= 1.0)) {
            emlrtErrorWithMessageIdR2018a(
                &d_st, &f_emlrtRTEI, "MATLAB:validateattributes:expectedArray",
                "MATLAB:interpolate:notLessEqual", 9, 4, 6, "ratios", 4, 2,
                "<=", 4, 1, "1");
          }
        }
        _mm_storeu_pd(&stateDiff[0], _mm_sub_pd(_mm_loadu_pd(&state2[0]),
                                                _mm_loadu_pd(&state1[0])));
        stateDiff[2] = state2[2] - state1[2];
        wrapToPi(&stateDiff[2]);
        r = _mm_loadu_pd(&stateDiff[0]);
        _mm_storeu_pd(&iState[0],
                      _mm_add_pd(_mm_loadu_pd(&state1[0]),
                                 _mm_mul_pd(_mm_set1_pd(dtheta), r)));
        iState[2] = state1[2] + dtheta * stateDiff[2];
        wrapToPi(&iState[2]);
        st.site = &ts_emlrtRSI;
        if (!c_validatorOccupancyMap_isState(&st, obj, iState)) {
          isValid = false;
          exitg1 = true;
        } else {
          n++;
          dtheta = delta * n;
        }
      }
      if (isValid) {
        st.site = &us_emlrtRSI;
        isValid = c_validatorOccupancyMap_isState(&st, obj, state2);
      }
    }
  }
  return isValid;
}

boolean_T c_validatorOccupancyMap_isState(const emlrtStack *sp,
                                          validatorOccupancyMap *obj,
                                          const real_T state[3])
{
  binaryOccupancyMap *c_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_boolean_T *r;
  emxArray_real_T b_localGridInd_data;
  stateSpaceSE2 *b_obj;
  real_T ssBounds[6];
  real_T localGridInd_data[2];
  real_T originIdx_tmp;
  real_T xyInd_idx_0;
  real_T xyInd_idx_1;
  int32_T localGridInd_size[2];
  int32_T i;
  int32_T k;
  int32_T tmp_size;
  boolean_T originIdx[2];
  boolean_T exitg1;
  boolean_T isInBounds;
  boolean_T isValid;
  boolean_T y;
  boolean_T *r1;
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
  xyInd_idx_0 = obj->XYIndices[0];
  xyInd_idx_1 = obj->XYIndices[1];
  originIdx[0] = (obj->XYIndices[0] > 3.0);
  originIdx[1] = (obj->XYIndices[1] > 3.0);
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (originIdx[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (y) {
    emlrtErrorWithMessageIdR2018a(
        sp, &w_emlrtRTEI, "nav:navalgs:statevalidoccmap:XYIndicesInvalid",
        "nav:navalgs:statevalidoccmap:XYIndicesInvalid", 2, 6, 3.0);
  }
  st.site = &og_emlrtRSI;
  b_obj = obj->StateSpace;
  for (i = 0; i < 6; i++) {
    ssBounds[i] = b_obj->StateBoundsInternal[i];
  }
  if (xyInd_idx_0 != (int32_T)muDoubleScalarFloor(xyInd_idx_0)) {
    emlrtIntegerCheckR2012b(xyInd_idx_0, &f_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (((int32_T)xyInd_idx_0 < 1) || ((int32_T)xyInd_idx_0 > 3)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)xyInd_idx_0, 1, 3, &s_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (xyInd_idx_1 != (int32_T)muDoubleScalarFloor(xyInd_idx_1)) {
    emlrtIntegerCheckR2012b(xyInd_idx_1, &f_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (((int32_T)xyInd_idx_1 < 1) || ((int32_T)xyInd_idx_1 > 3)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)xyInd_idx_1, 1, 3, &s_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  originIdx_tmp = state[(int32_T)xyInd_idx_0 - 1];
  originIdx[0] =
      ((originIdx_tmp >= ssBounds[0]) && (originIdx_tmp <= ssBounds[3]));
  xyInd_idx_0 = state[(int32_T)xyInd_idx_1 - 1];
  originIdx[1] = ((xyInd_idx_0 >= ssBounds[1]) && (xyInd_idx_0 <= ssBounds[4]));
  isInBounds = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (!originIdx[k]) {
      isInBounds = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (obj->SkipStateValidation) {
    real_T ssUpperBounds_idx_0;
    boolean_T b;
    boolean_T b1;
    boolean_T b2;
    boolean_T b3;
    st.site = &pg_emlrtRSI;
    b = (originIdx_tmp >= obj->MapBounds[0]);
    b1 = (originIdx_tmp <= obj->MapBounds[2]);
    b2 = (xyInd_idx_0 >= obj->MapBounds[1]);
    b3 = (xyInd_idx_0 <= obj->MapBounds[3]);
    ssUpperBounds_idx_0 = -obj->MapBounds[1] + xyInd_idx_0;
    xyInd_idx_0 = -obj->MapBounds[0] + originIdx_tmp;
    xyInd_idx_1 = muDoubleScalarCeil(ssUpperBounds_idx_0);
    ssUpperBounds_idx_0 = xyInd_idx_1;
    originIdx[0] = (muDoubleScalarAbs(xyInd_idx_1) < 2.2204460492503131E-16);
    xyInd_idx_1 = muDoubleScalarCeil(xyInd_idx_0);
    xyInd_idx_0 = xyInd_idx_1;
    originIdx[1] = (muDoubleScalarAbs(xyInd_idx_1) < 2.2204460492503131E-16);
    y = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= 1)) {
      if (originIdx[k]) {
        y = true;
        exitg1 = true;
      } else {
        k++;
      }
    }
    if (y) {
      if (originIdx[0]) {
        ssUpperBounds_idx_0 = 1.0;
      }
      if (originIdx[1]) {
        xyInd_idx_0 = 1.0;
      }
    }
    ssUpperBounds_idx_0 = 101.0 - ssUpperBounds_idx_0;
    xyInd_idx_0 = 100.0 * (xyInd_idx_0 - 1.0) + ssUpperBounds_idx_0;
    if (b && b1 && b2 && b3) {
      if (xyInd_idx_0 != (int32_T)xyInd_idx_0) {
        emlrtIntegerCheckR2012b(xyInd_idx_0, &d_emlrtDCI, &st);
      }
      if (((int32_T)xyInd_idx_0 < 1) || ((int32_T)xyInd_idx_0 > 10000)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)xyInd_idx_0, 1, 10000,
                                      &q_emlrtBCI, &st);
      }
      y = !obj->ValidMatrix[(int32_T)xyInd_idx_0 - 1];
    } else {
      y = false;
    }
    isValid = (isInBounds && y);
  } else {
    real_T b_state[2];
    real_T localGridInd[2];
    real_T ssUpperBounds_idx_0;
    st.site = &qg_emlrtRSI;
    c_obj = obj->Map;
    b_st.site = &sg_emlrtRSI;
    c_st.site = &ug_emlrtRSI;
    d_st.site = &vg_emlrtRSI;
    xyInd_idx_1 = c_obj->SharedProperties.LocalOriginInWorld[0] +
                  c_obj->SharedProperties.GridOriginInLocal[0];
    d_st.site = &vg_emlrtRSI;
    ssUpperBounds_idx_0 = c_obj->SharedProperties.LocalOriginInWorld[1] +
                          c_obj->SharedProperties.GridOriginInLocal[1];
    b_st.site = &tg_emlrtRSI;
    c_st.site = &wg_emlrtRSI;
    b_state[0] = originIdx_tmp;
    b_state[1] = xyInd_idx_0;
    d_st.site = &xg_emlrtRSI;
    MapInterface_world2gridImpl(c_obj, b_state, localGridInd);
    d_st.site = &yg_emlrtRSI;
    k = -1;
    if ((localGridInd[0] > 0.0) && (localGridInd[0] < 101.0) &&
        (localGridInd[1] > 0.0) && (localGridInd[1] < 101.0)) {
      int32_T loop_ub;
      int8_T i1;
      int8_T tmp_data;
      k = 1;
      localGridInd_size[0] = 1;
      localGridInd_size[1] = 2;
      for (i = 0; i < 2; i++) {
        localGridInd_data[localGridInd_size[0] * i] = localGridInd[i];
      }
      b_localGridInd_data.data = &localGridInd_data[0];
      b_localGridInd_data.size = &localGridInd_size[0];
      b_localGridInd_data.allocatedSize = 2;
      b_localGridInd_data.numDimensions = 2;
      b_localGridInd_data.canFreeData = false;
      emxInit_boolean_T(&d_st, &r, 1, &qe_emlrtRTEI);
      e_st.site = &ch_emlrtRSI;
      c_CircularBuffer_getValueAtIndi(&e_st, &c_obj->Buffer,
                                      &b_localGridInd_data, r);
      r1 = r->data;
      tmp_size = r->size[0];
      loop_ub = r->size[0];
      for (i = 0; i < loop_ub; i++) {
        tmp_data = (int8_T)r1[i];
      }
      emxFree_boolean_T(&d_st, &r);
      emlrtSubAssignSizeCheckR2012b(&k, 1, &tmp_size, 1, &h_emlrtECI, &d_st);
      i1 = -1;
      for (i = 0; i < tmp_size; i++) {
        i1 = tmp_data;
      }
      k = i1;
    }
    st.site = &rg_emlrtRSI;
    isValid = (isInBounds &&
               ((originIdx_tmp >= xyInd_idx_1) &&
                (originIdx_tmp <= xyInd_idx_1 + 100.0) &&
                (xyInd_idx_0 >= ssUpperBounds_idx_0) &&
                (xyInd_idx_0 <= ssUpperBounds_idx_0 + 100.0)) &&
               (k == 0));
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return isValid;
}

validatorOccupancyMap *c_validatorOccupancyMap_validat(
    const emlrtStack *sp, validatorOccupancyMap *obj, stateSpaceSE2 *varargin_1,
    binaryOccupancyMap *varargin_3)
{
  emlrtStack b_st;
  emlrtStack st;
  validatorOccupancyMap *b_obj;
  boolean_T unusedExpr[10000];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_obj = obj;
  st.site = &dd_emlrtRSI;
  st.site = &ed_emlrtRSI;
  st.site = &ed_emlrtRSI;
  st.site = &fd_emlrtRSI;
  b_obj->StateSpace = varargin_1;
  st.site = &gd_emlrtRSI;
  b_obj->Map = varargin_3;
  st.site = &hd_emlrtRSI;
  b_obj->XYIndices[0] = 1.0;
  b_obj->XYIndices[1] = 2.0;
  st.site = &id_emlrtRSI;
  b_obj->ValidationDistance = rtInf;
  b_obj->SkipStateValidation = false;
  st.site = &jd_emlrtRSI;
  b_st.site = &ud_emlrtRSI;
  MapLayer_getValueAllImpl(&b_st, varargin_3, unusedExpr);
  return b_obj;
}

void d_and(const emlrtStack *sp, emxArray_boolean_T *in1,
           const emxArray_boolean_T *in2)
{
  emxArray_boolean_T *b_in1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  const boolean_T *in2_data;
  boolean_T *b_in1_data;
  boolean_T *in1_data;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_boolean_T(sp, &b_in1, 1, &fl_emlrtRTEI);
  if (in2->size[0] == 1) {
    loop_ub = in1->size[0];
  } else {
    loop_ub = in2->size[0];
  }
  i = b_in1->size[0];
  b_in1->size[0] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, b_in1, i, &fl_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[0] != 1);
  stride_1_0 = (in2->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in1_data[i] = (in1_data[i * stride_0_0] && in2_data[i * stride_1_0]);
  }
  i = in1->size[0];
  in1->size[0] = b_in1->size[0];
  emxEnsureCapacity_boolean_T(sp, in1, i, &fl_emlrtRTEI);
  in1_data = in1->data;
  loop_ub = b_in1->size[0];
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = b_in1_data[i];
  }
  emxFree_boolean_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void d_validatorOccupancyMap_isState(const emlrtStack *sp,
                                     validatorOccupancyMap *obj,
                                     const emxArray_real_T *state,
                                     emxArray_boolean_T *isValid)
{
  static real_T dv[2] = {0.0, 3.0};
  binaryOccupancyMap *c_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_boolean_T *idxInBounds;
  emxArray_boolean_T *r;
  emxArray_boolean_T *r1;
  emxArray_boolean_T *r2;
  emxArray_real_T *isOccupied;
  emxArray_real_T *ssLowerBounds;
  emxArray_real_T *ssUpperBounds;
  stateSpaceSE2 *b_obj;
  real_T ssBounds[6];
  real_T xyInd[2];
  const real_T *state_data;
  real_T *ssLowerBounds_data;
  real_T *ssUpperBounds_data;
  int32_T i;
  int32_T i1;
  int32_T k;
  boolean_T x[2];
  boolean_T exitg1;
  boolean_T p;
  boolean_T *idxInBounds_data;
  boolean_T *isValid_data;
  boolean_T *r3;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  dv[0U] = rtNaN;
  state_data = state->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  if (!obj->SkipStateValidation) {
    st.site = &wn_emlrtRSI;
    b_st.site = &bo_emlrtRSI;
    c_st.site = &ab_emlrtRSI;
    if (state->size[0] == 0) {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &pb_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedNonempty",
          "MATLAB:isStateValid:expectedNonempty", 3, 4, 5, "state");
    }
    c_st.site = &ab_emlrtRSI;
    p = true;
    for (k = 0; k < 2; k++) {
      if (p) {
        real_T d;
        d = dv[k];
        if ((!(d != d)) && (state->size[k] != 3)) {
          p = false;
        }
      } else {
        p = false;
      }
    }
    if (!p) {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &tb_emlrtRTEI, "Coder:toolbox:ValidateattributesincorrectSize",
          "MATLAB:isStateValid:incorrectSize", 3, 4, 5, "state");
    }
  }
  xyInd[0] = obj->XYIndices[0];
  xyInd[1] = obj->XYIndices[1];
  x[0] = (obj->XYIndices[0] > 3.0);
  x[1] = (obj->XYIndices[1] > 3.0);
  p = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (x[k]) {
      p = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(
        sp, &w_emlrtRTEI, "nav:navalgs:statevalidoccmap:XYIndicesInvalid",
        "nav:navalgs:statevalidoccmap:XYIndicesInvalid", 2, 6, 3.0);
  }
  st.site = &og_emlrtRSI;
  b_obj = obj->StateSpace;
  for (i = 0; i < 6; i++) {
    ssBounds[i] = b_obj->StateBoundsInternal[i];
  }
  emxInit_real_T(sp, &ssLowerBounds, 2, &qg_emlrtRTEI);
  st.site = &xn_emlrtRSI;
  repmat(&st, &ssBounds[0], state->size[0], ssLowerBounds);
  ssLowerBounds_data = ssLowerBounds->data;
  emxInit_real_T(sp, &ssUpperBounds, 2, &rg_emlrtRTEI);
  st.site = &yn_emlrtRSI;
  repmat(&st, &ssBounds[3], state->size[0], ssUpperBounds);
  ssUpperBounds_data = ssUpperBounds->data;
  if (xyInd[0] != (int32_T)muDoubleScalarFloor(xyInd[0])) {
    emlrtIntegerCheckR2012b(xyInd[0], &f_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (((int32_T)xyInd[0] < 1) || ((int32_T)xyInd[0] > 3)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)xyInd[0], 1, 3, &s_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (xyInd[1] != (int32_T)muDoubleScalarFloor(xyInd[1])) {
    emlrtIntegerCheckR2012b(xyInd[1], &f_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (((int32_T)xyInd[1] < 1) || ((int32_T)xyInd[1] > 3)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)xyInd[1], 1, 3, &s_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if ((state->size[0] != ssLowerBounds->size[0]) &&
      ((state->size[0] != 1) && (ssLowerBounds->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(state->size[0], ssLowerBounds->size[0],
                                &u_emlrtECI, (emlrtConstCTX)sp);
  }
  if ((state->size[0] != ssUpperBounds->size[0]) &&
      ((state->size[0] != 1) && (ssUpperBounds->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(state->size[0], ssUpperBounds->size[0],
                                &v_emlrtECI, (emlrtConstCTX)sp);
  }
  emxInit_boolean_T(sp, &r, 2, &lg_emlrtRTEI);
  if (state->size[0] == ssLowerBounds->size[0]) {
    i = r->size[0] * r->size[1];
    r->size[0] = state->size[0];
    r->size[1] = 2;
    emxEnsureCapacity_boolean_T(sp, r, i, &lg_emlrtRTEI);
    isValid_data = r->data;
    k = state->size[0];
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < k; i1++) {
        isValid_data[i1 + r->size[0] * i] =
            (state_data[i1 + state->size[0] * ((int32_T)xyInd[i] - 1)] >=
             ssLowerBounds_data[i1 + ssLowerBounds->size[0] * i]);
      }
    }
  } else {
    st.site = &ao_emlrtRSI;
    e_binary_expand_op(&st, r, state, xyInd, ssLowerBounds);
    isValid_data = r->data;
  }
  emxInit_boolean_T(sp, &r1, 2, &lg_emlrtRTEI);
  if (state->size[0] == ssUpperBounds->size[0]) {
    i = r1->size[0] * r1->size[1];
    r1->size[0] = state->size[0];
    r1->size[1] = 2;
    emxEnsureCapacity_boolean_T(sp, r1, i, &mg_emlrtRTEI);
    idxInBounds_data = r1->data;
    k = state->size[0];
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < k; i1++) {
        idxInBounds_data[i1 + r1->size[0] * i] =
            (state_data[i1 + state->size[0] * ((int32_T)xyInd[i] - 1)] <=
             ssUpperBounds_data[i1 + ssUpperBounds->size[0] * i]);
      }
    }
  } else {
    st.site = &ao_emlrtRSI;
    d_binary_expand_op(&st, r1, state, xyInd, ssUpperBounds);
    idxInBounds_data = r1->data;
  }
  emxFree_real_T(sp, &ssUpperBounds);
  if ((r->size[0] != r1->size[0]) &&
      ((r->size[0] != 1) && (r1->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(r->size[0], r1->size[0], &u_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  if (r->size[0] == r1->size[0]) {
    emxInit_boolean_T(sp, &r2, 2, &lg_emlrtRTEI);
    i = r2->size[0] * r2->size[1];
    r2->size[0] = r->size[0];
    r2->size[1] = 2;
    emxEnsureCapacity_boolean_T(sp, r2, i, &lg_emlrtRTEI);
    r3 = r2->data;
    k = r->size[0] << 1;
    for (i = 0; i < k; i++) {
      r3[i] = (isValid_data[i] && idxInBounds_data[i]);
    }
    st.site = &ao_emlrtRSI;
    b_all(&st, r2, isValid);
    isValid_data = isValid->data;
    emxFree_boolean_T(sp, &r2);
  } else {
    st.site = &ao_emlrtRSI;
    c_binary_expand_op(&st, isValid, ao_emlrtRSI, r, r1);
    isValid_data = isValid->data;
  }
  emxFree_boolean_T(sp, &r1);
  emxFree_boolean_T(sp, &r);
  emxInit_boolean_T(sp, &idxInBounds, 1, &tg_emlrtRTEI);
  if (obj->SkipStateValidation) {
    i = ssLowerBounds->size[0] * ssLowerBounds->size[1];
    ssLowerBounds->size[0] = state->size[0];
    ssLowerBounds->size[1] = 2;
    emxEnsureCapacity_real_T(sp, ssLowerBounds, i, &ng_emlrtRTEI);
    ssLowerBounds_data = ssLowerBounds->data;
    k = state->size[0];
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < k; i1++) {
        ssLowerBounds_data[i1 + ssLowerBounds->size[0] * i] =
            state_data[i1 + state->size[0] * ((int32_T)xyInd[i] - 1)];
      }
    }
    st.site = &pg_emlrtRSI;
    c_validatorOccupancyMap_checkMa(&st, obj, ssLowerBounds, idxInBounds);
    idxInBounds_data = idxInBounds->data;
    if ((isValid->size[0] != idxInBounds->size[0]) &&
        ((isValid->size[0] != 1) && (idxInBounds->size[0] != 1))) {
      emlrtDimSizeImpxCheckR2021b(isValid->size[0], idxInBounds->size[0],
                                  &w_emlrtECI, (emlrtConstCTX)sp);
    }
    if (isValid->size[0] == idxInBounds->size[0]) {
      k = isValid->size[0];
      for (i = 0; i < k; i++) {
        isValid_data[i] = (isValid_data[i] && idxInBounds_data[i]);
      }
    } else {
      st.site = &pg_emlrtRSI;
      d_and(&st, isValid, idxInBounds);
    }
  } else {
    st.site = &qg_emlrtRSI;
    c_obj = obj->Map;
    i = ssLowerBounds->size[0] * ssLowerBounds->size[1];
    ssLowerBounds->size[0] = state->size[0];
    ssLowerBounds->size[1] = 2;
    emxEnsureCapacity_real_T(&st, ssLowerBounds, i, &og_emlrtRTEI);
    ssLowerBounds_data = ssLowerBounds->data;
    k = state->size[0];
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < k; i1++) {
        ssLowerBounds_data[i1 + ssLowerBounds->size[0] * i] =
            state_data[i1 + state->size[0] * ((int32_T)xyInd[i] - 1)];
      }
    }
    b_st.site = &sg_emlrtRSI;
    MapInterface_getParser(&b_st, c_obj, ssLowerBounds, idxInBounds);
    idxInBounds_data = idxInBounds->data;
    emxInit_real_T(&st, &isOccupied, 1, &sg_emlrtRTEI);
    b_st.site = &tg_emlrtRSI;
    c_binaryOccupancyMap_checkOccup(&b_st, c_obj, ssLowerBounds, isOccupied);
    ssLowerBounds_data = isOccupied->data;
    if ((isValid->size[0] != idxInBounds->size[0]) &&
        ((isValid->size[0] != 1) && (idxInBounds->size[0] != 1))) {
      emlrtDimSizeImpxCheckR2021b(isValid->size[0], idxInBounds->size[0],
                                  &x_emlrtECI, (emlrtConstCTX)sp);
    }
    st.site = &rg_emlrtRSI;
    i = isOccupied->size[0];
    for (k = 0; k < i; k++) {
      if (muDoubleScalarIsNaN(ssLowerBounds_data[k])) {
        emlrtErrorWithMessageIdR2018a(&st, &t_emlrtRTEI, "MATLAB:nologicalnan",
                                      "MATLAB:nologicalnan", 0);
      }
    }
    if (isValid->size[0] == idxInBounds->size[0]) {
      k = isValid->size[0];
      for (i = 0; i < k; i++) {
        isValid_data[i] = (isValid_data[i] && idxInBounds_data[i]);
      }
    } else {
      st.site = &rg_emlrtRSI;
      d_and(&st, isValid, idxInBounds);
      isValid_data = isValid->data;
    }
    i = idxInBounds->size[0];
    idxInBounds->size[0] = isOccupied->size[0];
    emxEnsureCapacity_boolean_T(sp, idxInBounds, i, &pg_emlrtRTEI);
    idxInBounds_data = idxInBounds->data;
    k = isOccupied->size[0];
    for (i = 0; i < k; i++) {
      idxInBounds_data[i] = !(ssLowerBounds_data[i] != 0.0);
    }
    emxFree_real_T(sp, &isOccupied);
    if ((isValid->size[0] != idxInBounds->size[0]) &&
        ((isValid->size[0] != 1) && (idxInBounds->size[0] != 1))) {
      emlrtDimSizeImpxCheckR2021b(isValid->size[0], idxInBounds->size[0],
                                  &x_emlrtECI, (emlrtConstCTX)sp);
    }
    if (isValid->size[0] == idxInBounds->size[0]) {
      k = isValid->size[0];
      for (i = 0; i < k; i++) {
        isValid_data[i] = (isValid_data[i] && idxInBounds_data[i]);
      }
    } else {
      st.site = &rg_emlrtRSI;
      d_and(&st, isValid, idxInBounds);
    }
  }
  emxFree_boolean_T(sp, &idxInBounds);
  emxFree_real_T(sp, &ssLowerBounds);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (validatorOccupancyMap.c) */
