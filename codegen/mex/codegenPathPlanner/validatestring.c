/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * validatestring.c
 *
 * Code generation for function 'validatestring'
 *
 */

/* Include files */
#include "validatestring.h"
#include "codegenPathPlanner_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo hh_emlrtRSI =
    {
        74,               /* lineNo */
        "validatestring", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\lang\\validatestring"
        ".m" /* pathName */
};

static emlrtRSInfo
    ih_emlrtRSI =
        {
            91,       /* lineNo */
            "strcmp", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\strcmp.m" /* pathName */
};

static emlrtRSInfo
    jh_emlrtRSI =
        {
            167,          /* lineNo */
            "loc_strcmp", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\strcmp.m" /* pathName */
};

static emlrtRSInfo
    kh_emlrtRSI =
        {
            240,       /* lineNo */
            "charcmp", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\strcmp.m" /* pathName */
};

static emlrtRSInfo nh_emlrtRSI =
    {
        111,                  /* lineNo */
        "fullValidatestring", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\lang\\validatestring"
        ".m" /* pathName */
};

static emlrtRSInfo oh_emlrtRSI =
    {
        164,         /* lineNo */
        "get_match", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\lang\\validatestring"
        ".m" /* pathName */
};

static emlrtRTEInfo x_emlrtRTEI = {
    15,                      /* lineNo */
    9,                       /* colNo */
    "assertSupportedString", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertSupportedString.m" /* pName */
};

static emlrtRTEInfo y_emlrtRTEI =
    {
        131,                  /* lineNo */
        9,                    /* colNo */
        "fullValidatestring", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\lang\\validatestring"
        ".m" /* pName */
};

/* Function Definitions */
void validatestring(const emlrtStack *sp, const char_T str[6],
                    char_T out_data[], int32_T out_size[2])
{
  static const char_T cv1[128] = {
      '\x00', '\x01', '\x02', '\x03', '\x04', '\x05', '\x06', '\a',   '\b',
      '\t',   '\n',   '\v',   '\f',   '\r',   '\x0e', '\x0f', '\x10', '\x11',
      '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19', '\x1a',
      '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',    '!',    '\"',   '#',
      '$',    '%',    '&',    '\'',   '(',    ')',    '*',    '+',    ',',
      '-',    '.',    '/',    '0',    '1',    '2',    '3',    '4',    '5',
      '6',    '7',    '8',    '9',    ':',    ';',    '<',    '=',    '>',
      '?',    '@',    'a',    'b',    'c',    'd',    'e',    'f',    'g',
      'h',    'i',    'j',    'k',    'l',    'm',    'n',    'o',    'p',
      'q',    'r',    's',    't',    'u',    'v',    'w',    'x',    'y',
      'z',    '[',    '\\',   ']',    '^',    '_',    '`',    'a',    'b',
      'c',    'd',    'e',    'f',    'g',    'h',    'i',    'j',    'k',
      'l',    'm',    'n',    'o',    'p',    'q',    'r',    's',    't',
      'u',    'v',    'w',    'x',    'y',    'z',    '{',    '|',    '}',
      '~',    '\x7f'};
  static const char_T cv2[6] = {'g', 'r', 'e', 'e', 'd', 'y'};
  static const char_T vstr[6] = {'g', 'r', 'e', 'e', 'd', 'y'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  int32_T exitg1;
  int32_T i;
  int32_T kstr;
  int32_T nmatched;
  char_T partial_match_data[10];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &hh_emlrtRSI;
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
  b_st.site = &nh_emlrtRSI;
  nmatched = 0;
  c_st.site = &oh_emlrtRSI;
  d_st.site = &ih_emlrtRSI;
  e_st.site = &jh_emlrtRSI;
  b_bool = false;
  kstr = 0;
  do {
    exitg1 = 0;
    if (kstr < 6) {
      f_st.site = &kh_emlrtRSI;
      i = (uint8_T)str[kstr];
      if (i > 127) {
        emlrtErrorWithMessageIdR2018a(
            &f_st, &x_emlrtRTEI, "Coder:toolbox:unsupportedString",
            "Coder:toolbox:unsupportedString", 2, 12, 127);
      }
      if (cv1[i] != cv1[(int32_T)cv2[kstr]]) {
        exitg1 = 1;
      } else {
        kstr++;
      }
    } else {
      b_bool = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  if (b_bool) {
    nmatched = 1;
    kstr = 6;
    for (i = 0; i < 6; i++) {
      partial_match_data[i] = vstr[i];
    }
  } else {
    c_st.site = &oh_emlrtRSI;
    d_st.site = &ih_emlrtRSI;
    e_st.site = &jh_emlrtRSI;
    b_bool = false;
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 6) {
        f_st.site = &kh_emlrtRSI;
        i = (uint8_T)str[kstr];
        if (i > 127) {
          emlrtErrorWithMessageIdR2018a(
              &f_st, &x_emlrtRTEI, "Coder:toolbox:unsupportedString",
              "Coder:toolbox:unsupportedString", 2, 12, 127);
        }
        if (cv1[i] != cv1[(int32_T)cv[kstr]]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
    if (b_bool) {
      kstr = 10;
      for (i = 0; i < 10; i++) {
        partial_match_data[i] = cv[i];
      }
      nmatched = 1;
    } else {
      kstr = 6;
      for (i = 0; i < 6; i++) {
        partial_match_data[i] = ' ';
      }
    }
  }
  if (nmatched == 0) {
    char_T b_cv[11];
    b_cv[0] = ',';
    b_cv[1] = ' ';
    b_cv[2] = '\'';
    for (i = 0; i < 6; i++) {
      b_cv[i + 3] = str[i];
    }
    b_cv[9] = '\'';
    b_cv[10] = ',';
    emlrtErrorWithMessageIdR2018a(
        &st, &y_emlrtRTEI,
        "Coder:toolbox:ValidatestringUnrecognizedStringChoice",
        "MATLAB:plan:unrecognizedStringChoice", 9, 4, 10, "SearchMode", 4, 22,
        "\'greedy\', \'exhaustive\'", 4, 11, &b_cv[0]);
  } else {
    out_size[0] = 1;
    out_size[1] = kstr;
    memcpy(&out_data[0], &partial_match_data[0],
           (uint32_T)kstr * sizeof(char_T));
  }
}

/* End of code generation (validatestring.c) */
