#include "ni_modelframework.h"
#if defined VXWORKS || defined kNIOSLinux
#define DC_P                           DC_P DataSection(".NIVS.defaultparams")
#endif

/*
 * DC_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "DC".
 *
 * Model version              : 1.107
 * Simulink Coder version : 8.11 (R2016b) 25-Aug-2016
 * C source code generated on : Mon Jun 03 19:15:50 2019
 *
 * Target selection: NIVeriStand_Linux_ARM_32.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM 7
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */
#include "DC.h"
#include "DC_private.h"

/* Block parameters (auto storage) */
P_DC_T DC_P = {
  0.7,                                 /* Expression: 0.7
                                        * Referenced by: '<Root>/Integrator'
                                        */
  5.4                                  /* Expression: 5.4
                                        * Referenced by: '<Root>/Integrator1'
                                        */
};

/*========================================================================*
 * NI VeriStand Model Framework code generation
 *
 * Model : DC
 * Model version : 1.107
 * VeriStand Model Framework version : 2017.0.0.143 (2017)
 * Source generated on : Mon Jun 03 19:15:49 2019
 *========================================================================*/
#if defined VXWORKS || defined kNIOSLinux

typedef struct {
  int32_t size;
  int32_t width;
  int32_t basetype;
} NI_ParamSizeWidth;

NI_ParamSizeWidth P_DC_T_sizes[] DataSection(".NIVS.defaultparamsizes") = {
  { sizeof(P_DC_T), 1 },

  { sizeof(real_T), 1, 0 },

  { sizeof(real_T), 1, 0 },
};

#endif
