/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * codegenPathPlanner_types.h
 *
 * Code generation for function 'codegenPathPlanner'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_c_matlabshared_autonomous_inter
#define typedef_c_matlabshared_autonomous_inter
typedef struct {
  real_T GridOriginInLocal[2];
  real_T LocalOriginInWorld[2];
} c_matlabshared_autonomous_inter;
#endif /* typedef_c_matlabshared_autonomous_inter */

#ifndef typedef_d_matlabshared_autonomous_inter
#define typedef_d_matlabshared_autonomous_inter
typedef struct {
  real_T Head[2];
} d_matlabshared_autonomous_inter;
#endif /* typedef_d_matlabshared_autonomous_inter */

#ifndef typedef_e_matlabshared_autonomous_inter
#define typedef_e_matlabshared_autonomous_inter
typedef struct {
  d_matlabshared_autonomous_inter *Index;
  boolean_T Buffer[10000];
} e_matlabshared_autonomous_inter;
#endif /* typedef_e_matlabshared_autonomous_inter */

#ifndef typedef_binaryOccupancyMap
#define typedef_binaryOccupancyMap
typedef struct {
  c_matlabshared_autonomous_inter SharedProperties;
  d_matlabshared_autonomous_inter Index;
  e_matlabshared_autonomous_inter Buffer;
  boolean_T DefaultValueInternal;
  boolean_T HasParent;
} binaryOccupancyMap;
#endif /* typedef_binaryOccupancyMap */

#ifndef typedef_stateSpaceSE2
#define typedef_stateSpaceSE2
typedef struct {
  char_T Name[3];
  real_T StateBoundsInternal[6];
  real_T WeightXY;
  real_T WeightTheta;
  boolean_T SkipStateValidation;
} stateSpaceSE2;
#endif /* typedef_stateSpaceSE2 */

#ifndef typedef_validatorOccupancyMap
#define typedef_validatorOccupancyMap
typedef struct {
  stateSpaceSE2 *StateSpace;
  binaryOccupancyMap *Map;
  real_T ValidationDistance;
  real_T XYIndices[2];
  boolean_T SkipStateValidation;
  boolean_T ValidMatrix[10000];
  real_T MapBounds[4];
} validatorOccupancyMap;
#endif /* typedef_validatorOccupancyMap */

#ifndef typedef_c_nav_algs_internal_plannerASta
#define typedef_c_nav_algs_internal_plannerASta
typedef struct {
  real_T Map[10000];
  real_T TieBreaker;
  real_T DiagonalSearchFlag;
  real_T ObstacleThreshold;
  real_T MapResolution;
  real_T PathIndicesInternal[10000];
  real_T PathInternal[20000];
  real_T PathCost;
  real_T NumPathPoints;
  real_T NodesExploredIndicesInternal[10000];
  real_T NumNodesExplored;
  real_T GCostMatrix[10000];
  real_T ParentCol[10000];
  real_T ParentRow[10000];
  real_T MapIndex[10000];
  real_T AllNodes[20000];
  real_T UseCustomG;
  real_T UseCustomH;
  real_T GCostMethod;
  real_T HCostMethod;
} c_nav_algs_internal_plannerASta;
#endif /* typedef_c_nav_algs_internal_plannerASta */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#ifndef typedef_navPath
#define typedef_navPath
typedef struct {
  stateSpaceSE2 *StateSpace;
  real_T NumStates;
  emxArray_real_T *StateInternal;
} navPath;
#endif /* typedef_navPath */

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T
struct emxArray_boolean_T {
  boolean_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_boolean_T */
#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T
typedef struct emxArray_boolean_T emxArray_boolean_T;
#endif /* typedef_emxArray_boolean_T */

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
struct emxArray_int32_T {
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_int32_T */
#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T
typedef struct emxArray_int32_T emxArray_int32_T;
#endif /* typedef_emxArray_int32_T */

#ifndef struct_emxArray_int8_T
#define struct_emxArray_int8_T
struct emxArray_int8_T {
  int8_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_int8_T */
#ifndef typedef_emxArray_int8_T
#define typedef_emxArray_int8_T
typedef struct emxArray_int8_T emxArray_int8_T;
#endif /* typedef_emxArray_int8_T */

#ifndef struct_emxArray_uint16_T
#define struct_emxArray_uint16_T
struct emxArray_uint16_T {
  uint16_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_uint16_T */
#ifndef typedef_emxArray_uint16_T
#define typedef_emxArray_uint16_T
typedef struct emxArray_uint16_T emxArray_uint16_T;
#endif /* typedef_emxArray_uint16_T */

#ifndef typedef_plannerAStarGrid
#define typedef_plannerAStarGrid
typedef struct {
  binaryOccupancyMap *Map;
  real_T TieBreaker;
  real_T DiagonalSearch;
  real_T GCost;
  real_T HCost;
  real_T OccupiedThreshold;
  real_T GCostMatrix[10000];
  real_T IsGrid;
  real_T StartInGrid[2];
  real_T GoalInGrid[2];
  real_T Path[10000];
  real_T PathXY[20000];
  real_T PathInGrid[20000];
  real_T NumPathPoints;
  real_T PathCost;
  real_T NumNodesExplored;
  real_T OccupancyMatrix[10000];
  real_T IdPose[30000];
  real_T PoseId[10000];
  real_T UseCustomG;
  real_T UseCustomH;
  real_T isFirstRun;
} plannerAStarGrid;
#endif /* typedef_plannerAStarGrid */

#ifndef typedef_plannerHybridAStar
#define typedef_plannerHybridAStar
typedef struct {
  binaryOccupancyMap *Map;
  real_T Dimensions[2];
  real_T Neighbors[18];
  real_T GoalPoint[2];
  boolean_T visitedCellsFront[10000];
  boolean_T visitedCellsBack[10000];
  real_T Heuristic2DMat[10000];
  plannerAStarGrid *Heuristic2DObj;
  boolean_T PathFound;
  real_T NumPointsMotionPrimitive;
  real_T ExpansionPoint[3];
  real_T AnalyticPathLength;
  real_T AnalyticPathSegments[5];
  real_T AnalyticPathTypes[5];
  real_T StartPose[3];
  real_T GoalPose[3];
  real_T PrimitivesData[550000];
  real_T LinesData[350000];
  validatorOccupancyMap *StateValidator;
  real_T MinTurningRadius;
  real_T MotionPrimitiveLength;
  real_T ForwardCost;
  real_T ReverseCost;
  real_T DirectionSwitchingCost;
  real_T AnalyticExpansionInterval;
} plannerHybridAStar;
#endif /* typedef_plannerHybridAStar */

#ifndef typedef_d_plannerHybridAStar_getFinalPa
#define typedef_d_plannerHybridAStar_getFinalPa
typedef struct {
  real_T finalPathData[550000];
} d_plannerHybridAStar_getFinalPa;
#endif /* typedef_d_plannerHybridAStar_getFinalPa */

#ifndef typedef_d_plannerAStarGrid_reconstructP
#define typedef_d_plannerAStarGrid_reconstructP
typedef struct {
  real_T OptimalPath[20000];
  real_T pathIndices[10000];
} d_plannerAStarGrid_reconstructP;
#endif /* typedef_d_plannerAStarGrid_reconstructP */

#ifndef typedef_d_plannerAStarGrid_getHeuristic
#define typedef_d_plannerAStarGrid_getHeuristic
typedef struct {
  real_T difference[20000];
  real_T c[20000];
  real_T y[20000];
} d_plannerAStarGrid_getHeuristic;
#endif /* typedef_d_plannerAStarGrid_getHeuristic */

#ifndef typedef_c_plannerAStarGrid_plan
#define typedef_c_plannerAStarGrid_plan
typedef struct {
  real_T GScore[10000];
  real_T FScore[10000];
  real_T Hn2[10000];
} c_plannerAStarGrid_plan;
#endif /* typedef_c_plannerAStarGrid_plan */

#ifndef typedef_b_plannerAStarGrid_planOM
#define typedef_b_plannerAStarGrid_planOM
typedef struct {
  c_nav_algs_internal_plannerASta astarInternal;
  real_T pathXYTemp[20000];
  real_T pathTemp[10000];
  real_T b_pathTemp[10000];
} b_plannerAStarGrid_planOM;
#endif /* typedef_b_plannerAStarGrid_planOM */

#ifndef typedef_d_plannerAStarGrid_plan
#define typedef_d_plannerAStarGrid_plan
typedef struct {
  real_T expl_temp[10000];
} d_plannerAStarGrid_plan;
#endif /* typedef_d_plannerAStarGrid_plan */

#ifndef typedef_e_plannerHybridAStar_calculateC
#define typedef_e_plannerHybridAStar_calculateC
typedef struct {
  real_T minval[10000];
  real_T varargin_1[10000];
  real_T varargin_2[10000];
} e_plannerHybridAStar_calculateC;
#endif /* typedef_e_plannerHybridAStar_calculateC */

#ifndef typedef_e_plannerHybridAStar_get2DHeuri
#define typedef_e_plannerHybridAStar_get2DHeuri
typedef struct {
  real_T minval[10000];
  real_T varargin_1[10000];
  real_T varargin_2[10000];
} e_plannerHybridAStar_get2DHeuri;
#endif /* typedef_e_plannerHybridAStar_get2DHeuri */

#ifndef typedef_b_codegenPathPlanner
#define typedef_b_codegenPathPlanner
typedef struct {
  plannerHybridAStar planner;
  plannerAStarGrid lobj_5;
} b_codegenPathPlanner;
#endif /* typedef_b_codegenPathPlanner */

#ifndef typedef_codegenPathPlannerStackData
#define typedef_codegenPathPlannerStackData
typedef struct {
  union {
    d_plannerHybridAStar_getFinalPa f0;
    d_plannerAStarGrid_reconstructP f1;
    d_plannerAStarGrid_getHeuristic f2;
  } u1;
  c_plannerAStarGrid_plan f3;
  b_plannerAStarGrid_planOM f4;
  d_plannerAStarGrid_plan f5;
  union {
    e_plannerHybridAStar_calculateC f6;
    e_plannerHybridAStar_get2DHeuri f7;
    e_plannerHybridAStar_get2DHeuri f8;
  } u5;
  b_codegenPathPlanner f9;
} codegenPathPlannerStackData;
#endif /* typedef_codegenPathPlannerStackData */

/* End of code generation (codegenPathPlanner_types.h) */
