#ifndef _HUNGARIAN_ALGORITHM_H
#define _HUNGARIAN_ALGORITHM_H
#include "base_type.h"
#ifdef __cplusplus
extern "C"
{
#endif

	DLL_API int HungarianAlgorithm_getBufferSize(int nRows, int nCols);
	DLL_API void* HungarianAlgorithm_solveWithBuffer(const float* distMatrix, int nRows, int nCols, int* assignment, float* cost, void* buffer);
	// 内存分配失败时返回FALSE 
	DLL_API BOOL HungarianAlgorithm_solve(const float* distMatrix, int nRows, int nCols, int* assignment, float* cost);

#ifdef __cplusplus
}
#endif

#endif