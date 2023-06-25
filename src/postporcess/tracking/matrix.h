#ifndef _MATRIX_H
#define _MATRIX_H
#include <float.h>
#include <math.h>
#include "base_type.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef MAT_DOUBLE
typedef double MatType;
#define MAT_EPSILON DBL_EPSILON
#define MAT_MIN DBL_MIN
#define MAT_MAX DBL_MAX
#define Mat_sqrt sqrt
#else
typedef float MatType;
#define MAT_EPSILON FLT_EPSILON
#define MAT_MIN FLT_MIN
#define MAT_MAX FLT_MAX
#define Mat_sqrt sqrtf
#endif

typedef struct JmMat {
	MatType* data;
	int rows;
	int cols;
}JmMat;

#ifndef jmAlignPtr
#define jmAlignPtr(ptr, n) (((size_t)(ptr) + (n)-1) & -(n))
#endif

#define JmMat_alignSize() (sizeof(MatType))
#define JmMat_getAlignPtr(ptr) jmAlignPtr((ptr), (int)sizeof(MatType))
#define JmMat_getMemorySize(rows, cols) ((rows) * (cols) * sizeof(MatType))

// 不在内部开辟内存，也不需要调用JmMat_release释放内存
void* JmMat_createWithPtr(JmMat* o, int rows, int cols, void* ptr);

JmMat* JmMat_create(JmMat* o, int rows, int cols);
void JmMat_release(JmMat* o);

/******************************************************************************
矩阵数据深拷贝
matrix_B = matrix_A
******************************************************************************/
void JmMat_copy(const JmMat *from, JmMat *to);

void JmMat_print(const JmMat *o);

/******************************************************************************
matrix_C = matrix_A + matrix_B
******************************************************************************/
void JmMat_add(const JmMat *A, const JmMat *B, JmMat *C);

/******************************************************************************
matrix_C = matrix_A - matrix_B
******************************************************************************/
void JmMat_sub(const JmMat *A, const JmMat *B, JmMat *C);

/******************************************************************************
matrix_C = matrix_A * matrix_B
******************************************************************************/
void JmMat_multiply(const JmMat *A, const JmMat *B, JmMat *C);

/******************************************************************************
matrix_B = invert(matrix_A) 
******************************************************************************/
BOOL JmMat_invert(const JmMat *A, JmMat *invA, int method);
int JmMat_getInvertBufferSize(const JmMat *A, JmMat *invA, int method);
BOOL JmMat_invertWithBuffer(const JmMat *A, JmMat *invA, int method, void* buffer);

/******************************************************************************
matrix_B = scale * matrix_A
******************************************************************************/
void JmMat_scale(const JmMat *A, JmMat* scaleA, MatType scale);

/******************************************************************************
matrix_B = scale * transpose(matrix_A)
当矩阵AB的内存地址一样时，必须AB为方阵，这样才能把转置后的矩阵存储在原内存区
******************************************************************************/
void JmMat_transposeScale(const JmMat *A, JmMat* scaleAt, MatType scale);

// matrix_B = transpose(matrix_A);
void JmMat_transpose(const JmMat *A, JmMat* At);

/******************************************************************************
使用数组初始化矩阵
******************************************************************************/
void JmMat_setValue(JmMat *o, const MatType values[]);

void JmMat_zeros(JmMat *o);

void JmMat_ones(JmMat *o);

void JmMat_eye(JmMat *o);

void JmMat_setIdentity(JmMat *o, MatType value);

// flag: JM_GEMM_AT + JM_GEMM_BT (At, Bt)
// flag: 0                       (A,  B)
// flag: JM_GEMM_AT              (At, B)
// flag: JM_GEMM_BT              (A,  Bt)
// matrix_D = alpha * matrix_A[t] * matrix_B[t] + beta * matrix_C[t]
#define JM_GEMM_AT 1
#define JM_GEMM_BT 2
#define	JM_GEMM_CT 4
void JmMat_gemm(const JmMat *A, const JmMat *B, MatType alpha, 
	const JmMat *C, MatType beta, JmMat *D, int flag);

/******************************************************************************
solve: matrix_A * matrix_C = matrix_B;
matrix_C is the solution of matrix equation.
flag: JM_DECOMP_LU or JM_DECOMP_SVD
******************************************************************************/
#define JM_DECOMP_LU  0
#define JM_DECOMP_SVD 1
BOOL JmMat_solve(const JmMat* A, const JmMat* B, JmMat* C, int method);
int JmMat_getSolveBufferSize(const JmMat* A, const JmMat* B, const JmMat* C, int method);
int JmMat_getSolveBufferSize2(int A_rows, int A_cols, int C_cols, int method);
BOOL JmMat_solveWithBuffer(const JmMat* A, const JmMat* B, JmMat* C, int method, void* buffer);

#ifdef __cplusplus
}
#endif

#endif