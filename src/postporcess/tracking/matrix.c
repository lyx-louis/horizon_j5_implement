#include "matrix.h"

#include <stdlib.h>
#include <malloc.h>
#include <stdio.h>

#ifdef NDEBUG
#define jmAssert(o) (void)0
#else
#define jmAssert(o) do {if (!(o)) *((int*)0) = 0;} while (0)
#endif

#define jmPrintError(msg) perror("[""__FUNCTION__]"msg"\n")
#define NO_MEMORY "No enough memory"

MatType Mat_hypot(MatType a, MatType b)
{
	a = ABS(a);
	b = ABS(b);
	if (a > b) {
		b /= a;
		return a * Mat_sqrt(1 + b*b);
	} else if (b > 0) {
		a /= b;
		return b * Mat_sqrt(1 + a*a);
	} else {
		return 0;
	}
}

typedef struct JM_RNG {
	unsigned long long state;
}JM_RNG;

#define RNG_init(o) ((o)->state = 0xffffffff)
#define RNG_initVal(o, val) ((o)->state = (val) ? (val) : 0xffffffff)
#define RNG_next(o) ((unsigned)((o)->state = (unsigned long long)(unsigned)(o)->state * /*CV_RNG_COEFF*/ 4164903690U \
			 + (unsigned)((o)->state >> 32)))

void* JmMat_createWithPtr(JmMat* o, int rows, int cols, void* ptr)
{
	o->rows = rows;
	o->cols = cols;
	o->data = NULL;
	if (ptr == NULL)
		return NULL;
	o->data = (MatType*)JmMat_getAlignPtr(ptr);
	return (void*)((size_t)o->data + JmMat_getMemorySize(rows, cols));
}

JmMat* JmMat_create(JmMat* o, int rows, int cols)
{	
	o->rows = rows;
	o->cols = cols;
	o->data = (MatType*)malloc(rows * cols * sizeof(MatType));
	if (o->data == NULL)
		return NULL;
	else {
		return o;
	}
}

void JmMat_release(JmMat* o)
{
	if (o != NULL) {
		free(o->data);
		o->data = NULL;
	}
}

// A[mxm]*X[mxn]=b[mxn]
// [A|b] -->(elementary transformation) [U|Y] -->(calculate b)
// b --> X (b stores the answer of X)
static int _LU_Impl(MatType *A, MatType *b, int m, int n, MatType eps)
{
	int i, j, k, p = 1;
	MatType tmp;
	for (i = 0; i < m; i++)
	{
		k = i;

		for (j = i + 1; j < m; j++)
			if (ABS(A[j*m + i]) > ABS(A[k*m + i]))
				k = j;

		if (ABS(A[k*m + i]) < eps)
			return 0;

		if (k != i)
		{
			for (j = i; j < m; j++)
				SWAP(A[i*m + j], A[k*m + j], tmp);
			if (b)
				for (j = 0; j < n; j++)
					SWAP(b[i*n + j], b[k*n + j], tmp);
			p = -p;
		}

		MatType d = -1 / A[i*m + i];

		for (j = i + 1; j < m; j++)
		{
			MatType alpha = A[j*m + i] * d;

			for (k = i + 1; k < m; k++)
				A[j*m + k] += alpha*A[i*m + k];

			if (b)
				for (k = 0; k < n; k++)
					b[j*n + k] += alpha*b[i*n + k];
		}

		A[i*m + i] = -d;
	}

	if (b)
	{
		for (i = m - 1; i >= 0; i--)
			for (j = 0; j < n; j++)
			{
				MatType s = b[i*n + j];
				for (k = i + 1; k < m; k++)
					s -= A[i*m + k] * b[k*n + j];
				b[i*n + j] = s*A[i*m + i];
			}
	}

	return p;
}

// A is a square matrix, B is a right matrix
// AX = B 
static int JmMat_LU(JmMat* AtoU, JmMat* BtoX)
{
#ifndef MAT_DOUBLE
#define LU_NUM 10
#else
#define LU_NUM 100
#endif
	jmAssert(AtoU->rows == AtoU->cols);
	jmAssert(BtoX == NULL || (AtoU->rows == BtoX->rows && BtoX->data != NULL));
	return _LU_Impl(AtoU->data, BtoX ? BtoX->data : NULL, 
		AtoU->cols, BtoX ? BtoX->cols : 0, MAT_EPSILON * LU_NUM);
#undef LU_NUM
}

void JmMat_copy(const JmMat *from, JmMat *to)
{
	int len = from->rows * from->cols, i;
	jmAssert(from->rows == to->rows && from->cols == to->cols);
	if (from->data == to->data)
		return;
	for (i = 0; i < len; i++) {
		to->data[i] = from->data[i];
	}
}

void JmMat_print(const JmMat *o)
{
	int row, col;
	printf("Mat %dx%d:\n", o->rows, o->cols);
	for (row = 0; row < o->rows; row++) {
		for (col = 0; col < o->cols; col++) {
			printf("%.4f\t", o->data[row * o->cols + col]);
		}
		printf("\n");
	}
}

void JmMat_add(const JmMat *A, const JmMat *B, JmMat *C)
{
	int len, i;
	jmAssert(A->rows == B->rows && B->rows == C->rows &&
		A->cols == B->cols && B->cols == C->cols);

	len = A->rows * A->cols;
	for (i = 0; i < len; i++) {
		C->data[i] = A->data[i] + B->data[i];
	}
}

void JmMat_sub(const JmMat *A, const JmMat *B, JmMat *C)
{
	int len, i;
	jmAssert(A->rows == B->rows && B->rows == C->rows &&
		A->cols == B->cols && B->cols == C->cols);

	len = A->rows * A->cols;
	for (i = 0; i < len; i++) {
		C->data[i] = A->data[i] - B->data[i];
	}
}

void JmMat_multiply(const JmMat *A, const JmMat *B, JmMat *C)
{
	int m, n, k, M = A->rows, K = A->cols, N = C->cols;
	jmAssert(A->rows == C->rows && A->cols == B->rows && C->cols == B->cols);

	for (m = 0; m < M; m++) {
		for (n = 0; n < N; n++) {
			register MatType tmp = 0;
			for (k = 0; k < K; k++) 
				tmp += A->data[m * K + k] * B->data[k * N + n];
			C->data[m * N + n] = tmp;
		}
	}
}

BOOL JmMat_invert(const JmMat *A, JmMat *invA, int method)
{
	BOOL result = FALSE;
	void* buffer = malloc(JmMat_getInvertBufferSize(A, invA, method));
	if (buffer == NULL) {
		jmPrintError(NO_MEMORY);
		return FALSE;
	}
	result = JmMat_invertWithBuffer(A, invA, method, buffer);
	free(buffer);
	return result;
}

int JmMat_getInvertBufferSize(const JmMat *A, JmMat *invA, int method)
{
	return JmMat_getSolveBufferSize(A, NULL, invA, method);
}

BOOL JmMat_invertWithBuffer(const JmMat *A, JmMat *invA, int method, void* buffer)
{
	jmAssert(A->rows == invA->cols && A->cols == invA->rows);
	jmAssert(method == JM_DECOMP_SVD || A->rows == A->cols);
	return JmMat_solveWithBuffer(A, NULL, invA, method, buffer);
}

void JmMat_scale(const JmMat *A, JmMat* scaleA, MatType scale)
{
	int size = A->rows * A->cols, i;
	jmAssert(A->rows == scaleA->rows && A->cols == scaleA->cols);
	for (i = 0; i < size; i++)
		scaleA->data[i] = scale * A->data[i];
}

void JmMat_transposeScale(const JmMat *A, JmMat* scaleAt, MatType scale)
{
	int i, j, m = A->rows, n = A->cols;
	jmAssert(A->rows == scaleAt->cols && A->cols == scaleAt->rows);
	if (scale == 1) {
		JmMat_transpose(A, scaleAt);
		return;
	}

	if (A->data == scaleAt->data) {
		jmAssert(m == n);
		for (i = 0; i < m; i++) {
			for (j = i + 1; j < m; j++) {
				register MatType tmp = A->data[j * m + i];
				A->data[j * m + i] = scale * A->data[i * m + j];
				A->data[i * m + j] = scale * tmp;
			}
		}
	}
	else {
		for (i = 0; i < m; i++)
			for (j = 0; j < n; j++)
				scaleAt->data[j * m + i] = scale * A->data[i * n + j];
	}
}

void JmMat_transpose(const JmMat *A, JmMat* At)
{
	int i, j, m = A->rows, n = A->cols;
	jmAssert(A->rows == At->cols && A->cols == At->rows);

	if (A->data == At->data) {
		jmAssert(m == n);
		for (i = 0; i < m; i++) {
			for (j = i + 1; j < m; j++) {
				register MatType tmp = A->data[j * m + i];
				A->data[j * m + i] = A->data[i * m + j];
				A->data[i * m + j] = tmp;
			}
		}
	}
	else {
		for (i = 0; i < m; i++)
			for (j = 0; j < n; j++)
				At->data[j * m + i] = A->data[i * n + j];
	}
}

void JmMat_setValue(JmMat *o, const MatType values[])
{
	int len = o->rows * o->cols, i;
	for (i = 0; i < len; i++)
		o->data[i] = values[i];
}

void JmMat_zeros(JmMat *o)
{
	int len = o->rows * o->cols, i;
	for (i = 0; i < len; i++)
		o->data[i] = 0;
}

void JmMat_ones(JmMat *o)
{
	int len = o->rows * o->cols, i;
	for (i = 0; i < len; i++)
		o->data[i] = 1;
}

void JmMat_eye(JmMat *o)
{
	int i = 0, m = MIN(o->rows, o->cols);
	JmMat_zeros(o);
	for (i = 0; i < m; i++)
		o->data[i * o->cols + i] = 1;
}

void JmMat_setIdentity(JmMat *o, MatType value)
{
	int i = 0, m = MIN(o->rows, o->cols);
	JmMat_zeros(o);
	for (i = 0; i < m; i++)
		o->data[i * o->cols + i] = value;
}

static void gemm_nn(int M, int N, int K, MatType ALPHA, MatType *A, int lda,
	MatType *B, int ldb, MatType *C, int ldc)
{
	int i, j, k;
//#pragma omp parallel for
	for (i = 0; i < M; ++i) {
		for (k = 0; k < K; ++k) {
			register MatType A_PART = ALPHA*A[i*lda + k];
			for (j = 0; j < N; ++j) {
				C[i*ldc + j] += A_PART*B[k*ldb + j];
			}
		}
	}
}

static void gemm_nt(int M, int N, int K, MatType ALPHA, MatType *A, int lda,
	MatType *B, int ldb, MatType *C, int ldc)
{
	int i, j, k;
	//#pragma omp parallel for
	for (i = 0; i < M; ++i) {
		for (j = 0; j < N; ++j) {
			register MatType sum = 0;
			for (k = 0; k < K; ++k) {
				sum += ALPHA*A[i*lda + k] * B[j*ldb + k];
			}
			C[i*ldc + j] += sum;
		}
	}
}

static void gemm_tn(int M, int N, int K, MatType ALPHA, MatType *A, int lda, 
	MatType *B, int ldb, MatType *C, int ldc)
{
	int i, j, k;
//#pragma omp parallel for
	for (i = 0; i < M; ++i) {
		for (k = 0; k < K; ++k) {
			register MatType A_PART = ALPHA*A[k*lda + i];
			for (j = 0; j < N; ++j) {
				C[i*ldc + j] += A_PART*B[k*ldb + j];
			}
		}
	}
}

static void gemm_tt(int M, int N, int K, MatType ALPHA, MatType *A, int lda,
	MatType *B, int ldb, MatType *C, int ldc)
{
	int i, j, k;
//#pragma omp parallel for
	for (i = 0; i < M; ++i) {
		for (j = 0; j < N; ++j) {
			register MatType sum = 0;
			for (k = 0; k < K; ++k) {
				sum += ALPHA*A[i + k*lda] * B[k + j*ldb];
			}
			C[i*ldc + j] += sum;
		}
	}
}

void JmMat_gemm(const JmMat *A, const JmMat *B, MatType alpha, 
	const JmMat *C, MatType beta, JmMat *D, int flag)
{
	int m, n, k;
	MatType* a, *b, *d;
	int lda, ldb, ldd;
	lda = A->cols, ldb = B->cols, ldd = D->cols;
	a = A->data, b = B->data, d = D->data;
	m = D->rows, n = D->cols;
	k = (flag & JM_GEMM_AT) ? A->rows : A->cols;

	jmAssert(((flag & JM_GEMM_AT) != 0 && A->cols == m) 
		|| ((flag & JM_GEMM_AT) == 0 && A->rows == m));
	jmAssert(((flag & JM_GEMM_BT) != 0 && B->rows == n && B->cols == k) 
		|| ((flag & JM_GEMM_BT) == 0 && B->rows == k && B->cols == n));

	if (C != NULL) {
		jmAssert(((flag & JM_GEMM_CT) != 0 && C->rows == n && C->cols == m)
			|| ((flag & JM_GEMM_CT) == 0 && C->rows == m && C->cols == n));

		(flag & JM_GEMM_CT) ? JmMat_transposeScale(C, D, beta) : JmMat_scale(C, D, beta);		
	}
		
	flag &= ~JM_GEMM_CT;
	switch (flag) {
	case JM_GEMM_AT:
		gemm_tn(m, n, k, alpha, a, lda, b, ldb, d, ldd);
		break;
	case JM_GEMM_BT:
		gemm_nt(m, n, k, alpha, a, lda, b, ldb, d, ldd);
		break;
	case JM_GEMM_AT + JM_GEMM_BT:
		gemm_tt(m, n, k, alpha, a, lda, b, ldb, d, ldd);
		break;
	default:
		gemm_nn(m, n, k, alpha, a, lda, b, ldb, d, ldd);
		break;
	}
}

// A[mxn] = U[mxn] * diag(W[nxn]) * V[nxn]t
// At--> At[nxm] 
static void _SVD_Impl(MatType* At, MatType* W, MatType* Vt, int m, int n, int n1, MatType minval, MatType eps)
{
	JM_RNG rng;
	int i, j, k, iter, max_iter = MAX(m, 30);	

	for (i = 0; i < n; i++) {
		MatType sd;
		for (k = 0, sd = 0; k < m; k++) {
			MatType t = At[i*m + k];
			sd += t*t;
		}
		W[i] = sd;

		if (Vt) {
			for (k = 0; k < n; k++)
				Vt[i*n + k] = 0;
			Vt[i*n + i] = 1;
		}
	}

	for (iter = 0; iter < max_iter; iter++) {
		BOOL changed = FALSE;
		for (i = 0; i < n - 1; i++)
			for (j = i + 1; j < n; j++) {
				MatType *Ai = At + i*m, *Aj = At + j*m;
				MatType a = W[i], p = 0, b = W[j];
				MatType beta, gamma;
				MatType c, s;
				
				for (k = 0; k < m; k++)
					p += Ai[k] * Aj[k];

				if (ABS(p) <= eps*Mat_sqrt(a*b))
					continue;

				p *= 2;
				beta = a - b;
				gamma = Mat_hypot(p, beta);
				if (beta < 0) {
					MatType delta = (gamma - beta)*(MatType)0.5;
					s = Mat_sqrt(delta / gamma);
					c = p / (gamma*s * 2);
				}
				else {
					c = Mat_sqrt((gamma + beta) / (gamma * 2));
					s = p / (gamma*c * 2);
				}

				a = b = 0;
				for (k = 0; k < m; k++) {
					MatType t0 = c*Ai[k] + s*Aj[k];
					MatType t1 = -s*Ai[k] + c*Aj[k];
					Ai[k] = t0; Aj[k] = t1;

					a += t0*t0; b += t1*t1;
				}
				W[i] = a; W[j] = b;

				changed = TRUE;

				if (Vt) {
					MatType *Vi = Vt + i*n, *Vj = Vt + j*n;
					for (k = 0; k < n; k++) {
						MatType t0 = c*Vi[k] + s*Vj[k];
						MatType t1 = -s*Vi[k] + c*Vj[k];
						Vi[k] = t0; Vj[k] = t1;
					}
				}
			}
		if (!changed)
			break;
	}

	for (i = 0; i < n; i++) {
		MatType sd;
		for (k = 0, sd = 0; k < m; k++) {
			MatType t = At[i*m + k];
			sd += t*t;
		}
		W[i] = Mat_sqrt(sd);
	}

	for (i = 0; i < n - 1; i++) {
		j = i;
		for (k = i + 1; k < n; k++) {
			if (W[j] < W[k])
				j = k;
		}
		if (i != j) {
			MatType t;
			SWAP(W[i], W[j], t);
			if (Vt) {
				for (k = 0; k < m; k++)
					SWAP(At[i*m + k], At[j*m + k], t);

				for (k = 0; k < n; k++)
					SWAP(Vt[i*n + k], Vt[j*n + k], t);
			}
		}
	}

	if (!Vt)
		return;

	RNG_initVal(&rng, 0x12345678);
	for (i = 0; i < n1; i++) {
		int ii;
		MatType s, sd = i < n ? W[i] : 0;

		for (ii = 0; ii < 100 && sd <= minval; ii++) {
			// if we got a zero singular value, then in order to get the corresponding left singular vector
			// we generate a random vector, project it to the previously computed left singular vectors,
			// subtract the projection and normalize the difference.
			const MatType val0 = (MatType)1. / m;
			for (k = 0; k < m; k++) {
				MatType val = (RNG_next(&rng) & 256) != 0 ? val0 : -val0;
				At[i*m + k] = val;
			}
			for (iter = 0; iter < 2; iter++) {
				for (j = 0; j < i; j++) {
					sd = 0;
					for (k = 0; k < m; k++)
						sd += At[i*m + k] * At[j*m + k];
					MatType asum = 0;
					for (k = 0; k < m; k++) {
						MatType t = At[i*m + k] - sd*At[j*m + k];
						At[i*m + k] = t;
						asum += ABS(t);
					}
					asum = asum > eps * 100 ? 1 / asum : 0;
					for (k = 0; k < m; k++)
						At[i*m + k] *= asum;
				}
			}
			sd = 0;
			for (k = 0; k < m; k++) {
				MatType t = At[i*m + k];
				sd += t*t;
			}
			sd = Mat_sqrt(sd);
		}

		s = (sd > minval ? 1 / sd : 0);
		for (k = 0; k < m; k++)
			At[i*m + k] *= s;
	}
}

/* y[0:m,0:n] += diag(a[0:1,0:m]) * x[0:m,0:n] */
static void MatrAXPY(int m, int n, const MatType* x, int dx,
	const MatType* a, int inca, MatType* y, int dy)
{
	int i, j;
	for (i = 0; i < m; i++, x += dx, y += dy) {
		MatType s = a[i*inca];
		for (j = 0; j < n; j++)
			y[j] = (y[j] + s*x[j]);
	}
}

// A[mxn] = U[mxn] * diag(W[nxn]) * V[nxn]t
// A[mxn] * X[nxk] = b[mxk]
static void _SVBkSb_Impl_(int m, int n, const MatType* w, int incw,
	const MatType* u, int ldu, BOOL uT,
	const MatType* v, int ldv, BOOL vT,
	const MatType* b, int ldb, int nb,
	MatType* x, int ldx, MatType* buffer, MatType eps)
{
	MatType threshold = 0;
	int udelta0 = uT ? ldu : 1, udelta1 = uT ? 1 : ldu;
	int vdelta0 = vT ? ldv : 1, vdelta1 = vT ? 1 : ldv;
	int i, j, nm = MIN(m, n);

	if (!b)
		nb = m;

	for (i = 0; i < n; i++)
		for (j = 0; j < nb; j++)
			x[i*ldx + j] = 0;

	for (i = 0; i < nm; i++)
		threshold += w[i*incw];
	threshold *= eps;

	// v * inv(w) * uT * b
	for (i = 0; i < nm; i++, u += udelta0, v += vdelta0) {
		MatType wi = w[i*incw];
		if (ABS(wi) <= threshold)
			continue;
		wi = 1 / wi;

		if (nb == 1) {
			MatType s = 0;
			if (b)
				for (j = 0; j < m; j++)
					s += u[j*udelta1] * b[j*ldb];
			else
				s = u[0];
			s *= wi;

			for (j = 0; j < n; j++)
				x[j*ldx] = x[j*ldx] + s*v[j*vdelta1];
		}
		else {
			if (b) {
				for (j = 0; j < nb; j++)
					buffer[j] = 0;
				MatrAXPY(m, nb, b, ldb, u, udelta1, buffer, 0);
				for (j = 0; j < nb; j++)
					buffer[j] *= wi;
			}
			else {
				for (j = 0; j < nb; j++)
					buffer[j] = u[j*udelta1] * wi;
			}
			MatrAXPY(n, nb, buffer, 0, v, vdelta1, x, ldx);
		}
	}
}

static void JmMat_SVD(JmMat* At, JmMat* W, JmMat* Vt)
{
#ifndef MAT_DOUBLE
#define SVD_NUM 2
#else
#define SVD_NUM 10
#endif
	_SVD_Impl(At->data, W->data, !Vt ? NULL : Vt->data,
		At->cols, At->rows, !Vt ? 0 : At->rows, MAT_MIN, MAT_EPSILON *  SVD_NUM);
#undef SVD_NUM
}

static void JmMat_SVBkSb(int m, int n, const JmMat* W, const JmMat* u, BOOL uT,
	const JmMat* v, BOOL vT, const JmMat* b, JmMat* x, void* buffer)
{
	_SVBkSb_Impl_(m, n, W->data, 1, u->data, u->cols, uT,
		v->data, v->cols, vT, b ? b->data : NULL, b ? b->cols : 0, b ? b->cols : 0,
		x->data, x->cols, (MatType*)JmMat_getAlignPtr(buffer), (MatType)(MAT_EPSILON * 2));
}

BOOL JmMat_solveWithBuffer(const JmMat* A, const JmMat* B, JmMat* C, int method, void* buffer)
{
	BOOL result = TRUE;
	// A[mxk] * C[kxn] = B[mxn]
	int M = A->rows, K = A->cols, N = C->cols;
	jmAssert(B == NULL || (M == B->rows && N == B->cols && K == C->rows));
	if (method == JM_DECOMP_LU) {
		JmMat tmp;
		JmMat_createWithPtr(&tmp, K, K, buffer);
		jmAssert(B != NULL || (M == K && K == N));
		if (M == K) {
			JmMat_copy(A, &tmp);
			B != NULL ? JmMat_copy(B, C) : JmMat_eye(C);
		}
		else {
			JmMat_gemm(A, A, 1, NULL, 0, &tmp, JM_GEMM_AT);
			JmMat_gemm(A, B, 1, NULL, 0, C, JM_GEMM_AT);
		}
		if (JmMat_LU(&tmp, C) != 0)
			result = TRUE;
		else {
			JmMat_zeros(C);
			jmPrintError("the matrix A is singular matrix.");
			result = FALSE;
		}
	}
	else if (method == JM_DECOMP_SVD) {
		JmMat Ut, w, Vt;
		buffer = JmMat_createWithPtr(&Ut, K, M, buffer);
		buffer = JmMat_createWithPtr(&w, K, 1, buffer);
		buffer = JmMat_createWithPtr(&Vt, K, K, buffer);
		JmMat_transpose(A, &Ut);
		JmMat_SVD(&Ut, &w, &Vt);
		JmMat_SVBkSb(M, K, &w, &Ut, TRUE, &Vt, TRUE, B, C, buffer);
		result = TRUE;
	}
	else {
		jmAssert(FALSE);
		jmPrintError("the method is undefined.");
		result = FALSE;
	}
	return result;
}

BOOL JmMat_solve(const JmMat* A, const JmMat* B, JmMat* C, int method)
{
	BOOL result;
	void* buffer = malloc(JmMat_getSolveBufferSize(A, B, C, method));
	if (buffer == NULL) {
		jmPrintError(NO_MEMORY);
		return FALSE;
	}

	result = JmMat_solveWithBuffer(A, B, C, method, buffer);
	free(buffer);
	return result;
}

int JmMat_getSolveBufferSize(const JmMat* A, const JmMat* B, const JmMat* C, int method)
{
	return JmMat_getSolveBufferSize2(A->rows, A->cols, C->cols, method);
}

int JmMat_getSolveBufferSize2(int A_rows, int A_cols, int C_cols, int method)
{
	if (method == JM_DECOMP_LU) {
		return JmMat_getMemorySize(A_cols, A_cols);
	}
	else if (method == JM_DECOMP_SVD) {
		return
			  JmMat_getMemorySize(A_cols, A_rows)	/* Ut */
			+ JmMat_getMemorySize(A_cols, 1)		/* w  */
			+ JmMat_getMemorySize(A_cols, A_cols)	/* Vt */
			+ C_cols * sizeof(MatType);				/* SVBkSb buffer*/
	}
	else {
		jmAssert(FALSE);
		jmPrintError("the method is undefined.");
		return 0;
	}
}

