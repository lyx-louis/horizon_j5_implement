#include "kalman_filter.h"
#include <stdlib.h>
#include <memory.h>
#include <stdio.h>

#define METHOD JM_DECOMP_SVD
int JmKalmanFilter_getMemorySize(int DP, int MP, int CP)
{
	return 
		  JmMat_getMemorySize(DP, 1)  // statePre 
		+ JmMat_getMemorySize(DP, 1)  // statePost
		+ JmMat_getMemorySize(DP, DP) // transitionMatrix
		+ JmMat_getMemorySize(DP, DP) // processNoiseCov
		+ JmMat_getMemorySize(MP, DP) // measurementMatrix
		+ JmMat_getMemorySize(MP, MP) // measurementNoiseCov
		+ JmMat_getMemorySize(DP, DP) // errorCovPre 
		+ JmMat_getMemorySize(DP, DP) // errorCovPost
		+ JmMat_getMemorySize(DP, MP) // gain 
		+ JmMat_getMemorySize(DP, DP) // temp1
		+ JmMat_getMemorySize(MP, DP) // temp2
		+ JmMat_getMemorySize(MP, MP) // temp3
		+ JmMat_getMemorySize(MP, DP) // temp4
		+ JmMat_getMemorySize(MP, 1)  // temp5
		+ JmMat_getMemorySize(DP, CP > 0 ? CP : 0) // controlMatrix 
		+ JmMat_getSolveBufferSize2(MP, MP, DP, METHOD); // solve(temp3, temp2, temp4, METHOD)
}

void* JmKalmanFilter_createWithPtr(JmKalmanFilter* o, int DP, int MP, int CP, void* ptr)
{
	o->buffer = ptr;
	if (ptr == NULL)
		return NULL;

	ptr = (void*)(JmMat_getAlignPtr(ptr)
		+ JmMat_getSolveBufferSize2(MP, MP, DP, METHOD)); // buffer memory
	ptr = JmMat_createWithPtr(&o->statePre, DP, 1, ptr);
	ptr = JmMat_createWithPtr(&o->statePost, DP, 1, ptr);
	ptr = JmMat_createWithPtr(&o->transitionMatrix, DP, DP, ptr);
	ptr = JmMat_createWithPtr(&o->processNoiseCov, DP, DP, ptr);
	ptr = JmMat_createWithPtr(&o->measurementMatrix, MP, DP, ptr);
	ptr = JmMat_createWithPtr(&o->measurementNoiseCov, MP, MP, ptr);
	ptr = JmMat_createWithPtr(&o->errorCovPre, DP, DP, ptr);
	ptr = JmMat_createWithPtr(&o->errorCovPost, DP, DP, ptr);
	ptr = JmMat_createWithPtr(&o->gain, DP, MP, ptr);
	ptr = JmMat_createWithPtr(&o->temp1, DP, DP, ptr);
	ptr = JmMat_createWithPtr(&o->temp2, MP, DP, ptr);
	ptr = JmMat_createWithPtr(&o->temp3, MP, MP, ptr);
	ptr = JmMat_createWithPtr(&o->temp4, MP, DP, ptr);
	ptr = JmMat_createWithPtr(&o->temp5, MP, 1, ptr);
	ptr = JmMat_createWithPtr(&o->controlMatrix,
		CP > 0 ? DP : 0, CP > 0 ? CP : 0, ptr);

	JmMat_zeros(&o->statePre);
	JmMat_zeros(&o->statePost);
	JmMat_eye(&o->transitionMatrix);
	JmMat_eye(&o->processNoiseCov);
	JmMat_zeros(&o->measurementMatrix);
	JmMat_eye(&o->measurementNoiseCov);
	JmMat_zeros(&o->errorCovPre);
	JmMat_zeros(&o->errorCovPost);
	JmMat_zeros(&o->gain);
	JmMat_zeros(&o->controlMatrix);

	return ptr;
}

JmKalmanFilter* JmKalmanFilter_create(JmKalmanFilter* o, int DP, int MP, int CP)
{
	o->buffer = malloc(JmKalmanFilter_getMemorySize(DP, MP, CP));
	if (o->buffer == NULL)
		return NULL;
	JmKalmanFilter_createWithPtr(o, DP, MP, CP, o->buffer);
	return o;
}

void JmKalmanFilter_release(JmKalmanFilter* o)
{
	if (o != NULL) {
		free(o->buffer);
		o->buffer = NULL;
	}
}

const JmMat* JmKalmanFilter_predict(JmKalmanFilter* o, const JmMat* control)
{
	// update the state: x'(k) = A*x(k)
	// statePre = transitionMatrix*statePost;
	JmMat_multiply(&o->transitionMatrix, &o->statePost, &o->statePre);

	//	x'(k) = x'(k) + B*u(k)
	//	if (!control.empty())
	//		statePre += controlMatrix*control;
	if (control != NULL) 
		JmMat_gemm(&o->controlMatrix, control, 1, &o->statePre, 1, &o->statePre, 0);

	// update error covariance matrices: temp1 = A*P(k)
	//temp1 = transitionMatrix*errorCovPost;
	JmMat_multiply(&o->transitionMatrix, &o->errorCovPost, &o->temp1);

	// P'(k) = temp1*At + Q
	//gemm(temp1, transitionMatrix, 1, processNoiseCov, 1, errorCovPre, GEMM_2_T);
	JmMat_gemm(&o->temp1, &o->transitionMatrix, 1, &o->processNoiseCov, 1, &o->errorCovPre, JM_GEMM_BT);

	// handle the case when there will be measurement before the next predict.
	//statePre.copyTo(statePost);
	//errorCovPre.copyTo(errorCovPost);
	JmMat_copy(&o->statePre, &o->statePost);
	JmMat_copy(&o->errorCovPre, &o->errorCovPost);

	return &o->statePre;
}

const JmMat* JmKalmanFilter_correct(JmKalmanFilter* o, const JmMat* measurement)
{
	// temp2 = H*P'(k)
	// temp2 = measurementMatrix * errorCovPre;
	JmMat_multiply(&o->measurementMatrix, &o->errorCovPre, &o->temp2);

	// temp3 = temp2*Ht + R
	// gemm(temp2, measurementMatrix, 1, measurementNoiseCov, 1, temp3, GEMM_2_T);
	JmMat_gemm(&o->temp2, &o->measurementMatrix, 1, &o->measurementNoiseCov, 1, &o->temp3, JM_GEMM_BT);

	// temp4 = inv(temp3)*temp2 = Kt(k)
	// solve(temp3, temp2, temp4, DECOMP_SVD);
	JmMat_solveWithBuffer(&o->temp3, &o->temp2, &o->temp4, METHOD, o->buffer);

	// K(k)
	// gain = temp4.t();
	JmMat_transpose(&o->temp4, &o->gain);

	// temp5 = z(k) - H*x'(k)
	// temp5 = measurement - measurementMatrix*statePre;
	JmMat_gemm(&o->measurementMatrix, &o->statePre, -1, measurement, 1, &o->temp5, 0);

	// x(k) = x'(k) + K(k)*temp5
	// statePost = statePre + gain*temp5;
	JmMat_gemm(&o->gain, &o->temp5, 1, &o->statePre, 1, &o->statePost, 0);

	// P(k) = P'(k) - K(k)*temp2
	// errorCovPost = errorCovPre - gain*temp2;
	JmMat_gemm(&o->gain, &o->temp2, -1, &o->errorCovPre, 1, &o->errorCovPost, 0);

	return &o->statePost;
}

