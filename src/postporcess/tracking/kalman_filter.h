#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include "matrix.h"

#ifdef __cplusplus
extern "C"
{
#endif

	typedef struct JmKalmanFilter
	{
		JmMat statePre;		   //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
		JmMat statePost;		   //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
		JmMat transitionMatrix;	   //!< state transition matrix (A)
		JmMat controlMatrix;	   //!< control matrix (B) (not used if there is no control)
		JmMat measurementMatrix;   //!< measurement matrix (H)
		JmMat processNoiseCov;	   //!< process noise covariance matrix (Q)
		JmMat measurementNoiseCov; //!< measurement noise covariance matrix (R)
		JmMat errorCovPre;	   //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
		JmMat gain;			   //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
		JmMat errorCovPost;	   //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)

		// temporary matrices
		JmMat temp1;
		JmMat temp2;
		JmMat temp3;
		JmMat temp4;
		JmMat temp5;
		void *buffer;
	} JmKalmanFilter;

#define JmKalmanFilter_alignSize JmMat_alignSize
#define JmKalmanFilter_getAlignPtr JmMat_getAlignPtr
	int JmKalmanFilter_getMemorySize(int DP, int MP, int CP);

	// return next buffer pointer
	void *JmKalmanFilter_createWithPtr(JmKalmanFilter *o, int DP, int MP, int CP, void *ptr);

	JmKalmanFilter *JmKalmanFilter_create(JmKalmanFilter *o, int DP, int MP, int CP);
	void JmKalmanFilter_release(JmKalmanFilter *o);

	const JmMat *JmKalmanFilter_predict(JmKalmanFilter *o, const JmMat *control);
	const JmMat *JmKalmanFilter_correct(JmKalmanFilter *o, const JmMat *measurement);

#ifdef __cplusplus
}
#endif

#endif