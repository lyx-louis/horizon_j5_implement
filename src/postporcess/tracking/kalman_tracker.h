#ifndef _KALMAN_TRACKER_H
#define _KALMAN_TRACKER_H
#include "kalman_filter.h"

#ifdef __cplusplus
extern "C"
{
#endif

	typedef struct KalmanTracker
	{
		int m_time_since_update;
		int m_hits;
		int m_hit_streak;
		int m_age;
		int m_id;
		int m_clsId;
		float m_prob;
		int fwd;
		int m_isDetected;
		void *ptTracker;
		JmKalmanFilter m_kf;
		JmMat m_measurement;
	} KalmanTracker;

	KalmanTracker *KalmanTracker_create(RECT_F initRect);
	void KalmanTracker_release(KalmanTracker *o);

	RECT_F KalmanTracker_getState(KalmanTracker *o);
	RECT_F KalmanTracker_getMeasurement(KalmanTracker *o);
	RECT_F KalmanTracker_predict(KalmanTracker *o);
	void KalmanTracker_update(KalmanTracker *o, RECT_F stateMat);

#ifdef __cplusplus
}
#endif

#endif