#include "kalman_tracker.h"

#include <math.h>
#include <stdlib.h>
#include <malloc.h>
#include <stdio.h>

static const int stateNum = 6;
static const int measureNum = 4;
static const float transitionValues[] = {
	1, 0, 0, 0, 1, 0,
	0, 1, 0, 0, 0, 1,
	0, 0, 1, 0, 0, 0,
	0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 1,
};

#define KalmanTracker_alignSize JmMat_alignSize
#define KalmanTracker_getAlignPtr JmMat_getAlignPtr
int KalmanTracker_getMemorySize();

void* KalmanTracker_createWithPtr(KalmanTracker* o, RECT_F initRect, void* ptr);

// Convert bounding box from [cx,cy,s,r] to [x,y,w,h] style.
static RECT_F _convertCxXySR2Rect(float cx, float cy, float s, float r)
{
	RECT_F res;
	float w, h, x, y;
	res.x = res.y = res.width = res.height = -1;
	if (s > 0 && r > 0) {
		w = sqrtf(s * r);
		h = s / w;
		x = (cx - w / 2);
		y = (cy - h / 2);
		//if (x < 0 && cx > 0)
		//	x = 0;
		//if (y < 0 && cy > 0)
		//	y = 0;
		res.x = x; res.y = y; res.width = w; res.height = h;
	}

	return res;
}

int KalmanTracker_getMemorySize()
{
	return JmMat_getMemorySize(measureNum, 1) // measurement size
		// + JmKalmanFilter_alignSize()           // m_kf align size
		+ JmKalmanFilter_getMemorySize(stateNum, measureNum, 0);// m_kf size
}

static void _initKalmanTracker(KalmanTracker* o, const RECT_F *initRect)
{
	JmMat_zeros(&o->m_measurement);
	JmMat_setValue(&o->m_kf.transitionMatrix, transitionValues);
	JmMat_eye(&o->m_kf.measurementMatrix);
	JmMat_setIdentity(&o->m_kf.processNoiseCov, 1e-3f);
	JmMat_setIdentity(&o->m_kf.measurementNoiseCov, 1e-3f);
	JmMat_setIdentity(&o->m_kf.errorCovPost, 1.f);

	o->m_kf.statePost.data[0] = (float)initRect->x + (float)initRect->width / 2;
	o->m_kf.statePost.data[1] = (float)initRect->y + (float)initRect->height / 2;
	o->m_kf.statePost.data[2] = (float)initRect->width * (float)initRect->height;
	o->m_kf.statePost.data[3] = (float)initRect->width / (float)initRect->height;

	o->m_time_since_update = 0;
	o->m_hits = 0;
	o->m_hit_streak = 0;
	o->m_age = 0;
	o->m_id = -1;
}

void* KalmanTracker_createWithPtr(KalmanTracker* o, RECT_F initRect, void* ptr)
{
	ptr = JmMat_createWithPtr(&o->m_measurement, measureNum, 1, ptr);
	ptr = JmKalmanFilter_createWithPtr(&o->m_kf, stateNum, measureNum, 0, ptr);
	_initKalmanTracker(o, &initRect);
	return ptr;
}

KalmanTracker* KalmanTracker_create(RECT_F initRect)
{
	KalmanTracker* o = (KalmanTracker*)malloc(sizeof(KalmanTracker));
	if (o == NULL)
		return NULL;
	if (NULL == JmMat_create(&o->m_measurement, measureNum, 1)) {
		free(o);
		return NULL;
	}
	if (NULL == JmKalmanFilter_create(&o->m_kf, stateNum, measureNum, 0)) {
		JmMat_release(&o->m_measurement);
		free(o);
		return NULL;
	}
	_initKalmanTracker(o, &initRect);
	return o;
}

void KalmanTracker_release(KalmanTracker* o)
{
	if (o != NULL) {
		//free(o->buffer);
		JmMat_release(&o->m_measurement);
		JmKalmanFilter_release(&o->m_kf);
		free(o);
	}
}

// Predict the estimated bounding box.
RECT_F KalmanTracker_predict(KalmanTracker* o)
{
	// predict
	const JmMat* p = JmKalmanFilter_predict(&o->m_kf, NULL);
	o->m_age += 1;
	o->m_isDetected = FALSE;
	if (o->m_time_since_update > 0)
		o->m_hit_streak = 0;
	o->m_time_since_update += 1;

	//printf("predict[%.2f %.2f %.2f %.2f]\n\n", p->data[0], p->data[1], p->data[2], p->data[3]);
	return _convertCxXySR2Rect(p->data[0], p->data[1], p->data[2], p->data[3]);
}

// Update the state vector with observed bounding box.
void KalmanTracker_update(KalmanTracker* o, RECT_F stateMat)
{
	//const JmMat* p;
	o->m_hits += 1;
	o->m_isDetected = TRUE;
	o->m_time_since_update = 0;

	o->m_hit_streak += 1;

	// measurement
	o->m_measurement.data[0] = (float)stateMat.x + (float)stateMat.width / 2;
	o->m_measurement.data[1] = (float)stateMat.y + (float)stateMat.height / 2;
	o->m_measurement.data[2] = (float)stateMat.width * (float)stateMat.height;
	o->m_measurement.data[3] = (float)stateMat.width / (float)stateMat.height;

	// update
	/*p = */JmKalmanFilter_correct(&o->m_kf, &o->m_measurement);
	//printf("correct[%.2f %.2f %.2f %.2f]\n\n", p->data[0], p->data[1], p->data[2], p->data[3]);
}
RECT_F KalmanTracker_getMeasurement(KalmanTracker* o)
{
	return _convertCxXySR2Rect(o->m_measurement.data[0], o->m_measurement.data[1],
		o->m_measurement.data[2], o->m_measurement.data[3]);
}

// Return the current state vector
RECT_F KalmanTracker_getState(KalmanTracker* o)
{
	//printf("state[%.2f %.2f %.2f %.2f]\n\n", o->m_kf.statePost.data[0], o->m_kf.statePost.data[1],
	//	o->m_kf.statePost.data[2], o->m_kf.statePost.data[3]);
	return _convertCxXySR2Rect(o->m_kf.statePost.data[0], o->m_kf.statePost.data[1], 
		o->m_kf.statePost.data[2], o->m_kf.statePost.data[3]);
}
