#ifndef _OBJECT_TRACKING_3D_H
#define _OBJECT_TRACKING_3D_H
#include "base_type.h"
#include "track.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef TRACKING_OBJ_NUM_MAX
#define TRACKING_OBJ_NUM_MAX 100
#endif

	typedef struct TrackingBox
	{
		int objId;
		int clsId;
		float prob; // if prob < 0, it is detected
		int belongIdx;
		int fwd;
		RECT_F accurateBox;
		CV_RECT box;
		BOOL used3d;
		CV_RECT fb;
		CV_POINT side[4];
		int rider;
		int childIdx; // only used in internal code for rider.
	} TrackingBox;

	typedef struct TrackingBoxes
	{
		int numBox;
		TrackingBox trkBoxes[TRACKING_OBJ_NUM_MAX];
	} TrackingBoxes;

	typedef enum DownDirection
	{
		BottomIsDown,
		LeftIsDown,
		RightIsDown,
		TopIsDown
	} DownDirection;

	typedef struct _KM_TRACK_VERSION_
	{
		int major;
		int minor;
		int patch;
		char build_time[256];
		char description[256];
	} KM_TRACK_VERSION;

	DLL_API void *ObjectTracking3D_create();

	DLL_API void ObjectTracking_setDownDirection(void *handle, DownDirection downDirection);

	DLL_API void ObjectTracking3D_tracking(void *handle, DetectBoxes *detectBoxes, TrackingBoxes *trackingBoxes, int isBox3d);

	DLL_API void ObjectTracking3D_release(void *);

	DLL_API const KM_TRACK_VERSION *ObjectTracking3D_getVersion();

#ifdef __cplusplus
}
#endif

#endif