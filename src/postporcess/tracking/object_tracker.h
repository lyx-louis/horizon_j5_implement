#ifndef _OBJECT_TRACKER_H
#define _OBJECT_TRACKER_H

#include "base_type.h"
#include "object_tracking_3d.h"
#ifdef __cplusplus
extern "C"
{
#endif

	typedef enum KMTrackClass
	{
		KMTRACK_vehicle = 0,
		KMTRACK_person = 1,
		KMTRACK_bike = 2,
		KMTRACK_trafficSign = 3,
		KMTRACK_wheel = 4,
		KMTRACK_frontRear = 5
	} KMTrackClass;

	void *ObjectTracker_create();

	int ObjectTracker_updateTrackingBoxes(void *handle, DetectBox *detectBoxes, int detNum,
													  TrackingBox *trackBoxes, int maxTrackNum);

	int ObjectTracker_convertTrackingBoxes(void *handle, DetectBox *detectBoxes, int detNum,
														TrackingBox *trackBoxes, int maxTrackNum);

	void ObjectTracker_release(void *handle);

#ifdef __cplusplus
}
#endif
#endif // !_OBJECT_TRACKER_H_
