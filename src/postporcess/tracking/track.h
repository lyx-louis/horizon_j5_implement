#pragma once

#ifndef TRACK_H_
#define TRACK_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "base_type.h"

#ifndef MAX_OBJECT_NUM
#define MAX_OBJECT_NUM 100
#endif

#ifndef MAX_POINT_NUM
#define MAX_POINT_NUM 100
#endif

#ifndef FCW_SAVE
#define FCW_SAVE 0
#define FCW_SLIDE_ALARM 1
#define FCW_FOLLOW_ALARM 2
#define FCW_HMW_ALARM 3
#define FCW_HMW_DANGER 4
#define FCW_SOFT_TTC_ALARM 5 // L1
#define FCW_TTC_ALARM 6		  // L2
#define FCW_TTC_BRAKE 7		  // L3
#endif

#define VEH_SAME_DIRECTION 0
#define VEH_OPPO_DIRECTION 1
#define VEH_CROSS_DIRECTION 2
#define VEH_CUTIN_DIRECTION 3
#define VEH_CUTOUT_DIRECTION 4

	typedef enum object_class
	{
		car = 1,
		bus = 2,
		truck = 3,
		person = 4,
		bicycle = 5,
		wheel = 6,
		front_rear = 7,
		traffic_sign = 8,
		pure_humanlike = 9,
		tunnel = 10,
		zebra_crossing = 11,
		pavement_signs = 12,
		vehicle_like = 13,
		tricycle = 14,
		vehicle_side = 15,
		gate = 16,
		traffic_light = 17,
		traffic_cone = 18,
		obj_class_max
	} object_class;

#define MAX_OBJECT_CLASS_NUM (obj_class_max)

	typedef struct _TRACK_VERSION_
	{
		int major;
		int minor;
		int patch;
		char build_time[256];
		char description[256];
	} TRACK_VERSION;

	typedef struct _CAM_PARAM_
	{
		float left_length;
		float right_length;
		float cam_height;
		float cam_length;
		float focus_length;
		float center_x;
		float center_y;
		float distortion_x;
		float distortion_y;
		float distortionCoeff[4];
		int frame_rate;
		int needDistortion;
	} CAM_PARAM;

	// SSD roi object定义
	typedef struct DetectBox
	{
		int label;
		int fused;
		float prob;
		int forward;
		RECT_F rect;
	} DetectBox;

	typedef struct DetectBoxes
	{
		int numRect;								// rect个数
		DetectBox boxRects[MAX_OBJECT_NUM]; // 最多支持100个box rect
	} DetectBoxes;

	// LDW
	/*typedef struct _LANE_INFO_
	{
		POINT_F  center_pt;
		CV_POINT left_points[MAX_POINT_NUM];
		CV_POINT right_points[MAX_POINT_NUM];
		int left_pointNum;
		int right_pointNum;
		int left_valid;
		int right_valid;
		int left_lane_type;
		int right_lane_type;
	}LANE_INFO;*/

	typedef struct TagTrackObject
	{
		int objId;
		int label;
		int used3d;
		int center;
		int warn_flag;
		int fused;
		int rider;
		int direction;
		float prob;
		float v_dist;
		float h_dist;
		float v_speed;
		float h_speed;
		float v_acc; // 用于保存基于scale计算的TTC
		float h_acc;
		float width;
		float height;
		float warn_time;
		float prev_v_dist;
		float prev_h_dist;
		float prev_v_speed;
		float prev_h_speed;
		CV_RECT box;
		CV_RECT rear;
		CV_POINT side[4];
	} TTrackObject;

	typedef struct TagTrackObjects
	{
		int obj_num;
		TTrackObject objects[MAX_OBJECT_NUM];
	} TTrackObjects;

	// 相机外参
	typedef struct TagCamParamInfo
	{
		float left_length;
		float right_length;
		float cam_height;
		float cam_length;
		float focus_length;
		float center_x;
		float center_y;
	} CamParamInfo;

	DLL_API void *TrackInit(CAM_PARAM *cam_param, int image_w, int image_h);

	DLL_API int TrackSetParam(void *phandle, CAM_PARAM *cam_param);

	DLL_API int TrackWarnLevel(void *phandle, int level);

	DLL_API const TRACK_VERSION *Track_get_version(void);

	DLL_API int TrackProcess(void *phandle, unsigned char *image, unsigned char *mask, int image_w, int image_h,
									 LANE_INFO *plane, float vehicle_speed, DetectBoxes *detectBoxes,
									 TTrackObjects *trackingBoxes);

	DLL_API void TrackUninit(void *phandle);

#ifdef __cplusplus
}
#endif

#endif
