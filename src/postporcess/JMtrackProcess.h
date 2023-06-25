#pragma once
#include "tracking/object_tracker.h"
#include "zmq/oal_type.h"
#include "projection.h"
#include "comm_api.h"

#define LANE_POINT_NUM 1000
#define BEV_POINT_NUM 100

typedef struct
{
   f32 x = 0.;
   f32 y = 0;
   f32 z = 0.;
   int classId = 100;
   int trackId = 100;
} TF32Pointbev;

typedef struct
{
   f32 x = 0.;
   f32 y = 0.;
} TF32Point;

typedef struct
{
   f32 lanePointNum = LANE_POINT_NUM;
   TF32Point tLanePoint[LANE_POINT_NUM];
   f32 bevPointNum = BEV_POINT_NUM;
   TF32Pointbev tBevPoint[BEV_POINT_NUM][4];
} TShowPoint;

class ObjTrack
{
private:
   void *kalman_tracker; // = ObjectTracker_create();

public:
   void get_MaxMin(s32 objNum, TDetResult &allDetResult, DetectBoxes &detectBoxes);

   void initDetectorBoxs(float xmin, float ymin, float xmax, float ymax,
                         int fwd, int classid, DetectBoxes &detectBoxes);

   void sortPoints(int kalman_obj_num, TShowPoint &tShowPoint, TrackingBox *trackingBoxes);
   void sortPoints(TShowPoint &tShowPoint, DetectBoxes &trackingBoxes);
   void fowardUp(int i, TShowPoint &tShowPoint, TrackingBox *trackingBoxes);
   void fowardButtom(int i, TShowPoint &tShowPoint, TrackingBox *trackingBoxes);
   void fowardLeft(int i, TShowPoint &tShowPoint, TrackingBox *trackingBoxes);
   void fowardRight(int i, TShowPoint &tShowPoint, TrackingBox *trackingBoxes);

public:
   ObjTrack();
   ~ObjTrack();
   void runTrack(s32 objNum, TDetResult &allDetResult, TShowPoint &tShowPoint);
   void runTrack(TShowPoint &tShowPoint, DetectBoxes detectBoxes);
};

int post_init(TCommBuff tGridFile);

TDetResult post_process_test(std::vector<cv::Mat> img_lists, float *ptF32ScoreBuff,
                             float *ptF32BboxBuff, float *ptF32ClassBuff);

TDetResult post_process(float *ptF32ScoreBuff, float *ptF32BboxBuff, float *ptF32ClassBuff);
