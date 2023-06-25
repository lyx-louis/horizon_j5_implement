
#include "object_tracker.h"
#include <stdlib.h>
#include <stdio.h>
#include <memory.h>
#include <string.h>
#include "kalman_tracker.h"
#include "hungarian_algorithm.h"

#define DEFAULT_TRACK_NUM 20
#define DEFAULT_DETECT_NUM 20
#define MAX_TRACKER_NUM 100

#define MAX_AGE 7
#define MIN_HITS 3
#define IOU_THRESHOLD 0.3f
#define MERGE_THRESHOLD 0.5f

typedef struct ObjectTracker
{
   int frame_count;
   int max_age;
   int min_hits;
   float iouThreshold;
   void *buffer;
   int bufferSize;
   float mergeBoxThreshold;
   KalmanTracker *trackers[MAX_TRACKER_NUM];
   int trackerIdMap[MAX_TRACKER_NUM];
   int trackerNum;
   int w;
   int h;
   RECT_F predictedBoxes[MAX_TRACKER_NUM];
} ObjectTracker;

extern KMTrackClass kmTrackIdxTab[MAX_OBJECT_CLASS_NUM];

static CV_RECT rectF2CvRect(RECT_F rectf)
{
   CV_RECT rect;
   // rect.x = (int)rectf.x;
   // rect.y = (int)rectf.y;
   // rect.width = (rectf.width + 0.5);
   // rect.height = (rectf.height + 0.5);
   rect.x = rectf.x;
   rect.y = rectf.y;
   rect.width = (rectf.width + 0.5);
   rect.height = (rectf.height + 0.5);
   return rect;
}

static RECT_F cvRect2RectF(CV_RECT rect)
{
   RECT_F rectf;
   rectf.x = (float)rect.x;
   rectf.y = (float)rect.y;
   rectf.width = (float)rect.width;
   rectf.height = (float)rect.height;
   return rectf;
}

// Computes IOU between two bounding boxes
static float GetIOU(RECT_F a, RECT_F b)
{
   float x1 = MAX(a.x, b.x);
   float y1 = MAX(a.y, b.y);
   float width = MIN(a.x + a.width,
                     b.x + b.width) -
                 x1;
   float height = MIN(a.y + a.height,
                      b.y + b.height) -
                  y1;

   float in = (width > 0 && height > 0) ? width * height : 0;
   float un = a.width * a.height + b.width * b.height - in;
   float ret = 0;
   if (un <= 0)
      return 0;
   ret = in / un;
   return ret;
   return CLIP(ret, 0, 1);
}

// float GetIOU(RECT_F *a, RECT_F b)
// {
//    float x1 = MAX(a->x, b.x);
//    float y1 = MAX(a->y, b.y);
//    float width = MIN(a->x + a->width,
//                      b.x + b.width) -
//                  x1;
//    float height = MIN(a->y + a->height,
//                       b.y + b.height) -
//                   y1;

//    a->height = b.height;
//    a->width = b.width;
//    float in = (width > 0 && height > 0) ? width * height : 0;
//    float un = a->width * a->height + b.width * b.height - in;
//    float ret = 0;
//    if (un <= 0)
//       return 0;
//    ret = in / un;
//    return ret;
//    // return CLIP(ret, 0, 1);
// }

#define ObjectTracker_getBufferSize(trkNum, detNum) \
   (HungarianAlgorithm_getBufferSize((trkNum), (detNum)) + sizeof(float) * (trkNum) * (detNum) + sizeof(int) * (trkNum))

void *ObjectTracker_create()
{
   ObjectTracker *o = (ObjectTracker *)malloc(sizeof(ObjectTracker));
   if (o == NULL)
      return NULL;
   o->frame_count = 0;
   o->max_age = MAX_AGE;
   o->min_hits = MIN_HITS;
   o->iouThreshold = IOU_THRESHOLD;
   o->mergeBoxThreshold = MERGE_THRESHOLD;
   o->bufferSize = ObjectTracker_getBufferSize(DEFAULT_TRACK_NUM, DEFAULT_DETECT_NUM);
   o->buffer = malloc(o->bufferSize);
   if (o->buffer == NULL)
      o->bufferSize = 0;
   memset(o->trackers, 0, MAX_TRACKER_NUM * sizeof(KalmanTracker *));
   memset(o->trackerIdMap, 0, MAX_TRACKER_NUM * sizeof(int));
   o->trackerNum = 0;

   return o;
}

void ObjectTracker_release(void *handle)
{
   ObjectTracker *o = (ObjectTracker *)handle;
   if (o != NULL)
   {
      int i;
      free(o->buffer);
      for (i = 0; i < o->trackerNum; i++)
      {
         KalmanTracker_release(o->trackers[i]);
         o->trackers[i] = NULL;
      }
      free(o);
   }
}

static void _adjustKalmanTrackers(ObjectTracker *o)
{
   int i, j;
   for (i = 0, j = 0; i < o->trackerNum; i++)
   {
      if (o->trackers[i] != NULL)
      {
         o->trackers[j] = o->trackers[i];
         j++;
      }
   }
   o->trackerNum = j;
}

#define BOOL2SIGN(x) ((x) ? 1 : -1)

static int ObjectTracker_getTrackBoxes(ObjectTracker *o, TrackingBox trackBoxes[], int maxTrackerNum)
{
   // get trackers' output
   int i, j, trkNum = 0;

   for (i = 0; i < o->trackerNum; i++)
   {
      if (kmTrackIdxTab[o->trackers[i]->m_clsId] == KMTRACK_frontRear)
      {
         int max_index = -1;
         float max_iou = 0.f;
         for (j = 0; j < o->trackerNum; j++)
         {
            if (kmTrackIdxTab[o->trackers[j]->m_clsId] == KMTRACK_vehicle)
            {
               RECT_F track_xy = KalmanTracker_getState(o->trackers[i]);
               float iou = GetIOU((track_xy), KalmanTracker_getState(o->trackers[j]));
               if (max_iou < iou)
               {
                  max_iou = iou;
                  max_index = j;
               }
            }
         }
         if (max_index != -1)
         {
            o->trackers[max_index]->m_time_since_update = MIN(o->trackers[i]->m_time_since_update, o->trackers[max_index]->m_time_since_update);
         }
      }
   }

   for (i = 0; i < o->trackerNum; i++)
   {
      if (trkNum < maxTrackerNum && o->trackers[i]->m_hits >= o->min_hits && o->trackers[i]->m_time_since_update <= o->max_age)
      {
         TrackingBox *res = &trackBoxes[trkNum++];
         res->accurateBox = KalmanTracker_getState(o->trackers[i]);
         res->box = rectF2CvRect(res->accurateBox);
         res->objId = o->trackers[i]->m_id;
         res->clsId = o->trackers[i]->m_clsId;
         res->prob = BOOL2SIGN(o->trackers[i]->m_isDetected) * o->trackers[i]->m_prob;
         res->belongIdx = -1;
         res->rider = 0;
         res->childIdx = -1;
         res->used3d = FALSE;
         res->fwd = o->trackers[i]->fwd;
      }
      else if (o->trackers[i]->m_time_since_update > o->max_age)
      {
         KalmanTracker_release(o->trackers[i]);
         o->trackers[i] = NULL;
      }
   }
   _adjustKalmanTrackers(o);
   return trkNum;
}

static int ObjectTracker_getPredictedBoxes(ObjectTracker *o, RECT_F *predictedBoxes)
{
   // 3.1. get predicted locations from existing trackers.
   int i, j = 0;
   for (i = 0; i < o->trackerNum; i++)
   {
      RECT_F pBox = KalmanTracker_predict(o->trackers[i]);
      if (!isnan(pBox.x) && !isnan(pBox.y) && !isnan(pBox.width) && !isnan(pBox.height) && pBox.width >= 0 && pBox.height >= 0)
      {
         predictedBoxes[j++] = pBox;
      }
      else
      {
         KalmanTracker_release(o->trackers[i]);
         o->trackers[i] = NULL;
      }
   }
   if (j != o->trackerNum)
      _adjustKalmanTrackers(o);
   return j;
}

BOOL _checkBufferSize(ObjectTracker *o, int trkNum, int detNum)
{
   if (ObjectTracker_getBufferSize(trkNum, detNum) > o->bufferSize)
   {
      o->bufferSize = ObjectTracker_getBufferSize(trkNum + 5, detNum + 5);
      free(o->buffer);
      o->buffer = malloc(o->bufferSize);
      if (o->buffer == NULL)
         o->bufferSize = 0;
   }
   if (o->buffer == NULL)
      return FALSE;
   return TRUE;
}

void ObjectTracker_mergeTrackers(ObjectTracker *o, int trkNum, int *assignment)
{
   // merge iouMatched tracker
   int i, j;
   for (i = 0; i < trkNum; i++)
   {
      for (j = i + 1; j < trkNum; j++)
      {
         if (o->trackers[i] != NULL && o->trackers[j] != NULL && GetIOU((o->predictedBoxes[i]), o->predictedBoxes[j]) >= o->mergeBoxThreshold && kmTrackIdxTab[o->trackers[i]->m_clsId] == kmTrackIdxTab[o->trackers[j]->m_clsId])
         {
            if (assignment[i] == -1 && assignment[j] != -1)
            {
               KalmanTracker_release(o->trackers[i]);
               o->trackers[i] = NULL;
            }
            else if (assignment[i] != -1 && assignment[j] == -1)
            {
               KalmanTracker_release(o->trackers[j]);
               o->trackers[j] = NULL;
            }
            else if (assignment[i] == -1 && assignment[j] == -1)
            {
               if (o->trackers[i]->m_time_since_update < o->trackers[j]->m_time_since_update)
               {
                  KalmanTracker_release(o->trackers[j]);
                  o->trackers[j] = NULL;
               }
               else if (o->trackers[i]->m_time_since_update > o->trackers[j]->m_time_since_update)
               {
                  KalmanTracker_release(o->trackers[i]);
                  o->trackers[i] = NULL;
               }
            }
            else
            {
               // if (iouMatrix[i * detNum + assignment[i]] < iouMatrix[j * detNum + assignment[j]])
               //{
               //	KalmanTracker_release(o->trackers[j]);
               //	o->trackers[j] = NULL;
               // }
               // else {
               //	KalmanTracker_release(o->trackers[i]);
               //	o->trackers[i] = NULL;
               // }
            }
         }
      }
   }
   _adjustKalmanTrackers(o);
}

static void _resetTrackIdMap(ObjectTracker *o)
{
   int i;
   memset(o->trackerIdMap, 0, MAX_TRACKER_NUM * sizeof(int));
   for (i = 0; i < o->trackerNum; i++)
   {
      int id = o->trackers[i]->m_id;
      if (id >= 0 && id < MAX_TRACKER_NUM)
      {
         o->trackerIdMap[id] = 1;
      }
      else
      {
         printf("[%s, %d] logic error! should not enter this code.\n", __FILE__, __LINE__);
      }
   }
}

static int _findNewId(ObjectTracker *o)
{
   int i;
   for (i = 0; i < MAX_TRACKER_NUM; i++)
   {
      if (o->trackerIdMap[i] == 0)
      {
         o->trackerIdMap[i] = 1;
         return i;
      }
   }
   printf("[%s, %d] logic error! should not enter this code.\n", __FILE__, __LINE__);
   return -1;
}

static void ObjectTracker_pushTracker(ObjectTracker *o, DetectBox *detBox)
{
   KalmanTracker *trk = KalmanTracker_create(detBox->rect);
   if (o->trackerNum < MAX_TRACKER_NUM)
   {
      trk->m_prob = detBox->prob;
      trk->m_clsId = detBox->label;
      trk->fwd = detBox->forward;
      trk->m_id = _findNewId(o);
      // trk->
      o->trackers[o->trackerNum++] = trk;
   }
   else
   {
      KalmanTracker_release(trk);
   }
}

int ObjectTracker_updateTrackingBoxes(void *handle, DetectBox *detBoxes, int detNum,
                                      TrackingBox *trkBoxes, int maxTrkNum)
{
   ObjectTracker *o = (ObjectTracker *)handle;
   void *ptr = NULL;
   float *iouMatrix = NULL;
   int *assignment = NULL, *detMark = NULL;
   int trkNum, i, j;

   o->frame_count++;
   if (detBoxes == NULL)
      detNum = 0;

   _resetTrackIdMap(o);

   if (o->trackerNum == 0)
   { // the first frame met
      // initialize kalman trackers using first detections.
      for (i = 0; i < detNum; i++)
      {
         ObjectTracker_pushTracker(o, detBoxes + i);
      }
      return 0;
   }
   // printf("predict\n");
   trkNum = ObjectTracker_getPredictedBoxes(o, o->predictedBoxes);
   if (trkNum == 0)
      return 0;

   ///////////////////////////////////////
   // 3.2. associate detections to tracked object (both represented as bounding boxes)
   if (!_checkBufferSize(o, trkNum, detNum))
      return 0;
   // printf("iouMatrix\n");
   iouMatrix = (float *)o->buffer;
   ptr = (void *)(iouMatrix + trkNum * detNum);
   assignment = (int *)ptr;
   ptr = (void *)(assignment + trkNum);

   // solve the assignment problem using hungarian algorithm.
   for (i = 0; i < trkNum; i++)
   {
      for (j = 0; j < detNum; j++)
      {
         // use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
         iouMatrix[i * detNum + j] = 1 - GetIOU((o->predictedBoxes[i]), detBoxes[j].rect);
      }
   }
   // printf("HungarianAlgorithm_solveWithBuffer\n");
   HungarianAlgorithm_solveWithBuffer(iouMatrix, trkNum, detNum, assignment, NULL, ptr);
   // printf("end\n");
   detMark = (int *)ptr;
   memset(detMark, 0, detNum * sizeof(int));
   // printf("end %d\n", 100);

   // for (i = 0; i < trkNum; i++)
   // {
   //    printf("trackers %d \n", o->trackers[i]->m_clsId);
   // }
   for (i = 0; i < trkNum; i++)
   {
      if (assignment[i] == -1) // tracking box has no matched detected box.
         continue;
      // matched tracking box with detected box
      if (kmTrackIdxTab[o->trackers[i]->m_clsId] == kmTrackIdxTab[detBoxes[assignment[i]].label] &&
          1 - iouMatrix[i * detNum + assignment[i]] >= o->iouThreshold)
      {
         KalmanTracker_update(o->trackers[i], detBoxes[assignment[i]].rect);
         o->trackers[i]->m_clsId = detBoxes[assignment[i]].label;
         o->trackers[i]->m_prob = detBoxes[assignment[i]].prob;
         o->trackers[i]->fwd = detBoxes[assignment[i]].forward;
         detMark[assignment[i]] = 1; // mark the matched detected box
      }
      else
      {
         // printf("end %d\n", 300);
         assignment[i] = -1; // add to unmatched tracking box
      }
   }

   ObjectTracker_mergeTrackers(o, trkNum, assignment);
   for (i = 0; i < detNum; i++)
   {
      if (detMark[i] == 0)
      { // not matched detected box
         ObjectTracker_pushTracker(o, detBoxes + i);
      }
   }
   // printf("ObjectTracker_getTrackBoxes\n");
   return ObjectTracker_getTrackBoxes(o, trkBoxes, maxTrkNum);
}

int ObjectTracker_convertTrackingBoxes(void *handle, DetectBox *detectBoxes, int detNum, TrackingBox *trackBoxes, int maxTrackNum)
{
   int i, num = MIN(detNum, maxTrackNum);
   if (detectBoxes == NULL)
      return 0;

   for (i = 0; i < num; i++)
   {
      trackBoxes[i].box = rectF2CvRect(detectBoxes[i].rect);
      trackBoxes[i].accurateBox = detectBoxes[i].rect;
      trackBoxes[i].prob = detectBoxes[i].prob;
      trackBoxes[i].clsId = detectBoxes[i].label;
      trackBoxes[i].belongIdx = -1;
      trackBoxes[i].used3d = FALSE;
   }
   return num;
}
