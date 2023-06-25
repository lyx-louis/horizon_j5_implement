#include "JMtrackProcess.h"
#include <iostream>

#define addWidth 1.5
#define addHeight 2.5

void ObjTrack::get_MaxMin(s32 objNum, TDetResult &allDetResult, DetectBoxes &detectBoxes)
{

   for (int i = 0; i < objNum; i++)
   {
      float xmin = 0;
      float ymin = 0;

      float xmax = 0;
      float ymax = 0;

      float x0, y0;
      int class_id = allDetResult.bev.bev[i].cls;
      // std::cout << "class id : " << classID << std::endl;
      for (int j = 0; j < 4; j++)
      {
         if (j == 0)
         {
            x0 = -allDetResult.bev.bev[i].point[j].y;
            y0 = allDetResult.bev.bev[i].point[j].x;

            xmin = -allDetResult.bev.bev[i].point[j].y;
            xmax = -allDetResult.bev.bev[i].point[j].y;

            ymin = allDetResult.bev.bev[i].point[j].x;
            ymax = allDetResult.bev.bev[i].point[j].x;
         }
         else
         {
            // get ymax and ymin values
            if (allDetResult.bev.bev[i].point[j].x < ymin)
            {
               ymin = allDetResult.bev.bev[i].point[j].x;
            }
            else if (allDetResult.bev.bev[i].point[j].x > ymax)
            {
               ymax = allDetResult.bev.bev[i].point[j].x;
            }

            // get xmin and xmax values
            if (-allDetResult.bev.bev[i].point[j].y < xmin)
            {
               xmin = -allDetResult.bev.bev[i].point[j].y;
            }
            else if (-allDetResult.bev.bev[i].point[j].y > xmax)
            {
               xmax = -allDetResult.bev.bev[i].point[j].y;
            }
         }
      }

      float cx = (xmax + xmin) / 2;
      float cy = (ymax + ymin) / 2;

      int forward = 0;
      if (x0 < cx && y0 > cy)
      {
         forward = 0; // up
      }
      else if (x0 > cx && y0 < cy)
      {
         forward = 1; // buttom
      }
      else if (x0 < cx && y0 < cy)
      {
         forward = 2; // left
      }
      else if (x0 > cx && y0 > cy)
      {
         forward = 3; // right
      }

      initDetectorBoxs(xmin, ymin, xmax, ymax, forward, class_id, detectBoxes);
   }
}

/*
classID 从0开始， 最大类别ID为 5, 0: car, 1: person, 2：rider
*/
void ObjTrack::initDetectorBoxs(float xmin, float ymin, float xmax, float ymax,
                                int fwd, int classid, DetectBoxes &detectBoxes)
{
   if (classid >= 0 && classid <= 5)
   {
      detectBoxes.boxRects[detectBoxes.numRect].label = classid;
      detectBoxes.boxRects[detectBoxes.numRect].prob = 0;
      detectBoxes.boxRects[detectBoxes.numRect].fused = 0;
      detectBoxes.boxRects[detectBoxes.numRect].rect.x = xmin; // dets[i].xmin;
      detectBoxes.boxRects[detectBoxes.numRect].rect.y = ymin; // dets[i].ymin;
      if (classid == 1 || classid == 2)
      {
         detectBoxes.boxRects[detectBoxes.numRect].rect.width = (xmax - xmin) + addWidth;
         detectBoxes.boxRects[detectBoxes.numRect].rect.height = (ymax - ymin) + addHeight;
      }
      else
      {
         detectBoxes.boxRects[detectBoxes.numRect].rect.width = (xmax - xmin);
         detectBoxes.boxRects[detectBoxes.numRect].rect.height = (ymax - ymin);
      }

      detectBoxes.boxRects[detectBoxes.numRect].forward = fwd;
      detectBoxes.numRect++;
      if (detectBoxes.numRect >= MAX_OBJECT_NUM)
      {
         return;
      }
   }
}

void ObjTrack::fowardUp(int i, TShowPoint &tShowPoint, TrackingBox *trackingBoxes)
{
   for (int j = 0; j < 4; j++)
   {
      switch (j)
      {
      case 0:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y + trackingBoxes[i].box.height;
         break;
      }
      case 1:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x + trackingBoxes[i].box.width);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y + trackingBoxes[i].box.height;
         break;
      }
      case 2:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x + trackingBoxes[i].box.width);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y;
         break;
      }
      case 3:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y;
         break;
      }
      }

      tShowPoint.tBevPoint[i][j].z = 1.5; // none
      int class_id = trackingBoxes[i].clsId;
      if (class_id >= 3 && class_id <= 4)
         tShowPoint.tBevPoint[i][j].classId = 3; // classID
      else
      {
         tShowPoint.tBevPoint[i][j].classId = class_id; // data[ptsStep * (i + 1)]; // classID
      }
      tShowPoint.tBevPoint[i][j].trackId = trackingBoxes[i].objId;
   }
}

void ObjTrack::fowardButtom(int i, TShowPoint &tShowPoint, TrackingBox *trackingBoxes)
{
   for (int j = 0; j < 4; j++)
   {
      switch (j)
      {
      case 0:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x + trackingBoxes[i].box.width);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y;
         break;
      }
      case 1:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y;
         break;
      }
      case 2:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y + trackingBoxes[i].box.height;
         break;
      }
      case 3:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x + trackingBoxes[i].box.width);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y + trackingBoxes[i].box.height;
         break;
      }
      }

      tShowPoint.tBevPoint[i][j].z = 1.5; // none
      int class_id = trackingBoxes[i].clsId;
      if (class_id >= 2 && class_id <= 4)
         tShowPoint.tBevPoint[i][j].classId = 2; // classID
      else
      {
         tShowPoint.tBevPoint[i][j].classId = class_id; // data[ptsStep * (i + 1)]; // classID
      }
      tShowPoint.tBevPoint[i][j].trackId = trackingBoxes[i].objId;
   }
}

void ObjTrack::fowardLeft(int i, TShowPoint &tShowPoint, TrackingBox *trackingBoxes)
{
   for (int j = 0; j < 4; j++)
   {
      switch (j)
      {
      case 0:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y;
         break;
      }
      case 1:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y + trackingBoxes[i].box.height;
         break;
      }
      case 2:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x + trackingBoxes[i].box.width);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y + trackingBoxes[i].box.height;
         break;
      }
      case 3:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x + trackingBoxes[i].box.width);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y;
         break;
      }
      }

      tShowPoint.tBevPoint[i][j].z = 1.5; // none
      int class_id = trackingBoxes[i].clsId;
      if (class_id >= 2 && class_id <= 4)
         tShowPoint.tBevPoint[i][j].classId = 2; // classID
      else
      {
         tShowPoint.tBevPoint[i][j].classId = class_id; // data[ptsStep * (i + 1)]; // classID
      }
      tShowPoint.tBevPoint[i][j].trackId = trackingBoxes[i].objId;
   }
}

void ObjTrack::fowardRight(int i, TShowPoint &tShowPoint, TrackingBox *trackingBoxes)
{
   for (int j = 0; j < 4; j++)
   {
      switch (j)
      {
      case 0:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x + trackingBoxes[i].box.width);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y + trackingBoxes[i].box.height;
         break;
      }
      case 1:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x + trackingBoxes[i].box.width);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y;
         break;
      }
      case 2:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y;
         break;
      }
      case 3:
      {
         tShowPoint.tBevPoint[i][j].x = (trackingBoxes[i].box.x);
         tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y + trackingBoxes[i].box.height;
         break;
      }
      }

      tShowPoint.tBevPoint[i][j].z = 1.5; // none
      int class_id = trackingBoxes[i].clsId;
      if (class_id >= 2 && class_id <= 4)
         tShowPoint.tBevPoint[i][j].classId = 2; // classID
      else
      {
         tShowPoint.tBevPoint[i][j].classId = class_id; // data[ptsStep * (i + 1)]; // classID
      }
      tShowPoint.tBevPoint[i][j].trackId = trackingBoxes[i].objId;
   }
}

void ObjTrack::sortPoints(int kalman_obj_num, TShowPoint &tShowPoint, TrackingBox *trackingBoxes)
{
   for (int i = 0; i < kalman_obj_num; i++)
   {
      int forward = trackingBoxes[i].fwd;
      int class_id = trackingBoxes[i].clsId;
      if (class_id == 1 || class_id == 2)
      {
         trackingBoxes[i].box.width = trackingBoxes[i].box.width - addWidth;
         trackingBoxes[i].box.height = trackingBoxes[i].box.height - addHeight;
      }

      switch (forward)
      {
      case 0:
      {
         fowardUp(i, tShowPoint, trackingBoxes);
         break;
      }
      case 1:
      {
         fowardButtom(i, tShowPoint, trackingBoxes);
         break;
      }
      case 2:
      {
         fowardLeft(i, tShowPoint, trackingBoxes);
         break;
      }
      case 3:
      {
         fowardRight(i, tShowPoint, trackingBoxes);
         break;
      }
      }
   }
}

/*
void ObjTrack::sortPoints(int kalman_obj_num, TShowPoint &tShowPoint, TrackingBox *trackingBoxes)
{
   for (int i = 0; i < kalman_obj_num; i++)
   {
      int forward = trackingBoxes[i].fwd;
      int class_id = trackingBoxes[i].clsId;
      if (class_id == 0 || class_id == 1)
      {
         trackingBoxes[i].box.width = trackingBoxes[i].box.width - addWidth;
         trackingBoxes[i].box.height = trackingBoxes[i].box.height - addHeight;
      }

      for (int j = 0; j < 4; j++)
      {
         if (forward == -1)
         {
            switch (j)
            {
            case 0:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes[i].box.x);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y + trackingBoxes[i].box.height;
               break;
            }
            case 1:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes[i].box.x + trackingBoxes[i].box.width);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y + trackingBoxes[i].box.height;
               break;
            }
            case 2:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes[i].box.x + trackingBoxes[i].box.width);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y;
               break;
            }
            case 3:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes[i].box.x);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y;
               break;
            }
            }
         }

         if (forward == 1)
         {
            switch (j)
            {
            case 0:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes[i].box.x + trackingBoxes[i].box.width);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y;
               break;
            }
            case 1:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes[i].box.x);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y;
               break;
            }
            case 2:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes[i].box.x);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y + trackingBoxes[i].box.height;
               break;
            }
            case 3:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes[i].box.x + trackingBoxes[i].box.width);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes[i].box.y + trackingBoxes[i].box.height;
               break;
            }
            }
         }

         tShowPoint.tBevPoint[i][j].z = 1.5; // none

         if (class_id >= 2 && class_id <= 4)
            tShowPoint.tBevPoint[i][j].classId = 2; // classID
         else
         {
            tShowPoint.tBevPoint[i][j].classId = class_id; // data[ptsStep * (i + 1)]; // classID
         }
         tShowPoint.tBevPoint[i][j].trackId = trackingBoxes[i].objId;
      }
   }
}
*/

void ObjTrack::sortPoints(TShowPoint &tShowPoint, DetectBoxes &trackingBoxes)
{
   int bboxNums = trackingBoxes.numRect;
   for (int i = 0; i < bboxNums; i++)
   {
      int forward = 1;
      for (int j = 0; j < 4; j++)
      {
         if (forward == -1)
         {
            switch (j)
            {
            case 0:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes.boxRects[i].rect.x);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes.boxRects[i].rect.y + trackingBoxes.boxRects[i].rect.height;
               break;
            }
            case 1:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes.boxRects[i].rect.x + trackingBoxes.boxRects[i].rect.width);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes.boxRects[i].rect.y + trackingBoxes.boxRects[i].rect.height;
               break;
            }
            case 2:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes.boxRects[i].rect.x + trackingBoxes.boxRects[i].rect.width);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes.boxRects[i].rect.y;
               break;
            }
            case 3:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes.boxRects[i].rect.x);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes.boxRects[i].rect.y;
               break;
            }
            }
         }

         if (forward == 1)
         {
            switch (j)
            {
            case 0:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes.boxRects[i].rect.x + trackingBoxes.boxRects[i].rect.width);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes.boxRects[i].rect.y;
               break;
            }
            case 1:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes.boxRects[i].rect.x);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes.boxRects[i].rect.y;
               break;
            }
            case 2:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes.boxRects[i].rect.x);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes.boxRects[i].rect.y + trackingBoxes.boxRects[i].rect.height;
               break;
            }
            case 3:
            {
               tShowPoint.tBevPoint[i][j].x = -(trackingBoxes.boxRects[i].rect.x + trackingBoxes.boxRects[i].rect.width);
               tShowPoint.tBevPoint[i][j].y = trackingBoxes.boxRects[i].rect.y + trackingBoxes.boxRects[i].rect.height;
               break;
            }
            }
         }

         // tShowPoint.tBevPoint[i][j].z = 1.5; // none
         int classID = trackingBoxes.boxRects[i].label;

         /*
         if (classID >= 2 && classID <= 4)
            tShowPoint.tBevPoint[i][j].classId = 2;
         else
         {
            tShowPoint.tBevPoint[i][j].classId = classID; // data[ptsStep * (i + 1)]; // classID
         }
         */

         tShowPoint.tBevPoint[i][j].classId = classID;
         tShowPoint.tBevPoint[i][j].trackId = 1; // trackingBoxes[i].objId;
      }
   }
}

ObjTrack::ObjTrack()
{
   kalman_tracker = ObjectTracker_create();
}

ObjTrack::~ObjTrack()
{
   kalman_tracker = nullptr;
}

void ObjTrack::runTrack(s32 objNum, TDetResult &allDetResult, TShowPoint &tShowPoint)
{
   DetectBoxes detectBoxes;
   detectBoxes.numRect = 0;
   TrackingBox trackingBoxes[MAX_OBJECT_NUM];
   get_MaxMin(objNum, allDetResult, detectBoxes);
   if (kalman_tracker != nullptr)
   {
      int kalman_obj_num = ObjectTracker_updateTrackingBoxes(kalman_tracker, detectBoxes.boxRects,
                                                             detectBoxes.numRect, trackingBoxes,
                                                             MAX_OBJECT_NUM);
      sortPoints(kalman_obj_num, tShowPoint, trackingBoxes);
   }
}

void ObjTrack::runTrack(TShowPoint &tShowPoint, DetectBoxes detectBoxes)
{
   TrackingBox trackingBoxes[MAX_OBJECT_NUM];
   // sortPoints(tShowPoint, detectBoxes);
   // /*
   if (kalman_tracker != nullptr)
   {
      int kalman_obj_num = ObjectTracker_updateTrackingBoxes(kalman_tracker, detectBoxes.boxRects,
                                                             detectBoxes.numRect, trackingBoxes,
                                                             MAX_OBJECT_NUM);
      sortPoints(kalman_obj_num, tShowPoint, trackingBoxes);
   }
   // */
}