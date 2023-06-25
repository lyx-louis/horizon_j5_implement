#include "object_tracking_3d.h"
#include "object_tracker.h"

#include <memory.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#define Math_sqrt sqrtf
#define Math_asin asinf
#define Math_pow powf

#define MAX_PART_NUM (8+1)
#define LEFT 1
#define RIGHT 2

#define OVERLAPE_THRESHOLD  0.75f

typedef struct ObjectTracking3D
{
	void* objectTracker;
	DetectBoxes detect_objects;
	TrackingBoxes trackingBoxes;
	int wheelPart[TRACKING_OBJ_NUM_MAX][MAX_PART_NUM];
	int frontRearPart[TRACKING_OBJ_NUM_MAX][MAX_PART_NUM];
	DownDirection downDirection;
}ObjectTracking3D;

KMTrackClass kmTrackIdxTab[MAX_OBJECT_CLASS_NUM];

static void _initTrackIdxTab()
{
	kmTrackIdxTab[car] = KMTRACK_vehicle;
	kmTrackIdxTab[bus] = KMTRACK_vehicle;
	kmTrackIdxTab[truck] = KMTRACK_vehicle;
	kmTrackIdxTab[person] = KMTRACK_person;
	kmTrackIdxTab[bicycle] = KMTRACK_bike;
	kmTrackIdxTab[pure_humanlike] = KMTRACK_person;
	kmTrackIdxTab[vehicle_like] = KMTRACK_vehicle;
	kmTrackIdxTab[wheel] = KMTRACK_wheel;
}

void* ObjectTracking3D_create()
{
	ObjectTracking3D* o = (ObjectTracking3D*)malloc(sizeof(ObjectTracking3D));
	if (o == NULL)
		return NULL;
	o->objectTracker = ObjectTracker_create();
	if ( o->objectTracker == NULL )
	{
		free(o);
		return NULL;
	}
	o->downDirection = BottomIsDown;
	_initTrackIdxTab();
	return o;
}

void ObjectTracking_setDownDirection(void* handle, DownDirection downDirection)
{
	ObjectTracking3D* o = (ObjectTracking3D*)handle;
	if (o != NULL)
	{
		o->downDirection = downDirection;
	}
}

static CV_RECT _rectIntersection(CV_RECT a, CV_RECT b)
{
	CV_RECT c;
	c.x = MAX(a.x, b.x);
	c.y = MAX(a.y, b.y);
	c.width = MIN(a.x + a.width,
		b.x + b.width) - c.x;
	c.height = MIN(a.y + a.height,
		b.y + b.height) - c.y;
	return c;
}

static RECT_F _rectUnion(RECT_F a, RECT_F b)
{
	RECT_F c;
	c.x = MIN(a.x, b.x);
	c.y = MIN(a.y, b.y);
	c.width = MAX(a.x + a.width, b.x + b.width) - c.x;
	c.height = MAX(a.y + a.height, b.y + b.height) - c.y;
	return c;
}

static CV_RECT rectF2CvRect(RECT_F rectf) {
	CV_RECT rect;
	rect.x = (int)rectf.x;
	rect.y = (int)rectf.y;
	rect.width = (int)rectf.width;
	rect.height = (int)rectf.height;
	return rect;
}

static float overlap(int x1, int w1, int x2, int w2)
{
	int left = x1 > x2 ? x1 : x2; 
	int r1 = x1 + w1; 
	int r2 = x2 + w2; 
	int right = r1 < r2 ? r1 : r2; 
	return (float)(right - left);
}

static float box_intersection(CV_RECT a, CV_RECT b)
{
	float w = overlap(a.x, a.width, b.x, b.width); 
	float h = overlap(a.y, a.height, b.y, b.height); 
	if (w < 0 || h < 0) 
		return 0; 
	return w*h;
}

static float box_union(CV_RECT a, CV_RECT b)
{
	float i = box_intersection(a, b);
	float u = a.width*a.height + b.width*b.height - i;
	return u;
}

static float box_iou(CV_RECT a, CV_RECT b)
{
	return box_intersection(a, b) / box_union(a, b);
}

static void part_in_which_car(TrackingBoxes* bbs, int obj_id, int part[][MAX_PART_NUM])
{
	int i;
	for (i = 0; i < bbs->numBox; i++) {
		part[i][0] = 0;
	}
	for (i = 0; i < bbs->numBox; i++) {
		TrackingBox* bbox_0 = &bbs->trkBoxes[i];
		int max_idx = -1, j;
		float max_value = 0, iou, iow;
		if (bbox_0->clsId != obj_id)
			continue;
		for (j = 0; j < bbs->numBox; j++) {
			TrackingBox* bbox_1 = &bbs->trkBoxes[j];
			if (kmTrackIdxTab[bbox_1->clsId] != KMTRACK_vehicle)
				continue;
			iou = box_iou(bbox_0->box, bbox_1->box);
			iow = box_intersection(bbox_0->box, bbox_1->box) / box_union(bbox_0->box, bbox_0->box);
			if (iou > max_value && iow > 0.4f) {
				max_value = iou;
				max_idx = j;
			}
		}

		if (max_idx >= 0) {
			if (part[max_idx][0] + 1 < MAX_PART_NUM) {
				int num = (++part[max_idx][0]);
				part[max_idx][num] = i;
				bbox_0->belongIdx = max_idx;
			}
		}
	}
}

static BOOL line_intersection(int Ax, int Ay, int Bx, int By, 
	int Cx, int Cy, int Dx, int Dy, int *X, int *Y)
{
	float distAB, theCos, theSin, newCx, newCy, newDx, newDy, ABpos;

	//  Fail if either line is undefined.
	if ((Ax == Bx && Ay == By) || (Cx == Dx && Cy == Dy))
		return FALSE;

	//  (1) Translate the system so that point A is on the origin.
	Bx -= Ax; By -= Ay;
	Cx -= Ax; Cy -= Ay;
	Dx -= Ax; Dy -= Ay;

	//  Discover the length of segment A-B.
	distAB = Math_sqrt((float)(Bx*Bx + By*By));

	//  (2) Rotate the system so that point B is on the positive X axis.
	theCos = Bx / distAB;
	theSin = By / distAB;
	newCx = Cx*theCos + Cy*theSin;
	newCy = Cy*theCos - Cx*theSin;
	newDx = Dx*theCos + Dy*theSin;
	newDy = Dy*theCos - Dx*theSin;

	//  Fail if the lines are parallel.
	if (newCy == newDy)
		return FALSE;

	//  (3) Discover the position of the intersection point along line A-B.
	ABpos = newDx + (newCx - newDx) * newDy / (newDy - newCy);

	//  (4) Apply the discovered position to line A-B in the original coordinate system.
	*X = (int)(Ax + ABpos*theCos);
	*Y = (int)(Ay + ABpos*theSin);

	//  Success.
	return TRUE;
}

static int calc_side_by_wheels(TrackingBoxes* trkBoxes, int idx, int w_part[][MAX_PART_NUM], int *sx, int *y)
{
	//center of part's bottom side
	CV_POINT pt_max_y, pt_min_y;
	CV_POINT xy[MAX_PART_NUM];
	int n = w_part[idx][0], i;

	for (i = 1; i <= n; i++) {
		CV_RECT* bb = &trkBoxes->trkBoxes[w_part[idx][i]].box;
		xy[i].x = bb->x + bb->width / 2;
		xy[i].y = bb->y + bb->height;
	}

	//max and min y of center
	pt_max_y.x = pt_max_y.y = 0;
	pt_min_y.x = pt_min_y.y = 100000;
	for (i = 1; i <= n; i++) {
		pt_min_y = (xy[i].y < pt_min_y.y ? xy[i] : pt_min_y);
		pt_max_y = (xy[i].y > pt_max_y.y ? xy[i] : pt_max_y);
	}

	{
		CV_RECT* bb = &trkBoxes->trkBoxes[idx].box;
		int left = bb->x, right = bb->x + bb->width, top = bb->y, bottom = bb->y + bb->height;
		int X0, Y0, X1, Y1;
		BOOL li0 = line_intersection(pt_min_y.x, pt_min_y.y, pt_max_y.x, pt_max_y.y,
			left, bottom, right, bottom, &X0, &Y0);

		float dist = Math_sqrt((float)((pt_min_y.x - pt_max_y.x) * (pt_min_y.x - pt_max_y.x) + (pt_min_y.y - pt_max_y.y)*(pt_min_y.y - pt_max_y.y)));
		float angle = Math_asin(ABS(pt_min_y.y - pt_max_y.y) / dist);
		BOOL ignore = FALSE;

		if (angle<3.14159 / 24)
			ignore = TRUE;

		if (pt_min_y.x > pt_max_y.x) {
			BOOL li1 = line_intersection(pt_min_y.x, pt_min_y.y, pt_max_y.x, pt_max_y.y,
				right, top, right, bottom, &X1, &Y1);
			if (li0  && li1) {
				*sx = MAX(left, MIN(right, X0));
				*y = MAX(top, MIN(bottom, Y1));
				if (ignore)
					*sx = left;
				return RIGHT;
			}
		}
		else {
			BOOL li1 = line_intersection(pt_min_y.x, pt_min_y.y, pt_max_y.x, pt_max_y.y,
				left, top, left, bottom, &X1, &Y1);
			if (li0 && li1) {
				*sx = MAX(left, MIN(right, X0));
				*y = MAX(top, MIN(bottom, Y1));
				if (ignore)
					*sx = right;
				return LEFT;
			}
		}
	}
	return 0;
}

static int calc_side_by_front(TrackingBoxes* trkBoxes, int idx, int f_part[][MAX_PART_NUM], int *sx, int *y)
{	
	CV_RECT rt_whole = trkBoxes->trkBoxes[idx].box, rt_part = { 0 };
	TrackingBox *bb = NULL;
	int n = f_part[idx][0], i;
	float max_prob = 0;
		
	for (i = 1; i <= n; i++) {
		bb = &trkBoxes->trkBoxes[f_part[idx][i]];
		if (1 == i) {
			rt_part = bb->box;
			max_prob = bb->prob;
		}
		else {
			if (bb->prob > max_prob) {
				rt_part = bb->box;
				max_prob = bb->prob;
			}
		}
	}

	rt_part = _rectIntersection(rt_part, rt_whole);
	if (rt_part.width > 0 && rt_part.height > 0)
	{
		int xw = rt_whole.x + rt_whole.width / 2;
		int xp = rt_part.x + rt_part.width / 2;
		float alpha;

		if ((rt_whole.width - rt_part.width) < 10 || rt_part.width*1.0 / rt_whole.width > 0.8)
			return -1;

		//experience
		alpha = (float)rt_part.width / rt_whole.width;
		*y = (int)(rt_part.y + (1 - Math_pow(alpha, 3))*rt_part.height);
		if (xw >= xp) {
			*sx = rt_part.x + rt_part.width;
			return RIGHT;
		}
		else {
			*sx = rt_part.x;
			return LEFT;
		}
	}
	return 0;
}

static void set_3d_box(TrackingBox* trkBox, int side, int sx, int y)
{
	int left = trkBox->box.x, right = trkBox->box.x + trkBox->box.width;
	int top = trkBox->box.y, bot = trkBox->box.y + trkBox->box.height;
	trkBox->used3d = 1;
	if (RIGHT == side) {
		trkBox->fb.x = left;
		trkBox->fb.y = trkBox->box.y;
		trkBox->fb.width = sx - left;
		trkBox->fb.height = trkBox->box.height;
		  
		trkBox->side[0].x = sx;
		trkBox->side[0].y = top;
		trkBox->side[1].x = right;
		trkBox->side[1].y = MIN(top + (bot - y) / 3, (y + top) / 2);
		trkBox->side[2].x = right;
		trkBox->side[2].y = y;
		trkBox->side[3].x = sx;
		trkBox->side[3].y = bot;
	}
	else if (LEFT == side) {
		trkBox->fb.x = sx;
		trkBox->fb.y = trkBox->box.y;
		trkBox->fb.width = right - sx;
		trkBox->fb.height = trkBox->box.height;

		trkBox->side[0].x = sx;
		trkBox->side[0].y = top;
		trkBox->side[1].x = left;
		trkBox->side[1].y = MIN(top + (bot - y) / 3, (y + top) / 2);
		trkBox->side[2].x = left;
		trkBox->side[2].y = y;
		trkBox->side[3].x = sx;
		trkBox->side[3].y = bot;

	}
	else {
		// undefined.
	}
}

static void get_3d_boxes(TrackingBoxes* trkBoxes, int w_part[][MAX_PART_NUM], int f_part[][MAX_PART_NUM])
{
	//init
	int i;
	for (i = 0; i < trkBoxes->numBox; i++) {
		TrackingBox* bbox = &trkBoxes->trkBoxes[i];
		bbox->fb.x = bbox->box.x;
		bbox->fb.y = bbox->box.y;
		bbox->fb.width = bbox->box.width;
		bbox->fb.height = bbox->box.height;
		bbox->side[0].x = -100;
		bbox->side[0].y = -100;
	}

	for (i = 0; i < trkBoxes->numBox; i++) {
		TrackingBox* bbox = &trkBoxes->trkBoxes[i];
		int n_w_part = w_part[i][0];
		int n_f_part = f_part[i][0];
		int w_side = 0, f_side = 0, sx_w = 0, y_w = 0, sx_f = 0, y_f = 0;
		if (n_w_part > 1)
			w_side = calc_side_by_wheels(trkBoxes, i, w_part, &sx_w, &y_w);
		if (n_f_part > 0)
			f_side = calc_side_by_front(trkBoxes, i, f_part, &sx_f, &y_f);

		//Multiple wheels and front rear
		if (f_side > 0)
			set_3d_box(bbox, f_side, sx_f, y_f);
		else if (w_side > 0)
			set_3d_box(bbox, w_side, sx_w, y_w);
		else if (1 == n_w_part)
			set_3d_box(bbox, RIGHT, bbox->box.x, bbox->box.y + bbox->box.height);

		//smooth
	}
}

//void non_maximal_suppression(DetectBoxes* bbox, float nms_T/* = 0.4*/)
//{
//	for (int i = 0; i < bbox.size(); i++)
//	{
//		for (int j = i + 1; j < bbox.size(); j++)
//		{
//			if (is_same_type(bbox[i].obj_id, bbox[j].obj_id) && (rect_overlap_ratio(&bbox[i], &bbox[j]) >= 700
//				|| rect_position_check(&bbox[i], &bbox[j]) == 1))
//			{
//
//				if (bbox[i].prob > bbox[j].prob)
//				{
//					bbox[i].prob += bbox[j].prob;
//					bbox[j].prob = -1;
//				}
//				else
//				{
//					bbox[i].prob = -1;
//					bbox[j].prob += bbox[i].prob;
//				}
//			}
//		}
//	}
//
//	for (std::vector<bbox_dl>::iterator it = bbox.begin(); it != bbox.end();)
//	{
//		if (it->prob < 0) it = bbox.erase(it);	else it++;
//	}
//
//	return;
//}
static BOOL rect_position_check(CV_RECT r1, CV_RECT r2)
{
	if (r1.x <= r2.x && r1.x + r1.width >= r2.x + r2.width
		&& r1.y <= r2.y && r1.y + r1.height >= r2.y + r2.height)
		return TRUE;
	else if (r1.x >= r2.x && r1.x + r1.width <= r2.x + r2.width
		&& r1.y >= r2.y && r1.y + r1.height <= r2.y + r2.height)
		return TRUE;
	else
		return FALSE;
}

static int _getDown(DownDirection downDirection, TrackingBox* trkBox)
{
	int down = trkBox->box.y + trkBox->box.height - 1;
	if (downDirection == LeftIsDown) {
		down = trkBox->box.x;
	}
	else if (downDirection == RightIsDown) {
		down = trkBox->box.x + trkBox->box.width - 1;
	}
	else if (downDirection == TopIsDown) {
		down = trkBox->box.y;
	}
	return down;
}
static void ObjectTracking3D_findRiderAndMark(ObjectTracking3D* o, TrackingBoxes* trackingBoxes)
{
	int i, j;
	for (i = 0; i < trackingBoxes->numBox; i++) {
		TrackingBox* bikeBox = &trackingBoxes->trkBoxes[i];
		if (bikeBox->clsId == bicycle) {
			float max_iou = 0;
			int index = -1;
			int down = _getDown(o->downDirection, bikeBox);
			for (j = 0; j < trackingBoxes->numBox; j++) {
				TrackingBox* personBox = &trackingBoxes->trkBoxes[j];
				if (personBox->clsId == person) {
					float iou = box_iou(bikeBox->box, personBox->box);
					int downInner = _getDown(o->downDirection, personBox);
					int personHeight = personBox->box.height;
					if (o->downDirection == LeftIsDown || o->downDirection == RightIsDown)
						personHeight = personBox->box.width;
					if (iou > max_iou && ABS(down - downInner) <= personHeight / 8) {
						index = j;
						max_iou = iou;
					}
				}
			}

			if (max_iou > 0.01 && index >= 0)
			{
				TrackingBox* matchedPersonBox = &trackingBoxes->trkBoxes[index];
				if (matchedPersonBox->childIdx >= 0 && matchedPersonBox->childIdx != i) 
				{// the matched person has matched the other bike
					TrackingBox* lastMatchedBikeBox = &trackingBoxes->trkBoxes[matchedPersonBox->childIdx];
					float iou = box_iou(lastMatchedBikeBox->box, matchedPersonBox->box);
					if (iou < max_iou) {
						// match person with this bike, and remove last matched bike
						lastMatchedBikeBox->rider = 0;
						lastMatchedBikeBox->childIdx = -1;
						bikeBox->rider = 1;
						bikeBox->childIdx = index;
						matchedPersonBox->rider = 1;
						matchedPersonBox->childIdx = i;
					}
					else {
						// skip match person with this bike.
					}
				}
				else {
					bikeBox->rider = 1;
					bikeBox->childIdx = index;
					matchedPersonBox->rider = 1;
					matchedPersonBox->childIdx = i;
				}
			}
		}
	}
}

void ObjectTracking3D_tracking(void* handle, DetectBoxes* detectBoxes, TrackingBoxes* trackingBoxes, int isBox3d)
{
	int  i;
	ObjectTracking3D* o = (ObjectTracking3D*)handle;
	TrackingBox* trkBox = NULL;

	if ( handle == NULL )
	{
		return ;
	}
	memcpy(&o->detect_objects, detectBoxes, sizeof(DetectBoxes));

	trkBox = o->trackingBoxes.trkBoxes;
	o->trackingBoxes.numBox = 0;

	o->trackingBoxes.numBox = ObjectTracker_updateTrackingBoxes(o->objectTracker,
		o->detect_objects.boxRects, o->detect_objects.numRect, trkBox, TRACKING_OBJ_NUM_MAX);

	ObjectTracking3D_findRiderAndMark(o, &o->trackingBoxes);

	trackingBoxes->numBox = 0;
	for (i = 0; i < o->trackingBoxes.numBox; i++) {
		trkBox = &o->trackingBoxes.trkBoxes[i];
		if (trkBox->clsId == bicycle && trkBox->rider && trkBox->childIdx >= 0) {
			continue;
		}
		if (trkBox->clsId == person && trkBox->rider && trkBox->childIdx >= 0) {
			trkBox->accurateBox = _rectUnion(trkBox->accurateBox,
				o->trackingBoxes.trkBoxes[trkBox->childIdx].accurateBox);
			trkBox->box = rectF2CvRect(trkBox->accurateBox);
		}
		trackingBoxes->trkBoxes[trackingBoxes->numBox] = *trkBox;
		trackingBoxes->numBox++;
	}

	part_in_which_car(trackingBoxes, wheel, o->wheelPart);
	//part_in_which_car(trackingBoxes, km_frontRear, o->frontRearPart);
	part_in_which_car(trackingBoxes, front_rear, o->frontRearPart);
	if (isBox3d)
		get_3d_boxes(trackingBoxes, o->wheelPart, o->frontRearPart);
}

void ObjectTracking3D_release(void* handle)
{
	ObjectTracking3D* o = (ObjectTracking3D*)handle;
	if (o != NULL) 
	{
		ObjectTracker_release(o->objectTracker);
		free(o);
	}
}

static KM_TRACK_VERSION g_km_track_version =
{
	1, 2, 0, "2021/10/26", "alpha"
};

const KM_TRACK_VERSION* ObjectTracking3D_getVersion()
{
	return &g_km_track_version;
}

