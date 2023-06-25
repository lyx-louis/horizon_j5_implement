

#ifndef _BASE_TYPE_INCLUDE_H_
#define _BASE_TYPE_INCLUDE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef DLL_API_DEFAULT
#define DLL_API_DEFAULT __attribute__((visibility("default")))
#endif
#ifndef DLL_API_HIDDEN
#define DLL_API_HIDDEN /*__attribute__((visibility("hidden")))*/
#endif

#ifndef DLL_LOCAL
#if (defined WIN32 || defined _WINDLL)
#define DLL_LOCAL _declspec(dllimport)
#else
#define DLL_LOCAL __attribute__((visibility("hidden")))
#endif
#else
// #pragma message("--------base_type.h already defined DLL_LOCAL" )
#endif

#undef DLL_API
#ifndef DLL_API
#if (defined WIN32 || defined _WINDLL)
#define DLL_API _declspec(dllexport)
#else
#define DLL_API __attribute__((visibility("default")))
#endif
#else
// #pragma message("--------base_type.h already defined DLL_API")
#endif

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifndef RECT_EPS
#define RECT_EPS 308
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef ABS
#define ABS(a) (((a) > 0) ? (a) : (-(a)))
#endif

#ifndef SWAP
#define SWAP(a, b, t) ((t) = (a), (a) = (b), (b) = (t))
#endif

#ifndef CLIP
#define CLIP(x, minval, maxval) (x) < (minval) ? (minval) : ((x) > (maxval) ? (maxval) : (x))
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef NULL
#define NULL 0
#endif

#if !defined(DWORD) && !defined(_DWORD_)
#ifdef _WIN32
	typedef unsigned long DWORD;
#else
	typedef unsigned int DWORD;
#endif
#define _DWORD_ DWORD
#endif

#ifndef BOOL
	typedef int BOOL;
#endif

#ifndef BYTE
	typedef unsigned char BYTE;
#endif

#ifndef PBYTE
	typedef unsigned char *PBYTE;
#endif

#ifndef WORD
	typedef unsigned short WORD;
#endif

#if defined(_WIN32) || defined(_WINDOWS)
	typedef __int64 s64;
	typedef unsigned __int64 u64;
#else
typedef long long s64;
typedef unsigned long long u64;
#endif

#define CHECK_ERROR(state, error_code) \
	if (state)                          \
	{                                   \
		return error_code;               \
	}

	typedef struct _POINT_
	{
		int x;
		int y;
	} CV_POINT;

	typedef struct _POINT_F_
	{
		float x;
		float y;
	} POINT_F;

	typedef struct _BLOB_
	{
		int left;
		int right;
		int top;
		int bottom;
	} CV_BLOB;

	typedef struct _BLOB_F_
	{
		float left;
		float right;
		float top;
		float bottom;
	} BLOB_F;

	// LDW
	typedef struct _LANE_INFO_
	{
		CV_POINT left_lane[2];
		CV_POINT right_lane[2];
		int left_valid;
		int right_valid;
		int left_lane_type;
		int right_lane_type;
	} LANE_INFO;

#define BLOB_WIDTH(_blob) ((_blob)->right - (_blob)->left + 1)
#define BLOB_HEIGHT(_blob) ((_blob)->bottom - (_blob)->top + 1)

	typedef struct _RECT_
	{
		float x;
		float y;
		float width;
		float height;
	} CV_RECT;

	typedef struct _RECT_F_
	{
		float x;
		float y;
		float width;
		float height;
	} RECT_F;

	typedef struct _SIZE_
	{
		int width;
		int height;
	} CV_SIZE;

#define MAX_VERTEX 20

	typedef struct _POLYGON_
	{
		int vertex_num;
		CV_POINT point[MAX_VERTEX + 1];
	} POLYGON;

	typedef struct _POLYGON_F_
	{
		int vertex_num;
		POINT_F point[MAX_VERTEX + 1];
	} POLYGON_F;

	typedef struct _LINE_
	{
		int rho;
		int angle;
		CV_POINT line_start;
		CV_POINT line_end;
		int confidence;
	} LINE;

	typedef struct LIST
	{
		void *ptData;
		struct LIST *ptNext;
	} LIST;

	typedef struct SEQ_LIST
	{
		LIST *ptHead;
		LIST *ptTail;
		int total;
	} SEQ_LIST;

	typedef struct _OBJ_RECT_
	{
		union
		{
			CV_BLOB r;
			BLOB_F rf;
		};
		int neighbors;
		unsigned int light_on;
		float scaleshift;
		POINT_F displacements;
	} OBJ_RECT;

#ifndef MAX_OBJECT_RECT_NUM
#define MAX_OBJECT_RECT_NUM 200
#endif

	typedef struct _OBJ_RECT_LIST_
	{
		int rect_num;
		OBJ_RECT rect[MAX_OBJECT_RECT_NUM];
	} OBJ_RECT_LIST;

#ifndef MAX_RECT_NUM
#define MAX_RECT_NUM 200
#endif

	typedef struct _RECT_LIST_
	{
		int rect_num;
		OBJ_RECT rect[MAX_RECT_NUM];
	} RECT_LIST;

#define TREE_NODE_FIELDS(node_type)                           \
	int flags;					  /**< Miscellaneous flags.     */ \
	int header_size;			  /**< Size of sequence header. */ \
	struct node_type *h_prev; /**< Previous sequence.       */ \
	struct node_type *h_next; /**< Next sequence.           */ \
	struct node_type *v_prev; /**< 2nd previous sequence.   */ \
	struct node_type *v_next  /**< 2nd next sequence.       */

	/**
	Read/Write sequence.
	Elements can be dynamically inserted to or deleted from the sequence.
	*/
	typedef struct CV_SEQ
	{
		TREE_NODE_FIELDS(CV_SEQ);
		SEQ_LIST list;
		CV_RECT rect;
	} CV_SEQ;

#define PIXEL_FORMAT_UNKNOWN (0)			// reserved
#define PIXEL_FORMAT_BGR (1)				// B0G0R0B1G0R0
#define PIXEL_FORMAT_GRAY (2)				// G0G1G2G3
#define PIXEL_FORMAT_BLOCK_YUV (3)		// Y0Y1Y2...,  U0U1U2..., V0V1V2...
#define PIXEL_FORMAT_YPUPVP_444 (4)		// pRaw[0] pointers to Y Block; pRaw[1] pointers to U Block; pRaw[2] pointers to V Block
#define PIXEL_FORMAT_BLOCK_YUV_422 (5) // Y0Y1Y2..., U01U23..., V01V23...
#define PIXEL_FORMAT_BLOCK_YUV_420 (6) // Y0Y1Y2..., U0U1..., V0V1...

	typedef struct _IMAGE
	{
		int image_w;
		int image_h;
		int wideStep;
		int pix_format;
		PBYTE imageData[3];
	} IMAGE;

	typedef enum
	{
		DATA_FORMAT_NV12 = 0,
		DATA_FORMAT_RGB = 1,
		DATA_FORMAT_BGR = 2,
		DATA_FORMAT_GRAY = 3,
		DATA_FORMAT_NV16 = 4,
		DATA_FORMAT_YUV420P = 5,
	} EDataFormat;

	typedef struct TagFrameHeader
	{
		int net_id;
		int data_num;
		int dst_use_phy;
		unsigned long dst_phy_addr;
		u64 frame_no;
		int camera_id;
		int dl_width;
		int dl_height;
	} TFrameHeader;

	typedef struct TagDateHeader
	{
		int width;
		int pitch;
		int height;
		int dim;
		int data_format;
		int data_size;
		int use_roi;
		int src_use_phy;
		unsigned long src_phy_addr;
		CV_RECT roi;
	} TDataHeader;

#ifdef __cplusplus
}
#endif

#endif
