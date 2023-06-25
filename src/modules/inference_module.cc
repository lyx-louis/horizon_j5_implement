/**
 * Copyright Horizon Robotics
inference_module_15fps_roi_success_track_add.cc
 
 */
#include "inference_module.h"

#include <fstream>
#include <iomanip>

#include "hobotlog/hobotlog.hpp"
#include "image_utils.h"
#include <cstdlib>
#include <ctime>
#include "dnn/hb_dnn.h"
#include <numeric> 
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include "ipc_demo.h"
#include <opencv2/dnn/dnn.hpp>
#include "comm_api.h"
#include "rte_client_api.h"
#include <vector>

// #include "common_api.h"
//#include "post_process.h"

TDetResult allChanDetResult;

namespace J5Sample {

#define DUMP_IMG_FOR_TEST
#define INPUTHEIGHT 256
#define INPUTWIDTH 704
#define PI 3.141592653589793
#define NUM_CAM 6
#define LANE_POINT_NUM	1000
#define BEV_POINT_NUM	100

#ifdef DUMP_IMG_FOR_TEST
using namespace std;
using namespace cv;
static std::map<int, int> _dumpImgCountMap = {{0, 0}, {1, 0}, {2, 0}, {3, 0},
                                              {4, 0}, {5, 0}, {6, 0}, {7, 0}};
#endif
PTQFcosConfig default_ptq_fcos_config = {
    {{8, 16, 32, 64, 128}},
    80,
    {"person",        "bicycle",      "car",
     "motorcycle",    "airplane",     "bus",
     "train",         "truck",        "boat",
     "traffic light", "fire hydrant", "stop sign",
     "parking meter", "bench",        "bird",
     "cat",           "dog",          "horse",
     "sheep",         "cow",          "elephant",
     "bear",          "zebra",        "giraffe",
     "backpack",      "umbrella",     "handbag",
     "tie",           "suitcase",     "frisbee",
     "skis",          "snowboard",    "sports ball",
     "kite",          "baseball bat", "baseball glove",
     "skateboard",    "surfboard",    "tennis racket",
     "bottle",        "wine glass",   "cup",
     "fork",          "knife",        "spoon",
     "bowl",          "banana",       "apple",
     "sandwich",      "orange",       "broccoli",
     "carrot",        "hot dog",      "pizza",
     "donut",         "cake",         "chair",
     "couch",         "potted plant", "bed",
     "dining table",  "toilet",       "tv",
     "laptop",        "mouse",        "remote",
     "keyboard",      "cell phone",   "microwave",
     "oven",          "toaster",      "sink",
     "refrigerator",  "book",         "clock",
     "vase",          "scissors",     "teddy bear",
     "hair drier",    "toothbrush"},
    ""};


struct ScoreId {
  float score;
  int id;
};

static s32 g_s32Fps = 30;					// 设置输出帧率
static u32 g_u32Topic = 1;					// 设置消息发布ID

#define PUB_MESSAGE_NUM_MAX		100

typedef struct {
	s32 s32CmdStart;		// 收到开始命令
	s32 s32CmdDataHead;		// 第一帧数据
	s32 s32CheckEndCount;	// 比较结束编号
	s32 s32CmdEnd;			// 收到结束命令
	s32 s32RecvPlace;		// 0 --- (PUB_MESSAGE_NUM_MAX-1)
	s32 s32RecvSize[PUB_MESSAGE_NUM_MAX]; // 记录已接收数据长度
} TRecordInfo;
/*
typedef struct {
	f32 x = 0.;
	f32 y = 0.;
} TF32Point;
*/
typedef struct {
	s32 s32Type;
	s32 s32Size;
} TMessageInfo;

typedef struct {
	s32 s32PubNum;
	TMessageInfo tPubInfo[PUB_MESSAGE_NUM_MAX];
	u8 *ptPubData[PUB_MESSAGE_NUM_MAX]; // 数据存储
	TRecordInfo tRecode;
} TMessageData;
/*
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
   f32 lanePointNum = LANE_POINT_NUM;
   TF32Point tLanePoint[LANE_POINT_NUM];
   f32 bevPointNum = BEV_POINT_NUM;
   TF32Pointbev tBevPoint[BEV_POINT_NUM][4];
} TShowPoint;

*/

/*
typedef struct {
	f32 lanePointNum = LANE_POINT_NUM;
	TF32Point tLanePoint[LANE_POINT_NUM];
	f32 bevPointNum = BEV_POINT_NUM;
	TF32Point tBevPoint[BEV_POINT_NUM][4];
} TShowPoint;
*/

static u32 PUB_HEAD_START = 0x5A5AA5A5;
static u32 PUB_HEAD_OK_END = 0xA5AA5A5A;
static u32 PUB_HEAD_FAIL_END = 0xA5AA5A5B;

s32 pub_data(u32 topic, TMessageData *ptPubData)
{
	s32 s32PubFail = 0;

	if (ptPubData == NULL) {
		return -1;
	}

	{
		s8 *ptr = (s8 *)&PUB_HEAD_START;
		if (RTE_API_SendMsg(topic, ptr, sizeof(u32)) != 0) {
			log_color_err("RTE_API_SendMsg head start fail");
			return -1;
		}
	}

	do {
		if (RTE_API_SendMsg(topic, (s8 *)ptPubData, sizeof(s32) + sizeof(TMessageInfo)*PUB_MESSAGE_NUM_MAX) != 0) {
			log_color_err("RTE_API_SendMsg data head fail");
			s32PubFail = 1;
			break;
		}

		for (s32 i=0; i<ptPubData->s32PubNum; i++) {
			s32 offset = 0;
			s32 s32SendMax = RTE_MSG_MAX_LENGTH / 2;

			while(offset < ptPubData->tPubInfo[i].s32Size) {
				s32 sendLen = ((ptPubData->tPubInfo[i].s32Size - offset) <= s32SendMax) ? (ptPubData->tPubInfo[i].s32Size - offset) : s32SendMax;
				if (RTE_API_SendMsg(topic, (s8 *)ptPubData->ptPubData[i] + offset, sendLen) != 0) {
					log_color_err("RTE_API_SendMsg data(%d) fail", i);
					s32PubFail = 1;
					break;
				}
				offset += sendLen;
				// log_debug("send %d", sendLen);
			}

			if (s32PubFail) {
				break;
			}
		}
	} while(0);

	{
		s8 *ptr = (s32PubFail) ? (s8 *)&PUB_HEAD_FAIL_END : (s8 *)&PUB_HEAD_OK_END;
		if (RTE_API_SendMsg(topic, ptr, sizeof(u32)) != 0) {
			log_color_err("RTE_API_SendMsg head end fail");
		}
	}

	return 0;
}

#define CLIENT_IP_ADDR	NULL

bool cmp1(pair<int, int>a, pair<int, int>b)
{
    return a.first > b.first;
}

#define HB_CHECK_SUCCESS(value, errmsg)              \
  do {                                               \
    /*value can be call of function*/                \
    auto ret_code = value;                           \
    if (ret_code != 0) {                             \
      LOGE << errmsg << ", error code:" << ret_code; \
      return ret_code;                               \
    }                                                \
  } while (0);

int InferenceModule::LoadConfig() {
  std::string default_value = "";
  LOGI << "Load Inference config file:" << config_file_;
  Json::Value cfg_jv;
  std::ifstream infile(config_file_);
  if (!infile) {
    LOGE << "error!!! Inference config file is not exist";
    return -1;
  }
  infile >> cfg_jv;
  config_.reset(new JsonConfigWrapper(cfg_jv));

  if (config_->HasMember("model_file_path")) {
    model_file_path_ = config_->GetSTDStringValue("model_file_path");
  } else {
    LOGE << "config has not model_file_path";
    return -1;
  }

  if (config_->HasMember("score_threshold")) {
    score_threshold_ = config_->GetFloatValue("score_threshold");
  }

  if (config_->HasMember("topk")) {
    topk_ = config_->GetIntValue("topk");
  }

  if (config_->HasMember("community_qat")) {
    community_qat_ = config_->GetBoolValue("community_qat");
    ;
  }

  LOGD << "Load config suc";
  return 0;
}

std::string tensorShape2str(const hbDNNTensorShape &tensor) {
  std::stringstream ss;
  for (int32_t i = 0; i < tensor.numDimensions; ++i) {
    ss << tensor.dimensionSize[i]
       << (i == (tensor.numDimensions - 1) ? "" : "x");
  }
  return ss.str();
}


template<typename T>
std::vector<int> argsort(const std::vector<T>& a) {
    // 返回从大到小的排序对应的序号
    int Len = a.size();
    std::vector<int> idx(Len, 0);
    for (int i = 0; i < Len; i++)
    {
        idx[i] = i;
    }
    std::sort(idx.begin(), idx.end(), [&a](int i1, int i2) {return a[i1] > a[i2]; });
    return idx;
}

inline float sigmoid(float x)
{
    return (1 / (1 + exp(-x)));
}

std::vector<int> TopKIndex(const std::vector<float>& vec, int topk)
{
    std::vector<int> topKIndex;
    topKIndex.clear();
    std::vector<size_t> vec_index(vec.size());
    std::iota(vec_index.begin(), vec_index.end(), 0);
    std::sort(vec_index.begin(), vec_index.end(), [&vec](size_t index_1, size_t index_2) { return vec[index_1] > vec[index_2]; });
    int k_num = std::min<int>(vec.size(), topk);
    for (int i = 0; i < k_num; ++i)
    {
        topKIndex.emplace_back(vec_index[i]);
    }
    return topKIndex;
}

void topk(std::vector<float> hm, int top_num, std::vector<float>& topk_scores, std::vector<int>& topk_inds)
{
    topk_inds = TopKIndex(hm, top_num);
    // for (int i = 0; i < top_num; i++)
    for (int i = 0; i < 20; i++)
    {
        topk_scores[i] = hm[topk_inds[i]];
        std::cout << "topk scores" << i << " : -> " << topk_scores[i] << std::endl;
        //std::cout << " hm values" << " : -> " << hm[i] << std::endl;
        //std::cout << " topk indices" << " : -> " << topk_inds[i] << std::endl;
    }
}

void filter_scores_new(std::vector<float> hm, std::vector<std::pair<float, int>>& hm_pair) {
    for (int i = 0; i < hm.size(); i++)
    {
        float threshold = 0.1;

        if (hm[i] > threshold)
        {
            float fi = hm[i++];
            float se = i++;
            std::pair<float, float> p(fi, se);
            hm_pair.emplace_back(p);
        }

    }
}

void filter_scores(std::vector<float> topk_scores, std::vector<int> topk_inds, std::vector<float>& topk_scores_new, std::vector<int>& topk_inds_new)
{
    for (int i = 0; i < topk_scores.size(); i++)
    {
        float threshold = 0.1;

        if (topk_scores[i] > threshold)
        {
            topk_scores_new.emplace_back(topk_scores[i]);

            topk_inds_new.emplace_back(topk_inds[i]);
        }
    }
}

void get_row_col(std::vector<int> topk_inds_new, std::vector<int>& obj_rows, std::vector<int>& obj_cols)
{
    for (int i = 0; i < topk_inds_new.size(); i++)
    {
        obj_rows.emplace_back(topk_inds_new[i] / 200);  //?
        obj_cols.emplace_back(topk_inds_new[i] % 200);
    }
}

void circle_nms(std::vector<int> obj_rows, std::vector<int> obj_cols, std::vector<float> topk_scores_new, int nms_dist_thre, std::vector<int>& keep)  
{ // 对一列数进行比较，首先对最大值，计算其他数和他的距离，如果距离太近，判断为是同一个框，因此排除掉那些和他相近的，再依次递归下去找到第二个最大的数，对其他数做同样的操作。
    std::vector<int> order;
    order = argsort(topk_scores_new); //[3.4 3.3 3.1]
    int ndets = topk_scores_new.size(); //3
    std::vector<int> suppressed(ndets, 0);//[0 0 0]
    for (int i_ = 0; i_ < ndets; i_++)
    {
        int i = order[i_];                      //  3.4start with highest score box
        if (suppressed[i] == 1)   continue;   // if any box have enough iou with this, remove it.
        keep.emplace_back(i);
        for (int j_ = i_ + 1; j_ < ndets; j_++)
        {
            int j = order[j_];
            if (suppressed[j] == 1)  continue;
            int dist = pow(obj_rows[i] - obj_rows[j], 2) + pow(obj_cols[i] - obj_cols[j], 2);  // pow(x,y)  返回x的y次方
            if (dist <= nms_dist_thre)      suppressed[j] = 1;
        }
    }
}

void filter_obj(std::vector<int> obj_rows, std::vector<int> obj_cols, std::vector<int> keep, std::vector<int>& filter_obj_rows, std::vector<int>& filter_obj_cols)
{
    for (int i = 0; i < keep.size(); i++)
    {
        filter_obj_rows.emplace_back(obj_rows[keep[i]]);
        filter_obj_cols.emplace_back(obj_cols[keep[i]]);
    }
}

void get_rotation_matrix(std::vector<std::vector<float>> product_matrix, std::vector<std::vector<float>>& rotation_matrix) {
    rotation_matrix[0][0] = product_matrix[1][1];
    rotation_matrix[0][1] = product_matrix[1][2];
    rotation_matrix[0][2] = product_matrix[1][3];
    rotation_matrix[1][0] = product_matrix[2][1];
    rotation_matrix[1][1] = product_matrix[2][2];
    rotation_matrix[1][2] = product_matrix[2][3];
    rotation_matrix[2][0] = product_matrix[3][1];
    rotation_matrix[2][1] = product_matrix[3][2];
    rotation_matrix[2][2] = product_matrix[3][3];
}

float norm_value(std::vector<float> vector, int size) {
    int i;
    float square_sum = 0.;
    for (i = 0; i < size; i++) {
        square_sum += vector[i] * vector[i];
    }
    return sqrt(square_sum);
}

void vec_norm(std::vector<float> vector, std::vector<float>& vector_new, int size) {
    /*
    Note: 'vector' and 'vector_new' must have the same size.
    */
    int i;
    float square_sum = 0.;
    for (i = 0; i < size; i++) {
        square_sum += vector[i] * vector[i];
    }
    square_sum = sqrt(square_sum);
    if (square_sum < 1.0E-7)    square_sum = 1.0E-7;
    for (i = 0; i < size; i++) {
        vector_new[i] = vector[i] / square_sum;
    }
}

void get_quat_norm(std::vector<float> quat, std::vector<float>& quat_norm) {
    float n = norm_value(quat, 4);
    if (n > 0)  vec_norm(quat, quat_norm, 4);
}

void get_q_matrix(std::vector<float> quat_norm, std::vector<std::vector<float>>& q_matrix) {
    q_matrix[0][0] = quat_norm[0];
    q_matrix[0][1] = -quat_norm[1];
    q_matrix[0][2] = -quat_norm[2];
    q_matrix[0][3] = -quat_norm[3];
    q_matrix[1][0] = quat_norm[1];
    q_matrix[1][1] = quat_norm[0];
    q_matrix[1][2] = -quat_norm[3];
    q_matrix[1][3] = quat_norm[2];
    q_matrix[2][0] = quat_norm[2];
    q_matrix[2][1] = quat_norm[3];
    q_matrix[2][2] = quat_norm[0];
    q_matrix[2][3] = -quat_norm[1];
    q_matrix[3][0] = quat_norm[3];
    q_matrix[3][1] = -quat_norm[2];
    q_matrix[3][2] = quat_norm[1];
    q_matrix[3][3] = quat_norm[0];
}

void get_q_bar_matrix(std::vector<float> quat_norm, std::vector<std::vector<float>>& q_bar_matrix) {
    q_bar_matrix[0][0] = quat_norm[0];
    q_bar_matrix[0][1] = -quat_norm[1];
    q_bar_matrix[0][2] = -quat_norm[2];
    q_bar_matrix[0][3] = -quat_norm[3];
    q_bar_matrix[1][0] = quat_norm[1];
    q_bar_matrix[1][1] = quat_norm[0];
    q_bar_matrix[1][2] = quat_norm[3];
    q_bar_matrix[1][3] = -quat_norm[2];
    q_bar_matrix[2][0] = quat_norm[2];
    q_bar_matrix[2][1] = -quat_norm[3];
    q_bar_matrix[2][2] = quat_norm[0];
    q_bar_matrix[2][3] = quat_norm[1];
    q_bar_matrix[3][0] = quat_norm[3];
    q_bar_matrix[3][1] = quat_norm[2];
    q_bar_matrix[3][2] = -quat_norm[1];
    q_bar_matrix[3][3] = quat_norm[0];
}

void get_q_bar_matrix_trans(std::vector<float> quat_norm, std::vector<std::vector<float>>& q_bar_matrix_trans) {
    q_bar_matrix_trans[0][0] = quat_norm[0];
    q_bar_matrix_trans[1][0] = -quat_norm[1];
    q_bar_matrix_trans[2][0] = -quat_norm[2];
    q_bar_matrix_trans[3][0] = -quat_norm[3];
    q_bar_matrix_trans[0][1] = quat_norm[1];
    q_bar_matrix_trans[1][1] = quat_norm[0];
    q_bar_matrix_trans[2][1] = quat_norm[3];
    q_bar_matrix_trans[3][1] = -quat_norm[2];
    q_bar_matrix_trans[0][2] = quat_norm[2];
    q_bar_matrix_trans[1][2] = -quat_norm[3];
    q_bar_matrix_trans[2][2] = quat_norm[0];
    q_bar_matrix_trans[3][2] = quat_norm[1];
    q_bar_matrix_trans[0][3] = quat_norm[3];
    q_bar_matrix_trans[1][3] = quat_norm[2];
    q_bar_matrix_trans[2][3] = -quat_norm[1];
    q_bar_matrix_trans[3][3] = quat_norm[0];
}

void matrix_mul(std::vector<std::vector<float>> A, int num_x_A, int num_y_A,
    std::vector<std::vector<float>> B, int num_x_B, int num_y_B,
    std::vector<std::vector<float>>& C) {
    /*
    Note:
    1. 'num_x_A' must be equal to 'num_y_B';
    2. the size of 'float* C' is 'num_y_A*num_xB', that is 'float C[num_y_A * num_x_B];'.
    */
    int i, j, m;
    for (j = 0; j < num_y_A; j++) {          // j is row
        for (i = 0; i < num_x_B; i++) {       // i is col
            float sum_tmp = 0.;
            for (m = 0; m < num_x_A; m++) {
                sum_tmp += A[j][m] * B[m][i];
            }
            C[j][i] = sum_tmp;
        }
    }
}

void quat_to_rotMatrix(std::vector<float>& quat, std::vector<std::vector<float>>& rotMatrix)
{
    // step1: get Rotation_matrix
    std::vector<float> quat_norm(4);
    get_quat_norm(quat, quat_norm);
    std::vector<std::vector<float>> q_matrix(4, std::vector<float>(4)), q_bar_matrix_trans(4, std::vector<float>(4));
    get_q_matrix(quat_norm, q_matrix);
    get_q_bar_matrix_trans(quat_norm, q_bar_matrix_trans);
    std::vector<std::vector<float>> product_matrix(4, std::vector<float>(4));
    matrix_mul(q_matrix, 4, 4, q_bar_matrix_trans, 4, 4, product_matrix);
    get_rotation_matrix(product_matrix, rotMatrix);
}

void quat_to_rotMatrix_scipy(std::vector<float>& quat, std::vector<std::vector<float>>& rotMatrix)
{
    std::vector<float> quat_norm(4);
    get_quat_norm(quat, quat_norm);
    float x = quat_norm[0];
    float y = quat_norm[1];
    float z = quat_norm[2];
    float w = quat_norm[3];
    float x2 = x * x;
    float y2 = y * y;
    float z2 = z * z;
    float w2 = w * w;
    float xy = x * y;
    float zw = z * w;
    float xz = x * z;
    float yw = y * w;
    float yz = y * z;
    float xw = x * w;
    rotMatrix[0][0] = x2 - y2 - z2 + w2;
    rotMatrix[1][0] = 2 * (xy + zw);
    rotMatrix[2][0] = 2 * (xz - yw);
    rotMatrix[0][1] = 2 * (xy - zw);
    rotMatrix[1][1] = -x2 + y2 - z2 + w2;
    rotMatrix[2][1] = 2 * (yz + xw);
    rotMatrix[0][2] = 2 * (xz + yw);
    rotMatrix[1][2] = 2 * (yz - xw);
    rotMatrix[2][2] = -x2 - y2 + z2 + w2;
}

void get_boxPts(std::vector<float> center, std::vector<float> size, std::vector<float> quat,
    std::vector<cv::Point3d>& box_3d, std::vector<cv::Point>& box_2d)
{
    float w = size[0];
    float l = size[1];
    float h = size[2];
    std::vector<float> x_corners = { 1,  1,  1,  1, -1, -1, -1, -1 };
    std::vector<float> y_corners = { 1, -1, -1,  1,  1, -1, -1,  1 };
    std::vector<float> z_corners = { 1,  1, -1, -1,  1,  1, -1, -1 };
    for (int i = 0; i < 8; i++) {
        x_corners[i] *= l / 2;
        y_corners[i] *= w / 2;
        z_corners[i] *= h / 2;
    }
    std::vector<std::vector<float>> corners;
    std::vector<std::vector<float>> rotMatrix(3, std::vector<float>(3));
    std::vector<std::vector<float>> corners_new(3, std::vector<float>(8));
    corners.emplace_back(x_corners);
    corners.emplace_back(y_corners);
    corners.emplace_back(z_corners);
    quat_to_rotMatrix(quat, rotMatrix);
    // Rotate
    matrix_mul(rotMatrix, 3, 3, corners, 8, 3, corners_new);
    // Translate
    for (int i = 0; i < center.size(); i++) {
        for (int j = 0; j < corners[i].size(); j++) {
            corners_new[i][j] += center[i];
        }
    }
    // get box points
    box_3d[0] = cv::Point3d(round(corners_new[0][2]), round(corners_new[1][2]), round(corners_new[2][2]));
    box_3d[1] = cv::Point3d(round(corners_new[0][3]), round(corners_new[1][3]), round(corners_new[2][3]));
    box_3d[2] = cv::Point3d(round(corners_new[0][7]), round(corners_new[1][7]), round(corners_new[2][7]));
    box_3d[3] = cv::Point3d(round(corners_new[0][6]), round(corners_new[1][6]), (corners_new[2][6]));
    box_2d[0] = cv::Point(round(corners_new[0][2]), round(corners_new[1][2]));
    box_2d[1] = cv::Point(round(corners_new[0][3]), round(corners_new[1][3]));
    box_2d[2] = cv::Point(round(corners_new[0][7]), round(corners_new[1][7]));
    box_2d[3] = cv::Point(round(corners_new[0][6]), round(corners_new[1][6]));
}

void get_obj_z_rows_cols(std::vector<float> hm, std::vector<int>& obj_z_rows, std::vector<int>& obj_z_cols)
{
    for (int i = 0; i < hm.size(); i++)
    {
        if (hm[i] > 0) {
            obj_z_rows.emplace_back(i / 200);
            obj_z_cols.emplace_back(i % 200);
        }
    }
}

void Mat_to_vec(cv::Mat rot, std::vector<std::vector<float>>& rot_)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rot_[i][j] = rot.ptr<float>(i)[j];
        }
    }
}

void matrix_inverse(std::vector<std::vector<float>> a, int N, std::vector<std::vector<float>>& b)
{
    /*
    求矩阵的逆, a表示输入的
    */
    using namespace std;
    int i, j, k;
    double max, temp;
    // 定义一个临时矩阵t
    double t[N][N];
    // 将a矩阵临时存放在矩阵t[n][n]中
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            t[i][j] = a[i][j];
        }
    }
    // 初始化B矩阵为单位矩阵
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            b[i][j] = (i == j) ? (double)1 : 0;
        }
    }
    // 进行列主消元，找到每一列的主元
    for (i = 0; i < N; i++)
    {
        max = t[i][i];
        // 用于记录每一列中的第几个元素为主元
        k = i;
        // 寻找每一列中的主元元素
        for (j = i + 1; j < N; j++)
        {
            if (fabs(t[j][i]) > fabs(max))
            {
                max = t[j][i];
                k = j;
            }
        }
        //cout<<"the max number is "<<max<<endl;
        // 如果主元所在的行不是第i行，则进行行交换
        if (k != i)
        {
            // 进行行交换
            for (j = 0; j < N; j++)
            {
                temp = t[i][j];
                t[i][j] = t[k][j];
                t[k][j] = temp;
                // 伴随矩阵B也要进行行交换
                temp = b[i][j];
                b[i][j] = b[k][j];
                b[k][j] = temp;
            }
        }
        if (t[i][i] == 0)
        {
            cout << "\nthe matrix does not exist inverse matrix\n";
            break;
        }
        // 获取列主元素
        temp = t[i][i];
        // 将主元所在的行进行单位化处理
        //cout<<"\nthe temp is "<<temp<<endl;
        for (j = 0; j < N; j++)
        {
            t[i][j] = t[i][j] / temp;
            b[i][j] = b[i][j] / temp;
        }
        for (j = 0; j < N; j++)
        {
            if (j != i)
            {
                temp = t[j][i];
                //消去该列的其他元素
                for (k = 0; k < N; k++)
                {
                    t[j][k] = t[j][k] - temp * t[i][k];
                    b[j][k] = b[j][k] - temp * b[i][k];
                }
            }
        }
    }
}

void draw_cv(cv::Mat& frame, std::vector<std::vector<float>> selected_corners)
{
    int n = selected_corners.size();
    std::vector<float> prev = selected_corners[n - 1];
    for (int i = 0; i < n; i++) {
        std::vector<float> corner = selected_corners[i];
        cv::line(frame, cv::Point(round(prev[0]), round(prev[1])), cv::Point(round(corner[0]), round(corner[1])), cv::Scalar(0, 0, 255), 2);
        prev[0] = corner[0];
        prev[1] = corner[1];
    }
}

std::vector<float> get_center_bottom_forward(std::vector<std::vector<float>> corners_2d_homo)
{
    std::vector<float> center_bottom_forward(2);
    for (int i = 0; i < 2; i++) {
        center_bottom_forward[i] = (corners_2d_homo[i][2] + corners_2d_homo[i][3]) / 2;
    }
    return center_bottom_forward;
}

std::vector<float> get_center_bottom(std::vector<std::vector<float>> corners_2d_homo)
{
    std::vector<float> center_bottom(2);
    for (int i = 0; i < 2; i++) {
        center_bottom[i] = (corners_2d_homo[i][2] + corners_2d_homo[i][3] + corners_2d_homo[i][7] + corners_2d_homo[i][6]) / 4;
    }
    return center_bottom;
}

std::string name_string_same_length(std::string ori_name, int length)
{
    int ori_name_len = strlen(ori_name.c_str());
    std::string new_name(length, '0');
    for (int i = 0; i < ori_name_len; i++)
    {
        new_name[length - (i + 1)] = ori_name[ori_name_len - (i + 1)];
    }
    return new_name;
}

void projection(cv::Mat frame, std::vector<std::vector<float>> centers, std::vector<std::vector<float>> quats,
    std::vector<std::vector<float>> whls, cv::Mat intrin, cv::Mat tran, cv::Mat rot, int view_id, int imgs_id, cv::Mat& frame_imshow)
{
    std::vector<std::vector<float>> rot_(3, std::vector<float>(3));
    std::vector<std::vector<float>> rot_inv(3, std::vector<float>(3));
    Mat_to_vec(rot, rot_);
    matrix_inverse(rot_, 3, rot_inv);
    std::vector<std::vector<float>> intrin_(3, std::vector<float>(3));
    Mat_to_vec(intrin, intrin_);
    std::vector<std::vector<float>> corners_2d_homo_head4(4, std::vector<float>(3));
    std::vector<std::vector<float>> corners_2d_homo_tail4(4, std::vector<float>(3));
    for (int ll = 0; ll < centers.size(); ll++)
    {
        std::vector<float> center = centers[ll];
        std::vector<float> quat(4);
        std::vector<float> r;
        std::vector<std::vector<float>> rotz(3, std::vector<float>(3));
        std::vector<std::vector<float>> rotz_new(3, std::vector<float>(3));
        quat[0] = quats[ll][1];
        quat[1] = quats[ll][2];
        quat[2] = quats[ll][3];
        quat[3] = quats[ll][0];
        quat_to_rotMatrix_scipy(quat, rotz);
        std::vector<float> whl = whls[ll];

        // 由自车坐标系转为相机坐标系
        for (int i = 0; i < center.size(); i++)
        {
            center[i] -= tran.ptr<float>(i)[0];
        }
        std::vector<std::vector<float>> center_(3, std::vector<float>(1));
        std::vector<std::vector<float>> center_new(3, std::vector<float>(1));
        for (int i = 0; i < center.size(); i++)
        {
            center_[i][0] = center[i];
        }
        matrix_mul(rot_inv, 3, 3, center_, 1, 3, center_new);                      // 自车坐标系下box旋转至相机坐标系
        matrix_mul(rot_inv, 3, 3, rotz, 3, 3, rotz_new);

        if (center_new[2][0] < 0)    continue;

        // 生成自车坐标系的八个坐标
        float w, h, l;
        w = whl[0];
        l = whl[1];
        h = whl[2];
        std::vector<float> x_corners = { 1,  1,  1,  1, -1, -1, -1, -1 };
        std::vector<float> y_corners = { 1, -1, -1,  1,  1, -1, -1,  1 };
        std::vector<float> z_corners = { 1,  1, -1, -1,  1,  1, -1, -1 };
        for (int i = 0; i < 8; i++) {
            x_corners[i] *= l / 2;
            y_corners[i] *= w / 2;
            z_corners[i] *= h / 2;
        }
        std::vector<std::vector<float>> corners;
        std::vector<std::vector<float>> corners_new(3, std::vector<float>(8));
        corners.emplace_back(x_corners);
        corners.emplace_back(y_corners);
        corners.emplace_back(z_corners);                             // 生成8个点
        matrix_mul(rotz_new, 3, 3, corners, 8, 3, corners_new);   // 8个点加入旋转角
        for (int i = 0; i < center.size(); i++) {
            for (int j = 0; j < corners[i].size(); j++) {
                corners_new[i][j] += center_new[i][0];                   // 8个点平移至相机坐标系
            }
        }

        std::vector<std::vector<float>> corners_2d_homo(3, std::vector<float>(8));
        matrix_mul(intrin_, 3, 3, corners_new, 8, 3, corners_2d_homo);
        for (int i = 0; i < 8; i++) {
            corners_2d_homo[0][i] /= corners_2d_homo[2][i];
            corners_2d_homo[1][i] /= corners_2d_homo[2][i];
            corners_2d_homo[2][i] /= corners_2d_homo[2][i];       // 齐次化
        }
        int nn = 0;
        for (int g = 0; g < 8; g++)
        {
            if (corners_2d_homo[0][g] > 0 && corners_2d_homo[0][g] < 1600 && corners_2d_homo[1][g] > 0 && corners_2d_homo[1][g] < 960)
            {
                nn += 1;
                if (nn >= 2)
                {
                    for (int i = 0; i < 4; i++) {
                        cv::line(frame, cv::Point((int)(corners_2d_homo[0][i]), (int)(corners_2d_homo[1][i])), cv::Point((int)(corners_2d_homo[0][i + 4]), (int)(corners_2d_homo[1][i + 4])), cv::Scalar(0, 0, 255), 2);
                    }
                    // get head and tail corners
                    for (int id = 0; id < 4; id++) {
                        corners_2d_homo_head4[id][0] = corners_2d_homo[0][id];
                        corners_2d_homo_head4[id][1] = corners_2d_homo[1][id];
                        corners_2d_homo_head4[id][2] = corners_2d_homo[2][id];
                        corners_2d_homo_tail4[id][0] = corners_2d_homo[0][id + 4];
                        corners_2d_homo_tail4[id][1] = corners_2d_homo[1][id + 4];
                        corners_2d_homo_tail4[id][2] = corners_2d_homo[2][id + 4];
                    }
                    draw_cv(frame, corners_2d_homo_head4);
                    draw_cv(frame, corners_2d_homo_tail4);
                    std::vector<float> center_bottom_forward = get_center_bottom_forward(corners_2d_homo);
                    std::vector<float> center_bottom = get_center_bottom(corners_2d_homo);
                    cv::line(frame, cv::Point((int)(center_bottom[0]), (int)(center_bottom[1])),
                        cv::Point((int)(center_bottom_forward[0]), (int)(center_bottom_forward[1])), cv::Scalar(0, 0, 255), 2);
                }
            }
        }
    }
}



void postprocess(hbDNNTensor* j5tensor, std::vector<cv::Mat> intrins_list, std::vector<cv::Mat> trans_list, std::vector<cv::Mat> rots_list) {

    std::vector<std::vector<float>> outputTensorValues_list; // 将tensor转换成vector
    for (int i = 0; i < 5; i++) { // 读取5个output
        hbSysFlushMem(&(j5tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
        int n_dim = j5tensor->properties.validShape.numDimensions; // 4
        int* shape = j5tensor->properties.validShape.dimensionSize; //reg: 1，2，200，200; height:1,1,200,200; dim:1,3,200,200; rot:1,2,200,200; hm:1,1,200,200
        int output_batch = shape[0];
        int output_channel = shape[1];
        int output_height = shape[2];
        int output_width = shape[3];
        int output_batch_size = output_batch * output_channel * output_height * output_width;
        auto info_tensor = reinterpret_cast<float*>(j5tensor->sysMem[0].virAddr); //地址起始点
        std::vector<float> output_tensor(info_tensor, info_tensor + output_batch_size); //用该地址起始点初始化vector
        outputTensorValues_list.emplace_back(output_tensor);
        j5tensor++;
    }
    std::vector<float> reg = outputTensorValues_list[0]; //直接读取tensor
    std::vector<float> height = outputTensorValues_list[1];
    std::vector<float> dim = outputTensorValues_list[2];
    std::vector<float> rot = outputTensorValues_list[3];
    std::vector<float> hm = outputTensorValues_list[4];

    int top_num = 200;
    int nms_dist_thre = 5;
    int nx[2] = { 200, 200 };
    cv::Mat binimgs(cv::Size(nx[0], nx[1]), CV_8UC3, cv::Scalar(0, 0, 0));
  /*  std::vector<float> topk_scores(top_num, 0);
    std::vector<int> topk_inds(top_num, 0);
    std::vector<float> topk_scores_new;
    std::vector<int> topk_inds_new;
    std::vector<int> obj_rows;
    std::vector<int> obj_cols;
    std::vector<int> filter_obj_rows;
    std::vector<int> filter_obj_cols;
    std::vector<int> keep;
  
    topk(hm, top_num, topk_scores, topk_inds);
    filter_scores(topk_scores, topk_inds, topk_scores_new, topk_inds_new);
*/
    std::vector<float> topk_scores(top_num, 0);
    std::vector<int> topk_inds(top_num, 0);

    std::vector<int> obj_rows;
    std::vector<int> obj_cols;
    std::vector<int> filter_obj_rows;
    std::vector<int> filter_obj_cols;
    std::vector<int> keep;
    std::vector<float> hm_new;
    std::vector<float> hm_new_index;
    std::vector<std::pair<float, int>> hm_pair;

    filter_scores_new(hm, hm_pair);
    int k_num = std::min<int>(hm_pair.size(), top_num); //
    std::sort(hm_pair.begin(), hm_pair.end(), cmp1);
    std::vector<std::pair<float, int>> hm_pair_k_num(hm_pair.begin(), hm_pair.begin() + k_num);

    std::vector<float> topk_scores_new(hm_pair_k_num.size());
    std::vector<int> topk_inds_new(hm_pair_k_num.size());

    // ʹ��std::transform��lambda������hm_pair����vecFirst��vecSecond
    std::transform(hm_pair_k_num.begin(), hm_pair_k_num.end(), topk_scores_new.begin(), [](const std::pair<float, int>& p) { return p.first; });
    std::transform(hm_pair_k_num.begin(), hm_pair_k_num.end(), topk_inds_new.begin(), [](const std::pair<float, int>& p) { return p.second; });



    //Start = clock();
    //topk(hm, top_num, topk_scores, topk_inds);
   // filter_scores(topk_scores, topk_inds, topk_scores_new, topk_inds_new);
    //for (int i = 0; i < hm_pair.size(); i++) {
    //    cout <<"topk_scores"<< topk_scores_new[i] << endl;
    //    cout << "topk_inds" << topk_inds_new[i] << endl;
    //}
   /* 
   Start = clock();
   End = clock();
    double Sort_filter_time = (double)(End - Start) / CLOCKS_PER_SEC;
    std::cout << "Sort filter time:" << Sort_filter_time * 1000 << "ms" << std::endl;*/

    get_row_col(topk_inds_new, obj_rows, obj_cols);

    // Start = clock();
    circle_nms(obj_rows, obj_cols, topk_scores_new, nms_dist_thre, keep);
    /*   End = clock();
       double Cnms_time = (double)(End - Start) / CLOCKS_PER_SEC;
       std::cout << "Cnms time:" << Cnms_time * 1000 << "ms" << std::endl;*/


    filter_obj(obj_rows, obj_cols, keep, filter_obj_rows, filter_obj_cols); // 对刚刚nms完的框，保留下来

     

    //Start = clock();
    std::vector<std::vector<float>> centers_pre;
    std::vector<std::vector<float>> whls_pre;
    std::vector<std::vector<float>> quats_pre;
    for (int i = 0; i < filter_obj_rows.size(); i++)
    {
        std::vector<float> center, center1, size;
        std::vector<float> quat(4, 0);
        std::vector<float> quat1(4, 0);
        std::vector<cv::Point3d> pts_3d(4);
        std::vector<cv::Point> pts_2d(4);
        center.emplace_back(nx[1] - filter_obj_cols[i] - 2 * reg[filter_obj_rows[i] * 200 + filter_obj_cols[i]]);
        center.emplace_back(nx[0] - filter_obj_rows[i] - 2 * reg[200 * 200 + filter_obj_rows[i] * 200 + filter_obj_cols[i]]);
        center.emplace_back(0);
        center1.emplace_back(filter_obj_rows[i] / 2.0 + reg[filter_obj_rows[i] * 200 + filter_obj_cols[i]] - 50);
        center1.emplace_back(filter_obj_cols[i] / 2.0 + reg[200 * 200 + filter_obj_rows[i] * 200 + filter_obj_cols[i]] - 50);
        center1.emplace_back(height[filter_obj_rows[i] * 200 + filter_obj_cols[i]]);
        size.emplace_back(2 - dim[filter_obj_rows[i] * 200 + filter_obj_cols[i]]);
        size.emplace_back(6 - dim[200 * 200 + filter_obj_rows[i] * 200 + filter_obj_cols[i]]);
        size.emplace_back(2.5 - dim[2 * 200 * 200 + filter_obj_rows[i] * 200 + filter_obj_cols[i]]);
        float ori_rad = 0;
        float ori_rad1 = 0;
        if (rot[200 * 200 + filter_obj_rows[i] * 200 + filter_obj_cols[i]] < 0)
        {
            ori_rad = -acos(rot[filter_obj_rows[i] * 200 + filter_obj_cols[i]]) - PI / 2;
            ori_rad1 = -acos(rot[filter_obj_rows[i] * 200 + filter_obj_cols[i]]);
        }
        else
        {
            ori_rad = acos(rot[filter_obj_rows[i] * 200 + filter_obj_cols[i]]) - PI / 2;
            ori_rad1 = acos(rot[filter_obj_rows[i] * 200 + filter_obj_cols[i]]);
        }
        quat[0] = cos(ori_rad / 2);
        quat[3] = sin(ori_rad / 2);
        quat1[0] = cos(ori_rad1 / 2);
        quat1[3] = sin(ori_rad1 / 2);
        centers_pre.emplace_back(center1);
        whls_pre.emplace_back(size);
        quats_pre.emplace_back(quat1);
        get_boxPts(center, size, quat, pts_3d, pts_2d);

        cv::Point root_pts[4];
        for (int id = 0; id < 4; id++) {
            root_pts[id] = pts_2d[id];
        }
        const cv::Point* ppt[1] = { root_pts };
        int npt[] = { 4 };
        cv::fillPoly(binimgs, ppt, npt, 1, cv::Scalar(0, 0, 255));
    }
    /*   End = clock();
       double for_boucle = (double)(End - Start) / CLOCKS_PER_SEC;
       std::cout << "fb time:" << for_boucle * 1000 << "ms" << std::endl;
    */

    // -- add_ego --
   // Start = clock();
    cv::Point root_ego_pts[4];
    std::vector<std::vector<float>> pts_ego_ori(4, std::vector<float>(2));
    std::vector<float> bx_ego = { -49.75, -49.75 };
    std::vector<float> dx_ego = { 0.5, 0.5 };

    float W = 1.25;
    pts_ego_ori[0][0] = W / 2.0;
    pts_ego_ori[0][1] = -2.484 / 2. + 0.5;
    pts_ego_ori[1][0] = W / 2.0;
    pts_ego_ori[1][1] = 2.484 / 2. + 0.5;
    pts_ego_ori[2][0] = -W / 2.0;
    pts_ego_ori[2][1] = 2.484 / 2. + 0.5;
    pts_ego_ori[3][0] = -W / 2.0;
    pts_ego_ori[3][1] = -2.484 / 2. + 0.5;
    for (int i = 0; i < 4; i++) {
        root_ego_pts[i].x = (pts_ego_ori[i][0] - bx_ego[0]) / dx_ego[0];
        root_ego_pts[i].y = (pts_ego_ori[i][1] - bx_ego[1]) / dx_ego[1];
    }
    const cv::Point* ppt_ego[1] = { root_ego_pts };
    int npt_ego[] = { 4 };
    cv::fillPoly(binimgs, ppt_ego, npt_ego, 1, cv::Scalar(0, 255, 0));
    //End = clock();
    //double Ego_time = (double)(End - Start) / CLOCKS_PER_SEC;
    //std::cout << "Ego time:" << Ego_time * 1000 << "ms" << std::endl;


    // -- show --
   // Start = clock();
    cv::Mat showimg;
    std::vector<cv::Mat> img_imshow_lists(NUM_CAM);
    std::vector<cv::Mat> frame123(3), frame456(3), frame123_456(2), frame_union(2);
/*
    for (int view_id = 0; view_id < NUM_CAM; view_id++)
    {
        projection(img_lists[view_id], centers_pre, quats_pre, whls_pre, intrins_list[view_id], trans_list[view_id], rots_list[view_id], view_id, imgs_id, img_imshow_lists[view_id]);
    }
*/


    cv::resize(binimgs, binimgs, cv::Size(400, 400));
    binimgs.copyTo(frame_union[1]);
    // --- save bin_img ---
    std::string bevimg_file_path = "perfbin.jpg";

    char fntmp3[1024] = {0};

    sprintf(fntmp3, "perfbin-%02d.jpg", rand());
    //cv::imwrite(fntmp,mat_dest);
    std::string str(fntmp3);
    cv::imwrite(str, binimgs);
}

std::string getCurrentTimeMicrosecondsString()
{
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::microseconds>(now);

    // 转换为time_t
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d%H_%M_%S");

    // 输出微秒部分
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(now_ms.time_since_epoch()) -
                  std::chrono::duration_cast<std::chrono::seconds>(now_ms.time_since_epoch());
    ss << '_' << std::setw(6) << std::setfill('0') << micros.count();

    return ss.str();
}



int InferenceModule::Init(hobot::RunContext *context) {
  // load model
  LOGD << "Enter InferenceModule::Init";
  HB_CHECK_SUCCESS(LoadConfig(), "LoadConfig failed.");
  LOGD << "Configuration info : model_file_path_=" << model_file_path_
       << ",   topk_=" << topk_ << ", score_threshold_=" << score_threshold_;

  auto modelFileName = model_file_path_.c_str();
  HB_CHECK_SUCCESS(
      hbDNNInitializeFromFiles(&packed_dnn_handle_, &modelFileName, 1),
      "hbDNNInitializeFromFiles failed");
  LOGD << "hbDNNInitializeFromFiles success";

  // get dnn handle
  const char **model_name_list;
  int model_count = 0;
  HB_CHECK_SUCCESS(
      hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle_),
      "hbDNNGetModelNameList failed");
  LOGD << "hbDNNGetModelNameList success";
  HOBOT_CHECK(model_name_list[0]) << "model name is invalid.";

  // get model handle
  HB_CHECK_SUCCESS(
      hbDNNGetModelHandle(&dnn_handle_, packed_dnn_handle_, model_name_list[0]),
      "hbDNNGetModelHandle failed");
  LOGD << "hbDNNGetModelHandle success";
  HB_CHECK_SUCCESS(
      hbDNNGetInputTensorProperties(&input_properties_, dnn_handle_, 0),
      "hbDNNGetInputTensorProperties failed");

  // get required image width & height by model
  int h_idx, w_idx, c_idx;
  get_hwc_index(input_properties_.tensorLayout, &h_idx, &w_idx, &c_idx);
  model_intput_width_ = INPUTWIDTH;
  model_intput_height_ = INPUTHEIGHT;

  // create infer threads
  const int thread_count = 8;
  worker_ = std::make_shared<CThreadPool>();
  worker_->CreatThread(thread_count, "InferProcess");
  LOGW << thread_count << " threads are created and used to inference.";
  LOGD << "Leave InferenceModule::Init";
  is_inited_ = true;

  // post process init
  TCommBuff tGridFile = api_comm_readFileToBuff("grid.bin");
  post_init(tGridFile);

  return 0;
}

class Vec_vioMessage : public hobot::Message {
  public:
  std::vector<spVioMessage> vios_;
  Vec_vioMessage(){}
  explicit Vec_vioMessage(std::vector<spVioMessage> vios) : vios_(vios) {}
};


FORWARD_DEFINE(InferenceModule, 0) {
  srand((unsigned) time(NULL));
  auto vio_msg_list = input[0];

  LOGE << "#############################################"<<vio_msg_list->size();
  hobot::spMessage in = (*input[0])[0];
  std::vector<spVioMessage> res;
  res = static_cast<Vec_vioMessage *>(in.get())->vios_;
  //std::cout<<res.size()<<std::endl;
  //std::cout<<res[0]->channel_<<std::endl;
 // std::cout<<res[1]->channel_<<std::endl;
 // std::cout<<res[2]->channel_<<std::endl;
  /*  auto &pym_img = (pym_level_ == -1)
                      ? pyramid_message->pym_image_->src_info_
                      : pyramid_message->pym_image_->bl_ds_[pym_level_];
  // class pym_img: width,height,stride,y_paddr,c_paddr,y_vaddr,c_vaddr
  int width = pym_img.width;
  int height = pym_img.height;
  uint8_t *y_data = reinterpret_cast<uint8_t *>(pym_img.y_vaddr);
  uint8_t *c_data = reinterpret_cast<uint8_t *>(pym_img.c_vaddr);*/


                   //  can write the correct img

/*0
  auto &pym_res0 = res[0]->pym_image_->src_info_;
  int res0width = pym_res0.width;
  int res0height = pym_res0.height;
  uint8_t *res0y_data = reinterpret_cast<uint8_t *>(pym_res0.y_vaddr);
  uint8_t *res0c_data = reinterpret_cast<uint8_t *>(pym_res0.c_vaddr);
  std::cout<<"new start"<<std::endl;
  uint8_t * yuv_all = new uint8_t[res0height*res0width*3/2];
  std::cout<<"new success"<<std::endl;
  memcpy(yuv_all,res0y_data,res0width*res0height);
  memcpy(yuv_all+res0height*res0width,res0c_data,res0width*res0height*0.5);
  cv::Mat mat_src;
  int mat_src_size = res0height*res0width*3/2;
  mat_src.create(res0height*3/2,res0width,CV_8UC1);
  memcpy(mat_src.data,yuv_all,mat_src_size);
  cv::Mat mat_dest;

  cv::cvtColor(mat_src,mat_dest,cv::COLOR_YUV2BGR_NV12);
  char fntmp[1024] = {0};

  sprintf(fntmp, "chnl1-%02d-Infer_result.jpg", rand());
  cv::imwrite(fntmp,mat_dest);
  delete[] yuv_all;
*/

  // Try to Get&Process Sensors Message
  if (input.size() > 1) {
    auto sio_msg_list = input[1];
    for (uint32_t msg_idx = 0; msg_idx < sio_msg_list->size(); ++msg_idx) {
      auto muti_sio_message =
          std::dynamic_pointer_cast<MultiSioMessage>(sio_msg_list->at(msg_idx));
      for (uint32_t sensor_chnl = 0;
           sensor_chnl < muti_sio_message->multi_sio_msg_.size();
           ++sensor_chnl) {
        auto sio_message = muti_sio_message->multi_sio_msg_[sensor_chnl];
        if (sio_message) {  // do something
          auto sensor_data = sio_message->data_buf_;
          LOGE << "Recv sensor data chnl_id =" << sensor_chnl
               << ", seq_id = " << sio_message->sequnce_id_
               << ", data_len = " << sensor_data->size();
        }
      }
      muti_sio_message = nullptr;
    }
  }

  // Get&Process Video Message 
  //for (uint32_t msg_idx = 0; msg_idx < vio_msg_list->size(); ++msg_idx) {
  //  auto vio_message =
  //      std::dynamic_pointer_cast<VioMessage>(vio_msg_list->at(msg_idx));
    // do something
    if (res.size()!=0) {
      worker_->PostTask(std::bind(&InferenceModule::DoProcess, this,
                                  res, workflow, this, 0, context));
    } else {
      LOGD << "VioMessage is nullptr.";
      workflow->Return(this, 0, nullptr, context);
    }
 // }

}

void InferenceModule::Reset() {
  LOGD << "Enter InferenceModule::Reset";
  if (false == is_inited_) {
    return;
  }
  is_inited_ = false;
  worker_->ClearTask();
  worker_ = nullptr;
  // release model
  if (packed_dnn_handle_) {
    if (0 != hbDNNRelease(packed_dnn_handle_)) {
      LOGE << "hbDNNRelease failed";
    }
  }
  LOGD << "leave InferenceModule::Reset";
}

#ifdef DUMP_IMG_FOR_TEST
static int DumpToFile2Plane(const char *filename, uint8_t *srcBuf,
                            uint8_t *srcBuf1, int size, int size1) {
  FILE *yuvFd = NULL;
  {
    static std::map<std::string, FILE *> _fhs_map;
    static std::mutex _fhs_map_mutex_;
    std::lock_guard<std::mutex> _fhs_map_guard(_fhs_map_mutex_);
    if (_fhs_map.end() == _fhs_map.find(filename)) {
      _fhs_map[filename] = fopen(filename, "w+");
    }
    yuvFd = _fhs_map[filename];
  }

  if (yuvFd == NULL) {
    std::cout << "open file: " << filename << " failed" << std::endl;
    return -1;
  }

  char *buffer = reinterpret_cast<char *>(malloc(size + size1));
  if (buffer == NULL) {
    std::cout << "malloc failed " << std::endl;
    fclose(yuvFd);
    return -1;
  }

  memcpy(buffer, srcBuf, size);
  memcpy(buffer + size, srcBuf1, size1);

  fwrite(buffer, 1, size + size1, yuvFd);
  fflush(yuvFd);

  // if (yuvFd) fclose(yuvFd);
  if (buffer) free(buffer);

  return 0;
}

static void DrawImageByNV12(int width, int height, uint8_t *buff,
                            std::vector<spTarget> draw_target,
                            const char *save_path) {
  // 1. cvt nv12 to bgr mat
  cv::Mat yuvNV12;
  cv::Mat bgr24;

  int yuvNV12_size = width * height * 3 / 2;
  yuvNV12.create(height * 3 / 2, width, CV_8UC1);
  memcpy(yuvNV12.data, buff, yuvNV12_size);
  cv::cvtColor(yuvNV12, bgr24, cv::COLOR_YUV2BGR_NV12);

  static cv::Scalar colors[] = {
      cv::Scalar(255, 0, 0),     // red
      cv::Scalar(255, 165, 0),   // orange
      cv::Scalar(255, 255, 0),   // yellow
      cv::Scalar(0, 255, 0),     // green
      cv::Scalar(0, 0, 255),     // blue
      cv::Scalar(75, 0, 130),    // indigo
      cv::Scalar(238, 130, 238)  // violet
  };

  // 2. draw
  for (auto spTar : draw_target) {
    // if (spTar->type_ == "BBox") {  // 目标检测框
    auto &vBbox = spTar->boxs_;
    for (auto bbox : vBbox) {
      auto &color = colors[bbox->id_ % 7];
      cv::rectangle(bgr24, cv::Point(bbox->x1_, bbox->y1_),
                    cv::Point(bbox->x2_, bbox->y2_), color);
      std::stringstream text_ss;
      std::string class_name = bbox->specific_type_;
      text_ss << bbox->id_ << " " << class_name << ":" << std::fixed
              << std::setprecision(4) << bbox->score_;
      cv::putText(bgr24, text_ss.str(),
                  cv::Point(bbox->x1_, (bbox->y1_ > 5) ? (bbox->y1_ - 5) : 0),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }
    //} else if (spTar->type_ == "Attribute") {  // 分类检测
    auto &vCls = spTar->attrs_;
    for (uint32_t i = 0; i < vCls.size(); ++i) {
      auto &c = vCls[i];
      auto &color = colors[c->value_ % 7];
      std::stringstream text_ss;
      text_ss << c->value_ << ":" << std::fixed << std::setprecision(5)
              << c->score_;
      cv::putText(bgr24, text_ss.str(), cv::Point(5, 20 + 10 * i),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }
    //} else {
    //  continue;
    //}
  }

  // 3.imwrite
  cv::imwrite(save_path, bgr24);
}
#endif

// Find a suitable pym level for padding.
static int FindPymLevelForPadding(spPyramidFrame pym_img, int width,
                                  int height) {
  if (pym_img->src_info_.width > 0 && pym_img->src_info_.height > 0 &&
      width >= pym_img->src_info_.width &&
      height >= pym_img->src_info_.height) {
    return -1;
  }

  for (int i = 0; i < 5; ++i) {
    auto &tmp_width = pym_img->bl_ds_[i].width;
    auto &tmp_height = pym_img->bl_ds_[i].height;
    if (tmp_width > 0 && tmp_height > 0) {
      if ((width >= tmp_width) && (height >= tmp_height)) {
        return i;
      }
    }
  }
  return -2;
}

void img_trans(cv::Mat& img, cv::Mat& img_crop, cv::Size resize_dims, int crop_w, int crop_h, int fW, int fH) {
    cv::Mat img_resize;
    cv::resize(img, img_resize, resize_dims,0,0,INTER_NEAREST);
    img_resize(cv::Range(crop_h, crop_h + fH), cv::Range(crop_w, crop_w + fW)).copyTo(img_crop); // 96:352, 0:704
}


void normalize_img(cv::Mat& img, cv::Mat& img_blob) {
    const float b_mean = 0.485;
    const float g_mean = 0.456;
    const float r_mean = 0.406;
    const float b_std = 4.3668122;
    const float g_std = 4.4642857;
    const float r_std = 4.4444444;
    cv::Mat input;
    cv::Mat input_f;
    cv::Mat output = cv::Mat::zeros(INPUTHEIGHT, INPUTWIDTH, CV_32FC3);
    img.copyTo(input);

    input.convertTo(input_f, CV_32FC3, 1.0/255, 0);
    float* ptr = input_f.ptr<float>();

    for (int i = 0; i < img.rows; i++) {
        cv::Vec3f* pOut = output.ptr<cv::Vec3f>(i);
        cv::Vec3f* pIn = input_f.ptr<cv::Vec3f>(i);
        for (int j = 0; j < img.cols; ++j) {
            pOut[j][0] = (pIn[j][0] - b_mean) * b_std;
            pOut[j][1] = (pIn[j][1] - g_mean) * g_std;
            pOut[j][2] = (pIn[j][2] - r_mean) * r_std;
        }
    }
    float* ptr2 = output.ptr<float>();

    img_blob = output;
    std::cout<<img_blob.ptr<float>(0)[0]<<std::endl;
}
void preprocess(std::vector<cv::Mat>& img_lists, std::vector<cv::Mat>& img_norm_lists) {
    /*
    int H0 = 540; int W0 = 960;
    int Hother = 640; int Wother = 1024;
    int fH = INPUTHEIGHT; int fW = INPUTWIDTH;
    // part for 800w camera
    float resize0 = fmax(fH/(1.0 * H0), fW/(1.0*W0));
    cv::Size resize_dims_0;
    resize_dims_0.width = (int)(W0 * resize0);
    resize_dims_0.height = (int)(H0 * resize0);
    int crop_h_0 = fmax(0, (int)((1 - (0.0 + 0.22) / 2) * resize_dims_0.height) - fH);
    int crop_w_0 = (int)(fmax(0,resize_dims_0.width - fW) / (2.0));

    // part for 200w camera
    float resize = fmax(fH / (1.0 * Hother), fW / (1.0 * Wother)); // 256/
    cv::Size resize_dims_other;
    resize_dims_other.width = (int)(Wother * resize);
    resize_dims_other.height = (int)(Hother * resize);
    int crop_h_other = fmax(0, (int)((1 - (0.0 + 0.22) / 2) * resize_dims_other.height) - fH);
    int crop_w_other = (int)(fmax(0, resize_dims_other.width - fW) / (2.0));
    */
    /*
    cv::Mat img_tmp0,img_norm0;
    img_trans(img_lists[0], img_tmp0, resize_dims_0, crop_w_0, crop_h_0, fW, fH);
    normalize_img(img_tmp0,img_norm0);
    img_norm_lists.emplace_back(img_norm0);
*/
    cv::Mat img_0,img_norm_0;
    //img_trans(img_lists[0], img_0, resize_dims_0, crop_w_0, crop_h_0, fW, fH);
    img_lists[0](cv::Range(96,352),cv::Range(0,704)).copyTo(img_0);
    normalize_img(img_0, img_norm_0);
    img_norm_lists.emplace_back(img_norm_0);

    cv::Mat img_1, img_norm_1;
    img_lists[1](cv::Range(96,352),cv::Range(0,704)).copyTo(img_1);
    normalize_img(img_1,img_norm_1);
    img_norm_lists.emplace_back(img_norm_1);

    cv::Mat img_2,img_norm_2;
    //img_trans(img_lists[2], img_2,resize_dims_0, crop_w_0, crop_h_0, fW, fH);
    img_lists[2](cv::Range(96,352),cv::Range(0,704)).copyTo(img_2);    
    normalize_img(img_2, img_norm_2);
    img_norm_lists.emplace_back(img_norm_2);

    cv::Mat img_3,img_norm_3;
    //img_trans(img_lists[3], img_3, resize_dims_0, crop_w_0, crop_h_0, fW, fH);
    img_lists[3](cv::Range(96,352),cv::Range(0,704)).copyTo(img_3);
    normalize_img(img_3, img_norm_3);
    img_norm_lists.emplace_back(img_norm_3); 

    cv::Mat img_4, img_norm_4;
    img_lists[4](cv::Range(0,256),cv::Range(0,704)).copyTo(img_4);
    normalize_img(img_4,img_norm_4);
    img_norm_lists.emplace_back(img_norm_4);

    cv::Mat img_5,img_norm_5;
//    img_trans(img_lists[5], img_5, resize_dims_0, crop_w_0, crop_h_0, fW, fH);
    img_lists[5](cv::Range(96,352),cv::Range(0,704)).copyTo(img_5);
    normalize_img(img_5, img_norm_5);
    img_norm_lists.emplace_back(img_norm_5); 


 /*   for (int i = 0; i < img_lists.size(); i++) {
        cv::Mat img_tmp, img_norm;
        img_trans(img_lists[i], img_tmp, resize_dims_other, crop_w_other, crop_h_other, fW, fH);

        normalize_img(img_tmp, img_norm);
        img_norm_lists.emplace_back(img_norm);
    }
*/
}


int JM_prepare_tensor(hbDNNTensor* input_tensor,
    hbDNNTensor* output_tensor,
    hbDNNHandle_t dnn_handle) {
    int input_count = 0;
    int output_count = 0;
    hbDNNGetInputCount(&input_count, dnn_handle);
    hbDNNGetOutputCount(&output_count, dnn_handle);

    hbDNNTensor* input = input_tensor;
    int input_memSize;
    // prepare input tensor for HB_DNN_IMG_TYPE_NV12.
    for (int i = 0; i < input_count; i++) {
        HB_CHECK_SUCCESS(
            hbDNNGetInputTensorProperties(&input[i].properties, dnn_handle, i),
            "hbDNNGetInputTensorProperties failed");
        input_memSize = input[i].properties.alignedByteSize;
        HB_CHECK_SUCCESS(hbSysAllocCachedMem(&input[i].sysMem[0], input_memSize),
            "hbSysAllocCachedMem failed");
        input[i].properties.alignedShape = input[i].properties.validShape;

    }

    hbDNNTensor* output = output_tensor;
    for (int i = 0; i < output_count; i++) {
        HB_CHECK_SUCCESS(
            hbDNNGetOutputTensorProperties(&output[i].properties, dnn_handle, i),
            "hbDNNGetOutputTensorProperties failed");
        int output_memSize = output[i].properties.alignedByteSize;
        HB_CHECK_SUCCESS(hbSysAllocCachedMem(&output[i].sysMem[0], output_memSize),
            "hbSysAllocCachedMem failed");

    }
    return 0;
}

/*
  auto &pym_res0 = res[0]->pym_image_->src_info_;
  int res0width = pym_res0.width;
  int res0height = pym_res0.height;
  uint8_t *res0y_data = reinterpret_cast<uint8_t *>(pym_res0.y_vaddr);
  uint8_t *res0c_data = reinterpret_cast<uint8_t *>(pym_res0.c_vaddr);
  std::cout<<"new start"<<std::endl;
  uint8_t * yuv_all = new uint8_t[res0height*res0width*3/2];
  std::cout<<"new success"<<std::endl;
  memcpy(yuv_all,res0y_data,res0width*res0height);
  memcpy(yuv_all+res0height*res0width,res0c_data,res0width*res0height*0.5);
  cv::Mat mat_src;
  int mat_src_size = res0height*res0width*3/2;
  mat_src.create(res0height*3/2,res0width,CV_8UC1);
  memcpy(mat_src.data,yuv_all,mat_src_size);
  cv::Mat mat_dest;

  cv::cvtColor(mat_src,mat_dest,cv::COLOR_YUV2BGR_NV12);
  char fntmp[1024] = {0};

  sprintf(fntmp, "chnl0-%02d-Infer_result.jpg", rand());
  cv::imwrite(fntmp,mat_dest);
  delete[] yuv_all;
*/
#define GET_FILE_NUM	9
int InferenceModule::DoProcess(std::vector<spVioMessage> pyramid_messages,
                               hobot::Workflow *workflow, Module *from,
                               int output_slot, hobot::spRunContext context) {

  //std::cout<<"start process"<<std::endl;
  clock_t Start, End;
  
  if (RTE_API_CreateSendAddr(CLIENT_IP_ADDR) != 0) {
		log_color_err("RTE_API_CreateSendAddr fail");
		return -1;
	}

	TMessageData tPubData = {0};
  TCommBuff tGetFile[GET_FILE_NUM] = {0};


  ModuleProfile profile("InferenceModule");

  ModuleProfile Preprocess_profile("preprocess profile");


 Start = clock();

/*
  auto out_message = std::make_shared<J5FrameMessage>();
  out_message->channel_id_ = pyramid_message->channel_;
  out_message->time_stamp_ = pyramid_message->time_stamp_;
  out_message->frame_id_ = pyramid_message->frame_id_;
*/
  std::vector<hbDNNTensor> input_tensors; // vector<Tensor>

  std::vector<hbDNNTensor> output_tensors;

//1. prepare tensor
  int input_count = 0;
  int output_count = 0;
  HB_CHECK_SUCCESS(hbDNNGetInputCount(&input_count, dnn_handle_), //获取指向模型输入张量的个数
            "hbDNNGetInputCount failed");
  HB_CHECK_SUCCESS(hbDNNGetOutputCount(&output_count, dnn_handle_),// 获取输出张量个数
            "hbDNNGetOutputCount failed");
  
  input_tensors.resize(input_count);
  output_tensors.resize(output_count);
  JM_prepare_tensor(input_tensors.data(), output_tensors.data(), dnn_handle_);
//  std::cout<<"prepare tensor !"<<std::endl;



//2. Read tensor 
  std::vector<cv::Mat> rgb_input_list;
  std::vector<cv::Mat> img_norm_list; //data after normalization
  //int32_t imgsize = 3 * INPUTHEIGHT * INPUTWIDTH;
  hbDNNTensor& imgs_ptr = input_tensors[0];
  hbDNNTensorProperties& Properties_imgs_ptr = imgs_ptr.properties;

  auto imgs_data_for_write_tensor = reinterpret_cast<float *>(imgs_ptr.sysMem[0].virAddr);
//  int batch = 6;
//  int input_h = 256; // h值
//  int input_w = 704; // w值

  auto batch_size = Properties_imgs_ptr.alignedByteSize / 6;
  int pym_id = pyramid_messages[0]->pym_image_->frame_id_*0.1333333;
//960 540 
/*
  auto &pym_img_0 = pyramid_messages[0]->pym_image_->bl_ds_[1];

  int width0 = pym_img_0.width;
  int height0 = pym_img_0.height;

  int imgsize0 = width0*height0;
  uint8_t *y_data_0 = reinterpret_cast<uint8_t *>(pym_img_0.y_vaddr);
  cv::Mat matsrc1(1.5*height0 ,width0,  CV_8UC1, y_data_0);
  cv::cvtColor(matsrc1,matsrc1,cv::COLOR_YUV2BGR_NV12); //model input is rgb
  //char fntmp1[1024] = {0};
  cv::cvtColor(matsrc1, matsrc1, cv::COLOR_RGB2BGR); 
  //sprintf(fntmp1, "chnl0-%d-Infer_result.jpg", pym_id);
  //cv::imwrite(fntmp1,mat_dest0);
  rgb_input_list.emplace_back(matsrc1);

*/

  //Ipc_sendData(0,pym_id,y_data_0,width0*height0*1.5,width0,height0,0,FORMAT_NV12);
  //delete[] yuv_all_0;


  //Ipc_sendData(0,pym_id,y_data_1,imgsize1*1.5,width1,height1,0,FORMAT_NV12);
 
  //log_color_info("<%d> --- %d", 0, imgsize1*1.5);

  //cv::Mat matsrc1(1.5*height1 ,width1,  CV_8UC1, y_data_1);


//1024 640  200w -> 1920 1080
/*
  for(int i=0;i<pyramid_messages.size();i++){
    auto &pym_img = pyramid_messages[i]->pym_image_->bl_ds_[0];
    int width_for = pym_img.width;
    int height_for = pym_img.height;

  //  std::cout<<width<<" "<<height<<std::endl;
    uint8_t *y_data = reinterpret_cast<uint8_t *>(pym_img.y_vaddr);

    cv::Mat mat_src(1.5*height_for ,width_for,  CV_8UC1, y_data);
    cv::cvtColor(mat_src,mat_src,cv::COLOR_YUV2BGR_NV12); //model input is rgb
    
  //  char fntmp2[1024] = {0};
    cv::cvtColor(mat_src, mat_src, cv::COLOR_RGB2BGR); 
    //sprintf(fntmp2, "chnl%d-%d-Infer_result.jpg",i, pym_id);
    //cv::imwrite(fntmp2,mat_dest);

    rgb_input_list.emplace_back(mat_src);


  }
*/
    auto &pym_img_lf = pyramid_messages[1]->pym_image_->roi_ds_[0];
    int width_for = pym_img_lf.width;
    int height_for = pym_img_lf.height;
    int imgsize_for = width_for*height_for;
    uint8_t *y_data_lf = reinterpret_cast<uint8_t *>(pym_img_lf.y_vaddr);
    cv::Mat mat_src_lf(1.5*height_for ,width_for,  CV_8UC1, y_data_lf);
    cv::cvtColor(mat_src_lf,mat_src_lf,cv::COLOR_YUV2BGR_NV12); //model input is rgb
    cv::cvtColor(mat_src_lf, mat_src_lf, cv::COLOR_RGB2BGR); 
    rgb_input_list.emplace_back(mat_src_lf);


    auto &pym_img_f = pyramid_messages[4]->pym_image_->roi_ds_[0];
    uint8_t *y_data_f = reinterpret_cast<uint8_t *>(pym_img_f.y_vaddr);
    cv::Mat mat_src_f(1.5*height_for ,width_for,  CV_8UC1, y_data_f);
    cv::cvtColor(mat_src_f,mat_src_f,cv::COLOR_YUV2BGR_NV12); //model input is rgb
    cv::cvtColor(mat_src_f, mat_src_f, cv::COLOR_RGB2BGR); 
    rgb_input_list.emplace_back(mat_src_f);


    auto &pym_img_rf = pyramid_messages[0]->pym_image_->roi_ds_[0];
    uint8_t *y_data_rf = reinterpret_cast<uint8_t *>(pym_img_rf.y_vaddr);
    cv::Mat mat_src_rf(1.5*height_for ,width_for,  CV_8UC1, y_data_rf);
    cv::cvtColor(mat_src_rf,mat_src_rf,cv::COLOR_YUV2BGR_NV12); //model input is rgb
    cv::cvtColor(mat_src_rf, mat_src_rf, cv::COLOR_RGB2BGR); 
    rgb_input_list.emplace_back(mat_src_rf);

    auto &pym_img_lb = pyramid_messages[2]->pym_image_->roi_ds_[0];
    uint8_t *y_data_lb = reinterpret_cast<uint8_t *>(pym_img_lb.y_vaddr);
    cv::Mat mat_src_lb(1.5*height_for ,width_for,  CV_8UC1, y_data_lb);
    cv::cvtColor(mat_src_lb,mat_src_lb,cv::COLOR_YUV2BGR_NV12); //model input is rgb
    cv::cvtColor(mat_src_lb, mat_src_lb, cv::COLOR_RGB2BGR); 
    rgb_input_list.emplace_back(mat_src_lb);

    auto &pym_img_b = pyramid_messages[5]->pym_image_->roi_ds_[0];
    uint8_t *y_data_b = reinterpret_cast<uint8_t *>(pym_img_b.y_vaddr);
    cv::Mat mat_src_b(1.5*height_for ,width_for,  CV_8UC1, y_data_b);
    cv::cvtColor(mat_src_b,mat_src_b,cv::COLOR_YUV2BGR_NV12); //model input is rgb
    cv::cvtColor(mat_src_b, mat_src_b, cv::COLOR_RGB2BGR); 
    rgb_input_list.emplace_back(mat_src_b);

    auto &pym_img_rb = pyramid_messages[3]->pym_image_->roi_ds_[0];
    uint8_t *y_data_rb = reinterpret_cast<uint8_t *>(pym_img_rb.y_vaddr);
    cv::Mat mat_src_rb(1.5*height_for ,width_for,  CV_8UC1, y_data_rb);
    cv::cvtColor(mat_src_rb,mat_src_rb,cv::COLOR_YUV2BGR_NV12); //model input is rgb
    cv::cvtColor(mat_src_rb, mat_src_rb, cv::COLOR_RGB2BGR); 
    rgb_input_list.emplace_back(mat_src_rb);

   End = clock();
  double color_cvt_time = (double)(End - Start) / CLOCKS_PER_SEC;
  std::cout << "color convert time:" << color_cvt_time * 1000 << "ms" << std::endl;


//
  



  Start = clock();
  // 480 270
/*  auto &pym_img_1 = pyramid_messages[0]->pym_image_->bl_ds_[2];
  int width1 = pym_img_1.width;
  int height1 = pym_img_1.height;
  int imgsize1 = width1*height1;
  uint8_t *y_data_1 = reinterpret_cast<uint8_t *>(pym_img_1.y_vaddr);

	tPubData.s32PubNum = GET_FILE_NUM; 
  tPubData.tPubInfo[0].s32Type = 0;
  tPubData.tPubInfo[0].s32Size = imgsize1*1.5;
  tPubData.ptPubData[0] = y_data_1;
*/



/**/
/*
  auto &pym_img_1 = pyramid_messages[0]->pym_image_->roi_ds_[0];
  int width1 = pym_img_1.width;
  int height1 = pym_img_1.height;
  int imgsize1 = width1*height1;
  uint8_t *y_data_1 = reinterpret_cast<uint8_t *>(pym_img_1.y_vaddr);

	tPubData.s32PubNum = GET_FILE_NUM; 
  tPubData.tPubInfo[0].s32Type = 0;
  tPubData.tPubInfo[0].s32Size = imgsize1*1.5;
  tPubData.ptPubData[0] = y_data_1;

//512 320
  auto &pym_img_2 = pyramid_messages[1]->pym_image_->roi_ds_[0];
  int width = pym_img_2.width;
  int height = pym_img_2.height;

  uint8_t *y_data_2 = reinterpret_cast<uint8_t *>(pym_img_2.y_vaddr);

  tPubData.tPubInfo[1].s32Type = 1;
  tPubData.tPubInfo[1].s32Size = width*height*1.5;
  tPubData.ptPubData[1] = y_data_2;
  //cv::Mat matsrc2(1.5*height ,width,  CV_8UC1, y_data_2);


//
  auto &pym_img_3 = pyramid_messages[2]->pym_image_->roi_ds_[0];
  uint8_t *y_data_3 = reinterpret_cast<uint8_t *>(pym_img_3.y_vaddr);

  tPubData.tPubInfo[2].s32Type = 2;
  tPubData.tPubInfo[2].s32Size = width*height*1.5;
  tPubData.ptPubData[2] = y_data_3;
  //cv::Mat matsrc3(1.5*height ,width,  CV_8UC1, y_data_3);

  auto &pym_img_4 = pyramid_messages[3]->pym_image_->roi_ds_[0];
  uint8_t *y_data_4 = reinterpret_cast<uint8_t *>(pym_img_4.y_vaddr);



  tPubData.tPubInfo[3].s32Type = 3;
  tPubData.tPubInfo[3].s32Size = width*height*1.5;
  tPubData.ptPubData[3] = y_data_4;
   // cv::Mat matsrc4(1.5*height ,width,  CV_8UC1, y_data_4);

  auto &pym_img_5 = pyramid_messages[4]->pym_image_->roi_ds_[0];
  uint8_t *y_data_5 = reinterpret_cast<uint8_t *>(pym_img_5.y_vaddr);

  tPubData.tPubInfo[4].s32Type = 4;
  tPubData.tPubInfo[4].s32Size = width*height*1.5;
  tPubData.ptPubData[4] = y_data_5;
 // cv::Mat matsrc5(1.5*height ,width,  CV_8UC1, y_data_5);

  auto &pym_img_6 = pyramid_messages[5]->pym_image_->roi_ds_[0];
  uint8_t *y_data_6 = reinterpret_cast<uint8_t *>(pym_img_6.y_vaddr);

  tPubData.tPubInfo[5].s32Type = 5;
  tPubData.tPubInfo[5].s32Size = width*height*1.5;
  tPubData.ptPubData[5] = y_data_6;
 // cv::Mat matsrc6(1.5*height ,width,  CV_8UC1, y_data_6);

  End = clock();
  double com_with_pc_time = (double)(End - Start) / CLOCKS_PER_SEC;
  std::cout << "com with pc time:" << com_with_pc_time * 1000 << "ms" << std::endl;

*/



// test imread
    Start = clock();
    preprocess(rgb_input_list,img_norm_list);
    End = clock();
    double preprocess_time = (double)(End - Start) / CLOCKS_PER_SEC;
    std::cout << "preprcess time:" << preprocess_time * 1000 << "ms" << std::endl;
    
    Start = clock();
    int32_t imgsize = 3*INPUTHEIGHT * INPUTWIDTH;
    std::vector<cv::Mat> img_blob_lists(NUM_CAM);
    cv::Mat blob0, blob1, blob2, blob3, blob4, blob5;
    cv::dnn::blobFromImage(img_norm_list[0], blob0, 1.0, cv::Size(INPUTWIDTH, INPUTHEIGHT), cv::Scalar(0, 0, 0), false, false, CV_32F);
    cv::dnn::blobFromImage(img_norm_list[1], blob1, 1.0, cv::Size(INPUTWIDTH, INPUTHEIGHT), cv::Scalar(0, 0, 0), false, false, CV_32F);
    cv::dnn::blobFromImage(img_norm_list[2], blob2, 1.0, cv::Size(INPUTWIDTH, INPUTHEIGHT), cv::Scalar(0, 0, 0), false, false, CV_32F);
    cv::dnn::blobFromImage(img_norm_list[3], blob3, 1.0, cv::Size(INPUTWIDTH, INPUTHEIGHT), cv::Scalar(0, 0, 0), false, false, CV_32F);
    cv::dnn::blobFromImage(img_norm_list[4], blob4, 1.0, cv::Size(INPUTWIDTH, INPUTHEIGHT), cv::Scalar(0, 0, 0), false, false, CV_32F);
    cv::dnn::blobFromImage(img_norm_list[5], blob5, 1.0, cv::Size(INPUTWIDTH, INPUTHEIGHT), cv::Scalar(0, 0, 0), false, false, CV_32F);

    img_blob_lists[0] = blob0;           
    img_blob_lists[1] = blob1;
    img_blob_lists[2] = blob2;
    img_blob_lists[3] = blob3;
    img_blob_lists[4] = blob4;
    img_blob_lists[5] = blob5;
       End = clock();
    double blobimg_time = (double)(End - Start) / CLOCKS_PER_SEC;
    std::cout << "blobimg time:" << blobimg_time * 1000 << "ms" << std::endl;

    Start = clock();
    for (int32_t i{ 0 }; i < 6;i++) {

        float* img_blob_data = img_blob_lists[i].ptr<float>();
        //std::cout << img_norm_list[i].size() << std::endl;
        //std::cout << "img_norm_data = " << *img_blob_data << std::endl;
        memcpy(imgs_data_for_write_tensor, img_blob_data, 4*imgsize);
        //std::cout << "input_tensor_data = " << *imgs_data_for_write_tensor << std::endl;
        imgs_data_for_write_tensor += batch_size/4;//++
    }
       End = clock();
    double cpimgdata_time = (double)(End - Start) / CLOCKS_PER_SEC;
    std::cout << "cpimgdata time:" << cpimgdata_time * 1000 << "ms" << std::endl;

 /* char fntmp1[1024] = {0};

  sprintf(fntmp1, "chnl0-%02d-Infer_result.jpg", rand());
  cv::imwrite(fntmp1,rgb_input_list[0]);
*/

/*
//  std::cout<<"rgb is ready "<<rgb_input_list.size()<<std::endl;
  // 960 540
  int width0 = 1600; int height0 = 900;
  int width = 1600; int height = 900;
  int fH = INPUTHEIGHT; int fW = INPUTWIDTH;
  float resize0 = fmax(fH/(1.0*height0),fW/(1.0*width0));
  cv::Size resize_dims0;
  resize_dims0.width = (int)(width0*resize0);
  resize_dims0.height = (int)(height0*resize0);
  int crop_h0 = fmax(0, (int)((1 - (0.0 + 0.22) / 2) * resize_dims0.height) - fH);
  int crop_w0 = (int)(fmax(0, resize_dims0.width - fW) / (2.0));
  img_trans(rgb_input_list[0],rgb_input_list[0],resize_dims0,crop_w0,crop_h0,fW,fH);
  
  //1024 640
  for(int i=1;i<rgb_input_list.size();i++){
    float resize = fmax(fH/(1.0*height),fW/(1.0*width));
    cv::Size resize_dims;
    resize_dims.width = (int)(width * resize);
    resize_dims.height = (int)(height * resize);
    int crop_h = fmax(0, (int)((1 - (0.0 + 0.22) / 2) * resize_dims.height) - fH);
    int crop_w = (int)(fmax(0, resize_dims.width - fW) / (2.0));
    img_trans(rgb_input_list[i],rgb_input_list[i],resize_dims,crop_w,crop_h,fW,fH);
  }

//  for(int i=0;i<rgb_input_list.size();i++){
//    char fntmp1[1024] = {0};

//    sprintf(fntmp1, "chnl%d-%02d-Infer_result.jpg",i, rand());
//    cv::imwrite(fntmp1,rgb_input_list[i]);
//  }


  for(int i=0;i<rgb_input_list.size();i++){
    //std::cout<<"normalize "<<i <<std::endl;
    //std::cout<<rgb_input_list[i].size()<<std::endl;
    cv::Mat img_norm;
    cv::Mat img_blob;
    normalize_img(rgb_input_list[i],img_norm);
    img_norm_list.emplace_back(img_norm);
    // to do nhcw->nchw
    cv::dnn::blobFromImage(img_norm, img_blob, 1.0, cv::Size(width,height), cv::Scalar(0, 0, 0),false, false, CV_32F);
    float* img_blob_data = img_blob.ptr<float>();
    //float* img_norm_data = img_norm.ptr<float>();
    memcpy(imgs_data_for_write_tensor,img_blob_data,4*INPUTHEIGHT*INPUTWIDTH*3);
    imgs_data_for_write_tensor += batch_size /4;

  }
  */
//  Preprocess_profile.EndRecord();

 // std::cout<<Preprocess_profile<<std::endl;
/*   End = clock();
  double Sort_filter_time = (double)(End - Start) / CLOCKS_PER_SEC;
  std::cout << "preprocess time:" << Sort_filter_time * 1000 << "ms" << std::endl;
*/

  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNTensor* output = output_tensors.data();

  for (int i = 0; i < input_tensors.size(); i++) {
      HB_CHECK_SUCCESS(hbSysFlushMem(&input_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_CLEAN), "hbsysFlushmem failed"); // 对缓存的BPU内存进行刷新
  }



  ModuleProfile infer_profile_doing("Infer-Doing-Module");
  hbDNNInferCtrlParam infer_ctrl_param;
  //Start = clock();
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param); // 开展推理任务 初始化模型控制参数
  HB_CHECK_SUCCESS(hbDNNInfer(&task_handle,    //任务句柄指针
      &output, // 推理任务的输出
      input_tensors.data(), // 推理任务的输入
      dnn_handle_, // DNN句柄指针
      &infer_ctrl_param), // 控制推理任务的参数
      "hbDNNInfer failed"); // 根据输入的参数执行推理任务

  HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0), "hbDNNWaitTaskDone failed");
  //infer_profile_doing.EndRecord();

  /*End = clock();
  double Sort_filter_time = (double)(End - Start) / CLOCKS_PER_SEC;
  std::cout << "INfer time:" << Sort_filter_time * 1000 << "ms" << std::endl;
  */
  //std::cout<<infer_profile_doing<<std::endl;

  for (int i = 0; i < output_tensors.size(); i++) {
      //Output tensors -> reg,height,dim,rot,hm
    hbSysFlushMem(&output_tensors[i].sysMem[0],
        HB_SYS_MEM_CACHE_INVALIDATE);
  }
  auto output_ptr0 = reinterpret_cast<uint8_t*>(output_tensors[0].sysMem[0].virAddr);
  auto output_ptr1 = reinterpret_cast<uint8_t*>(output_tensors[1].sysMem[0].virAddr);
  auto output_ptr2 = reinterpret_cast<uint8_t*>(output_tensors[2].sysMem[0].virAddr);





  tPubData.tPubInfo[6].s32Type = 6;
  tPubData.tPubInfo[6].s32Size = 24*100*100*4;
	tPubData.ptPubData[6] = output_ptr0;
	log_color_info("<%d> --- %d", 6, 24*100*100*4);

  tPubData.tPubInfo[7].s32Type = 7;
  tPubData.tPubInfo[7].s32Size = 72*100*100*4;
	tPubData.ptPubData[7] = output_ptr1;
	log_color_info("<%d> --- %d", 7, 72*100*100*4);

  tPubData.tPubInfo[8].s32Type = 8;
  tPubData.tPubInfo[8].s32Size = 16*100*100*4;
	tPubData.ptPubData[8] = output_ptr2; 
	log_color_info("<%d> --- %d", 8, 16*100*100*4);

  pub_data(g_u32Topic, &tPubData);  

  // post process run
  static s32 s32SaveFile = 1;
	TMessageData tSubData = {0};

	TShowPoint tShowPoint;
  //ModuleProfile post_process_profile("Postprocess-MODULE");
  //Start = clock();
  allChanDetResult = post_process((float *)output_ptr0, (float *)output_ptr1, (float *)output_ptr2); 
  /*End = clock();
  double Sort_filter_time = (double)(End - Start) / CLOCKS_PER_SEC;
  std::cout << "post time:" << Sort_filter_time * 1000 << "ms" << std::endl;
  */
  //post_process_profile.EndRecord();
  //std::cout<<post_process_profile<<std::endl;
  s32 s32ObjNum = ((s32)allChanDetResult.bev.bev.size() < BEV_POINT_NUM) ? (s32)allChanDetResult.bev.bev.size() : BEV_POINT_NUM;
	memset(&tShowPoint.tBevPoint, 0, sizeof(tShowPoint.tBevPoint));
	
  /*
  for (s32 i=0; i<s32ObjNum; i++) {
			for (s32 j=0; j<4; j++) {
				tShowPoint.tBevPoint[i][j].x = -allChanDetResult.bev.bev[i].point[j].y;
				tShowPoint.tBevPoint[i][j].y = allChanDetResult.bev.bev[i].point[j].x;
				tShowPoint.tBevPoint[i][j].z = 1.5;
        tShowPoint.tBevPoint[i][j].classId = allChanDetResult.bev.bev[i].cls;
        tShowPoint.tBevPoint[i][j].trackId = j;
				// log_trace("<%d> <%d> (%0.1f, %0.1f)", i, j, tShowPoint.tBevPoint[i][j].x, tShowPoint.tBevPoint[i][j].y);
			}
		}
*/

  objtracker.runTrack(s32ObjNum, allChanDetResult, tShowPoint);


//  auto output_ptr3 = reinterpret_cast<float*>(output_tensors[3].sysMem[0].virAddr);
//  auto output_ptr4 = reinterpret_cast<float*>(output_tensors[4].sysMem[0].virAddr);
/*
    FILE* pd = NULL;
    char fntmp[1024] = {0};

    sprintf(fntmp, "fbtest-%d-Infer_result.bin",pym_id);

    //cv::imwrite(fntmp,mat_dest);

    std::string timeline = getCurrentTimeMicrosecondsString();
    std::string filename = "fbtest_"+timeline+".bin";
    std::string str(fntmp);
    //pd = fopen(filename.c_str(), "wb");
    pd = fopen(fntmp,"wb");
    fwrite(output_ptr0, sizeof(float), 2 * 200 * 200, pd);
    fwrite(output_ptr1, sizeof(float), 1 * 200 * 200, pd);
    fwrite(output_ptr2, sizeof(float), 3 * 200 * 200, pd);
    fwrite(output_ptr3, sizeof(float), 2 * 200 * 200, pd);
    fwrite(output_ptr4, sizeof(float), 1 * 200 * 200, pd);
    fclose(pd);
*/

  //------------------test postporcess part ---------------------------
  
  //Ipc_sendData(channel,pym_id,data,datasize,width,height,pts,FORMAT_BIN);




  HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle),"hbDNNReleaseTask failed");

  for(int i=0;i<input_tensors.size();i++) {
        HB_CHECK_SUCCESS(hbSysFreeMem(&(input_tensors[i].sysMem[0])),
            "hbSysFreeMem failed");    
  }
  for (int i = 0; i < output_tensors.size(); i++) { //释放BPU内存。
        HB_CHECK_SUCCESS(hbSysFreeMem(&(output_tensors[i].sysMem[0])),
            "hbSysFreeMem failed");
  }

  //HB_CHECK_SUCCESS(hbDNNRelease(packed_dnn_handle_), "hbDNNRelease failed");// 将 packedDNNHandle 所指向的模型释放。

  // set pyramid_message = nullptr
//------------------------------------------------------------------------




  // get result and process
/*  ModuleProfile infer_profile_post("Infer-Post-Module");
  //Postprocess part
  PostProcess(output_tensor, output_count, out_message);


  int src_image_width = pyramid_message->pym_image_->src_info_.width;
  int src_image_height = pyramid_message->pym_image_->src_info_.height;
  CoordinateTransform(out_message, src_image_width, src_image_height,
                      width_without_pad, height_without_pad);
  LOGD <<" task post process finished";
  pyramid_message = nullptr;
  infer_profile_post.EndRecord();
  LOGD << infer_profile_post;
TShowPoint


  workflow->Return(from, output_slot, out_message, context);
 */
Ipc_sendData(0,pym_id,y_data_lf,imgsize_for*1.5,width_for,height_for,0,FORMAT_NV12);  // 1 ->RF
Ipc_sendData(1,pym_id,y_data_f,imgsize_for*1.5,width_for,height_for,0,FORMAT_NV12); // 2->LF
Ipc_sendData(2,pym_id,y_data_rf,imgsize_for*1.5,width_for,height_for,0,FORMAT_NV12); // 3->LB
Ipc_sendData(3,pym_id,y_data_lb,imgsize_for*1.5,width_for,height_for,0,FORMAT_NV12); // 4->RB
Ipc_sendData(4,pym_id,y_data_b,imgsize_for*1.5,width_for,height_for,0,FORMAT_NV12); //5->front
Ipc_sendData(5,pym_id,y_data_rb,imgsize_for*1.5,width_for,height_for,0,FORMAT_NV12); //6->back
Ipc_sendData(8, 0, (u8 *)&tShowPoint, sizeof(TShowPoint), 0, 0, 0, FORMAT_BIN);

  return 0;
}

int InferenceModule::get_hwc_index(int32_t layout, int *h_idx, int *w_idx,
                                   int *c_idx) {
  if (layout == HB_DNN_LAYOUT_NHWC) {
    *h_idx = 1;
    *w_idx = 2;
    *c_idx = 3;
  } else {
    *c_idx = 1;
    *h_idx = 2;
    *w_idx = 3;
  }
  return 0;
}

int InferenceModule::padding_nv12_image(uint8_t **pOutputImg, void *yuv_data,
                                        void *uv_data, int image_height,
                                        int image_width) {
  int output_img_size = 0;
  int first_stride = 0;
  int second_stride = 0;
  int output_img_width = 0;
  int output_img_height = 0;

  int y_size = image_height * image_width;
  int uv_size = y_size >> 1;

  const uint8_t *input_nv12_data[3] = {reinterpret_cast<uint8_t *>(yuv_data),
                                       reinterpret_cast<uint8_t *>(uv_data),
                                       nullptr};
  const int input_nv12_size[3] = {y_size, uv_size, 0};
  auto ret = HobotXStreamCropYuvImageWithPaddingBlack(
      input_nv12_data, input_nv12_size, image_width, image_height, image_width,
      image_width, IMAGE_TOOLS_RAW_YUV_NV12, 0, 0, model_intput_width_ - 1,
      model_intput_height_ - 1, pOutputImg, &output_img_size, &output_img_width,
      &output_img_height, &first_stride, &second_stride);

  if (ret < 0) {
    LOGE << "fail to crop image";
    free(*pOutputImg);
    return -1;
  }
  HOBOT_CHECK_EQ(output_img_height, model_intput_height_)
      << "padding error, output_img_height not eq model_input_height";
  HOBOT_CHECK_EQ(output_img_width, model_intput_width_)
      << "padding error, output_img_width not eq model_intput_width";
  return 0;
}

int InferenceModule::prepare_input_tensor(hbDNNTensor *tensors,
                                          int intput_count) {
  for (int i = 0; i < intput_count; ++i) {
    auto tensor = tensors + i;
    tensor->properties = input_properties_;
    auto &tensor_property = tensor->properties;
    int32_t image_data_type = tensor_property.tensorType;
    int32_t layout = tensor_property.tensorLayout;
    int h_idx, w_idx, c_idx;
    get_hwc_index(layout, &h_idx, &w_idx, &c_idx);
    int channel = tensor_property.validShape.dimensionSize[c_idx];
    int height = tensor_property.validShape.dimensionSize[h_idx];
    int width = tensor_property.validShape.dimensionSize[w_idx];
    int stride = tensor_property.alignedShape.dimensionSize[w_idx];
    LOGD << "input[" << i + 1 << "/" << intput_count << "], layout[" << layout
         << "] = " << channel << "x" << width << "x" << height
         << ", stride=" << stride;
    if (image_data_type == HB_DNN_IMG_TYPE_Y) {
      hbSysAllocCachedMem(&tensor->sysMem[0], height * stride);
    } else if (image_data_type == HB_DNN_IMG_TYPE_YUV444 ||
               image_data_type == HB_DNN_IMG_TYPE_BGR ||
               image_data_type == HB_DNN_IMG_TYPE_RGB) {
      hbSysAllocCachedMem(&tensor->sysMem[0], height * width * 3);
    } else if (image_data_type == HB_DNN_IMG_TYPE_NV12) {
      int y_length = height * stride;
      int uv_length = height / 2 * stride;
      hbSysAllocCachedMem(&tensor->sysMem[0], y_length + uv_length);
    } else if (image_data_type == HB_DNN_IMG_TYPE_NV12_SEPARATE) {
      int y_length = height * stride;
      int uv_length = height / 2 * stride;
      hbSysAllocCachedMem(&tensor->sysMem[0], y_length);
      hbSysAllocCachedMem(&tensor->sysMem[1], uv_length);
    } else if (image_data_type == HB_DNN_TENSOR_TYPE_F32 ||
               image_data_type == HB_DNN_TENSOR_TYPE_S32 ||
               image_data_type == HB_DNN_TENSOR_TYPE_U32) {
      hbSysAllocCachedMem(&tensor->sysMem[0], height * width * 4);
    } else if (image_data_type == HB_DNN_TENSOR_TYPE_U8 ||
               image_data_type == HB_DNN_TENSOR_TYPE_S8) {
      hbSysAllocCachedMem(&tensor->sysMem[0], height * width);
    }
  }
  return 0;
}

void InferenceModule::prepare_image_data(void *image_data,
                                         hbDNNTensor *tensor) {
  auto &tensor_property = tensor->properties;
  int32_t image_data_type = tensor_property.tensorType;
  int32_t layout = tensor_property.tensorLayout;
  int h_idx, w_idx, c_idx;
  get_hwc_index(layout, &h_idx, &w_idx, &c_idx);
  int image_height = tensor_property.validShape.dimensionSize[h_idx];
  int image_width = tensor_property.validShape.dimensionSize[w_idx];
  int stride = tensor_property.alignedShape.dimensionSize[w_idx];
  LOGD << "layout = " << image_width << ", " << image_height << ", " << stride;
  if (image_data_type == HB_DNN_IMG_TYPE_Y) {
    uint8_t *data = reinterpret_cast<uint8_t *>(image_data);
    uint8_t *data0 = reinterpret_cast<uint8_t *>(tensor->sysMem[0].virAddr);
    for (int h = 0; h < image_height; ++h) {
      auto *raw = data0 + h * stride;
      for (int w = 0; w < image_width; ++w) {
        *raw++ = *data++;
      }
    }
    hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
  } else if (image_data_type == HB_DNN_IMG_TYPE_YUV444 ||
             image_data_type == HB_DNN_IMG_TYPE_BGR ||
             image_data_type == HB_DNN_IMG_TYPE_RGB) {
    if (layout == HB_DNN_LAYOUT_NHWC) {
      uint8_t *data = reinterpret_cast<uint8_t *>(image_data);
      int image_length = image_height * stride * 3;
      void *data0 = tensor->sysMem[0].virAddr;
      memcpy(data0, data, image_length);
    } else {
      int channel_size = image_height * image_width;
      uint8_t *mem = reinterpret_cast<uint8_t *>(tensor->sysMem[0].virAddr);
      nhwc_to_nchw(mem, mem + channel_size, mem + channel_size * 2,
                   reinterpret_cast<uint8_t *>(image_data), image_height,
                   image_width);
    }
    hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
  } else if (image_data_type == HB_DNN_IMG_TYPE_NV12) {
    auto *img_data = reinterpret_cast<uint8_t *>(image_data);
    auto *tensor_data = reinterpret_cast<uint8_t *>(tensor->sysMem[0].virAddr);

    // Copy data
    for (int h = 0; h < image_height * 3 / 2; ++h) {
      memcpy(tensor_data, img_data, image_width);
      tensor_data += stride;
      img_data += image_width;
    }
    hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
  } else if (image_data_type == HB_DNN_IMG_TYPE_NV12_SEPARATE) {
    uint8_t *data = reinterpret_cast<uint8_t *>(image_data);

    // Copy y data to data0
    auto *y = reinterpret_cast<uint8_t *>(tensor->sysMem[0].virAddr);
    for (int h = 0; h < image_height; ++h) {
      memcpy(y, data, image_width);
      y += stride;
      data += image_width;
    }
    hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
    // Copy c data to data_ext
    uint8_t *c = reinterpret_cast<uint8_t *>(tensor->sysMem[1].virAddr);
    int c_height = image_height / 2;
    for (int i = 0; i < c_height; ++i) {
      memcpy(c, data, image_width);
      c += stride;
      data += image_width;
    }
    hbSysFlushMem(&(tensor->sysMem[1]), HB_SYS_MEM_CACHE_CLEAN);
  }
  return;
}

void InferenceModule::nhwc_to_nchw(uint8_t *out_data0, uint8_t *out_data1,
                                   uint8_t *out_data2, uint8_t *in_data,
                                   int height, int width) {
  for (int hh = 0; hh < height; ++hh) {
    for (int ww = 0; ww < width; ++ww) {
      *out_data0++ = *(in_data++);
      *out_data1++ = *(in_data++);
      *out_data2++ = *(in_data++);
    }
  }
}

void InferenceModule::release_tensor(hbDNNTensor *tensor) {
  switch (tensor->properties.tensorType) {
    case HB_DNN_IMG_TYPE_Y:
    case HB_DNN_IMG_TYPE_RGB:
    case HB_DNN_IMG_TYPE_BGR:
    case HB_DNN_IMG_TYPE_YUV444:
    case HB_DNN_IMG_TYPE_NV12:
      hbSysFreeMem(&(tensor->sysMem[0]));
      break;
    case HB_DNN_IMG_TYPE_NV12_SEPARATE:
      hbSysFreeMem(&(tensor->sysMem[0]));
      hbSysFreeMem(&(tensor->sysMem[1]));
      break;
    default:
      break;
  }
}

int InferenceModule::prepare_nv12_tensor(hbDNNTensor *input_tensor,
                                         void *yuv_data, void *uv_data,
                                         int image_height, int image_width) {
  auto &tensor_property = input_tensor->properties;
  int h_idx, w_idx, c_idx;
  auto layout = tensor_property.tensorLayout;
  get_hwc_index(layout, &h_idx, &w_idx, &c_idx);
  int tmp_height = tensor_property.validShape.dimensionSize[h_idx];
  int stride = tensor_property.alignedShape.dimensionSize[w_idx];
  auto y_length = uint32_t(tmp_height * stride);
  auto data = reinterpret_cast<uint8_t *>(input_tensor->sysMem[0].virAddr);
  auto len = input_tensor->sysMem[0].memSize;
  assert(len == (y_length * 3 / 2));
  memset(data, 0, len);

  // Copy y data to data
  auto y_data = reinterpret_cast<uint8_t *>(yuv_data);
  for (int h = 0; h < image_height; ++h) {
    memcpy(data, y_data, image_width);
    data += stride;
    y_data += image_width;
  }

  // Copy uv data
  auto c_data = reinterpret_cast<uint8_t *>(uv_data);
  int c_height = image_height / 2;
  for (int i = 0; i < c_height; ++i) {
    memcpy(data, c_data, image_width);
    data += stride;
    c_data += image_width;
  }

  // sync to mem
  hbSysFlushMem(&(input_tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
  return 0;
}

int InferenceModule::prepare_output_tensor(hbDNNTensor **output_tensor,
                                           int output_count) {
  hbDNNTensor *output = *output_tensor;
  for (int i = 0; i < output_count; i++) {
    hbDNNTensorProperties &output_properties = output[i].properties;
    HB_CHECK_SUCCESS(
        hbDNNGetOutputTensorProperties(&output_properties, dnn_handle_, i),
        "hbDNN_getOutputTensorProperties failed");

    // get all aligned size
    int out_aligned_size = 1;
    if (output_properties.tensorType >= HB_DNN_TENSOR_TYPE_F16 &&
        output_properties.tensorType <= HB_DNN_TENSOR_TYPE_U16) {
      out_aligned_size = 2;
    } else if (output_properties.tensorType >= HB_DNN_TENSOR_TYPE_F32 &&
               output_properties.tensorType <= HB_DNN_TENSOR_TYPE_U32) {
      out_aligned_size = 4;
    } else if (output_properties.tensorType >= HB_DNN_TENSOR_TYPE_F64 &&
               output_properties.tensorType <= HB_DNN_TENSOR_TYPE_U64) {
      out_aligned_size = 8;
    }
    for (int j = 0; j < output_properties.alignedShape.numDimensions; j++) {
      out_aligned_size =
          out_aligned_size * output_properties.alignedShape.dimensionSize[j];
    }
    out_aligned_size = ((out_aligned_size + (16 - 1)) / 16 * 16);
    hbSysMem &mem = output[i].sysMem[0];
    HB_CHECK_SUCCESS(hbSysAllocCachedMem(&mem, out_aligned_size),
                     "hbSysAllocCachedMem failed");
  }

  return 0;
}

void InferenceModule::yolo5_nms(std::vector<BBox> &input, float iou_threshold,
                                int top_k, std::vector<BBox> &result,
                                bool suppress) {
  // sort order by score desc
  std::stable_sort(
      input.begin(), input.end(),
      [](const BBox &b1, const BBox &b2) { return b1.score_ > b2.score_; });

  std::vector<bool> skip(input.size(), false);

  // pre-calculate boxes area
  std::vector<float> areas;
  areas.reserve(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    float width = input[i].x2_ - input[i].x1_;
    float height = input[i].y2_ - input[i].y1_;
    areas.push_back(width * height);
  }

  int count = 0;
  for (size_t i = 0; count < top_k && i < skip.size(); i++) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    ++count;

    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      if (suppress == false) {
        if (input[i].id_ != input[j].id_) {
          continue;
        }
      }

      // intersection area
      float xx1 = std::max(input[i].x1_, input[j].x1_);
      float yy1 = std::max(input[i].y1_, input[j].y1_);
      float xx2 = std::min(input[i].x2_, input[j].x2_);
      float yy2 = std::min(input[i].y2_, input[j].y2_);

      if (xx2 > xx1 && yy2 > yy1) {
        float area_intersection = (xx2 - xx1) * (yy2 - yy1);
        float iou_ratio =
            area_intersection / (areas[j] + areas[i] - area_intersection);
        if (iou_ratio > iou_threshold) {
          skip[j] = true;
        }
      }
    }
    result.push_back(input[i]);
  }
}

void InferenceModule::CoordinateTransform(spJ5FrameMessage out_message,
                                          int src_image_width,
                                          int src_image_height,
                                          int model_input_width,
                                          int model_input_height) {
  float ratio_w = static_cast<float>(src_image_width) / model_input_width;
  float ratio_h = static_cast<float>(src_image_height) / model_input_height;
  for (auto org_target : out_message->targets_) {
    for (auto org_box : org_target->boxs_) {
      org_box->x1_ *= ratio_w;
      org_box->y1_ *= ratio_h;
      org_box->x2_ *= ratio_w;
      org_box->y2_ *= ratio_h;
    }
  }
}

int InferenceModule::PostProcess(hbDNNTensor *tensors, int tensors_count,
                                 spJ5FrameMessage out_message) {
  int h_index, w_index, c_index;
  int ret = get_hwc_index(tensors[0].properties.tensorLayout, &h_index,
                          &w_index, &c_index);
  if (ret != 0 &&
      fcos_config_.class_names.size() !=
          uint32_t(tensors[0].properties.alignedShape.dimensionSize[c_index])) {
    LOGD << "User det_name_list in config file: '" << fcos_config_.det_name_list
         << "', is not compatible with this model. "
         << fcos_config_.class_names.size()
         << " != " << tensors[0].properties.alignedShape.dimensionSize[c_index]
         << ".";
  }
  for (int i = 0; i < tensors_count; i++) {
    hbSysFlushMem(&(tensors[i].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  }

  std::vector<BBox> dets;
  if (community_qat_) {
    CqatGetBboxAndScoresScaleNHWC(tensors, tensors_count, dets);
  } else {
    auto quanti_type = tensors[0].properties.quantiType;
    if (quanti_type == hbDNNQuantiType::SCALE) {
      if (tensors[0].properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
        GetBboxAndScoresScaleNHWC(tensors, tensors_count, dets);
      } else if (tensors[0].properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
        LOGW << "NCHW type is not supported.";
      } else {
        LOGE << "tensor layout error.";
      }
    } else if (quanti_type == hbDNNQuantiType::NONE) {
      if (tensors[0].properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
        GetBboxAndScoresNoneNHWC(tensors, tensors_count, dets);
      } else if (tensors[0].properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
        GetBboxAndScoresNoneNCHW(tensors, tensors_count, dets);
      } else {
        LOGE << "tensor layout error.";
      }
    } else {
      LOGE << "error quanti_type: " << quanti_type;
      return -1;
    }
  }

  std::vector<BBox> result;
  yolo5_nms(dets, 0.6, topk_, result, false);
  for (auto bbx : result) {
    LOGD << "dst box info: " << bbx;
    auto box = std::make_shared<BBox>(bbx);
    auto target = std::make_shared<Target>();
    target->type_ = box->specific_type_;
    target->boxs_.push_back(box);
    out_message->targets_.push_back(target);
  }
  return 0;
}

void InferenceModule::GetBboxAndScoresScaleNHWC(const hbDNNTensor *tensors,
                                                int tensors_count,
                                                std::vector<BBox> &dets) {
  // fcos stride is {8, 16, 32, 64, 128}
  for (int i = 0; i < 5; i++) {
    auto *cls_data = reinterpret_cast<int32_t *>(tensors[i].sysMem[0].virAddr);
    auto *bbox_data =
        reinterpret_cast<int32_t *>(tensors[i + 5].sysMem[0].virAddr);
    auto *ce_data =
        reinterpret_cast<int32_t *>(tensors[i + 10].sysMem[0].virAddr);
    float *cls_scale = tensors[i].properties.scale.scaleData;
    float *bbox_scale = tensors[i + 5].properties.scale.scaleData;
    float *ce_scale = tensors[i + 10].properties.scale.scaleData;

    // 同一个尺度下，tensor[i],tensor[i+5],tensor[i+10]出来的hw都一致，64*64/32*32/...
    auto shape = tensors[i].properties.alignedShape.dimensionSize;
    int tensor_h = shape[1];
    int tensor_w = shape[2];
    int tensor_c = shape[3];
    int32_t bbox_c_stride{
        tensors[i + 5].properties.alignedShape.dimensionSize[3]};
    int32_t ce_c_stride{
        tensors[i + 10].properties.alignedShape.dimensionSize[3]};

    for (int h = 0; h < tensor_h; h++) {
      for (int w = 0; w < tensor_w; w++) {
        // get score
        int ce_offset = (h * tensor_w + w) * ce_c_stride;
        float ce_data_offset =
            1.0 / (1.0 + exp(-ce_data[ce_offset] * ce_scale[0]));

        int cls_offset = (h * tensor_w + w) * tensor_c;
        ScoreId tmp_score = {cls_data[cls_offset] * cls_scale[0], 0};
        for (int cls_c = 1; cls_c < tensor_c; cls_c++) {
          int cls_index = cls_offset + cls_c;
          float score = cls_data[cls_index] * cls_scale[cls_c];
          if (score > tmp_score.score) {
            tmp_score.id = cls_c;
            tmp_score.score = score;
          }
        }
        tmp_score.score = 1.0 / (1.0 + exp(-tmp_score.score));
        tmp_score.score = std::sqrt(tmp_score.score * ce_data_offset);
        if (tmp_score.score <= score_threshold_) continue;

        // get detection box
        BBox detection;
        int index = bbox_c_stride * (h * tensor_w + w);
        auto &strides = fcos_config_.strides;

        detection.x1_ =
            ((w + 0.5) * strides[i] - (bbox_data[index] * bbox_scale[0]));
        detection.y1_ =
            ((h + 0.5) * strides[i] - (bbox_data[index + 1] * bbox_scale[1]));
        detection.x2_ =
            ((w + 0.5) * strides[i] + (bbox_data[index + 2] * bbox_scale[2]));
        detection.y2_ =
            ((h + 0.5) * strides[i] + (bbox_data[index + 3] * bbox_scale[3]));

        detection.score_ = tmp_score.score;
        detection.id_ = tmp_score.id;
        detection.specific_type_ =
            fcos_config_.class_names[detection.id_].c_str();
        dets.push_back(detection);
      }
    }
  }
}

void InferenceModule::GetBboxAndScoresNoneNHWC(const hbDNNTensor *tensors,
                                               int tensors_count,
                                               std::vector<BBox> &dets) {
  // fcos stride is {8, 16, 32, 64, 128}
  for (int i = 0; i < 5; i++) {
    auto *cls_data = reinterpret_cast<float *>(tensors[i].sysMem[0].virAddr);
    auto *bbox_data =
        reinterpret_cast<float *>(tensors[i + 5].sysMem[0].virAddr);
    auto *ce_data =
        reinterpret_cast<float *>(tensors[i + 10].sysMem[0].virAddr);

    // 同一个尺度下，tensor[i],tensor[i+5],tensor[i+10]出来的hw都一致，64*64/32*32/...
    auto shape = tensors[i].properties.alignedShape.dimensionSize;
    int tensor_h = shape[1];
    int tensor_w = shape[2];
    int tensor_c = shape[3];

    for (int h = 0; h < tensor_h; h++) {
      int offset = h * tensor_w;
      for (int w = 0; w < tensor_w; w++) {
        // get score
        int ce_offset = offset + w;
        ce_data[ce_offset] = 1.0 / (1.0 + exp(-ce_data[ce_offset]));

        int cls_offset = ce_offset * tensor_c;
        ScoreId tmp_score = {cls_data[cls_offset], 0};
        for (int cls_c = 1; cls_c < tensor_c; cls_c++) {
          int cls_index = cls_offset + cls_c;
          if (cls_data[cls_index] > tmp_score.score) {
            tmp_score.id = cls_c;
            tmp_score.score = cls_data[cls_index];
          }
        }
        tmp_score.score = 1.0 / (1.0 + exp(-tmp_score.score));
        tmp_score.score = std::sqrt(tmp_score.score * ce_data[ce_offset]);
        if (tmp_score.score <= score_threshold_) continue;

        // get detection box
        BBox detection;
        int index = 4 * (h * tensor_w + w);
        auto &strides = fcos_config_.strides;

        detection.x1_ = ((w + 0.5) * strides[i] - bbox_data[index]);
        detection.y1_ = ((h + 0.5) * strides[i] - bbox_data[index + 1]);
        detection.x2_ = ((w + 0.5) * strides[i] + bbox_data[index + 2]);
        detection.y2_ = ((h + 0.5) * strides[i] + bbox_data[index + 3]);

        detection.score_ = tmp_score.score;
        detection.id_ = tmp_score.id;
        detection.specific_type_ =
            fcos_config_.class_names[detection.id_].c_str();
        dets.push_back(detection);
      }
    }
  }
}

void InferenceModule::GetBboxAndScoresNoneNCHW(const hbDNNTensor *tensors,
                                               int tensors_count,
                                               std::vector<BBox> &dets) {
  for (int i = 0; i < 5; i++) {
    auto *cls_data = reinterpret_cast<float *>(tensors[i].sysMem[0].virAddr);
    auto *bbox_data =
        reinterpret_cast<float *>(tensors[i + 5].sysMem[0].virAddr);
    auto *ce_data =
        reinterpret_cast<float *>(tensors[i + 10].sysMem[0].virAddr);

    // 同一个尺度下，tensor[i],tensor[i+5],tensor[i+10]出来的hw都一致，64*64/32*32/...
    auto shape = tensors[i].properties.alignedShape.dimensionSize;
    int tensor_c = shape[1];
    int tensor_h = shape[2];
    int tensor_w = shape[3];
    int aligned_hw = tensor_h * tensor_w;

    for (int h = 0; h < tensor_h; h++) {
      int offset = h * tensor_w;
      for (int w = 0; w < tensor_w; w++) {
        // get score
        int ce_offset = offset + w;
        ce_data[ce_offset] = 1.0 / (1.0 + exp(-ce_data[ce_offset]));

        ScoreId tmp_score = {cls_data[offset + w], 0};
        for (int cls_c = 1; cls_c < tensor_c; cls_c++) {
          int cls_index = cls_c * aligned_hw + offset + w;
          if (cls_data[cls_index] > tmp_score.score) {
            tmp_score.id = cls_c;
            tmp_score.score = cls_data[cls_index];
          }
        }
        tmp_score.score = 1.0 / (1.0 + exp(-tmp_score.score));
        tmp_score.score = std::sqrt(tmp_score.score * ce_data[ce_offset]);
        if (tmp_score.score <= score_threshold_) continue;

        // get detection box
        auto &strides = fcos_config_.strides;
        BBox detection;
        detection.x1_ = ((w + 0.5) * strides[i] - bbox_data[offset + w]);
        detection.y1_ =
            ((h + 0.5) * strides[i] - bbox_data[1 * aligned_hw + offset + w]);
        detection.x2_ =
            ((w + 0.5) * strides[i] + bbox_data[2 * aligned_hw + offset + w]);
        detection.y2_ =
            ((h + 0.5) * strides[i] + bbox_data[3 * aligned_hw + offset + w]);

        detection.score_ = tmp_score.score;
        detection.id_ = tmp_score.id;
        detection.specific_type_ =
            fcos_config_.class_names[detection.id_].c_str();
        dets.push_back(detection);
      }
    }
  }
}

static inline uint32x4x4_t CalculateIndex(uint32_t idx, float32x4_t a,
                                          float32x4_t b, uint32x4x4_t c) {
  uint32x4_t mask{0};
  mask = vcltq_f32(b, a);
  uint32x4_t vec_idx = {idx, idx + 1, idx + 2, idx + 3};
  uint32x4x4_t res = {{vbslq_u32(mask, vec_idx, c.val[0]), 0, 0, 0}};
  return res;
}

static inline float32x2_t CalculateMax(float32x4_t in) {
  auto pmax = vpmax_f32(vget_high_f32(in), vget_low_f32(in));
  return vpmax_f32(pmax, pmax);
}

static inline uint32_t CalculateVectorIndex(uint32x4x4_t vec_res_idx,
                                            float32x4_t vec_res_value) {
  uint32x4_t res_idx_mask{0};
  uint32x4_t mask_ones = vdupq_n_u32(0xFFFFFFFF);

  auto pmax = CalculateMax(vec_res_value);
  auto mask = vceqq_f32(vec_res_value, vcombine_f32(pmax, pmax));
  res_idx_mask = vandq_u32(vec_res_idx.val[0], mask);
  res_idx_mask = vaddq_u32(res_idx_mask, mask_ones);
  auto pmin =
      vpmin_u32(vget_high_u32(res_idx_mask), vget_low_u32(res_idx_mask));
  pmin = vpmin_u32(pmin, pmin);
  uint32_t res = vget_lane_u32(pmin, 0);
  return (res - 0xFFFFFFFF);
}

static std::pair<float, int> MaxScoreID(int32_t *input, float *scale,
                                        int length) {
  float init_res_value = input[0] * scale[0];
  float32x4_t vec_res_value = vdupq_n_f32(init_res_value);
  uint32x4x4_t vec_res_idx{{0}};
  int i = 0;
  for (; i <= (length - 4); i += 4) {
    int32x4_t vec_input = vld1q_s32(input + i);
    float32x4_t vec_scale = vld1q_f32(scale + i);

    float32x4_t vec_elements = vmulq_f32(vcvtq_f32_s32(vec_input), vec_scale);
    float32x4_t temp_vec_res_value = vmaxq_f32(vec_elements, vec_res_value);
    vec_res_idx =
        CalculateIndex(i, temp_vec_res_value, vec_res_value, vec_res_idx);
    vec_res_value = temp_vec_res_value;
  }

  uint32_t idx = CalculateVectorIndex(vec_res_idx, vec_res_value);
  float res = vget_lane_f32(CalculateMax(vec_res_value), 0);

  // Compute left elements
  for (; i < length; ++i) {
    float score = input[i] * scale[i];
    if (score > res) {
      idx = i;
      res = score;
    }
  }
  std::pair<float, int> result_id_score = {res, idx};
  return result_id_score;
}

void InferenceModule::CqatGetBboxAndScoresScaleNHWC(const hbDNNTensor *tensors,
                                                    int tensors_count,
                                                    std::vector<BBox> &dets) {
  // fcos stride is {8, 16, 32, 64, 128}
  for (int i = 0; i < 5; i++) {
    auto *cls_data = reinterpret_cast<int32_t *>(tensors[i].sysMem[0].virAddr);
    auto *bbox_data =
        reinterpret_cast<int32_t *>(tensors[i + 5].sysMem[0].virAddr);
    auto *ce_data =
        reinterpret_cast<int32_t *>(tensors[i + 10].sysMem[0].virAddr);
    float *cls_scale = tensors[i].properties.scale.scaleData;
    float *bbox_scale = tensors[i + 5].properties.scale.scaleData;
    float *ce_scale = tensors[i + 10].properties.scale.scaleData;

    // 同一个尺度下，tensor[i],tensor[i+5],tensor[i+10]出来的hw都一致，64*64/32*32/...
    auto shape = tensors[i].properties.alignedShape.dimensionSize;
    int tensor_h = shape[1];
    int tensor_w = shape[2];
    int tensor_c = shape[3];
    int32_t bbox_c_stride{
        tensors[i + 5].properties.alignedShape.dimensionSize[3]};
    int32_t ce_c_stride{
        tensors[i + 10].properties.alignedShape.dimensionSize[3]};

    for (int h = 0; h < tensor_h; h++) {
      for (int w = 0; w < tensor_w; w++) {
        // get score
        int ce_offset = (h * tensor_w + w) * ce_c_stride;
        float ce_data_offset =
            1.0 / (1.0 + exp(-ce_data[ce_offset] * ce_scale[0]));
        // argmax + neon
        int cls_offset = (h * tensor_w + w) * tensor_c;
        auto max_score_id =
            MaxScoreID(cls_data + cls_offset, cls_scale, tensor_c);

        // filter
        float cls_data_offset = 1.0 / (1.0 + exp(-max_score_id.first));
        float score = std::sqrt(cls_data_offset * ce_data_offset);
        if (score <= score_threshold_) continue;

        // get detection box
        BBox detection;
        int index = bbox_c_stride * (h * tensor_w + w);
        auto &strides = fcos_config_.strides;

        float xmin = std::max(0.f, bbox_data[index] * bbox_scale[0]);
        float ymin = std::max(0.f, bbox_data[index + 1] * bbox_scale[1]);
        float xmax = std::max(0.f, bbox_data[index + 2] * bbox_scale[2]);
        float ymax = std::max(0.f, bbox_data[index + 3] * bbox_scale[3]);

        detection.x1_ = ((w + 0.5) - xmin) * strides[i];
        detection.y1_ = ((h + 0.5) - ymin) * strides[i];
        detection.x2_ = ((w + 0.5) + xmax) * strides[i];
        detection.y2_ = ((h + 0.5) + ymax) * strides[i];

        detection.score_ = score;
        detection.id_ = max_score_id.second;
        detection.specific_type_ =
            fcos_config_.class_names[detection.id_].c_str();
        dets.push_back(detection);
      }
    }
  }
}


}  // namespace J5Sample
