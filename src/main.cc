
/* Luo Yixin made it! 2023/2/17
   I hate C/C++!
*/
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <queue>
#include <utility>
#include <vector>
#include "dnn/hb_dnn.h"

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <thread>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "post_process.h"
#include "ipc_demo.h"
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>
#include "comm_api.h"
#include <sys/types.h>
#include <ctime>
#include <dirent.h>


#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <iomanip>
#include <ctime>
#include <chrono>

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



using namespace std;
using namespace cv;
#define INPUTHEIGHT 256
#define INPUTWIDTH  704
#define NUM_CAM     6
#define IMAGE_HEIGHT_ORI  1080
#define IMAGE_WIDTH_ORI  1920
#define EMPTY ""
#define PI 3.141592653589793

#define LANE_POINT_NUM	1000
#define BEV_POINT_NUM	100
#define PUB_MESSAGE_NUM_MAX		100
clock_t Start, End;
clock_t Start_all, End_all;

DEFINE_string(model_file, "fastbev_6_3_256_704_xian.bin", "model file path");
DEFINE_string(csv_file_directory, "data/", "csv file path");
DEFINE_string(imageID, "000001", "IMAGE ID:");

enum VLOG_LEVEL {
    EXAMPLE_SYSTEM = 0,
    EXAMPLE_REPORT = 1,
    EXAMPLE_DETAIL = 2,
    EXAMPLE_DEBUG = 3
};


#define HB_CHECK_SUCCESS(value, errmsg)                              \
  do {                                                               \
    /*value can be call of function*/                                \
    auto ret_code = value;                                           \
    if (ret_code != 0) {                                             \
      VLOG(EXAMPLE_SYSTEM) << errmsg << ", error code:" << ret_code; \
      return ret_code;                                               \
    }                                                                \
  } while (0);

std::map<int32_t, int32_t> element_size{ {HB_DNN_IMG_TYPE_Y, 1},
                                        {HB_DNN_IMG_TYPE_NV12, 1},
                                        {HB_DNN_IMG_TYPE_NV12_SEPARATE, 1},
                                        {HB_DNN_IMG_TYPE_YUV444, 1},
                                        {HB_DNN_IMG_TYPE_RGB, 1},
                                        {HB_DNN_IMG_TYPE_BGR, 1},
                                        {HB_DNN_TENSOR_TYPE_S8, 1},
                                        {HB_DNN_TENSOR_TYPE_U8, 1},
                                        {HB_DNN_TENSOR_TYPE_F16, 2},
                                        {HB_DNN_TENSOR_TYPE_S16, 2},
                                        {HB_DNN_TENSOR_TYPE_U16, 2},
                                        {HB_DNN_TENSOR_TYPE_F32, 4},
                                        {HB_DNN_TENSOR_TYPE_S32, 4},
                                        {HB_DNN_TENSOR_TYPE_U32, 4},
                                         {HB_DNN_TENSOR_TYPE_F64, 8},
                                        {HB_DNN_TENSOR_TYPE_S64, 8},
                                        {HB_DNN_TENSOR_TYPE_U64, 8} };


// ��������ͷ
typedef struct {
	s32 s32CmdStart;		// 收到开始命令
	s32 s32CmdDataHead;		// 第一帧数据
	s32 s32CheckEndCount;	// 比较结束编号
	s32 s32CmdEnd;			// 收到结束命令
	s32 s32RecvPlace;		// 0 --- (PUB_MESSAGE_NUM_MAX-1)
	s32 s32RecvSize[PUB_MESSAGE_NUM_MAX]; // 记录已接收数据长度
} TRecordInfo;

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

int JM_prepare_tensor(hbDNNTensor* input_tensor,
    hbDNNTensor* output_tensor,
    hbDNNHandle_t dnn_handle);

void split(std::string& str,
    char sep,
    std::vector<std::string>& tokens,
    int limit = -1);


void img_trans(cv::Mat& img, cv::Mat& img_crop, cv::Size resize_dims, int crop_w, int crop_h, int fW, int fH);

void normalize_img(cv::Mat& img, cv::Mat& img_blob);


void preprocess(std::vector<cv::Mat>& img_lists, std::vector<cv::Mat>& img_norm_lists);

void filter_scores_new(std::vector<float> hm, std::vector<std::pair<float, int>>& hm_pair);

void filter_scores(std::vector<float> topk_scores, std::vector<int> topk_inds, std::vector<float>& topk_scores_new, std::vector<int>& topk_inds_new);

int32_t Read_input_tensor(
    std::vector<string>& input_image_file, std::vector<hbDNNTensor>& input_tensor, std::vector<cv::Mat>& img_lists, std::vector<cv::Mat>& img_ori_lists, std::vector<cv::Mat>& img_norm_lists);



// ������
int main(int argc, char** argv) {
    // Step0: prepare the enviorment�� Time costs:0.34ms
    ipc_set_channel_t camera_init = {9, 0, 1, 2, 3, 4, 5,6,7,8};
    Ipc_init(camera_init);


    Start = clock();
    Start_all = clock();
    gflags::SetUsageMessage(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::cout << gflags::GetArgv() << std::endl;

    // Init logging
    google::InitGoogleLogging("");
    google::SetStderrLogging(0);
    google::SetVLOGLevel("*", 3);
    FLAGS_colorlogtostderr = true;
    FLAGS_minloglevel = google::INFO;
    FLAGS_logtostderr = true;

    hbPackedDNNHandle_t packed_dnn_handle;
    hbDNNHandle_t dnn_handle;
    const char** model_name_list;
    auto modelFileName = FLAGS_model_file.c_str();
    int model_count = 0;
    // prepare data 
    // intrins ���� 
    float c0_intrins[3][3] = { 1.2579e+03, 0.0000e+00, 8.2724e+02, 0.0000e+00, 1.2579e+03, 4.5092e+02, 0.0000e+00, 0.0000e+00, 1.0000e+00 };
    float c1_intrins[3][3] = { 1.2528e+03, 0.0000e+00, 8.2659e+02, 0.0000e+00, 1.2528e+03, 4.6998e+02, 0.0000e+00, 0.0000e+00, 1.0000e+00 };
    float c2_intrins[3][3] = { 1.2567e+03, 0.0000e+00, 8.1779e+02, 0.0000e+00, 1.2567e+03, 4.5195e+02, 0.0000e+00, 0.0000e+00, 1.0000e+00 };
    float c3_intrins[3][3] = { 1.2550e+03, 0.0000e+00, 8.2958e+02, 0.0000e+00, 1.2550e+03, 4.6717e+02, 0.0000e+00, 0.0000e+00, 1.0000e+00 };
    float c4_intrins[3][3] = { 7.9689e+02, 0.0000e+00, 8.5778e+02, 0.0000e+00, 7.9689e+02, 4.7688e+02, 0.0000e+00, 0.0000e+00, 1.0000e+00 };
    float c5_intrins[3][3] = { 1.2500e+03, 0.0000e+00, 8.2538e+02, 0.0000e+00, 1.2500e+03, 4.6255e+02, 0.0000e+00, 0.0000e+00, 1.0000e+00 };
    std::vector<cv::Mat> intrins_list(6);
    int size2_intrins_list[] = { 3,3 };
    intrins_list[0] = cv::Mat(2, size2_intrins_list, CV_32FC1, c0_intrins);
    intrins_list[1] = cv::Mat(2, size2_intrins_list, CV_32FC1, c1_intrins);
    intrins_list[2] = cv::Mat(2, size2_intrins_list, CV_32FC1, c2_intrins);
    intrins_list[3] = cv::Mat(2, size2_intrins_list, CV_32FC1, c3_intrins);
    intrins_list[4] = cv::Mat(2, size2_intrins_list, CV_32FC1, c4_intrins);
    intrins_list[5] = cv::Mat(2, size2_intrins_list, CV_32FC1, c5_intrins);


    // trans ����
    float c0_trans[3] = { 1.5753,  0.5005,  1.5070 };
    float c1_trans[3] = { 1.7220,  0.0048,  1.4949 };
    float c2_trans[3] = { 1.5808, -0.4991,  1.5175 };
    float c3_trans[3] = { 1.0485,  0.4831,  1.5621 };
    float c4_trans[3] = { 0.0552,  0.0108,  1.5679 };
    float c5_trans[3] = { 1.0595, -0.4672,  1.5505 };

    std::vector<cv::Mat> trans_list(6);
    int size1_trans_list[] = { 3 };
    trans_list[0] = cv::Mat(1, size1_trans_list, CV_32FC1, c0_trans);
    trans_list[1] = cv::Mat(1, size1_trans_list, CV_32FC1, c1_trans);
    trans_list[2] = cv::Mat(1, size1_trans_list, CV_32FC1, c2_trans);
    trans_list[3] = cv::Mat(1, size1_trans_list, CV_32FC1, c3_trans);
    trans_list[4] = cv::Mat(1, size1_trans_list, CV_32FC1, c4_trans);
    trans_list[5] = cv::Mat(1, size1_trans_list, CV_32FC1, c5_trans);

    // rots ����
    float c0_rots[3][3] = { 0.8225,  0.0065,  0.5687, -0.5687,  0.0164,  0.8224, -0.0040, -0.9998,  0.0172 };
    float c1_rots[3][3] = { 0.0103,  0.0084,  0.9999, -0.9999,  0.0123,  0.0102, -0.0122, -0.9999,  0.0086 };
    float c2_rots[3][3] = { -0.8440,  0.0165,  0.5361, -0.5361,  0.0036, -0.8441, -0.0158, -0.9999,  0.0058 };
    float c3_rots[3][3] = { 0.9479, -0.0089, -0.3185, 0.3186,  0.0188,  0.9477, -0.0025, -0.9998,  0.0207 };
    float c4_rots[3][3] = { 0.0092, -0.0068, -0.9999, 0.9999,  0.0113,  0.0091, 0.0112, -0.9999,  0.0069 };
    float c5_rots[3][3] = { -0.9237, -0.0026, -0.3830, 0.3830, -0.0114, -0.9237, -0.0020, -0.9999,  0.0116 };

    std::vector<cv::Mat> rots_list(6);
    int size2_rots_list[] = { 3,3 };
    rots_list[0] = cv::Mat(2, size2_rots_list, CV_32FC1, c0_rots);
    rots_list[1] = cv::Mat(2, size2_rots_list, CV_32FC1, c1_rots);
    rots_list[2] = cv::Mat(2, size2_rots_list, CV_32FC1, c2_rots);
    rots_list[3] = cv::Mat(2, size2_rots_list, CV_32FC1, c3_rots);
    rots_list[4] = cv::Mat(2, size2_rots_list, CV_32FC1, c4_rots);
    rots_list[5] = cv::Mat(2, size2_rots_list, CV_32FC1, c5_rots);
    End = clock();
    double preparetime = (double)(End - Start) / CLOCKS_PER_SEC;
    std::cout << "Prepare time:" << preparetime << std::endl;
    std::cout << "Prepare time:" << preparetime * 1000 << "ms" << std::endl; //Time costs:0.34ms

    // Step1: get model handle, Time costs:2.1s
    {
        Start = clock();
        HB_CHECK_SUCCESS( // packed_dnn_handle: Horizon DNN �����ָ����ģ�ͣ�modelFileName ģ���ļ���·���� 1��ģ���ļ��ĸ���
            hbDNNInitializeFromFiles(&packed_dnn_handle, &modelFileName, 1),
            "hbDNNInitializeFromFiles failed"); // ���ļ���ɶ�packedDNNHandle�Ĵ����ͳ�ʼ��

        HB_CHECK_SUCCESS(hbDNNGetModelNameList(//model_name_listģ�������б���model_countģ�����Ƹ�����packed_dnn_handle: horizon dnn�����ָ����ģ��
            &model_name_list, &model_count, packed_dnn_handle),
            "hbDNNGetModelNameList failed");

        HB_CHECK_SUCCESS(// ��packed_dnn_handle��ָ���ģ���б��л�ȡһ��ģ�͵ľ�� 
            hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name_list[0]), // dnn_handle��DNN�����ָ��һ��ģ��
            "hbDNNGetModelHandle failed");
        End = clock();
        double READ_Model_time = (double)(End - Start) / CLOCKS_PER_SEC;
        std::cout << "Read Model time:" << READ_Model_time << std::endl;
        std::cout << "Read Model time:" << READ_Model_time * 1000 << "ms" << std::endl;
    }

    //To prepare input data

    VLOG(EXAMPLE_DEBUG) << "Inference start";

    std::vector<hbDNNTensor> input_tensors; // vector<Tensor>

    std::vector<hbDNNTensor> output_tensors;
    // Step2: prepare input and output tensor, Time cost: 0.527ms
    {
        Start = clock();
        int input_count = 0;
        int output_count = 0;
        std::vector<hbDNNTensor> input_tosave; // vector<Tensor>
        HB_CHECK_SUCCESS(hbDNNGetInputCount(&input_count, dnn_handle), //��ȡָ��ģ�����������ĸ���
            "hbDNNGetInputCount failed");
        HB_CHECK_SUCCESS(hbDNNGetOutputCount(&output_count, dnn_handle),// ��ȡ�����������
            "hbDNNGetOutputCount failed");
        input_tensors.resize(input_count);
        output_tensors.resize(output_count);
        JM_prepare_tensor(input_tensors.data(), output_tensors.data(), dnn_handle);
        End = clock();
        double Prepare_Tensor_time = (double)(End - Start) / CLOCKS_PER_SEC;
        std::cout << "Prepare tensor time:" << Prepare_Tensor_time << std::endl;
        std::cout << "Prepare tensor time:" << Prepare_Tensor_time * 1000 << "ms" << std::endl;

    }

    // Step3: set input data to input tensor, Time cost: 515ms

    Start = clock();
    std::vector<cv::Mat> img_lists(NUM_CAM);
    std::vector<cv::Mat> img_ori_lists(NUM_CAM);
    std::vector<cv::Mat> img_norm_lists(NUM_CAM);
    TCommBuff tGridFile = api_comm_readFileToBuff("grid.bin");
    post_init(tGridFile);
    std::string root_path = "/userdata/2023-05-09/";

    for(int ii =0;ii<100;ii++){
        char ss[10];
        sprintf(ss, "%d", 24*ii);
        std::string file_count_str = ss;
        std::string imgpath0 = root_path + "left_front/" + "chnl_leftfront-"+ss+"-Infer_result.jpg";
        std::string imgpath1 = root_path + "front/" + "chnl_front-"+ss+"-Infer_result.jpg";
        std::string imgpath2 = root_path + "right_front/" + "chnl_rightfront-"+ss+"-Infer_result.jpg";
        std::string imgpath3 = root_path + "left_back/" + "chnl_leftback-"+ss+"-Infer_result.jpg";
        std::string imgpath4 = root_path + "back/" + "chnl_back-"+ss+"-Infer_result.jpg";
        std::string imgpath5 = root_path + "right_back/" + "chnl_rightback-"+ss+"-Infer_result.jpg";


        vector<string> img_file_path;
        img_file_path.emplace_back(imgpath0);
        img_file_path.emplace_back(imgpath1);
        img_file_path.emplace_back(imgpath2);
        img_file_path.emplace_back(imgpath3);
        img_file_path.emplace_back(imgpath4);
        img_file_path.emplace_back(imgpath5);
        
        HB_CHECK_SUCCESS(Read_input_tensor( //��input��д��ͼ������
            img_file_path, input_tensors, img_lists, img_ori_lists, img_norm_lists),
            "read_image_2_tensor_as_nv12 failed");

        // img_lists -> read mat
        VLOG(EXAMPLE_DEBUG) << "read image to tensor as nv12 success";
        End = clock();
        double Read_Tensor_time = (double)(End - Start) / CLOCKS_PER_SEC;
        std::cout << "Read tensor time:" << Read_Tensor_time << std::endl;
        std::cout << "Read tensor time:" << Read_Tensor_time * 1000 << "ms" << std::endl;

        hbDNNTaskHandle_t task_handle = nullptr;
        hbDNNTensor* output = output_tensors.data();

        Start = clock();
            // make sure memory data is flushed to DDR before inference
            for (int i = 0; i < input_tensors.size(); i++) {
                HB_CHECK_SUCCESS(hbSysFlushMem(&input_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_CLEAN), "hbsysFlushmem failed"); // �Ի����BPU�ڴ����ˢ��
            }

            hbDNNInferCtrlParam infer_ctrl_param; // ģ���������Ʋ���

            HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param); // ��չ�������� ��ʼ��ģ�Ϳ��Ʋ���
            std::cout << "BPU core ID: " << infer_ctrl_param.bpuCoreId << " dspCoreId: " << infer_ctrl_param.dspCoreId << std::endl;
            std::cout << "Infer ctrl param intialize" << std::endl;
            HB_CHECK_SUCCESS(hbDNNInfer(&task_handle,    //������ָ��
                &output, // ������������
                input_tensors.data(), // �������������
                dnn_handle, // DNN���ָ��
                &infer_ctrl_param), // ������������Ĳ���
                "hbDNNInfer failed"); // ��������Ĳ���ִ����������
            std::cout << "Infer success" << std::endl;

            // wait task done �ȴ�������ɻ�ʱ
            HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0), "hbDNNWaitTaskDone failed");
            std::cout << " Check task done" << std::endl;
            End = clock();
            double Infer_time = (double)(End - Start) / CLOCKS_PER_SEC;
            std::cout << "Infer time:" << Infer_time << std::endl;
            std::cout << "Infer time:" << Infer_time * 1000 << "ms" << std::endl;


            for (int i = 0; i < output_tensors.size(); i++) {
              //Output tensors -> reg,height,dim,rot,hm
              hbSysFlushMem(&output_tensors[i].sysMem[0],
              HB_SYS_MEM_CACHE_INVALIDATE);
            }


            auto output_ptr0 = reinterpret_cast<uint8_t*>(output_tensors[0].sysMem[0].virAddr);
            auto output_ptr1 = reinterpret_cast<uint8_t*>(output_tensors[1].sysMem[0].virAddr);
            auto output_ptr2 = reinterpret_cast<uint8_t*>(output_tensors[2].sysMem[0].virAddr);

            Start = clock();
        	TShowPoint tShowPoint;
            TDetResult allChanDetResult;
            allChanDetResult = post_process((float *)output_ptr0, (float *)output_ptr1, (float *)output_ptr2); 
            End = clock();
            
/*
            double Post_process_time = (double)(End - Start) / CLOCKS_PER_SEC;
              std::cout << "Post_process_time:" << Post_process_time << std::endl;
              std::cout << "Post_process_time:" << Post_process_time * 1000 << "ms" << std::endl;
*/


            s32 s32ObjNum = ((s32)allChanDetResult.bev.bev.size() < BEV_POINT_NUM) ? (s32)allChanDetResult.bev.bev.size() : BEV_POINT_NUM;
                memset(&tShowPoint.tBevPoint, 0, sizeof(tShowPoint.tBevPoint));
                
            // without tracker
            
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
            int imgsize_for=1920*1080;
            int width_for = 1920;
            int height_for= 1080;



            Ipc_sendData(0,0,(u8 *)&img_lists[0],imgsize_for*1.5,width_for,height_for,0,FORMAT_NV12);  // 1 ->RF
            Ipc_sendData(1,0,(u8 *)&img_lists[1],imgsize_for*1.5,width_for,height_for,0,FORMAT_NV12); // 2->LF
            Ipc_sendData(2,0,(u8 *)&img_lists[2],imgsize_for*1.5,width_for,height_for,0,FORMAT_NV12); // 3->LB
            Ipc_sendData(3,0,(u8 *)&img_lists[3],imgsize_for*1.5,width_for,height_for,0,FORMAT_NV12); // 4->RB
            Ipc_sendData(4,0,(u8 *)&img_lists[4],imgsize_for*1.5,width_for,height_for,0,FORMAT_NV12); //5->front
            Ipc_sendData(5,0,(u8 *)&img_lists[5],imgsize_for*1.5,width_for,height_for,0,FORMAT_NV12); //6->back
            Ipc_sendData(8, 0, (u8 *)&tShowPoint, sizeof(TShowPoint), 0, 0, 0, FORMAT_BIN);




   //           FILE* pd = NULL;
   //           char fntmp[1024] = {0};

   //           sprintf(fntmp, "Infer_result/fbtest-%d-output_result.bin",24*ii);

   //           std::string str(fntmp);
              //pd = fopen(filename.c_str(), "wb");
   //           pd = fopen(fntmp,"wb");
  //            fwrite(output_ptr0, sizeof(float), 24 * 100 * 100, pd);
  //            fwrite(output_ptr1, sizeof(float), 72 * 100 * 100, pd);
//              fwrite(output_ptr2, sizeof(float), 16 * 100 * 100, pd);
   //           fclose(pd);




              // release task handle
              //�ͷ������������δִ�����ֱ��ȡ�����ͷ���������Ѿ�ִ����������е�ĳЩ�ڵ��ȡ�����ͷ�����
              HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle),
                  "hbDNNReleaseTask failed");

    }

    Start = clock();

    for (int i = 0; i < input_tensors.size(); i++) { //�ͷ�BPU�ڴ档
        HB_CHECK_SUCCESS(hbSysFreeMem(&(input_tensors[i].sysMem[0])),
            "hbSysFreeMem failed");
    }
    // free output mem
    for (int i = 0; i < output_tensors.size(); i++) { //�ͷ�BPU�ڴ档
        HB_CHECK_SUCCESS(hbSysFreeMem(&(output_tensors[i].sysMem[0])),
            "hbSysFreeMem failed");
    }


    // Step7: release model resource
    { HB_CHECK_SUCCESS(hbDNNRelease(packed_dnn_handle), "hbDNNRelease failed"); } // �� packedDNNHandle ��ָ���ģ���ͷš�

    End = clock();
    End_all = clock();
    double endtime = (double)(End - Start) / CLOCKS_PER_SEC;
    std::cout << "Release time:" << endtime << std::endl;
    std::cout << "Release time:" << endtime * 1000 << "ms" << std::endl;
    double endtime_all = (double)(End_all - Start_all) / CLOCKS_PER_SEC;
    std::cout << "Release time:" << endtime_all << std::endl;
    std::cout << "Release time:" << endtime_all * 1000 << "ms" << std::endl;
    return 0;
}


//
// �������룺
//
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

void img_trans(cv::Mat& img, cv::Mat& img_crop, cv::Size resize_dims, int crop_w, int crop_h, int fW, int fH) {
    cv::Mat img_resize;
    cv::resize(img, img_resize, resize_dims);
    img_resize(cv::Range(crop_h, crop_h + fH), cv::Range(crop_w, crop_w + fW)).copyTo(img_crop); // 0 ->396 0->704
}

void normalize_img(cv::Mat& img, cv::Mat& img_blob) {
    float b_mean = 0.485;
    float g_mean = 0.456;
    float r_mean = 0.406;
    float b_std = 0.229;
    float g_std = 0.224;
    float r_std = 0.225;
    cv::Mat input;
    cv::Mat input_f;
    cv::Mat output = cv::Mat::zeros(INPUTHEIGHT, INPUTWIDTH, CV_32FC3);
    img.copyTo(input);

    input.convertTo(input_f, CV_32FC3, 1.0 / 255, 0);
    float* ptr = input_f.ptr<float>();

    for (int i = 0; i < img.rows; i++) {
        cv::Vec3f* pOut = output.ptr<cv::Vec3f>(i);
        cv::Vec3f* pIn = input_f.ptr<cv::Vec3f>(i);
        for (int j = 0; j < img.cols; ++j) {
            pOut[j][0] = (pIn[j][0] - b_mean) / b_std;
            pOut[j][1] = (pIn[j][1] - g_mean) / g_std;
            pOut[j][2] = (pIn[j][2] - r_mean) / r_std;
        }
    }
    float* ptr2 = output.ptr<float>();

    img_blob = output;
}

std::time_t getTimeStamp()
{
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto tmp=std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    std::time_t timestamp = tmp.count();
    //std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
    return timestamp;
}

void preprocess(std::vector<cv::Mat>& img_lists, std::vector<cv::Mat>& img_norm_lists) {
/*
    auto now = std::chrono::system_clock::now();
    //通过不同精度获取相差的毫秒数
    uint64_t dis_millseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()
        - std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count() * 1000;
    time_t tt = std::chrono::system_clock::to_time_t(now);
    auto time_tm = localtime(&tt);
    char strTime[40] = { 0 };


    using namespace std::chrono;

    auto duration_since_epoch = system_clock::now().time_since_epoch(); // 从1970-01-01 00:00:00到当前时间点的时长
    auto microseconds_since_epoch = duration_cast<microseconds>(duration_since_epoch).count(); // 将时长转换为微秒数
    time_t seconds_since_epoch = static_cast<time_t>(microseconds_since_epoch / 1000000); // 将时长转换为秒数

    std::tm current_time;
    localtime_r(&seconds_since_epoch, &current_time ); // 获取当前时间（精确到秒）
    auto tm_microsec = microseconds_since_epoch % 1000; // 当前时间的微妙数
    auto tm_millisec = microseconds_since_epoch / 1000 % 1000; // 当前时间的毫秒数
    std::cout<<tm_millisec<<std::endl;
    std::cout<<tm_microsec<<std::endl;

    sprintf(strTime, "%d-%02d-%02d %02d:%02d:%02d-%03d-%03d", time_tm->tm_year + 1900,
        time_tm->tm_mon + 1, time_tm->tm_mday, time_tm->tm_hour,
        time_tm->tm_min, time_tm->tm_sec, (int)dis_millseconds,tm_millisec,tm_microsec);
    std::cout << strTime << std::endl;
*/

    int H = IMAGE_HEIGHT_ORI; int W = IMAGE_WIDTH_ORI; //1080 1920
    int fH = INPUTHEIGHT; int fW = INPUTWIDTH; // 256 704
    float resize = fmax(fH / (1.0 * H), fW / (1.0 * W)); // 0.36666667
    cv::Size resize_dims_other;
    resize_dims_other.width = (int)(W * resize); //704    
    resize_dims_other.height = (int)(H * resize);//396
    int crop_h_other = fmax(0, (int)((1 - (0.0 + 0.22) / 2) * resize_dims_other.height) - fH); //96
    int crop_w_other = (int)(fmax(0, resize_dims_other.width - fW) / (2.0));//0

    cv::Mat img_0,img_norm_0;
    img_trans(img_lists[0], img_0, resize_dims_other, crop_w_other, crop_h_other, fW, fH);
    normalize_img(img_0, img_norm_0);
    cv::cvtColor(img_0, img_0, cv::COLOR_BGR2RGB);
    cv::cvtColor(img_norm_0, img_norm_0, cv::COLOR_BGR2RGB);
    img_norm_lists[0]=img_norm_0;
    char fntmp0[1024] = {0};
    sprintf(fntmp0, "lf_%d.jpg", rand());
    std::string str0(fntmp0);
    cv::imwrite(str0, img_0);

    cv::Mat img_1, img_norm_1;
    img_trans(img_lists[1], img_1, resize_dims_other, crop_w_other, crop_h_other, fW, fH);
    normalize_img(img_1,img_norm_1);
    cv::cvtColor(img_1, img_1, cv::COLOR_BGR2RGB);
    cv::cvtColor(img_norm_1, img_norm_1, cv::COLOR_BGR2RGB);
    img_norm_lists[1]=img_norm_1;
    char fntmp1[1024] = {0};
    sprintf(fntmp1, "f_%d.jpg", rand());
    std::string str1(fntmp1);
    cv::imwrite(str1, img_1);

    cv::Mat img_2,img_norm_2;
    img_trans(img_lists[2], img_2, resize_dims_other, crop_w_other, crop_h_other, fW, fH);
    normalize_img(img_2, img_norm_2);
    cv::cvtColor(img_2, img_2, cv::COLOR_BGR2RGB);
    cv::cvtColor(img_norm_2, img_norm_2, cv::COLOR_BGR2RGB);    
    img_norm_lists[2]=img_norm_2;
    char fntmp2[1024] = {0};
    sprintf(fntmp2, "rf_%d.jpg", rand());
    std::string str2(fntmp2);
    cv::imwrite(str2, img_2);

    cv::Mat img_3,img_norm_3;
    img_trans(img_lists[3], img_3, resize_dims_other, crop_w_other, crop_h_other, fW, fH);
    normalize_img(img_3, img_norm_3);
    cv::cvtColor(img_3, img_3, cv::COLOR_BGR2RGB);
    cv::cvtColor(img_norm_3, img_norm_3, cv::COLOR_BGR2RGB);
    img_norm_lists[3]=img_norm_3; 
    char fntmp3[1024] = {0};
    sprintf(fntmp3, "lb_%d.jpg", rand());
    std::string str3(fntmp3);
    cv::imwrite(str3, img_3);

    cv::Mat img_4, img_norm_4;
    img_trans(img_lists[4], img_4, resize_dims_other, crop_w_other, 0, fW, fH);
    normalize_img(img_4,img_norm_4);
    cv::cvtColor(img_4, img_4, cv::COLOR_BGR2RGB);
    cv::cvtColor(img_norm_4, img_norm_4, cv::COLOR_BGR2RGB);
    img_norm_lists[4]=img_norm_4;
    char fntmp4[1024] = {0};
    sprintf(fntmp4, "back_%d.jpg", rand());
    std::string str4(fntmp4);
    cv::imwrite(str4, img_4);

    cv::Mat img_5,img_norm_5;
    img_trans(img_lists[5], img_5, resize_dims_other, crop_w_other, crop_h_other, fW, fH);
    normalize_img(img_5, img_norm_5);
    cv::cvtColor(img_5, img_5, cv::COLOR_BGR2RGB);
    cv::cvtColor(img_norm_5, img_norm_5, cv::COLOR_BGR2RGB);
    img_norm_lists[5]=img_norm_5; 
    char fntmp5[1024] = {0};
    sprintf(fntmp5, "rb_%d.jpg", rand());
    std::string str5(fntmp5);
    cv::imwrite(str5, img_5);

/*
    for (int i = 0; i < img_lists.size(); i++) { // 
        cv::Mat img_tmp, img_norm;
        img_trans(img_lists[i], img_tmp, resize_dims, crop_w, crop_h, fW, fH);
        normalize_img(img_tmp, img_norm);
        cv::cvtColor(img_norm, img_norm, cv::COLOR_BGR2RGB); // 
        img_norm_lists[i] = img_norm;
    }
*/
}


int32_t Read_input_tensor(
    std::vector<string>& input_image_file, std::vector<hbDNNTensor>& input_tensor, std::vector<cv::Mat>& img_lists, std::vector<cv::Mat>& img_ori_lists, std::vector<cv::Mat>& img_norm_lists) {
    // Input 1: trans: 6*3*396*704
    int batch = 6;
    int input_h = INPUTHEIGHT; // hֵ
    int input_w = INPUTWIDTH; // wֵ

    cout << "Read Input tensor!" << endl;
    // READ IMAGEs

    hbDNNTensor& imgs_ptr = input_tensor[0]; //inputָ��
    hbDNNTensorProperties& Properties_imgs_ptr = imgs_ptr.properties; // ָ��inputָ���properties��ָ��

    if (input_image_file.size() != batch) { // �������ͼƬ����������batch������ 
        VLOG(EXAMPLE_SYSTEM)
            << "please check parameter 'image_file', make sure there are " << batch
            << " pictures!";
        return -1;
    }

    auto imgs_data = reinterpret_cast<float*>(imgs_ptr.sysMem[0].virAddr);// reinterpret_cast ����ת��
    auto batch_size = Properties_imgs_ptr.alignedByteSize / batch; //alingedByteSize-> �����������ݵ��ڴ��С, ����batch���õ�ÿ��batch��ռ���ڴ��С;

    // read rgb image 
    for (int32_t i{ 0 }; i < batch; i++) {
        cv::Mat frame_mat = cv::imread(input_image_file[i]); // imread ������bgr
        if (frame_mat.empty()) {
            VLOG(EXAMPLE_SYSTEM) << "image file not exist!";
            return -1;
        }
        img_ori_lists[i] = frame_mat; // ԭͼ
        cv::cvtColor(frame_mat, frame_mat, cv::COLOR_BGR2RGB); // 

        img_lists[i] = frame_mat; // �洢ת�����ͼ
    }




    //Start = clock();
    preprocess(img_lists, img_norm_lists); // img_norm_lists -> 704*256 
    /*End = clock();
    double Pre_process_time = (double)(End - Start) / CLOCKS_PER_SEC;
    std::cout << "Pre_process_time:" << Pre_process_time << std::endl;
    std::cout << "Pre_process_time:" << Pre_process_time * 1000 << "ms" << std::endl;*/

    cout<<"preprocess finish"<<endl;
    int32_t imgsize = 3 * INPUTHEIGHT * INPUTWIDTH;
    std::vector<cv::Mat> img_blob_lists(NUM_CAM);
    cv::Mat blob0, blob1, blob2, blob3, blob4, blob5;
    cv::dnn::blobFromImage(img_norm_lists[0], blob0, 1.0, cv::Size(INPUTWIDTH, INPUTHEIGHT), cv::Scalar(0, 0, 0), false, false, CV_32F);
    cv::dnn::blobFromImage(img_norm_lists[1], blob1, 1.0, cv::Size(INPUTWIDTH, INPUTHEIGHT), cv::Scalar(0, 0, 0), false, false, CV_32F);
    cv::dnn::blobFromImage(img_norm_lists[2], blob2, 1.0, cv::Size(INPUTWIDTH, INPUTHEIGHT), cv::Scalar(0, 0, 0), false, false, CV_32F);
    cv::dnn::blobFromImage(img_norm_lists[3], blob3, 1.0, cv::Size(INPUTWIDTH, INPUTHEIGHT), cv::Scalar(0, 0, 0), false, false, CV_32F);
    cv::dnn::blobFromImage(img_norm_lists[4], blob4, 1.0, cv::Size(INPUTWIDTH, INPUTHEIGHT), cv::Scalar(0, 0, 0), false, false, CV_32F);
    cv::dnn::blobFromImage(img_norm_lists[5], blob5, 1.0, cv::Size(INPUTWIDTH, INPUTHEIGHT), cv::Scalar(0, 0, 0), false, false, CV_32F);

    img_blob_lists[0] = blob0;
    img_blob_lists[1] = blob1;
    img_blob_lists[2] = blob2;
    img_blob_lists[3] = blob3;
    img_blob_lists[4] = blob4;
    img_blob_lists[5] = blob5;

    for (int32_t i{ 0 }; i < batch; i++) {

        float* img_norm_data = img_blob_lists[i].ptr<float>();

        memcpy(imgs_data, img_norm_data, 4 * imgsize);
        imgs_data += batch_size / 4;//++
    }

    return 0;
}


