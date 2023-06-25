#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ccalib/omnidir.hpp>

#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

typedef struct {
    std::vector<cv::Mat> rots_list;
    std::vector<cv::Mat> rots_inv_list;
    std::vector<cv::Mat> intrins_list;
    std::vector<cv::Mat> intrins_inv_list;
    std::vector<cv::Mat> post_rots_list;
} CalibParmsLists;

typedef struct {
    std::vector<std::vector<float>> centers;
    std::vector<std::vector<float>> whls;
    std::vector<std::vector<float>> quats;   
    std::vector<float> scores;   
    std::vector<int> labels;   
} TensorConvert;

/*
 * For Bev struct
*/ 
typedef struct {
    float x;
    float y;
} TF32Rect;

typedef struct {
    int cls;
    float sorce;
    TF32Rect point[4];
} TBevObj;

typedef struct {
    std::vector<TBevObj> bev; 
} TBevResult;

/*
 * For Detection Result struct
*/
typedef struct {
    int x;
    int y;
} TS32Rect;

typedef struct {
    int cls;
    float sorce;
    TS32Rect point[8];
} TImgObj;

typedef struct {
    int id;
    std::vector<TImgObj> img; 
} TImgResult;

typedef struct {
    TBevResult bev;
    std::vector<TImgResult> img;
} TDetResult;

TBevResult bev_proc(TensorConvert *parms, cv::Mat intrin, cv::Mat rot);
TImgResult projection_test(cv::Mat frame, TensorConvert *parms, cv::Mat intrin, cv::Mat tran, cv::Mat rot, int view_id, int imgs_id, cv::Mat& frame_imshow, cv::Mat xuan, cv::Mat jibian, double xi_);
TImgResult projection(TensorConvert *parms, cv::Mat intrin, cv::Mat tran, cv::Mat rot, int view_id, int imgs_id, cv::Mat& frame_imshow, cv::Mat xuan, cv::Mat jibian, double xi_);