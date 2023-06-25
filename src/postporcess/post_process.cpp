
#include "comm_api.h"

#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ccalib/omnidir.hpp>

#include <cmath>
#define PI 3.141592653589793
#include "ATen/ATen.h"
#include "post_process.h"
using namespace at;
using namespace std;

#define NUM_CAM 6
Tensor tTensorGrid;
vector<int> nms_array = {10,4,4};
/*
cv::Mat intrins_new_list[NUM_CAM];
cv::Mat rots_new_list[NUM_CAM];
cv::Mat trans_new_list[NUM_CAM];
cv::Mat xuan_new_list[NUM_CAM];
cv::Mat jibian_new_list[NUM_CAM];
double xi_new_list[NUM_CAM];
*/
int reshape_2T(float *ptData, float *ptOut, int H, int W)
{
   for (int i = 0; i < W; i++)
   {
      for (int j = 0; j < H; j++)
      {
         ptOut[j + i * H] = ptData[i + j * W];
      }
   }

   return 0;
}

int reshape_hwc2chw(float *ptData, float *ptOut, int H, int W, int C)
{
   int N = 1;
   // [N,H,W,C] -> [N,C,H,W]
   // equal to cv::dnn::blobFromImages或cv::dnn::blobFromImage
   for (int n = 0; n < N; ++n)
   {
      for (int c = 0; c < C; ++c)
      {
         for (int h = 0; h < H; ++h)
         {
            for (int w = 0; w < W; ++w)
            {
               int old_index = n * H * W * C + h * W * C + w * C + c;
               int new_index = n * C * H * W + c * H * W + h * W + w;

               ptOut[new_index] = ptData[old_index];
            }
         }
      }
   }
   return 0;
}

int reshape_chw2hwc(float *ptData, float *ptOut, int H, int W, int C)
{
   int N = 1;

   for (int n = 0; n < N; ++n)
   {
      for (int c = 0; c < C; ++c)
      {
         for (int h = 0; h < H; ++h)
         {
            for (int w = 0; w < W; ++w)
            {
               int old_index = n * H * W * C + h * W * C + w * C + c;
               int new_index = n * C * H * W + c * H * W + h * W + w;

               ptOut[old_index] = ptData[new_index];
            }
         }
      }
   }

   return 0;
}

/*
void calibParmsInit (void)
{
    float c0_intrins_new[3][3] = { 2081.425921, -0.271840, 956.589223, 0.0, 2084.591354, 532.759085, 0.0000e+00, 0.0000e+00, 1.0000e+00 };
    float c1_intrins_new[3][3] = { 1914.101042, 2.446897, 940.025280, 0.0, 1915.748709, 457.671140, 0.0000e+00, 0.0000e+00, 1.0000e+00 };
    float c2_intrins_new[3][3] = { 2199.410475, -0.345196, 978.837268, 0.0, 2203.397887, 494.145799, 0.0000e+00, 0.0000e+00, 1.0000e+00 };
    float c3_intrins_new[3][3] = { 2159.473077, 0.626668, 957.695888, 0.0, 2164.443408, 488.093810, 0.0000e+00, 0.0000e+00, 1.0000e+00 };
    float c4_intrins_new[3][3] = { 2163.506095, -1.962711, 958.192672, 0.0, 2166.670560, 492.363364, 0.0000e+00, 0.0000e+00, 1.0000e+00 };
    float c5_intrins_new[3][3] = { 3363.942066, -0.740026, 990.333652, 0.0, 3368.022589, 504.578520, 0.0000e+00, 0.0000e+00, 1.0000e+00 };
    int size2_intrins_new_list[] = { 3,3 };
    //std::vector<cv::Mat> intrins_new_list(NUM_CAM);
    //cv::Mat intrins_new_list[NUM_CAM];
    intrins_new_list[0] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c0_intrins_new);
    intrins_new_list[1] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c1_intrins_new);
    intrins_new_list[2] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c2_intrins_new);
    intrins_new_list[3] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c3_intrins_new);
    intrins_new_list[4] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c4_intrins_new);
    intrins_new_list[5] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c5_intrins_new);

    float c0_rots_new[3][3] = { 0.992830, 0.119376, 0.006143, 0.005965, 0.001847, -0.999981, -0.119385, 0.992847, 0.001121 };
    float c1_rots_new[3][3] = { -0.066909,-0.997631,-0.015993, 0.021592,0.014578,-0.999661, 0.997525,-0.067232, 0.020565 };
    float c2_rots_new[3][3] = { -0.991153,0.117363,0.06198, -0.048126,0.117447,-0.991912, -0.123694,-0.986120, -0.110760 };
    float c3_rots_new[3][3] = { 0.812144,0.580348,-0.060154, 0.036225,-0.153055,-0.987554, -0.582332,0.799856,-0.145326 };
    float c4_rots_new[3][3] = { -0.035734,0.999321,-0.008969, 0.408918,0.006432,-0.912549, -0.911871,-0.036277,-0.408870 };
    float c5_rots_new[3][3] = { -0.734650,0.677282,0.039721, -0.000211,0.058319,-0.998298, -0.678446,-0.733408,-0.042701 };
    int size2_rots_new_list[] = { 3,3 };
    //std::vector<cv::Mat> rots_new_list(NUM_CAM);
    //cv::Mat rots_new_list[NUM_CAM];
    rots_new_list[0] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c0_rots_new);
    rots_new_list[1] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c1_rots_new);
    rots_new_list[2] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c2_rots_new);
    rots_new_list[3] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c3_rots_new);
    rots_new_list[4] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c4_rots_new);
    rots_new_list[5] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c5_rots_new);

    float c0_trans_new[3] = { -1.089340428, 0.686766352, -2.201202640 };
    float c1_trans_new[3] = { 0.008972003, 1.420332455, -1.978304061 };
    float c2_trans_new[3] = { 1.427882435, 1.041115114, -1.865937482 };
    float c3_trans_new[3] = { -2.381890176, 0.781288427, 0.662920280 };
    float c4_trans_new[3] = { 0.022401206, 1.593815104, 0.206479688 };
    float c5_trans_new[3] = { 2.303284437, 0.727899900, 0.859485636 };
    int size1_trans_new_list[] = { 3 };
    //std::vector<cv::Mat> trans_new_list(NUM_CAM);
    //cv::Mat trans_new_list[NUM_CAM];
    trans_new_list[0] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c0_trans_new);
    trans_new_list[1] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c1_trans_new);
    trans_new_list[2] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c2_trans_new);
    trans_new_list[3] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c3_trans_new);
    trans_new_list[4] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c4_trans_new);
    trans_new_list[5] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c5_trans_new);

    float c0_xuan_new[3] = { 1.528651, -0.533603, 0.624108 };
    float c1_xuan_new[3] = { 1.149855, -1.249854, 1.256888 };
    float c2_xuan_new[3] = { 0.554496, -2.014202, 1.856897 };
    float c3_xuan_new[3] = { 1.680724, 0.476488, -0.483243 };
    float c4_xuan_new[3] = { 1.505609, 1.537199, -0.985874 };
    float c5_xuan_new[3] = { 0.665384, 1.837873, -1.734261 };
    int size1_xuan_new_list[] = { 3 };
    //std::vector<cv::Mat> xuan_new_list(NUM_CAM);
    //cv::Mat xuan_new_list[NUM_CAM];
    xuan_new_list[0] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c0_xuan_new);
    xuan_new_list[1] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c1_xuan_new);
    xuan_new_list[2] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c2_xuan_new);
    xuan_new_list[3] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c3_xuan_new);
    xuan_new_list[4] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c4_xuan_new);
    xuan_new_list[5] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c5_xuan_new);

    float c0_jibian_new[4] = { -0.567330, 0.228672, 0.000416, 0.000191 };
    float c1_jibian_new[4] = { -0.588660, 0.338600, -0.003191, 0.000425 };
    float c2_jibian_new[4] = { -0.570360, 0.192660, -0.000469, 0.000227 };
    float c3_jibian_new[4] = { -0.571261, 0.203645, -0.001499, -0.001872 };
    float c4_jibian_new[4] = { -0.579305, 0.245817, 0.000122, 0.000212 };
    float c5_jibian_new[4] = { -0.450573, -0.767328, 0.000100, -0.000612 };
    int size1_jibian_new_list[] = { 4 };
    //std::vector<cv::Mat> jibian_new_list(NUM_CAM);
    //cv::Mat jibian_new_list[NUM_CAM];
    jibian_new_list[0] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c0_jibian_new);
    jibian_new_list[1] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c1_jibian_new);
    jibian_new_list[2] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c2_jibian_new);
    jibian_new_list[3] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c3_jibian_new);
    jibian_new_list[4] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c4_jibian_new);
    jibian_new_list[5] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c5_jibian_new);

    //std::vector<double> xi_new_list = { 1.057374,0.609629,1.182572,1.127962,1.144298,2.331714 };
    xi_new_list[0] = 1.057374;
    xi_new_list[1] = 0.609629;
    xi_new_list[2] = 1.182572;
    xi_new_list[3] = 1.127962;
    xi_new_list[4] = 1.144298;
    xi_new_list[5] = 2.331714;
}
*/

#if 1
Tensor decode(Tensor anchors, Tensor deltas)
{
   Tensor xa = split(anchors, 1, 1)[0];
   Tensor ya = split(anchors, 1, 1)[1];
   Tensor za = split(anchors, 1, 1)[2];
   Tensor wa = split(anchors, 1, 1)[3];
   Tensor la = split(anchors, 1, 1)[4];
   Tensor ha = split(anchors, 1, 1)[5];
   Tensor ra = split(anchors, 1, 1)[6];
   Tensor cas1 = split(anchors, 1, 1)[7];
   Tensor cas2 = split(anchors, 1, 1)[8];

   Tensor xt = split(deltas, 1, 1)[0];
   Tensor yt = split(deltas, 1, 1)[1];
   Tensor zt = split(deltas, 1, 1)[2];
   Tensor wt = split(deltas, 1, 1)[3];
   Tensor lt = split(deltas, 1, 1)[4];
   Tensor ht = split(deltas, 1, 1)[5];
   Tensor rt = split(deltas, 1, 1)[6];
   Tensor cts1 = split(deltas, 1, 1)[7];
   Tensor cts2 = split(deltas, 1, 1)[8];

   za = za + ha / 2;
   Tensor diagonal = sqrt(la * la + wa * wa);
   Tensor xg = xt * diagonal + xa;
   Tensor yg = yt * diagonal + ya;
   Tensor zg = zt * ha + za;

   Tensor lg = exp(lt) * la;
   Tensor wg = exp(wt) * wa;
   Tensor hg = exp(ht) * ha;
   Tensor rg = rt + ra;
   zg = zg - hg / 2;
   Tensor cgs1 = cas1 + cts1;
   Tensor cgs2 = cas2 + cts2;
   Tensor box = cat({xg, yg, zg, wg, lg, hg, rg, cgs1, cgs2}, -1);

   return box;
}

std::vector<long> nms_cpu(Tensor dets, Tensor scores, double thresh, int nms_value)
{
   float f32Out[dets.sizes()[0] * dets.sizes()[1]];
   float *ptr_tmp = (float *)dets.data_ptr();
   reshape_2T(ptr_tmp, f32Out, dets.sizes()[0], dets.sizes()[1]);
   float *ptr = f32Out;

   // Tensor tmp = dets.permute({ 1, 0 });
   // float *ptr = (float *)tmp.data_ptr();
   Tensor x1 = CPU(kFloat).tensorFromBlob(ptr, {dets.sizes()[0], 1});

   float *ptrX1 = (float *)x1.data_ptr();
   Tensor y1 = CPU(kFloat).tensorFromBlob(ptr + dets.sizes()[0], {dets.sizes()[0], 1});

   float *ptrY1 = (float *)y1.data_ptr();
   Tensor x2 = CPU(kFloat).tensorFromBlob(ptr + dets.sizes()[0] * 2, {dets.sizes()[0], 1});

   float *ptrX2 = (float *)x2.data_ptr();
   Tensor y2 = CPU(kFloat).tensorFromBlob(ptr + dets.sizes()[0] * 3, {dets.sizes()[0], 1});

   float *ptrY2 = (float *)y2.data_ptr();
   int data[scores.sizes()[0]];
   for (int i = 0; i < scores.sizes()[0]; i++)
   {
      data[i] = i;
   }

   Tensor order = CPU(kInt).tensorFromBlob(data, {scores.sizes()[0]});

   int *ptrOrder = (int *)order.data_ptr();

   int ndets = order.sizes()[0];
   std::vector<int> suppressed(ndets, 0);

   std::vector<long> keep;
   for (int i_ = 0; i_ < ndets; i_++)
   {
      int i = ptrOrder[i_]; // start with highest score box
      if (suppressed[i] == 1)
         continue; // if any box have enough iou with this, remove it.
      keep.push_back(i);
      for (int j_ = i_ + 1; j_ < ndets; j_++)
      {
         int j = ptrOrder[j_];
         if (suppressed[j] == 1)
            continue;
         int dist = pow((ptrX1[i] + ptrX2[i] - ptrX1[j] - ptrX2[j]) / 2, 2) + pow((ptrY1[i] + ptrY2[i] - ptrY1[j] - ptrY2[j]) / 2, 2);

         if (dist <= nms_value)
            suppressed[j] = 1;
      }
   }
   return keep;
}

std::vector<std::tuple<Tensor, Tensor, Tensor, Tensor>> box3d_multiclass_nms(Tensor mlvl_bboxes, Tensor mlvl_bboxes_for_nms, Tensor mlvl_scores, Tensor mlvl_dir_scores)
{
   const int nms_pre = 1000;
   const int num_classes = 3;
   const double score_thr = 0.4;
   std::vector<Tensor> bboxes;
   std::vector<Tensor> scores;
   std::vector<Tensor> labels;
   std::vector<Tensor> dir_scores;

   float f32Out[4000];
   float *ptr = (float *)mlvl_scores.data_ptr();
   reshape_2T(ptr, f32Out, 1000, 4);
   float *ptr_mlvl_scores = f32Out;
   
   for (int i = 0; i < 3; i++)
   {
      Tensor mlvlScore = CPU(kFloat).tensorFromBlob(ptr_mlvl_scores + i * nms_pre, {nms_pre, 1});
      float *ptScore = (float *)mlvlScore.data_ptr();
      // cout << "mlvlScore strides " << mlvlScore.strides() << " sizes " << mlvlScore.sizes() << endl;
      Tensor cls_inds = mlvlScore > score_thr;
      // cout << "cls_inds strides " << cls_inds.strides() << " sizes " << cls_inds.sizes() << endl;
      // std::cout << "=========== cls_inds " << cls_inds << std::endl;
      // 这里还要加一个判断
      Tensor _scores = mlvlScore.index({cls_inds});
      if (_scores.sizes()[0] == 0)
      {
         continue;
      }

      long data[nms_pre] = {0};
      for (int k = 0; k < nms_pre; k++)
      {
         if (ptScore[k] > score_thr)
         {
            data[k] = 1000 - k;
         }
      }
      Tensor cls = CPU(kLong).tensorFromBlob(data, {nms_pre});
      std::tuple<Tensor, Tensor> cls2 = cls.topk(_scores.sizes()[0]);
      // cout << "=====std::get<1>(cls2)=======" << std::get<1>(cls2) << endl;

      Tensor _bboxes_for_nms = mlvl_bboxes_for_nms.index({std::get<1>(cls2)});

      std::vector<long> keep = nms_cpu(_bboxes_for_nms, _scores, 0.2,nms_array[i]);
      Tensor selected = CPU(kLong).tensorFromBlob(&keep[0], {(int)keep.size()});

      Tensor _mlvl_bboxes = mlvl_bboxes.index({std::get<1>(cls2)});

      bboxes.push_back(_mlvl_bboxes.index({selected}));

      scores.push_back(_scores.index({selected}));

      Tensor cls_label = CPU(kFloat).ones(selected.sizes()[0]) * i;

      labels.push_back(cls_label);
      Tensor _mlvl_dir_scores = mlvl_dir_scores.index({std::get<1>(cls2)});

      dir_scores.push_back(_mlvl_dir_scores.index({selected}));
   }

   std::vector<std::tuple<Tensor, Tensor, Tensor, Tensor>> vResult;

   if (bboxes.size() > 0)
   {
      Tensor bboxes1 = cat(bboxes, 0);
      // cout << "-==================end1===================" << endl;
      Tensor scores1 = cat(scores, 0);
      // cout << "-==================end2===================" << endl;
      Tensor labels1 = cat(labels, 0);
      // cout << "-==================end3===================" << endl;
      Tensor dir_scores1 = cat(dir_scores, 0);
      // cout << "-==================end4===================" << endl;
      std::tuple<Tensor, Tensor, Tensor, Tensor> results(bboxes1, scores1, labels1, dir_scores1);
      vResult.push_back(results);
   }

   return vResult;
}

std::vector<Tensor> get_bboxes_single(Tensor &cls_scores, Tensor &bbox_preds, Tensor &dir_cls_preds, Tensor &mlvl_anchors)
{
   const int num_classes = 3;
   const int box_code_size = 9;
   const double dir_offset = 0.7854;
   const double dir_limit_offset = 0;
   const int nms_pre = 1000;

   Tensor dir_cls_pred = dir_cls_preds.reshape_({80000, 2}, {2, 1});
   std::tuple<Tensor, Tensor> max_classes = max(dir_cls_pred, 1);
   auto dir_cls_score = std::get<1>(max_classes);

   Tensor cls_score = cls_scores.reshape_({80000, num_classes}, {num_classes, 1});
   Tensor scores = cls_score;

   Tensor bbox_pred = bbox_preds.reshape_({80000, box_code_size}, {box_code_size, 1});

   std::tuple<Tensor, Tensor> max_classes1 = max(scores, 1);
   auto max_scores = std::get<0>(max_classes1);
   std::tuple<Tensor, Tensor> max_classes2 = max_scores.topk(nms_pre);

   Tensor topk_inds = std::get<1>(max_classes2);

   Tensor anchors = mlvl_anchors.index({topk_inds});

   bbox_pred = bbox_pred.index({topk_inds});
   scores = scores.index({topk_inds});
   dir_cls_score = dir_cls_score.index({topk_inds});
   Tensor bboxes = decode(anchors, bbox_pred);
   Tensor padding = CPU(kFloat).zeros({1000, 1});

   Tensor mlvl_scores = cat({scores, padding}, 1);

   // Tensor tmp = bboxes.permute({ 1, 0 });
   float *ptr = (float *)bboxes.data_ptr();

   float f32Out[9000];
   reshape_2T(ptr, f32Out, 1000, 9);
   Tensor tensor1 = CPU(kFloat).tensorFromBlob(f32Out, {nms_pre, 1});
   Tensor tensor2 = CPU(kFloat).tensorFromBlob(f32Out + 1000, {nms_pre, 1});
   Tensor tensor3 = CPU(kFloat).tensorFromBlob(f32Out + 3000, {nms_pre, 1}) / 2;
   Tensor tensor4 = CPU(kFloat).tensorFromBlob(f32Out + 4000, {nms_pre, 1}) / 2;
   Tensor tensor5 = CPU(kFloat).tensorFromBlob(f32Out + 6000, {nms_pre, 1});

   Tensor tensor11 = tensor1 - tensor3;
   Tensor tensor22 = tensor2 - tensor4;
   Tensor tensor33 = tensor1 + tensor3;
   Tensor tensor44 = tensor2 + tensor4;

   Tensor mlvl_bboxes_for_nms = cat({tensor11, tensor22, tensor33, tensor44, tensor5}, 1);
   // cout << "mlvl_bboxes_for_nms strides " << mlvl_bboxes_for_nms.strides() << " sizes " << mlvl_bboxes_for_nms.sizes() << endl;
   std::vector<std::tuple<Tensor, Tensor, Tensor, Tensor>> results = box3d_multiclass_nms(bboxes, mlvl_bboxes_for_nms, mlvl_scores, dir_cls_score);

   if (results.size() > 0)
   {
      auto bboxes1 = std::get<0>(results[0]);
      auto scores1 = std::get<1>(results[0]);
      auto labels1 = std::get<2>(results[0]);
      auto rotate = std::get<3>(results[0]);

      // Tensor &bboxcpy = bboxes1;
      if (bboxes1.sizes()[0] > 0)
      {
         for (int i = 0; i < bboxes1.sizes()[0]; i++)
         {
            float dir_scores = Scalar(rotate[i]).toFloat();
            float value6 = Scalar(bboxes1[i][6]).toFloat();
            float dir_rot = value6 - dir_offset - floor((value6 - dir_offset) / PI + dir_limit_offset) * PI;
            value6 = dir_rot + dir_offset + PI * dir_scores;
            bboxes1[i][6] = value6;
         }
      }
      std::vector<Tensor> results1 = {bboxes1, scores1, labels1};
      return results1;
   }
   std::vector<Tensor> vResults;
   return vResults;
}
#endif

TDetResult _test(std::vector<cv::Mat> img_lists, float *ptF32ScoreBuff, float *ptF32BboxBuff, float *ptF32ClassBuff)
{
   TDetResult detResult;

   if ((img_lists.size() != 6) || (ptF32ScoreBuff == NULL) || (ptF32BboxBuff == NULL) || (ptF32ClassBuff == NULL))
   {
      printf("==========================error");
      return detResult;
   }

#if 1
   float c0_intrins_new[3][3] = {2081.425921, -0.271840, 956.589223, 0.0, 2084.591354, 532.759085, 0.0000e+00, 0.0000e+00, 1.0000e+00};
   float c1_intrins_new[3][3] = {3.64448929e+03, 1.94223400e+00, 0.00000000e+00, 3.64988558e+03, 1.09198174e+03, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00};
   float c2_intrins_new[3][3] = {2199.410475, -0.345196, 978.837268, 0.0, 2203.397887, 494.145799, 0.0000e+00, 0.0000e+00, 1.0000e+00};
   float c3_intrins_new[3][3] = {2159.473077, 0.626668, 957.695888, 0.0, 2164.443408, 488.093810, 0.0000e+00, 0.0000e+00, 1.0000e+00};
   float c4_intrins_new[3][3] = {2163.506095, -1.962711, 958.192672, 0.0, 2166.670560, 492.363364, 0.0000e+00, 0.0000e+00, 1.0000e+00};
   float c5_intrins_new[3][3] = {3363.942066, -0.740026, 990.333652, 0.0, 3368.022589, 504.578520, 0.0000e+00, 0.0000e+00, 1.0000e+00};
   std::vector<cv::Mat> intrins_new_list(NUM_CAM);
   int size2_intrins_new_list[] = {3, 3};
   intrins_new_list[0] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c0_intrins_new);
   intrins_new_list[1] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c1_intrins_new);
   intrins_new_list[2] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c2_intrins_new);
   intrins_new_list[3] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c3_intrins_new);
   intrins_new_list[4] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c4_intrins_new);
   intrins_new_list[5] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c5_intrins_new);

   float c0_rots_new[3][3] = {0.741427, -0.667844, 0.065353, 0.039731, -0.05353, -0.997776, 0.669857, 0.742374, -0.013154};
   float c1_rots_new[3][3] = {-0.091742, -0.995396, -0.02774, 0.122503, 0.016364, -0.992333, 0.988219, -0.094437, 0.120438};
   float c2_rots_new[3][3] = {-0.847614, -0.526868, -0.062933, 0.033247, 0.065636, -0.99729, 0.529571, -0.847409, -0.038117};
   float c3_rots_new[3][3] = {0.821169, 0.568387, -0.051168, 0.044649, -0.153374, -0.987159, -0.568936, 0.80834, -0.151324};
   float c4_rots_new[3][3] = {-2.27410e-02, 9.99741e-01, 3.10000e-05, 4.16974e-01, 9.51300e-03, -9.08869e-01, -9.08634e-01, -2.06560e-02, -4.17083e-01};
   float c5_rots_new[3][3] = {-7.42725e-01, 6.68418e-01, 3.97210e-02, -9.11000e-04, 5.83120e-02, -9.98298e-01, -6.69596e-01, -7.41497e-01, -4.27010e-02};
   std::vector<cv::Mat> rots_new_list(NUM_CAM);
   int size2_rots_new_list[] = {3, 3};
   rots_new_list[0] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c0_rots_new);
   rots_new_list[1] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c1_rots_new);
   rots_new_list[2] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c2_rots_new);
   rots_new_list[3] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c3_rots_new);
   rots_new_list[4] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c4_rots_new);
   rots_new_list[5] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c5_rots_new);

   float c0_trans_new[3] = {-1.09595161, 0.68629192, -2.19806668};
   float c1_trans_new[3] = {0.03236304, 1.24888652, -2.17298289};
   float c2_trans_new[3] = {1.43212709, 0.85146336, -1.95665882};
   float c3_trans_new[3] = {-2.381890176, 0.781288427, 0.662920280};
   float c4_trans_new[3] = {0.022401206, 1.593815104, 0.206479688};
   float c5_trans_new[3] = {2.303284437, 0.727899900, 0.859485636};
   std::vector<cv::Mat> trans_new_list(NUM_CAM);
   int size1_trans_new_list[] = {3};
   trans_new_list[0] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c0_trans_new);
   trans_new_list[1] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c1_trans_new);
   trans_new_list[2] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c2_trans_new);
   trans_new_list[3] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c3_trans_new);
   trans_new_list[4] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c4_trans_new);
   trans_new_list[5] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c5_trans_new);

   float c0_xuan_new[3] = {1.529199, -0.531222, 0.621799};
   float c1_xuan_new[3] = {1.056942, -1.195918, 1.315916};
   float c2_xuan_new[3] = {0.490714, -1.939874, 1.833833};
   float c3_xuan_new[3] = {1.679212, 0.484234, -0.489817};
   float c4_xuan_new[3] = {1.504345, 1.538984, -0.987019};
   float c5_xuan_new[3] = {0.665384, 1.837873, -1.734261};
   std::vector<cv::Mat> xuan_new_list(NUM_CAM);
   int size1_xuan_new_list[] = {3};
   xuan_new_list[0] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c0_xuan_new);
   xuan_new_list[1] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c1_xuan_new);
   xuan_new_list[2] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c2_xuan_new);
   xuan_new_list[3] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c3_xuan_new);
   xuan_new_list[4] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c4_xuan_new);
   xuan_new_list[5] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c5_xuan_new);

   float c0_jibian_new[4] = {-0.567330, 0.228672, 0.000416, 0.000191};
   float c1_jibian_new[4] = {-4.45664e-01, 1.91109e-01, 1.08000e-04, 2.77000e-04};
   float c2_jibian_new[4] = {-0.570360, 0.192660, -0.000469, 0.000227};
   float c3_jibian_new[4] = {-0.571261, 0.203645, -0.001499, -0.001872};
   float c4_jibian_new[4] = {-0.579305, 0.245817, 0.000122, 0.000212};
   float c5_jibian_new[4] = {-0.450573, -0.767328, 0.000100, -0.000612};
   std::vector<cv::Mat> jibian_new_list(NUM_CAM);
   int size1_jibian_new_list[] = {4};
   jibian_new_list[0] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c0_jibian_new);
   jibian_new_list[1] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c1_jibian_new);
   jibian_new_list[2] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c2_jibian_new);
   jibian_new_list[3] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c3_jibian_new);
   jibian_new_list[4] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c4_jibian_new);
   jibian_new_list[5] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c5_jibian_new);

   std::vector<double> xi_new_list = {1.057374, 0.609629, 1.182572, 1.127962, 1.144298, 2.331714};
#else
// calibParmsInit();
#endif

   // float f32Score[100 * 100 * 24] = {0};
   // float f32Bbox[100 * 100 * 72] = {0};
   // float f32Class[100 * 100 * 16] = {0};

   // reshape_chw2hwc(ptF32ScoreBuff, f32Score, 100, 100, 24);
   Tensor tTensorScore = CPU(kFloat).tensorFromBlob(ptF32ScoreBuff, {100, 100, 24});

   // reshape_chw2hwc(ptF32BboxBuff, f32Bbox, 100, 100, 72);
   Tensor tTensorBbox = CPU(kFloat).tensorFromBlob(ptF32BboxBuff, {100, 100, 72});

   // reshape_chw2hwc(ptF32ClassBuff, f32Class, 100, 100, 16);
   Tensor tTensorClass = CPU(kFloat).tensorFromBlob(ptF32ClassBuff, {100, 100, 16});

   std::vector<Tensor> result = get_bboxes_single(tTensorScore, tTensorBbox, tTensorClass, tTensorGrid);

   if (result.size() == 0)
   {
      return detResult;
   }

   // Tensor tmp = result[0].permute({1, 0});

   float f32Out[result[0].sizes()[0] * result[0].sizes()[1]];
   float *ptr_tmp = (float *)result[0].data_ptr();
   reshape_2T(ptr_tmp, f32Out, result[0].sizes()[0], result[0].sizes()[1]);
   // float *ptr = f32Out;
   Tensor result1 = CPU(kFloat).tensorFromBlob(f32Out, {result[0].sizes()[1], result[0].sizes()[0]}); // Tensor result1

   long data0[3] = {0, 1, 2};
   Tensor res0 = CPU(kLong).tensorFromBlob(data0, {3});
   Tensor centers1 = result1.index({res0});
   float f32Out1[centers1.sizes()[0] * centers1.sizes()[1]];
   float *ptr_tmp1 = (float *)centers1.data_ptr();
   reshape_2T(ptr_tmp1, f32Out1, centers1.sizes()[0], centers1.sizes()[1]);
   // float *ptr = f32Out1;
   Tensor centers = CPU(kFloat).tensorFromBlob(f32Out1, {centers1.sizes()[1], centers1.sizes()[0]}); // Tensor centers

   long data1[3] = {3, 4, 5};
   Tensor res1 = CPU(kLong).tensorFromBlob(data1, {3});
   Tensor sizess1 = result1.index({res1});
   float f32Out2[sizess1.sizes()[0] * sizess1.sizes()[1]];
   float *ptr_tmp2 = (float *)sizess1.data_ptr();
   reshape_2T(ptr_tmp2, f32Out2, sizess1.sizes()[0], sizess1.sizes()[1]);
   Tensor sizess = CPU(kFloat).tensorFromBlob(f32Out2, {sizess1.sizes()[1], sizess1.sizes()[0]}); // Tensor sizess

   long data2[3] = {6};
   Tensor res2 = CPU(kLong).tensorFromBlob(data2, {1});
   Tensor rot1 = result1.index({res2});
   float f32Out3[rot1.sizes()[0] * rot1.sizes()[1]];
   float *ptr_tmp3 = (float *)rot1.data_ptr();
   reshape_2T(ptr_tmp3, f32Out3, rot1.sizes()[0], rot1.sizes()[1]);
   Tensor rot = CPU(kFloat).tensorFromBlob(f32Out3, {rot1.sizes()[1], rot1.sizes()[0]}); // Tensor rot

   Tensor scores_3d = result[1];
   Tensor labels_3d = result[2];

   TensorConvert tensorParms;
   for (int iii = 0; iii < centers.sizes()[0]; iii++)
   {
      Tensor center = centers[iii];
      float *ptr_v1 = (float *)center.data_ptr();
      std::vector<float> v1 = {ptr_v1[0], ptr_v1[1], ptr_v1[2]};
      tensorParms.centers.push_back(v1);

      Tensor sizel = sizess[iii];
      float *ptr_v2 = (float *)sizel.data_ptr();
      std::vector<float> v2 = {ptr_v2[0], ptr_v2[1], ptr_v2[2]};
      tensorParms.whls.push_back(v2);

      std::vector<float> quat1(4, 0);
      float *ptr_rot = (float *)rot[iii].data_ptr();
      quat1[0] = cos(*ptr_rot / 2);
      quat1[3] = sin(*ptr_rot / 2);
      tensorParms.quats.push_back(quat1);

      Tensor score = scores_3d[iii];
      float *ptr_score = (float *)score.data_ptr();
      float v3 = *ptr_score;
      tensorParms.scores.push_back(v3);

      Tensor label = labels_3d[iii];
      int *ptr_label = (int *)label.data_ptr();
      int v4 = *ptr_label;
      tensorParms.labels.push_back(v4);
   }

   std::vector<cv::Mat> img_imshow_lists(NUM_CAM);

   detResult.bev = bev_proc(&tensorParms, intrins_new_list[0], rots_new_list[0]);
   for (int view_id = 0; view_id < NUM_CAM; view_id++)
   {
      TImgResult imgResult = projection_test(img_lists[view_id], &tensorParms, intrins_new_list[view_id],
                                             trans_new_list[view_id], rots_new_list[view_id], view_id, 0,
                                             img_imshow_lists[view_id], xuan_new_list[view_id],
                                             jibian_new_list[view_id], xi_new_list[view_id]);
      imgResult.id = view_id;
      detResult.img.push_back(imgResult);
   }
   return detResult;
}

TDetResult post_process(float *ptF32ScoreBuff, float *ptF32BboxBuff, float *ptF32ClassBuff)
{
   TDetResult detResult;

   if ((ptF32ScoreBuff == NULL) || (ptF32BboxBuff == NULL) || (ptF32ClassBuff == NULL))
   {
      printf("==========================error");
      return detResult;
   }

#if 1
   float c0_intrins_new[3][3] = {2081.425921, -0.271840, 956.589223, 0.0, 2084.591354, 532.759085, 0.0000e+00, 0.0000e+00, 1.0000e+00};
   float c1_intrins_new[3][3] = {3.64448929e+03, 1.94223400e+00, 0.00000000e+00, 3.64988558e+03, 1.09198174e+03, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00};
   float c2_intrins_new[3][3] = {2199.410475, -0.345196, 978.837268, 0.0, 2203.397887, 494.145799, 0.0000e+00, 0.0000e+00, 1.0000e+00};
   float c3_intrins_new[3][3] = {2159.473077, 0.626668, 957.695888, 0.0, 2164.443408, 488.093810, 0.0000e+00, 0.0000e+00, 1.0000e+00};
   float c4_intrins_new[3][3] = {2163.506095, -1.962711, 958.192672, 0.0, 2166.670560, 492.363364, 0.0000e+00, 0.0000e+00, 1.0000e+00};
   float c5_intrins_new[3][3] = {3363.942066, -0.740026, 990.333652, 0.0, 3368.022589, 504.578520, 0.0000e+00, 0.0000e+00, 1.0000e+00};
   std::vector<cv::Mat> intrins_new_list(NUM_CAM);
   int size2_intrins_new_list[] = {3, 3};
   intrins_new_list[0] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c0_intrins_new);
   intrins_new_list[1] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c1_intrins_new);
   intrins_new_list[2] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c2_intrins_new);
   intrins_new_list[3] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c3_intrins_new);
   intrins_new_list[4] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c4_intrins_new);
   intrins_new_list[5] = cv::Mat(2, size2_intrins_new_list, CV_32FC1, c5_intrins_new);

   float c0_rots_new[3][3] = {0.741427, -0.667844, 0.065353, 0.039731, -0.05353, -0.997776, 0.669857, 0.742374, -0.013154};
   float c1_rots_new[3][3] = {-0.091742, -0.995396, -0.02774, 0.122503, 0.016364, -0.992333, 0.988219, -0.094437, 0.120438};
   float c2_rots_new[3][3] = {-0.847614, -0.526868, -0.062933, 0.033247, 0.065636, -0.99729, 0.529571, -0.847409, -0.038117};
   float c3_rots_new[3][3] = {0.821169, 0.568387, -0.051168, 0.044649, -0.153374, -0.987159, -0.568936, 0.80834, -0.151324};
   float c4_rots_new[3][3] = {-2.27410e-02, 9.99741e-01, 3.10000e-05, 4.16974e-01, 9.51300e-03, -9.08869e-01, -9.08634e-01, -2.06560e-02, -4.17083e-01};
   float c5_rots_new[3][3] = {-7.42725e-01, 6.68418e-01, 3.97210e-02, -9.11000e-04, 5.83120e-02, -9.98298e-01, -6.69596e-01, -7.41497e-01, -4.27010e-02};
   std::vector<cv::Mat> rots_new_list(NUM_CAM);
   int size2_rots_new_list[] = {3, 3};
   rots_new_list[0] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c0_rots_new);
   rots_new_list[1] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c1_rots_new);
   rots_new_list[2] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c2_rots_new);
   rots_new_list[3] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c3_rots_new);
   rots_new_list[4] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c4_rots_new);
   rots_new_list[5] = cv::Mat(2, size2_rots_new_list, CV_32FC1, c5_rots_new);

   float c0_trans_new[3] = {-1.09595161, 0.68629192, -2.19806668};
   float c1_trans_new[3] = {0.03236304, 1.24888652, -2.17298289};
   float c2_trans_new[3] = {1.43212709, 0.85146336, -1.95665882};
   float c3_trans_new[3] = {-2.381890176, 0.781288427, 0.662920280};
   float c4_trans_new[3] = {0.022401206, 1.593815104, 0.206479688};
   float c5_trans_new[3] = {2.303284437, 0.727899900, 0.859485636};
   std::vector<cv::Mat> trans_new_list(NUM_CAM);
   int size1_trans_new_list[] = {3};
   trans_new_list[0] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c0_trans_new);
   trans_new_list[1] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c1_trans_new);
   trans_new_list[2] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c2_trans_new);
   trans_new_list[3] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c3_trans_new);
   trans_new_list[4] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c4_trans_new);
   trans_new_list[5] = cv::Mat(1, size1_trans_new_list, CV_32FC1, c5_trans_new);

   float c0_xuan_new[3] = {1.529199, -0.531222, 0.621799};
   float c1_xuan_new[3] = {1.056942, -1.195918, 1.315916};
   float c2_xuan_new[3] = {0.490714, -1.939874, 1.833833};
   float c3_xuan_new[3] = {1.679212, 0.484234, -0.489817};
   float c4_xuan_new[3] = {1.504345, 1.538984, -0.987019};
   float c5_xuan_new[3] = {0.665384, 1.837873, -1.734261};

   std::vector<cv::Mat> xuan_new_list(NUM_CAM);
   int size1_xuan_new_list[] = {3};
   xuan_new_list[0] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c0_xuan_new);
   xuan_new_list[1] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c1_xuan_new);
   xuan_new_list[2] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c2_xuan_new);
   xuan_new_list[3] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c3_xuan_new);
   xuan_new_list[4] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c4_xuan_new);
   xuan_new_list[5] = cv::Mat(1, size1_xuan_new_list, CV_32FC1, c5_xuan_new);

   float c0_jibian_new[4] = {-0.567330, 0.228672, 0.000416, 0.000191};
   float c1_jibian_new[4] = {-4.45664e-01, 1.91109e-01, 1.08000e-04, 2.77000e-04};
   float c2_jibian_new[4] = {-0.570360, 0.192660, -0.000469, 0.000227};
   float c3_jibian_new[4] = {-0.571261, 0.203645, -0.001499, -0.001872};
   float c4_jibian_new[4] = {-0.579305, 0.245817, 0.000122, 0.000212};
   float c5_jibian_new[4] = {-0.450573, -0.767328, 0.000100, -0.000612};
   std::vector<cv::Mat> jibian_new_list(NUM_CAM);
   int size1_jibian_new_list[] = {4};
   jibian_new_list[0] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c0_jibian_new);
   jibian_new_list[1] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c1_jibian_new);
   jibian_new_list[2] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c2_jibian_new);
   jibian_new_list[3] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c3_jibian_new);
   jibian_new_list[4] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c4_jibian_new);
   jibian_new_list[5] = cv::Mat(1, size1_jibian_new_list, CV_32FC1, c5_jibian_new);

   std::vector<double> xi_new_list = {1.057374, 0.609629, 1.182572, 1.127962, 1.144298, 2.331714};
#else
// calibParmsInit();
#endif

   // float f32Score[100 * 100 * 24] = {0};
   // float f32Bbox[100 * 100 * 72] = {0};
   // float f32Class[100 * 100 * 16] = {0};
   // reshape_chw2hwc(ptF32ScoreBuff, f32Score, 100, 100, 24);
   Tensor tTensorScore = CPU(kFloat).tensorFromBlob(ptF32ScoreBuff, {100, 100, 24});

   // reshape_chw2hwc(ptF32BboxBuff, f32Bbox, 100, 100, 72);
   Tensor tTensorBbox = CPU(kFloat).tensorFromBlob(ptF32BboxBuff, {100, 100, 72});

   // reshape_chw2hwc(ptF32ClassBuff, f32Class, 100, 100, 16);
   Tensor tTensorClass = CPU(kFloat).tensorFromBlob(ptF32ClassBuff, {100, 100, 16});

   std::vector<Tensor> result = get_bboxes_single(tTensorScore, tTensorBbox, tTensorClass, tTensorGrid);

   if (result.size() == 0)
   {
      return detResult;
   }

   // Tensor tmp = result[0].permute({1, 0});

   float f32Out[result[0].sizes()[0] * result[0].sizes()[1]];
   float *ptr_tmp = (float *)result[0].data_ptr();
   reshape_2T(ptr_tmp, f32Out, result[0].sizes()[0], result[0].sizes()[1]);
   // float *ptr = f32Out;
   Tensor result1 = CPU(kFloat).tensorFromBlob(f32Out, {result[0].sizes()[1], result[0].sizes()[0]}); // Tensor result1

   long data0[3] = {0, 1, 2};
   Tensor res0 = CPU(kLong).tensorFromBlob(data0, {3});
   Tensor centers1 = result1.index({res0});
   float f32Out1[centers1.sizes()[0] * centers1.sizes()[1]];
   float *ptr_tmp1 = (float *)centers1.data_ptr();
   reshape_2T(ptr_tmp1, f32Out1, centers1.sizes()[0], centers1.sizes()[1]);
   // float *ptr = f32Out1;
   Tensor centers = CPU(kFloat).tensorFromBlob(f32Out1, {centers1.sizes()[1], centers1.sizes()[0]}); // Tensor centers

   long data1[3] = {3, 4, 5};
   Tensor res1 = CPU(kLong).tensorFromBlob(data1, {3});
   Tensor sizess1 = result1.index({res1});
   float f32Out2[sizess1.sizes()[0] * sizess1.sizes()[1]];
   float *ptr_tmp2 = (float *)sizess1.data_ptr();
   reshape_2T(ptr_tmp2, f32Out2, sizess1.sizes()[0], sizess1.sizes()[1]);
   Tensor sizess = CPU(kFloat).tensorFromBlob(f32Out2, {sizess1.sizes()[1], sizess1.sizes()[0]}); // Tensor sizess

   long data2[3] = {6};
   Tensor res2 = CPU(kLong).tensorFromBlob(data2, {1});
   Tensor rot1 = result1.index({res2});
   float f32Out3[rot1.sizes()[0] * rot1.sizes()[1]];
   float *ptr_tmp3 = (float *)rot1.data_ptr();
   reshape_2T(ptr_tmp3, f32Out3, rot1.sizes()[0], rot1.sizes()[1]);
   Tensor rot = CPU(kFloat).tensorFromBlob(f32Out3, {rot1.sizes()[1], rot1.sizes()[0]}); // Tensor rot

   Tensor scores_3d = result[1];
   Tensor labels_3d = result[2];

   TensorConvert tensorParms;
   for (int iii = 0; iii < centers.sizes()[0]; iii++)
   {
      Tensor center = centers[iii];
      float *ptr_v1 = (float *)center.data_ptr();
      std::vector<float> v1 = {ptr_v1[0], ptr_v1[1], ptr_v1[2]};
      tensorParms.centers.push_back(v1);

      Tensor sizel = sizess[iii];
      float *ptr_v2 = (float *)sizel.data_ptr();
      std::vector<float> v2 = {ptr_v2[0], ptr_v2[1], ptr_v2[2]};
      tensorParms.whls.push_back(v2);

      std::vector<float> quat1(4, 0);
      float *ptr_rot = (float *)rot[iii].data_ptr();
      quat1[0] = cos(*ptr_rot / 2);
      quat1[3] = sin(*ptr_rot / 2);
      tensorParms.quats.push_back(quat1);

      Tensor score = scores_3d[iii];
      float *ptr_score = (float *)score.data_ptr();
      float v3 = *ptr_score;
      tensorParms.scores.push_back(v3);

      // Tensor label = labels_3d[iii];
      // int *ptr_label = (int *)label.data_ptr();
      // int v4 = *ptr_label;
      int v4 = Scalar(labels_3d[iii]).toInt();
      tensorParms.labels.push_back(v4);
   }

   std::vector<cv::Mat> img_imshow_lists(NUM_CAM);

   detResult.bev = bev_proc(&tensorParms, intrins_new_list[0], rots_new_list[0]);
   for (int view_id = 0; view_id < NUM_CAM; view_id++)
   {
      TImgResult imgResult = projection(&tensorParms, intrins_new_list[view_id],
                                        trans_new_list[view_id], rots_new_list[view_id], view_id, 0,
                                        img_imshow_lists[view_id], xuan_new_list[view_id],
                                        jibian_new_list[view_id], xi_new_list[view_id]);
      imgResult.id = view_id;
      detResult.img.push_back(imgResult);
   }
   return detResult;
}

int post_init(TCommBuff tGridFile)
{
   float *ptGrid = (float *)tGridFile.ptData;
   int gridSize = tGridFile.s32Size / 4;

   tTensorGrid = CPU(kFloat).tensorFromBlob(ptGrid, {80000, 9});
   return 0;
}

int demo(int argc, char **argv)
{
   api_comm_setWorkSpace();
   TDetResult allChanDetResult;

   TCommBuff tBinFile[3] = {0};
   tBinFile[0] = api_comm_readFileToBuff("./1.bin");
   tBinFile[1] = api_comm_readFileToBuff("./2.bin");
   tBinFile[2] = api_comm_readFileToBuff("./3.bin");
   float *ptF32ScoreBuff = (float *)tBinFile[0].ptData;
   int scoreSize = tBinFile[0].s32Size / 4;
   float *ptF32BboxBuff = (float *)tBinFile[1].ptData;
   int bboxSize = tBinFile[1].s32Size / 4;
   float *ptF32ClassBuff = (float *)tBinFile[2].ptData;
   int classSize = tBinFile[2].s32Size / 4;

   TCommBuff tGridFile = api_comm_readFileToBuff("grid.bin");
   post_init(tGridFile);

#if 0 // for test
    std::string imgdir0 = "./a.jpg";
    std::string imgdir1 = "./b.jpg";
    std::string imgdir2 = "./c.jpg";
    std::string imgdir3 = "./d.jpg";
    std::string imgdir4 = "./f.jpg";
    std::string imgdir5 = "./g.jpg";

    cv::Mat frame0, frame1, frame2, frame3, frame4, frame5;
    std::vector<cv::Mat> img_lists(NUM_CAM);
    frame0 = cv::imread(imgdir0);
    frame1 = cv::imread(imgdir1);
    frame2 = cv::imread(imgdir2);
    frame3 = cv::imread(imgdir3);
    frame4 = cv::imread(imgdir4);
    frame5 = cv::imread(imgdir5);    
    img_lists[0] = frame0;
    img_lists[1] = frame1;
    img_lists[2] = frame2;
    img_lists[3] = frame3;
    img_lists[4] = frame4;
    img_lists[5] = frame5;  

    allChanDetResult = post_process_test(img_lists, ptF32ScoreBuff, ptF32BboxBuff, ptF32ClassBuff);
#else
   allChanDetResult = post_process(ptF32ScoreBuff, ptF32BboxBuff, ptF32ClassBuff);
#endif

   return 0;
}