#include "projection.h"

int post_init(TCommBuff tGridFile);
TDetResult post_process_test(std::vector<cv::Mat> img_lists, float *ptF32ScoreBuff, float *ptF32BboxBuff, float *ptF32ClassBuff);
TDetResult post_process(float *ptF32ScoreBuff, float *ptF32BboxBuff, float *ptF32ClassBuff);