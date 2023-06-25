/**
 * @file comm_api.h
 * @brief : 公共 API
 * 
 * @author liuzhiquan (liuzhiquan@jmadas.com)
 * @version 0.1
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023, 武汉极目智能技术有限公司
 * 
 * @par 修改日志
 * 
 */

#ifndef __COMM_API_H__
#define __COMM_API_H__

// #include "utl_log_api.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_IMAGE_WIDTH		(1920)	// 设定图像最大分辨率
#define MAX_IMAGE_HEIGHT	(1088)	// 设定图像最大分辨率

#ifndef log_trace
#define log_trace(fmt, ...) printf("[TRACE] \033[1;30m" fmt "\033[0m", ##__VA_ARGS__);
#endif

#ifndef log_debug
#define log_debug(fmt, ...) printf("[DEBUG] \033[1;32m" fmt "\033[0m", ##__VA_ARGS__);
#endif

#ifndef log_info
#define log_info(fmt, ...) printf("[INFO] \033[1;34m" fmt "\033[0m", ##__VA_ARGS__);
#endif

#ifndef log_warn
#define log_warn(fmt, ...) printf("[WARN] \033[1;33m" fmt "\033[0m", ##__VA_ARGS__);
#endif

#ifndef log_err
#define log_err(fmt, ...) printf("[ERR] \033[1;31m" fmt "\033[0m", ##__VA_ARGS__);
#endif

#ifndef log_color_trace
#define log_color_trace(fmt, ...) printf("[TRACE] \033[1;30m" fmt "\033[0m", ##__VA_ARGS__);
#endif

#ifndef log_color_debug
#define log_color_debug(fmt, ...) printf("[DEBUG] \033[1;32m" fmt "\033[0m", ##__VA_ARGS__);
#endif

#ifndef log_color_info
#define log_color_info(fmt, ...) printf("[INFO] \033[1;34m" fmt "\033[0m", ##__VA_ARGS__);
#endif

#ifndef log_color_warn
#define log_color_warn(fmt, ...) printf("[WARN] \033[1;33m" fmt "\033[0m", ##__VA_ARGS__);
#endif

#ifndef log_color_err
#define log_color_err(fmt, ...) printf("[ERR] \033[1;31m" fmt "\033[0m", ##__VA_ARGS__);
#endif

/**
 * @brief : 设置工作空间路径为可执行文件所在路径
 */
void api_comm_setWorkSpace(void);

/**
 * @brief : 获取系统时间
 * @return double : 返回系统时间 (单位:ms)
 */
double api_comm_getSysMs(void);

/**
 * @brief : 设定延时时长
 * @param s32DelayMs : 延时时间 (单位:ms)
 */
void api_comm_delayMs(int s32DelayMs);

/**
 * @brief : 创建线程
 * @param start_routine : 注册函数
 * @param arg : 传参
 * @return int : 成功:0
 */
int api_comm_pthreadCreat(void *(*start_routine)(void*), void *arg);

/**
 * @brief : 写文件
 * @param ptPath : 文件路径
 * @param ptData : 数据内容
 * @param dataLen : 有效数据长度
 */
void api_comm_writeToFile(char* ptPath, char* ptData, int dataLen);

typedef struct {
	unsigned char *ptData;
	int s32Size;
} TCommBuff;

/**
 * @brief : 读文件
 * @param ptPath : 文件路径
 * @return TCommBuff : 返回文件数据
 */
TCommBuff api_comm_readFileToBuff(const char* ptPath);

#ifdef __cplusplus
}
#endif

#endif
