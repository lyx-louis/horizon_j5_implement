/**
 * @file comm_api.cpp
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

# include "comm_api.h"
# include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/stat.h>

/**
 * @brief : 设置工作空间路径为可执行文件所在路径
 */
void api_comm_setWorkSpace(void)
{
	char link[128] = {0};
	char path[128] = {0};

	// 命令输入时所在路径
    char *res = getcwd(path, sizeof(path));
    snprintf(link, sizeof(link), "/proc/%d/exe", getpid());

    // 可执行文件所在路径
    int ret = readlink(link, path, sizeof(path));
	if (ret > 0) {
    	path[ret] = '\0';
	} else {
		printf("Err, readlink fail\n");
		return;
	}

	// 切割掉路径最后一个'/'及其后面内容
	char *ptName = strrchr(path, '/');
	if (ptName != NULL) {
		int s32PathLen = (int)(ptName - path);
		char dir[128];
		snprintf(dir, s32PathLen+1, "%s", path);
		int ret = chdir(dir);
		printf("change work path : %s\n", dir);
	} else {
		printf("Err, strrchr fail\n");
	}
}

/**
 * @brief : 获取系统时间
 * @return double : 返回系统时间 (单位:ms)
 */
double api_comm_getSysMs(void)
{
	struct timespec tTimespec;

	clock_gettime(CLOCK_MONOTONIC, &tTimespec);

	return tTimespec.tv_sec * 1000. + tTimespec.tv_nsec / 1000000.;
}

/**
 * @brief : 设定延时时长
 * @param s32DelayMs : 延时时间 (单位:ms)
 */
void api_comm_delayMs(int s32DelayMs)
{
	struct timeval tDelay;
	
	tDelay.tv_sec = s32DelayMs / 1000;
	tDelay.tv_usec = (s32DelayMs % 1000 ) * 1000;

	select(0, NULL, NULL, NULL, &tDelay);
}

/**
 * @brief : 创建线程
 * @param start_routine : 注册函数
 * @param arg : 传参
 * @return int : 成功:0
 */
int api_comm_pthreadCreat(void *(*start_routine)(void*), void *arg)
{
	pthread_t pid;

	int ret = pthread_create(&pid, NULL, start_routine, arg);
	if(ret != 0) {
		log_color_err("pthread_create fail");
	}
	pthread_detach(pid);

	return ret;
}

/**
 * @brief : 写文件
 * @param ptPath : 文件路径
 * @param ptData : 数据内容
 * @param dataLen : 有效数据长度
 */
void api_comm_writeToFile(char* ptPath, char* ptData, int dataLen)
{
    if (ptPath == NULL || ptData == NULL)
        return;

    FILE *fp = fopen(ptPath, "wb");
    if (NULL == fp) {
        log_color_err("fopen file %s err !\n", ptPath);
        return;
    }

    fwrite(ptData, 1, dataLen, fp);
    fclose(fp);
	log_color_debug("save:%s, len:%d", ptPath, dataLen);
}

/**
 * @brief : 读文件
 * @param ptPath : 文件路径
 * @return TCommBuff : 返回文件数据
 */
TCommBuff api_comm_readFileToBuff(const char* ptPath)
{
	TCommBuff tOut = {0};

    if (ptPath == NULL) {
		log_err("ptPath is null");
        return tOut;
	}

    if (access(ptPath, F_OK) != 0) {
		log_err("access %s fail", ptPath);
        return tOut;
    }

    struct stat statbuf;
    stat((char*)ptPath, &statbuf);
    if(statbuf.st_size > 0) {
        FILE* fp = fopen(ptPath, "rb");
        if (NULL == fp) {
            log_err("fopen file %s err !", ptPath);
            return tOut;
        }

		tOut.s32Size = statbuf.st_size;
		tOut.ptData = (unsigned char *) malloc(tOut.s32Size);

		if (tOut.ptData != NULL) {
        	size_t ret = fread(tOut.ptData, 1, tOut.s32Size, fp);
		} else {
			log_err("malloc fail");
		}
		
        fclose(fp);
    }

    return tOut;
}
