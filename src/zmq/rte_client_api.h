/**											
 *    @file     rte_client_api.h								
 *    @brief    实现RTE消息的收发接口	
 * 
 *    MaYelong  Create in V1.0.0 [2023/2/9]						
 *    @li 创建文件
 *    MaYelong  change in V1.0.1 [2023/2/10]	
 *    @li 修改注释格式
 *											
 *    @author   MaYelong								
 *    @version  1.0.1									
 *    @date     2023/2/10									
 *    @copyright (C) Copyright 2023, 武汉极目智能技术有限公司			
 */
#ifndef _RTE_CLIENT_API_H_
#define _RTE_CLIENT_API_H_

#include "oal_type.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
********************************************************************************
* 定义模块宏和常量
********************************************************************************
*/
/** RTE消息最大长度 */
#define RTE_MSG_MAX_LENGTH   (1 * 1024 * 1024u)

#define LOG_DEBUG   printf
#define LOG_WARN    printf
#define LOG_ERROR   printf

/*
********************************************************************************
* 定义模块枚举类型
********************************************************************************
*/

/*
********************************************************************************
* 定义模块数据结构
********************************************************************************
*/

/** RTE接收消息的回调函数结构 */
typedef s32 (*RTE_API_HandleMsg)(u32 topic, const s8* pData, u32 dataLen, void *arg);

/*
********************************************************************************
* 定义模块接口函数
********************************************************************************
*/
/**
* @brief 创建RTE消息接收句柄
* @param [in] pTopic:      订阅topic列表首地址
* @param [in] topic_num:   topic个数
* @param [in] cbhandle:    回调函数
* @return 执行结果
* @retval 0 成功
* @retval -1 失败
* @note client和线程绑定，每个线程一个client
*/
s32 RTE_API_CreateRecvClient(const u32 *pTopic, u32 topic_num, const RTE_API_HandleMsg cbhandle, void *arg);

/**
 * @brief : 创建RTE消息接收句柄
 * @param addr : IP 地址, NULL:使用本机IP
 * @param pTopic : 订阅topic列表首地址
 * @param topic_num : topic个数
 * @param cbhandle : 回调函数
 * @param arg : 传参
 * @return s32 : 成功:0
 * @note client和线程绑定，每个线程一个client
 */
s32 RTE_API_CreateRecvAddr(const char *addr, const u32 *pTopic, u32 topic_num, const RTE_API_HandleMsg cbhandle, void *arg);

/**
* @brief 接收RTE消息
* @param
* @return 执行结果
* @retval 0 成功
* @retval -1 失败
* @note
*/
s32 RTE_API_RecvMsg();


/**
* @brief 销毁RTE接收句柄
* @param
* @return int32_t
* @retval 0 成功
* @retval -1 失败
* @note
*/
s32 RTE_API_DestroyRecvClient();

/**
* @brief 创建RTE消息发送句柄
* @param
* @return 执行结果
* @retval 0 成功
* @retval -1 失败
* @note   client和线程绑定，每个线程一个client
*/
s32 RTE_API_CreateSendClient();

/**
 * @brief : 创建RTE消息发送句柄
 * @param addr : IP 地址, NULL:使用本机IP
 * @return s32 : 成功:0
 * @note   client和线程绑定，每个线程一个client
 */
s32 RTE_API_CreateSendAddr(const char *addr);

/**
* @brief   发送RTE消息
* @param   [in]  topic:      发送消息topic
* @param   [in]  pData:      发送数据指针
* @param   [in]  dataLen:    发送数据长度
* @return 执行结果
* @retval 0 成功
* @retval -1 失败
* @note   
*/
s32 RTE_API_SendMsg(u32 topic, s8* pData, u32 dataLen);

/**
* @brief  销毁RTE发送句柄
* @param  
* @return 执行结果
* @retval 0 成功
* @retval -1 失败
* @note   
*/
s32 RTE_API_DestroySendClient();

#ifdef __cplusplus
}
#endif

#endif     /* end of _RTE_CLIENT_H_ */

