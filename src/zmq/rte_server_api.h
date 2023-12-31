/**											
 *    @file     rte_server_api.h							
 *    @brief    实现RTE server接口
 * 
 *    MaYelong  Create in V1.0.0 [2023/2/9]						
 *    @li 创建文件
 *											
 *    @author   MaYelong								
 *    @version  1.0.0								
 *    @date     2023/2/9									
 *    @copyright (C) Copyright 2023, 武汉极目智能技术有限公司			
 */
#ifndef _RTE_SERVER_API_H_
#define _RTE_SERVER_API_H_

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

/*
********************************************************************************
* 定义模块接口函数
********************************************************************************
*/
/**
* @brief 启动RTE SERVER
* @return 执行结果
* @retval 0： 成功
* @retval -1：失败
* @note 函数是阻塞函数，启动后一直运行，直到进程结束
*/
s32 RTE_API_ServerStart();

#ifdef __cplusplus
}
#endif


#endif  /* end of _RTE_SERVER_API_H_ */

