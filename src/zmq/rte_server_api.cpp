/**											
 *    @file     rte_server_api.cpp								
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
#include "zmq.h"
#include "rte_server_api.h"

/**
* @brief 启动RTE SERVER
* @return 执行结果
* @retval 0： 成功
* @retval -1：失败
* @note 函数是阻塞函数，启动后一直运行，直到进程结束
*/
s32 RTE_API_ServerStart()
{
	void *contex = zmq_ctx_new();
	void *frontend = zmq_socket(contex, ZMQ_XSUB);
	if (NULL == frontend)
	{
		return -1;
	}
	
	void *backend = zmq_socket(contex, ZMQ_XPUB);
	if (NULL == backend)
	{
		return -1;
	}
	
	s32 ret = zmq_bind(frontend,"tcp://*:7765");
	if (ret < 0)
	{
		return -1;
	}
	
	ret = zmq_bind(backend,"tcp://*:7766");
	if (ret < 0)
	{
		return -1;
	}
	
    zmq_proxy(frontend, backend, NULL);

	zmq_close(frontend);
	zmq_close(backend);
	zmq_ctx_destroy(contex);
	
	return 0;
}

