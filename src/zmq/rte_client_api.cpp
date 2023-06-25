/**											
 *    @file     rte_client_api.cpp							
 *    @brief    实现RTE消息的收发接口	
 * 
 *    MaYelong  Create in V1.0.0 [2023/2/9]						
 *    @li 创建文件
 *											
 *    @author   MaYelong								
 *    @version  1.0.0								
 *    @date     2023/2/9									
 *    @copyright (C) Copyright 2023, 武汉极目智能技术有限公司			
 */
#include "rte_client_api.h"
#include "zmq.h"

#include <map>
#include <vector>
#include <pthread.h>
#include <string>

using namespace std;


class RTE_RecvClient;
class RTE_SendClient;

/** 接收rte消息句柄列表，每个线程绑定一个接收client */
static map<pthread_t, RTE_RecvClient*> g_rte_recv_client_list;    

/** 发送rte消息句柄列表，每个线程绑定一个发送client */
static map<pthread_t, RTE_SendClient*> g_rte_send_client_list;   

class RTE_SendClient
{
public:
	RTE_SendClient()
	{
		m_pSendCtx = NULL;
		m_pSendSock = NULL;
	}

	~RTE_SendClient()
	{
	    if(NULL != m_pSendSock)
		{
		    zmq_close(m_pSendSock);
            m_pSendSock = NULL;
        }
		
        if(NULL != m_pSendCtx)
		{
		    zmq_ctx_destroy(m_pSendCtx);
            m_pSendCtx = NULL;
        }
	}

	s32 SendMsg(u32 msgId, s8* pData, u32 dataLen)
	{
		if (m_pSendSock == NULL)
		{
			return -1;
		}
	
		//add topic
		if (dataLen + sizeof(msgId) > RTE_MSG_MAX_LENGTH)
		{
			LOG_ERROR("send size too long,dataLen:%u\n", dataLen);
			return -1;
		}

		zmq_send(m_pSendSock, &msgId, sizeof(u32), ZMQ_SNDMORE);   //ZMQ_DONTWAIT|ZMQ_SNDMORE
		if (zmq_send(m_pSendSock, pData, dataLen, 0) < 0)
		{
			LOG_ERROR("zmq_send fail:%s\n",zmq_strerror(errno));
			return -1;
		}
		
		return 0;
	}

	s32 Init(const char *ip)
	{
		if (ip != NULL) {
			char addr[128];
			snprintf(addr, sizeof(addr), "tcp://%s:7765", ip);
			m_sendAddr = addr;
		}

		m_pSendCtx = zmq_ctx_new();
		if (NULL == m_pSendCtx)
		{
			LOG_ERROR("RTE_SendClient zmq_ctx_new fail:%s\n",zmq_strerror(errno));
			return -1;
		}
		
		m_pSendSock = zmq_socket(m_pSendCtx, ZMQ_PUB);
		if (NULL == m_pSendSock)
		{
			LOG_ERROR("RTE_SendClient zmq_socket fail:%s\n",zmq_strerror(errno));
            zmq_ctx_destroy(m_pSendCtx);  //m_pSendCtx must be exist
            m_pSendCtx = NULL;
			return -1;
		}
		if (zmq_connect(m_pSendSock, m_sendAddr.empty() ? m_pSendAddr : m_sendAddr.data()) < 0)
		{
			LOG_ERROR("RTE_SendClient zmq_connect fail:%s\n",zmq_strerror(errno));
		    zmq_close(m_pSendSock);       //m_pSendCtx must be exist
		    m_pSendSock = NULL;
		    zmq_ctx_destroy(m_pSendCtx);  //m_pSendCtx must be exist
		    m_pSendCtx = NULL;
			return -1;
		}

		LOG_DEBUG("RTE_SendClient Init success\n");
		
		return 0;
	}

private:
	void* m_pSendCtx;
	void* m_pSendSock;
	std::string m_sendAddr;
	const char* m_pSendAddr = "tcp://127.0.0.1:7765";
};

class RTE_RecvClient
{
public:
	RTE_RecvClient()
	{
		m_pRecvCtx = NULL;
		m_pRecvSock = NULL;
	}

	~RTE_RecvClient()
	{
	    if(NULL != m_pRecvSock)
		{
		    zmq_close(m_pRecvSock);
            m_pRecvSock = NULL;
        }

		if(NULL != m_pRecvCtx)
		{
		    zmq_ctx_destroy(m_pRecvCtx);
            m_pRecvCtx = NULL;
        }
	}

	s32 Init(const char *ip, const vector<u32> &sub_topics, const RTE_API_HandleMsg cbhandle, void *cbarg)
	{
		if (ip != NULL) {
			char addr[128];
			snprintf(addr, sizeof(addr), "tcp://%s:7766", ip);
			m_sendAddr = addr;
		}
		
		m_pRecvCtx = zmq_ctx_new();
		if (NULL == m_pRecvCtx)
		{
			LOG_ERROR("RTE_RecvClient zmq_ctx_new fail:%s\n",zmq_strerror(errno));
			return -1;
		}

		//init zmq socket 
		m_pRecvSock = zmq_socket(m_pRecvCtx, ZMQ_SUB);
		if (NULL == m_pRecvSock)
		{
			LOG_ERROR("RTE_RecvClient zmq_socket fail:%s\n",zmq_strerror(errno));
		    zmq_ctx_destroy(m_pRecvCtx);
	        m_pRecvCtx = NULL;
			return -1;
		}

		if (zmq_connect(m_pRecvSock, m_sendAddr.empty() ? m_pRecvAddr : m_sendAddr.data()) < 0)
		{
			LOG_ERROR("RTE_RecvClient zmq_connect fail:%s\n",zmq_strerror(errno));
		    zmq_close(m_pRecvSock);
	        m_pRecvSock = NULL;
		    zmq_ctx_destroy(m_pRecvCtx);
	        m_pRecvCtx = NULL;
			return -1;
		}

		for (auto it : sub_topics)
		{
			u32 topic_id = it;
			if (zmq_setsockopt(m_pRecvSock, ZMQ_SUBSCRIBE, &topic_id, sizeof(topic_id)) < 0)
			{
				LOG_ERROR("RTE_RecvClient zmq_setsockopt fail:%s\n",zmq_strerror(errno));
		    	zmq_close(m_pRecvSock);
	            m_pRecvSock = NULL;
	    	    zmq_ctx_destroy(m_pRecvCtx);
	            m_pRecvCtx = NULL;
				return -1;
			}
		}		

		m_cbarg = cbarg;
		m_cbhandle = cbhandle;

		LOG_DEBUG("RTE_RecvClient Init success\n");
		
		return 0;
	}

	s32 RecvMsg()
	{
		if (m_pRecvSock == NULL)
		{
			return -1;
		}
		
		u32 topic_id = 0; 
		s8 szMsg[RTE_MSG_MAX_LENGTH] = {0};
		
		while (zmq_recv(m_pRecvSock, &topic_id, sizeof(topic_id), 0) < 0)
		{
			continue;
		}
		
		s32 size = zmq_recv(m_pRecvSock, szMsg, sizeof(szMsg), 0); 
		if (size < 0)
		{
			LOG_ERROR("zmq_recv fail:%s\n",zmq_strerror(errno));
		}
		else
		{
			if (m_cbhandle)
			{
				m_cbhandle(topic_id, szMsg, size, m_cbarg);
			}
		}
		
		return 0;
	}

private:
	void* m_pRecvCtx;
	void* m_pRecvSock;
	std::string m_sendAddr;
	const char* m_pRecvAddr = "tcp://127.0.0.1:7766";
	void *m_cbarg;
	RTE_API_HandleMsg m_cbhandle;
};


/**
* @brief 创建RTE消息发送句柄
* @param
* @return 执行结果
* @retval 0 成功
* @retval -1 失败
* @note   client和线程绑定，每个线程一个client
*/
s32 RTE_API_CreateSendClient()
{
	return RTE_API_CreateSendAddr(NULL);
}

/**
 * @brief : 创建RTE消息发送句柄
 * @param addr : IP 地址, NULL:使用本机IP
 * @return s32 : 成功:0
 * @note   client和线程绑定，每个线程一个client
 */
s32 RTE_API_CreateSendAddr(const char *addr)
{
	pthread_t tid = pthread_self();
	if (g_rte_send_client_list.find(tid) != g_rte_send_client_list.end())
	{
		LOG_DEBUG("client already exist, tid:%ld\n", tid);
		return 0;
	}
	
	RTE_SendClient *pSendClient = new RTE_SendClient();
	
	s32 ret = pSendClient->Init(addr);

	g_rte_send_client_list[tid] = pSendClient;
	
	LOG_DEBUG("RTE_API_CreateSendClient tid:%u\n", (u32)tid);

	return ret;
}

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
s32 RTE_API_SendMsg(u32 topic, s8* pData, u32 dataLen)
{
	s32 nRet = -1;
	/* 4字节topic */
	if ((dataLen + 4) > RTE_MSG_MAX_LENGTH)
	{
		LOG_ERROR("send data too long, datalen:%u\n", dataLen);
		return -1;
	}

    auto tid = pthread_self();
	auto clientIt = g_rte_send_client_list.find(tid);
	if (clientIt != g_rte_send_client_list.end())
	{
		RTE_SendClient *pSendClient = clientIt->second;
		if (pSendClient)
		{
			nRet = pSendClient->SendMsg(topic, pData, dataLen);
		}
	}

	return nRet;
}

/**
* @brief  销毁RTE发送句柄
* @param  
* @return 执行结果
* @retval 0 成功
* @retval -1 失败
* @note   
*/
s32 RTE_API_DestroySendClient()
{
	for (auto &it : g_rte_send_client_list) 
	{
		if (it.second)
		{
        	delete it.second;
		}
    }

	return 0;
}

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
s32 RTE_API_CreateRecvClient(const u32 *pTopic, u32 topic_num, const RTE_API_HandleMsg cbhandle, void *arg)
{
	return RTE_API_CreateRecvAddr(NULL, pTopic, topic_num, cbhandle, arg);
}

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
s32 RTE_API_CreateRecvAddr(const char *addr, const u32 *pTopic, u32 topic_num, const RTE_API_HandleMsg cbhandle, void *arg)
{
	if ((NULL == pTopic) || (0 == topic_num))
	{
		LOG_ERROR("invalid parameter\n");
		return -1;
	}
	
	pthread_t tid = pthread_self();
	if (g_rte_recv_client_list.find(tid) != g_rte_recv_client_list.end())
	{
		LOG_DEBUG("client already exist, tid:%ld\n", tid);
		return 0;
	}

	vector<u32> sub_topics;
	for (u32 i = 0; i < topic_num; i++)
	{
		sub_topics.push_back(*pTopic);
		pTopic++;
	}
	
	RTE_RecvClient* pRecvClient = new RTE_RecvClient();
	s32 ret = pRecvClient->Init(addr, sub_topics, cbhandle, arg);
	g_rte_recv_client_list[tid] = pRecvClient;

	LOG_DEBUG("RTE_API_CreateRecvClient tid:%u\n", (u32)tid);

	return ret;
}

/**
* @brief 接收RTE消息
* @param
* @return 执行结果
* @retval 0 成功
* @retval -1 失败
* @note
*/
s32 RTE_API_RecvMsg()
{
	s32 ret = -1;
	pthread_t tid = pthread_self();

	auto clientIt = g_rte_recv_client_list.find(tid);
	if (clientIt != g_rte_recv_client_list.end())	
	{
		RTE_RecvClient* pRecvClient = clientIt->second;
		if (pRecvClient)
		{
			ret = pRecvClient->RecvMsg();
		}
	}

	return ret;
}

/**
* @brief 销毁RTE接收句柄
* @param
* @return 执行结果
* @retval 0 成功
* @retval -1 失败
* @note
*/
s32 RTE_API_DestroyRecvClient()
{
	for (auto &it : g_rte_recv_client_list) 
	{
		if (it.second)
		{
        	delete it.second;
		}
    }

	return 0;
}

