/***************************************************************
模块名	     ：socket
文件名	     ：socket_c.h
相关文件	 ：
文件实现功能 ：epoll
作者		 ：xuduan
版本		 ：V1.0
-----------------------------------------------------------------
修改记录:
日  期		    版本		修改人		修改内容
2013/03/18		1.0          xuduan     初始版本
*****************************************************************/

#include "ipc_socket.h"

/*C++标准库头文件*/
#ifdef __cplusplus
#include <string>
#include <map>
#include <vector>
#include <set>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <fstream>
#include <sstream>
#include <queue>
#include <iomanip>
#include <thread>
#include <memory>
#include <list>
#include <algorithm>
#include <cstdlib>
#include <csignal>
#endif

#define DEFAULT_LISTEN_NUM 5

#define sprintf_s snprintf

/******************************************************************************************
函数名称：Oal_SocketCreate
函数说明：创建socket
输入参数：s32Protocol区分udp与tcp
输出参数：无。
返 回 值：	成功，则返回socket值，失败，则返回OAL_FAILURE。
*******************************************************************************************/
SOCKET Oal_SocketCreate(ESocketType eSocketType)
{
	SOCKET s32Socket = 0;
	int s32StreamType = 0;
	int s32Protocol = 0;

#if (defined(WIN32) || defined(WIN64))
	WORD wVer;
	WSADATA wsaData;

	wVer = MAKEWORD(1, 1);
	if (WSAStartup(wVer, &wsaData) != SUCCESS)
	{
		return FAILURE;
	}
#endif

#ifndef USE_IPV6
	if (eSocketType == SOCKET_UDP)
	{
		s32StreamType = SOCK_DGRAM;
		s32Protocol = IPPROTO_UDP;
	}
	else
	{
		s32StreamType = SOCK_STREAM;
		s32Protocol = IPPROTO_TCP;
	}
#endif

	s32Socket = socket(AF_INET, s32StreamType, s32Protocol);

	return s32Socket;
}

/******************************************************************************************
函数名称：Oal_SocketClose
函数说明：关闭socket
输入参数：s32Socket值
输出参数：无。
返 回 值：	成功，则返回socket值，失败，则返回OAL_FAILURE。
*******************************************************************************************/
int Oal_SocketClose(SOCKET s32Socket)
{
#if (defined(WIN32) || defined(WIN64)) 
	closesocket(s32Socket);
#else 
	close(s32Socket);
#endif
	return SUCCESS;
}

/******************************************************************************************
函数名称：Oal_SocketBind
函数说明：绑定socket
输入参数：ps8Ip 绑定的ip ，u16Port绑定端口
输出参数：无。
返 回 值：	成功，则返回SUCCESS，失败，则返回OAL_FAILURE。
*******************************************************************************************/
int Oal_SocketBind(SOCKET s32Socket, char* ps8Ip, unsigned short u16Port)
{
	struct sockaddr_in tMyAddr;

	if (ps8Ip == NULL)
	{
		return FAILURE;
	}
	/*Add your code here*/
	memset(&tMyAddr, 0, sizeof(struct sockaddr_in));
	tMyAddr.sin_family = AF_INET;
	tMyAddr.sin_port = htons(u16Port);
	tMyAddr.sin_addr.s_addr = inet_addr(ps8Ip);

	return bind(s32Socket, (struct sockaddr*) & tMyAddr, sizeof(struct sockaddr_in));
}

/******************************************************************************************
函数名称：Oal_SocketSelect
函数说明：等待可读写函数,针对单一的socket设计
输入参数：
输出参数：无。
返 回 值：	成功，则返OAL_SUCCESS，失败，则返回OAL_FAILURE。
*******************************************************************************************/
int Oal_SocketSelect(SOCKET s32Socket, struct timeval* ptWaitTime, int s32Flag)
{
	fd_set tRdSet;
	fd_set tWdSet;
	fd_set tErSet;
	int s32Nfds = 0;
	int s32Max = 0;

	if (ptWaitTime == NULL)
	{
		return FAILURE;
	}
	FD_ZERO(&tRdSet);
	FD_ZERO(&tWdSet);
	FD_ZERO(&tErSet);

	if ((s32Flag & FD_SET_READ) != 0)
	{
		FD_SET(s32Socket, &tRdSet);
	}
	if ((s32Flag & FD_SET_WRITE) != 0)
	{
		FD_SET(s32Socket, &tWdSet);
	}
	if ((s32Flag & FD_SET_EXCEPT) != 0)
	{
		FD_SET(s32Socket, &tErSet);
	}
	s32Flag = 0;
	s32Max = ((int)s32Socket) + 1;
	s32Nfds = select(s32Max, &tRdSet, &tWdSet, &tErSet, ptWaitTime);
	if (s32Nfds > 0)
	{
		if (FD_ISSET(s32Socket, &tRdSet))
		{
			s32Flag |= FD_SET_READ;
		}
		if (FD_ISSET(s32Socket, &tWdSet))
		{
			s32Flag |= FD_SET_WRITE;
		}
		if (FD_ISSET(s32Socket, &tErSet))
		{
			s32Flag |= FD_SET_EXCEPT;
		}
	}
	return s32Flag;

}

/******************************************************************************************
函数名称：Oal_SocketSetAddrReuse
函数说明：设置端口为可复用端口
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。

输出参数：无。
返回值：	成功，则返回SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSetAddrReuse(SOCKET s32Socket)
{
	int s32Tmp = 1;

	return setsockopt(s32Socket, SOL_SOCKET, SO_REUSEADDR, (const char*)&s32Tmp, sizeof(int));
}

/******************************************************************************************
函数名称：Oal_SocketSetNoDelay
函数说明：设置TCP模式下无时延传输
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。

输出参数：无。
返回值：	成功，则返回SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSetNoDelay(SOCKET s32Socket)
{
	int s32Tmp = 1;

#if (defined(WIN32) || defined(WIN64))
	setsockopt(s32Socket, IPPROTO_TCP, TCP_NODELAY, (const char*)&s32Tmp, sizeof(s32Tmp));
#else
	setsockopt(s32Socket, IPPROTO_TCP, TCP_NODELAY, (void*)&s32Tmp, sizeof(s32Tmp));
#endif
	return 0;
}

/******************************************************************************************
函数名称：Oal_SocketSetNonBlock
函数说明：设置非阻塞方式
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。
输出参数：
返 回 值：	成功，则返回请求信息类型的个数，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSetNonBlock(SOCKET s32Socket)
{
	int s32Result = 0;

#if (defined(WIN32) || defined(WIN64)) 
	u32 u32Value = 1;
	s32Result = ioctlsocket(s32Socket, FIONBIO, (unsigned long*)&u32Value);

#else 
	s32Result = fcntl(s32Socket, F_SETFL, O_NONBLOCK);
#endif
	return s32Result;
}

/******************************************************************************************
函数名称：Oal_SocketListen
函数说明：设置socket监听
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。
输出参数：
返 回 值：	成功，则返回请求信息类型的个数，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketListen(SOCKET s32Socket)
{
	return listen(s32Socket, DEFAULT_LISTEN_NUM);
}

/******************************************************************************************
函数名称：Oal_SocketAccept
函数说明：接受远端客户端的链接。
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。
输出参数：无。
返 回 值：	成功，则返回一个新的TSocket ID用来和建立连接的客户端进行通信，失败,则返回FAILURE。
*******************************************************************************************/
SOCKET Oal_SocketAccept(SOCKET s32Socket, char* ps8Ip, unsigned short* pu16Port)
{
	struct sockaddr_in tRemoteAddr;
	socklen_t tSockAddrLen = 0;
	SOCKET s32AcpSock = 0;

	memset(&tRemoteAddr, 0, sizeof(struct sockaddr_in));
	tSockAddrLen = sizeof(struct sockaddr_in);

	s32AcpSock = accept(s32Socket, (struct sockaddr*) & tRemoteAddr, &tSockAddrLen);
	if (s32AcpSock == INVALID_SOCKET)
	{
		return FAILURE;
	}
	if (ps8Ip != NULL)
	{
		strncpy(ps8Ip, inet_ntoa(tRemoteAddr.sin_addr), IP_LEN);
	}
	if (pu16Port != NULL)
	{
		*pu16Port = ntohs(tRemoteAddr.sin_port);
	}

	return s32AcpSock;
}

/******************************************************************************************
函数名称：Oal_SocketAccept
函数说明：接受远端客户端的链接。
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。
输出参数：无。
返 回 值：	成功，则返回SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketConnect(SOCKET s32Socket, char* ps8Ip, unsigned short u16Port)
{
	struct sockaddr_in tDstAddr;

	tDstAddr.sin_port = htons(u16Port);
	tDstAddr.sin_addr.s_addr = inet_addr(ps8Ip);
	tDstAddr.sin_family = AF_INET;
	return connect(s32Socket, (struct sockaddr*) & tDstAddr, sizeof(tDstAddr));
}

void AddTimeVal(struct timeval* ptTimeVal, int s32Ms)
{
	int s32Tmp = 0;

	if (ptTimeVal == NULL)
	{
		return;
	}
	if (s32Ms >= 1000000)
	{
		ptTimeVal->tv_usec = 0;
		s32Tmp = s32Ms / 1000;
	}
	else
	{
		ptTimeVal->tv_usec += s32Ms * 1000;
		s32Tmp = ptTimeVal->tv_usec / 1000000;
		ptTimeVal->tv_usec = ptTimeVal->tv_usec % 1000000;
	}
	ptTimeVal->tv_sec += s32Tmp;
}

/******************************************************************************************
函数名称：Oal_SocketSendTo
函数说明： udp发送数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
u16DstPort目的端口 s32WaitTime等待时间，对于非阻塞方式使用
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSendTo(SOCKET s32Socket, void* ptBuffer, int s32BufLen, char* ps8DstIp, unsigned short u16DstPort, int s32WaitTime)
{
	struct sockaddr_in tDstAddr;
	struct timeval tTimeOut;
	int s32Flag = 0;
	int s32SndLen = 0;

	if (ps8DstIp == NULL || ptBuffer == NULL)
	{
		return FAILURE;
	}
	tDstAddr.sin_family = AF_INET;
	tDstAddr.sin_port = htons(u16DstPort);
	tDstAddr.sin_addr.s_addr = inet_addr(ps8DstIp);

	memset(&tTimeOut, 0, sizeof(struct timeval));
	AddTimeVal(&tTimeOut, s32WaitTime);

	s32Flag = Oal_SocketSelect(s32Socket, &tTimeOut, FD_SET_WRITE | FD_SET_EXCEPT);
	if ((s32Flag & FD_SET_WRITE) != 0)
	{
		s32SndLen = sendto(s32Socket, (char*)ptBuffer, s32BufLen, 0, (const struct sockaddr*) & tDstAddr, sizeof(tDstAddr));
	}
	return s32SndLen;
}

/******************************************************************************************
函数名称：Oal_SocketSendToEx
函数说明： udp发送数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
u16DstPort目的端口
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSendToEx(SOCKET s32Socket, void* ptBuffer, int s32BufLen, char* ps8DstIp, unsigned short u16DstPort)
{
	struct sockaddr_in tDstAddr;
	int s32SndLen = 0;

	if (ps8DstIp == NULL || ptBuffer == NULL)
	{
		return FAILURE;
	}
	tDstAddr.sin_family = AF_INET;
	tDstAddr.sin_port = htons(u16DstPort);
	tDstAddr.sin_addr.s_addr = inet_addr(ps8DstIp);

	s32SndLen = sendto(s32Socket, (char*)ptBuffer, s32BufLen, 0, (const struct sockaddr*) & tDstAddr, sizeof(tDstAddr));
	return s32SndLen;
}

/******************************************************************************************
函数名称：Oal_SocketSend
函数说明： tcp发送数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
u16DstPort目的端口 s32WaitTime等待时间，对于非阻塞方式使用
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSend(SOCKET s32Socket, void* ptBuffer, int s32BufLen, int s32WaitTime)
{
	struct timeval tTimeOut;
	int s32Flag = 0;
	int s32SndLen = 0;

	if (ptBuffer == NULL)
	{
		return FAILURE;
	}

	memset(&tTimeOut, 0, sizeof(struct timeval));
	AddTimeVal(&tTimeOut, s32WaitTime);

	s32Flag = Oal_SocketSelect(s32Socket, &tTimeOut, FD_SET_WRITE | FD_SET_EXCEPT);
	if ((s32Flag & FD_SET_WRITE) != 0)
	{
		s32SndLen = send(s32Socket, (char*)ptBuffer, s32BufLen, 0);
	}
	return s32SndLen;
}

/******************************************************************************************
函数名称：Oal_SocketSend
函数说明： tcp发送数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
u16DstPort目的端口 s32WaitTime等待时间，对于非阻塞方式使用
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSendEx(SOCKET s32Socket, void* ptBuffer, int s32BufLen)
{
	return send(s32Socket, (char*)ptBuffer, s32BufLen, 0);
}

/******************************************************************************************
函数名称：Oal_SocketSendTo
函数说明： udp接收数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
u16DstPort目的端口 s32WaitTime等待时间，对于非阻塞方式使用
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketRecvFrom(SOCKET s32Socket, void* ptBuffer, int s32BufLen, char* ps8SrcIp, unsigned short* pu16SrcPort, int s32WaitTime)
{
	struct sockaddr_in tDstAddr;
	struct timeval tTimeOut;
	int s32Flag = 0;
	int s32RcvLen = 0;
	socklen_t tAddrLen = sizeof(tDstAddr);

	if (ptBuffer == NULL)
	{
		return FAILURE;
	}
	tDstAddr.sin_family = AF_INET;

	memset(&tTimeOut, 0, sizeof(struct timeval));
	AddTimeVal(&tTimeOut, s32WaitTime);

	s32Flag = Oal_SocketSelect(s32Socket, &tTimeOut, FD_SET_READ | FD_SET_EXCEPT);
	if ((s32Flag & FD_SET_READ) != 0)
	{
		s32RcvLen = recvfrom(s32Socket, (char*)ptBuffer, s32BufLen, 0, (struct sockaddr*) & tDstAddr, &tAddrLen);
		if (s32RcvLen > 0)
		{
			/*获取接收ip和端口*/
			if (ps8SrcIp != NULL)
			{
				strncpy(ps8SrcIp, inet_ntoa(tDstAddr.sin_addr), IP_LEN);
			}
			if (pu16SrcPort != NULL)
			{
				*pu16SrcPort = ntohs(tDstAddr.sin_port);
			}
		}
	}
	return s32RcvLen;
}

/******************************************************************************************
函数名称：Oal_SocketRecvFrom
函数说明： udp接收数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
u16DstPort目的端口
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketRecvFromEx(SOCKET s32Socket, void* ptBuffer, int s32BufLen, char* ps8SrcIp, unsigned short* pu16SrcPort)
{
	// struct sockaddr_in tDstAddr;
	// socklen_t tRcvLen = 0;

	// if (ptBuffer == NULL)
	// {
	// 	return FAILURE;
	// }
	// tDstAddr.sin_family = AF_INET;

	// tRcvLen = recvfrom(s32Socket, (char*)ptBuffer, s32BufLen, 0, (struct sockaddr*) & tDstAddr, &tRcvLen);
	// if (tRcvLen > 0)
	// {
	// 	/*获取接收ip和端口*/
	// 	if (ps8SrcIp != NULL)
	// 	{
	// 		strncpy(ps8SrcIp, inet_ntoa(tDstAddr.sin_addr), IP_LEN);
	// 	}
	// 	if (pu16SrcPort != NULL)
	// 	{
	// 		*pu16SrcPort = ntohs(tDstAddr.sin_port);
	// 	}
	// }

	// return tRcvLen;

	// 修复无法获取正确的 IP　PORT　问题
	struct sockaddr_in tDstAddr;
	int s32RcvLen = 0;
	socklen_t s32AddrLen = sizeof(tDstAddr);

	if (ptBuffer == NULL)
	{
		return FAILURE;
	}
	tDstAddr.sin_family = AF_INET;

	s32RcvLen = recvfrom(s32Socket, ptBuffer, s32BufLen, 0, (struct sockaddr *)&tDstAddr, &s32AddrLen);
	if (s32RcvLen > 0)
	{
		/*获取接收ip和端口*/
		if (ps8SrcIp != NULL)
		{
			strncpy(ps8SrcIp, inet_ntoa(tDstAddr.sin_addr), IP_LEN);
		}
		if (pu16SrcPort != NULL)
		{
			*pu16SrcPort = ntohs(tDstAddr.sin_port);
		}
	}

	return s32RcvLen;
}

/******************************************************************************************
函数名称：Oal_SocketRecv
函数说明： tcp接收数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
		  u16DstPort目的端口 s32WaitTime等待时间，对于非阻塞方式使用
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketRecv(SOCKET s32Socket, void* ptBuffer, int s32BufLen, int s32WaitTime)
{
	struct timeval tTimeOut;
	int s32Flag = 0;
	int s32RcvLen = 0;

	if (ptBuffer == NULL)
	{
		return FAILURE;
	}

	memset(&tTimeOut, 0, sizeof(struct timeval));
	AddTimeVal(&tTimeOut, s32WaitTime);

	s32Flag = Oal_SocketSelect(s32Socket, &tTimeOut, FD_SET_READ | FD_SET_EXCEPT);
	if ((s32Flag & FD_SET_READ) != 0)
	{
		s32RcvLen = recv(s32Socket, (char*)ptBuffer, s32BufLen, 0);
	}
	return s32RcvLen;
}

/******************************************************************************************
函数名称：Oal_SocketRecv
函数说明： tcp接收数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
u16DstPort目的端口 s32WaitTime等待时间，对于非阻塞方式使用
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketRecvEx(SOCKET s32Socket, void* ptBuffer, int s32BufLen)
{
	return recv(s32Socket, (char*)ptBuffer, s32BufLen, 0);
}

/******************************************************************************************
函数名称：IsPrivateIp
函数说明：检测ip是否是内网ip
输入参数：ps8Ip 地址
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_IsPrivateAddr(char* ps8Ip)
{
	int s32Result = FAILURE;
	if (ps8Ip == NULL)
	{
		return FAILURE;
	}
#ifndef USE_IPV6
	if (strncmp(ps8Ip, "10.", 3) == 0)
	{
		s32Result = SUCCESS;
	}
	else if (strncmp(ps8Ip, "192.168.", 8) == 0)
	{
		s32Result = SUCCESS;
	}
	else if (strncmp(ps8Ip, "100.", 4) == 0)
	{
		s32Result = SUCCESS;
	}
	else if ((strncmp(ps8Ip, "172.", 4)) == 0 && (ps8Ip[6] == '.'))/*172.16.x.x至172.31.x.x*/
	{
		char as8Temp[2];
		int s32Value = 0;

		strncpy(as8Temp, ps8Ip + 4, 2);
		s32Value = atoi(as8Temp);
		if (s32Value >= 16 && s32Value <= 31)
		{
			s32Result = SUCCESS;
		}
	}
#else

#endif

	return s32Result;
}

/******************************************************************************************
函数名称：Oal_SocketSetBuffer
函数说明：设置socket缓冲区大小
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。s32BufLen 缓冲区大小
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSetBuffer(SOCKET s32Socket, int s32BufLen, int s32Direction)
{
	int s32OptVal = s32BufLen;
	int s32OptLen = sizeof(int);
	int s32Result = 0;

	if (s32Direction == SOCKET_RECV)
	{
		s32Result = setsockopt(s32Socket, SOL_SOCKET, SO_RCVBUF, (char*)&s32OptVal, s32OptLen);
	}
	else if (s32Direction == SOCKET_SEND)
	{
		s32Result = setsockopt(s32Socket, SOL_SOCKET, SO_SNDBUF, (char*)&s32OptVal, s32OptLen);
	}

	return s32Result;
}

/******************************************************************************************
函数名称：Oal_SocketErrorCode
函数说明：获取错误码
输入参数：
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketErrorCode(void)
{
#if (defined(WIN32) || defined(WIN64))
	return WSAGetLastError();
#else
	return errno;
#endif
}

/******************************************************************************************
函数名称：Oal_SocketSetBroadcast
函数说明：设置广播方式
输入参数：
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSetBroadcast(int s32Socket)
{
	int s32Broadcast = 1;
	return setsockopt(s32Socket, SOL_SOCKET, SO_BROADCAST, (char*)&s32Broadcast, sizeof(int));
}

/******************************************************************************************
函数名称：Oal_SocketSetMemberShip
函数说明：设置组播方式
输入参数：
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSetMemberShip(int s32Socket, char* ps8DstIp)
{
	struct ip_mreq mreq;

	if (ps8DstIp == NULL)
	{
		return FAILURE;
	}
	mreq.imr_multiaddr.s_addr = inet_addr(ps8DstIp);
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	if (setsockopt(s32Socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char*)&mreq, sizeof(mreq)) < 0)
	{
		return FAILURE;
	}
	return SUCCESS;
}

/******************************************************************************************
函数名称：Oal_SocketSetMemberShip
函数说明：设置组播方式
输入参数：
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketDropMemberShip(int s32Socket, char* ps8DstIp)
{
	struct ip_mreq mreq;

	if (ps8DstIp == NULL)
	{
		return FAILURE;
	}
	mreq.imr_multiaddr.s_addr = inet_addr(ps8DstIp);
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	if (setsockopt(s32Socket, IPPROTO_IP, IP_DROP_MEMBERSHIP, (const char*)&mreq, sizeof(mreq)) < 0)
	{
		return FAILURE;
	}
	return SUCCESS;
}


/******************************************************************************************
函数名称：Oal_SocketSendBroadcastData
函数说明：发送广播包
输入参数：ps8Buffer:包数据 s32BufLen包长度，ps8DstIp广播ip，u16DstPort广播端口
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSendBroadcastData(int s32Socket, char* ps8Buffer, int s32BufLen, char* ps8DstIp, unsigned short u16DstPort)
{
	int s32Broadcast = 1;

	if (ps8Buffer == NULL || u16DstPort == 0)
	{
		return FAILURE;
	}
	setsockopt(s32Socket, SOL_SOCKET, SO_BROADCAST, (char*)&s32Broadcast, sizeof(int));
	if (ps8DstIp != NULL)
	{
		if (Oal_SocketSendToEx(s32Socket, ps8Buffer, s32BufLen, ps8DstIp, u16DstPort) <= 0)
		{
			return FAILURE;
		}
		return SUCCESS;
	}
#if (defined(WIN32) || defined(WIN64))
	{
		PMIB_IPADDRTABLE ptIpTable = NULL;
		DWORD dwSize = 0;
		char as8Buffer[2000];
		char as8DstIp[IP_LEN];
		DWORD i = 0;
		DWORD dwIpAddr = 0;
		DWORD dwNetMask = 0;
		DWORD dwBroadcast = 0;

		memset(as8Buffer, 0, 2000);
		dwSize = sizeof(as8Buffer);
		ptIpTable = (PMIB_IPADDRTABLE)as8Buffer;
		if (GetIpAddrTable(ptIpTable, &dwSize, FALSE) == NO_ERROR)
		{
			for (i = 0; i < ptIpTable->dwNumEntries; i++)
			{
				dwIpAddr = ptIpTable->table[i].dwAddr;
				dwNetMask = ptIpTable->table[i].dwMask;
				dwBroadcast = dwIpAddr | (~dwNetMask);
				sprintf_s(as8DstIp, IP_LEN, "%d.%d.%d.%d", (dwBroadcast & 0xFF),
					((dwBroadcast >> 8) & 0xFF), ((dwBroadcast >> 16) & 0xFF), ((dwBroadcast >> 24) & 0xFF));
				Oal_SocketSendToEx(s32Socket, ps8Buffer, s32BufLen, as8DstIp, u16DstPort);
			}

		}
	}
#else
	{
		char as8DstIp[IP_LEN];
		struct sockaddr_in* ptNetMask = NULL;
		struct ifconf tIfc;
		struct ifreq tFreq[16];
		char as8Buffer[2000];
		int s32Num = 0;
		int i = 0;

		tIfc.ifc_len = sizeof(as8Buffer);
		tIfc.ifc_buf = (caddr_t)tFreq;
		if (ioctl(s32Socket, SIOCGIFCONF, (char*)&tIfc) == SUCCESS)
		{
			/*找出所有的网卡，每个都发送广播消息*/
			s32Num = tIfc.ifc_len / sizeof(struct ifreq);
			for (i = 0; i < s32Num; i++)
			{
				if ((ioctl(s32Socket, SIOCGIFNETMASK, &tFreq[i])) < 0)
				{
					DEBUG_ERR("ioctl SIOCGIFNETMASK failed");
					continue;
				}

				ptNetMask = (struct sockaddr_in*) & (tFreq[i].ifr_netmask);
				strncpy(as8DstIp, inet_ntoa(ptNetMask->sin_addr), IP_LEN);
				Oal_SocketSendToEx(s32Socket, ps8Buffer, s32BufLen, as8DstIp, u16DstPort);
			}
		}
		else
		{
			return FAILURE;
		}
	}
#endif
	return SUCCESS;
}

int Oal_SocketGetLocalIpByDstIp(const char* ps8DstIp, unsigned short u16DstPort, char* ps8LocalIp, int s32IpLen)
{
	int err, tmp;
	struct addrinfo hints;
	struct addrinfo* res = NULL;
	struct sockaddr_storage addr;
	int s32Sock = 0;
	char  as8Port[8];

	strncpy(ps8LocalIp, "127.0.0.1", s32IpLen);    /* always fallback to local loopback */

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = PF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;
	/*hints.ai_flags=AI_NUMERICHOST|AI_CANONNAME; */
	sprintf_s(as8Port, 8, "%d", u16DstPort);
	err = getaddrinfo(ps8DstIp, as8Port, &hints, &res);
	if (err != 0)
	{
		return FAILURE;
	}
	if (res == NULL)
	{
		return FAILURE;
	}
	s32Sock = (int)socket(res->ai_family, SOCK_DGRAM, 0);
	tmp = 1;
	err = setsockopt(s32Sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&tmp, sizeof(int));
	if (err < 0)
	{
		return FAILURE;
	}
	err = connect(s32Sock, res->ai_addr, (int)(res->ai_addrlen));
	if (err < 0)
	{
		freeaddrinfo(res);
		Oal_SocketClose(s32Sock);
		return FAILURE;
	}
	freeaddrinfo(res);
	res = NULL;
	socklen_t tSockLen = sizeof(addr);
	err = getsockname(s32Sock, (struct sockaddr*) & addr, &tSockLen);
	if (err != 0)
	{
		Oal_SocketClose(s32Sock);
		return FAILURE;
	}

	err = getnameinfo((struct sockaddr*) & addr, tSockLen, ps8LocalIp, s32IpLen, NULL, 0, NI_NUMERICHOST);
	if (err != 0)
	{
		Oal_SocketClose(s32Sock);
		return FAILURE;
	}
	Oal_SocketClose(s32Sock);
	return SUCCESS;
}

/******************************************************************************************
函数名称：SocketGetHostIP
函数说明：设置主机的IP地址相关信息
输入参数：ps8Addr 点分十进制ip地址, ip地址长度, ps8DiviceName网卡的名称(linux物理网卡为"eth0")
输出参数：
返 回 值：	成功，则返回请求信息类型的个数，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSetHostAddrInfo(char* ps8Addr, const char* ps8DiviceName, int s32Type)
{
#if !(defined(WIN32) || defined(WIN64))
	struct ifreq buf[16];
	struct ifconf ifc;
	int s32Socket = 0;
	int s32Num = 0;
	struct sockaddr_in* host;

	if ((s32Socket = socket(AF_INET, SOCK_DGRAM, 0)) >= 0)
	{
		ifc.ifc_len = sizeof(buf);
		ifc.ifc_buf = (caddr_t)buf;
		if (!ioctl(s32Socket, SIOCGIFCONF, (char*)&ifc))
		{
			s32Num = ifc.ifc_len / sizeof(struct ifreq);
			DEBUG_PRT("interface num is intrface=%d", s32Num);
			while (s32Num-- > 0)
			{
				if (strncmp(buf[s32Num].ifr_name, ps8DiviceName, strlen(ps8DiviceName)) == 0)
				{
					host = (struct sockaddr_in*) & buf[s32Num].ifr_addr;
					host->sin_family = AF_INET;
					if (inet_pton(AF_INET, ps8Addr, &(host->sin_addr)) < 0)
					{
						DEBUG_ERR("(%04d %s) inet_pton:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
						break;
					}
					if (s32Type == IPADDR_TYPE)
					{
						if (ioctl(s32Socket, SIOCSIFADDR, &buf[s32Num]) < 0)
						{
							DEBUG_ERR("(%04d %s) ioctl SIOCSIFADDR:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
							break;
						}
					}
					else if (s32Type == NETMASK_TYPE)
					{
						if (ioctl(s32Socket, SIOCSIFNETMASK, &buf[s32Num]) < 0)
						{
							DEBUG_ERR("(%04d %s) ioctl SIOCSIFNETMASK:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
							break;
						}
					}
					else
					{
						//gateway
						struct rtentry rt;
						memset(&rt, 0, sizeof(struct rtentry));
						memcpy(&rt.rt_gateway, host, sizeof(struct sockaddr_in));
						((struct sockaddr_in*) & rt.rt_dst)->sin_family = AF_INET;
						((struct sockaddr_in*) & rt.rt_genmask)->sin_family = AF_INET;
						rt.rt_flags = (RTF_UP | RTF_GATEWAY);
						if (ioctl(s32Socket, SIOCADDRT, &rt) < 0)
						{
							DEBUG_ERR("(%04d %s) ioctl SIOCADDRT:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
							break;
						}
					}

					ioctl(s32Socket, SIOCGIFFLAGS, (char*)&buf[s32Num]);
					if ((!buf[s32Num].ifr_flags) & IFF_UP)
					{
						buf[s32Num].ifr_flags |= IFF_UP;
					}
					else
					{
						ioctl(s32Socket, SIOCSIFFLAGS, (char*)&buf[s32Num]);
					}
					break;
				}
			}
		}
		else
		{
			DEBUG_ERR("(%04d %s) socke ioctl SIOCGIFCONF failed:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
		}
	}
	else
	{
		DEBUG_ERR("(%04d %s) socke failed:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
	}
	close(s32Socket);
#endif	
	return SUCCESS;
}

#if !(defined(WIN32) || defined(WIN64))
int RoutesParser(struct nlmsghdr* nlHdr, char* ps8GateWay)
{
	struct rtmsg* rtMsg;
	struct rtattr* rtAttr;
	int rtLen;
	struct in_addr gate;
	u_int u32GateWay = 0;

	if (nlHdr == NULL || ps8GateWay == NULL)
	{
		return FAILURE;
	}
	rtMsg = (struct rtmsg*)NLMSG_DATA(nlHdr);
	// If the route is not for AF_INET or does not belong to main routing table     
	//then return.      
	if ((rtMsg->rtm_family != AF_INET) || (rtMsg->rtm_table != RT_TABLE_MAIN))
	{
		return FAILURE;
	}
	rtAttr = (struct rtattr*)RTM_RTA(rtMsg);
	rtLen = RTM_PAYLOAD(nlHdr);
	for (; RTA_OK(rtAttr, rtLen); rtAttr = RTA_NEXT(rtAttr, rtLen))
	{
		if (rtAttr->rta_type == RTA_GATEWAY)
		{
			u32GateWay = *(u_int*)RTA_DATA(rtAttr);
			break;
		}
	}
	gate.s_addr = u32GateWay;
	sprintf_s(ps8GateWay, IP_LEN, "%s", (char*)inet_ntoa(gate));
	return SUCCESS;
}
#endif

int SocketGetGateWay(char* ps8GateWay)
{
#if !(defined(WIN32) || defined(WIN64)) 
	char* buf[BUFSIZ];
	int fd;
	struct sockaddr_nl nl;
	struct nlmsghdr msghdr, * msghdrp;
	struct msghdr msg, recev;
	struct iovec iov;
	int len;

	if (ps8GateWay == NULL)
	{
		return FAILURE;
	}
	fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
	if (fd < 0)
	{
		perror("open error");
		return FAILURE;
	}
	nl.nl_family = AF_NETLINK;
	nl.nl_pad = 0;
	nl.nl_pid = getpid();
	nl.nl_groups = 0;

	if (bind(fd, (struct sockaddr*) & nl, sizeof(nl)) < 0)
	{
		perror(" bind  error");
		close(fd);
		return FAILURE;
	}
	memset(&nl, 0, sizeof(struct rtmsg));
	memset(&msghdr, 0, sizeof(msghdr));
	msghdr.nlmsg_len = NLMSG_LENGTH(sizeof(struct rtmsg));
	msghdr.nlmsg_type = RTM_GETROUTE;
	msghdr.nlmsg_flags = NLM_F_MATCH | NLM_F_REQUEST;
	msghdr.nlmsg_pid = nl.nl_pid;

	iov.iov_base = (void*)&msghdr;
	iov.iov_len = msghdr.nlmsg_len;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;

	len = sendmsg(fd, &msg, 0);
	if (len < 0)
	{
		DEBUG_ERR("send error!");
		close(fd);
		return FAILURE;
	}
	else
	{
		DEBUG_PRT("%d bytes send!", len);
	}
	msghdrp = (struct nlmsghdr*)buf;

	iov.iov_base = (void*)buf;
	iov.iov_len = BUFSIZ;
	recev.msg_name = (void*)&msghdrp;
	recev.msg_namelen = sizeof(struct nlmsghdr);
	recev.msg_iov = &iov;
	recev.msg_iovlen = 1;

	len = recvmsg(fd, &recev, 0);
	if (len < 0)
	{
		perror("received failed!");
		close(fd);
		return FAILURE;
	}
	DEBUG_PRT("%d bytes received!", len);

	for (msghdrp = (struct nlmsghdr*)buf;
		(msghdrp->nlmsg_type != NLMSG_DONE) && NLMSG_OK(msghdrp, len);
		msghdrp = NLMSG_NEXT(msghdrp, len))
	{
		RoutesParser(msghdrp, ps8GateWay);
		/*忽略0.0.0.0这个ip*/
		if (strcmp(ps8GateWay, "0.0.0.0") != 0)
		{
			break;
		}
	}
	close(fd);
#endif
	return SUCCESS;
}

int Oal_SocketGetHostAddrInfo(char* ps8Addr, const char* ps8DiviceName, int s32Type)
{
#if !(defined(WIN32) || defined(WIN64))
	struct ifreq buf[16];
	struct ifconf ifc;
	int s32Socket = 0;
	int s32Num = 0;
	struct sockaddr_in* host;

	if (s32Type == GATEWAY_TYPE)
	{
		//gateway
		SocketGetGateWay(ps8Addr);
		return SUCCESS;
	}

	if ((s32Socket = socket(AF_INET, SOCK_DGRAM, 0)) >= 0)
	{
		ifc.ifc_len = sizeof(buf);
		ifc.ifc_buf = (caddr_t)buf;
		if (!ioctl(s32Socket, SIOCGIFCONF, (char*)&ifc))
		{
			s32Num = ifc.ifc_len / sizeof(struct ifreq);
			DEBUG_PRT("interface num is intrface=%d", s32Num);
			while (s32Num-- > 0)
			{
				if (strncmp(buf[s32Num].ifr_name, ps8DiviceName, strlen(ps8DiviceName)) == 0)
				{
					host = (struct sockaddr_in*) & buf[s32Num].ifr_addr;
					host->sin_family = AF_INET;
					if (s32Type == IPADDR_TYPE)
					{
						if (ioctl(s32Socket, SIOCGIFADDR, &buf[s32Num]) < 0)
						{
							DEBUG_ERR("(%04d %s) ioctl SIOCSIFADDR:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
							break;
						}
						inet_ntop(AF_INET, &host->sin_addr, ps8Addr, 16);
					}
					else if (s32Type == NETMASK_TYPE)
					{
						if (ioctl(s32Socket, SIOCGIFNETMASK, &buf[s32Num]) < 0)
						{
							DEBUG_ERR("(%04d %s) ioctl SIOCSIFNETMASK:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
							break;
						}
						inet_ntop(AF_INET, &host->sin_addr, ps8Addr, 16);
					}
					break;
				}
			}
		}
		else
		{
			DEBUG_ERR("(%04d %s) socket ioctl SIOCGIFCONF failed:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
		}
	}
	else
	{
		DEBUG_ERR("(%04d %s) socket failed:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
	}
	close(s32Socket);
#endif	
	return SUCCESS;
}

int Oal_SocketGetDeviceName(char *ps8DiviceName)
{
	#ifndef WIN32
	struct ifreq buf[16];
	struct ifconf ifc;
	int s32Socket = 0;
	int s32Num = 0;
	int s32Result = 0;
	struct sockaddr_in *host;

	if ( (s32Socket = socket(AF_INET, SOCK_DGRAM, 0)) >= 0 )
	{
		ifc.ifc_len = sizeof(buf);
		ifc.ifc_buf = (caddr_t)buf;
		if ( !ioctl(s32Socket, SIOCGIFCONF, (char *)&ifc) )
		{
			s32Num = ifc.ifc_len / sizeof(struct ifreq);
			DEBUG_PRT("interface num is intrface=%d", s32Num);
			while ( s32Num-- > 0 )
			{
				if ( strncmp(buf[s32Num].ifr_name, "lo", strlen("lo")) != 0 )
				{
					strncpy(ps8DiviceName, buf[s32Num].ifr_name, strlen(buf[s32Num].ifr_name));
					break;
				}
			}
		}
		else
		{
			DEBUG_ERR("(%04d %s) socket ioctl SIOCGIFCONF failed:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
			s32Result = FAILURE;
		}
	}
	else
	{
		DEBUG_ERR("(%04d %s) socket failed:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
		s32Result = FAILURE;
	}
	close(s32Socket);
	#endif	
	return SUCCESS;
}

/******************************************************************************************
函数名称：SocketPrioritySet
函数说明：设置Socket 优先级和稳定性
输入参数：s32Socket
输出参数：
返 回 值：	成功，则返回请求信息类型的个数，失败,则返回FAILURE。
*******************************************************************************************/
int Oal_SocketSetHighPriority(int s32Socket)
{
	//SO_PRIORITY对套接字优先权的设置，高优先权的数据被优处理，优先权范围0-6
#if !(defined(WIN32) || defined(WIN64))	
	int s32Priority = 6;
	char  u8ServiceType = 0xE0 | IPTOS_LOWDELAY | IPTOS_RELIABILITY | IPTOS_THROUGHPUT;

	if (setsockopt(s32Socket, SOL_SOCKET, SO_PRIORITY, (void*)&s32Priority, sizeof(s32Priority)) < 0)
	{
		DEBUG_ERR("(%04d %s) setsockopt SO_PRIORITY failed:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
	}
	if (setsockopt(s32Socket, SOL_IP/*IPPROTO_IP*/, IP_TOS, (void*)&u8ServiceType, sizeof(u8ServiceType)) < 0)
	{
		DEBUG_ERR("(%04d %s) setsockopt IP_TOS failed:%d, %s!", __LINE__, __FILE__, errno, strerror(errno));
	}
#endif

	return SUCCESS;
}
