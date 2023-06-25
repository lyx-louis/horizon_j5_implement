/***************************************************************
模块名	     ：socket
文件名	     ：socket.h
相关文件	 ：
文件实现功能 ：epoll
作者		 ：xuduan
版本		 ：V1.0
-----------------------------------------------------------------
修改记录:
日  期		    版本		修改人		修改内容
2013/03/18		1.0          xuduan     初始版本
*****************************************************************/

#ifndef __OAL_SOCKET__
#define __OAL_SOCKET__

#include "base_define.h"

/*错误*/
#ifndef FAILURE
#	define FAILURE				-1
#endif

/*成功*/
#ifndef SUCCESS
#	define SUCCESS				0
#endif

/*真*/
#ifndef TRUE
#	define TRUE					1
#endif

/*假*/
#ifndef FALSE
#	define FALSE				0
#endif

/*定义IP长度*/
#ifndef IP_LEN
#	define IP_LEN				16
#endif

#define FD_SET_READ		0x01
#define FD_SET_WRITE	0x02
#define FD_SET_EXCEPT	0x04

#define SOCKET_SEND    	0
#define SOCKET_RECV    	1

#define IPADDR_TYPE  	0
#define NETMASK_TYPE 	1
#define GATEWAY_TYPE 	2

#define SOCKET			int
#define INVALID_SOCKET	-1 
#define SOCKET_ERROR	-1
#define DLL_LOCAL  		__attribute__ ((visibility("hidden")))
#define DLL_API			__attribute__ ((visibility("default")))

typedef enum ESocketType
{
	SOCKET_UDP = 0,
	SOCKET_TCP = 1
}ESocketType;

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************************
函数名称：Oal_SocketCreate
函数说明：创建socket
输入参数：s32Protocol区分udp与tcp
输出参数：无。
返 回 值：	成功，则返回socket值，失败，则返回OAL_FAILURE。
*******************************************************************************************/
DLL_API SOCKET Oal_SocketCreate(ESocketType eSocketType);

/******************************************************************************************
函数名称：Oal_SocketClose
函数说明：关闭socket
输入参数：s32Socket值
输出参数：无。
返 回 值：	成功，则返回socket值，失败，则返回OAL_FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketClose(SOCKET s32Socket);

/******************************************************************************************
函数名称：Oal_SocketBind
函数说明：绑定socket
输入参数：ps8Ip 绑定的ip ，u16Port绑定端口
输出参数：无。
返 回 值：	成功，则返回SUCCESS，失败，则返回OAL_FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketBind(SOCKET s32Socket, char* ps8Ip, unsigned short u16Port);

/******************************************************************************************
函数名称：Oal_SocketSelect
函数说明：等待可读写函数,针对单一的socket设计
输入参数：
输出参数：无。
返 回 值：	成功，则返OAL_SUCCESS，失败，则返回OAL_FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSelect(SOCKET s32Socket, struct timeval* ptWaitTime, int s32Flag);

/******************************************************************************************
函数名称：Oal_SocketSetAddrReuse
函数说明：设置端口为可复用端口
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。

输出参数：无。
返回值：	成功，则返回SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSetAddrReuse(SOCKET s32Socket);

/******************************************************************************************
函数名称：Oal_SocketSetNoDelay
函数说明：设置TCP模式下无时延传输
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。

输出参数：无。
返回值：	成功，则返回SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSetNoDelay(SOCKET s32Socket);

/******************************************************************************************
函数名称：Oal_SocketSetNonBlock
函数说明：设置非阻塞方式
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。
输出参数：
返 回 值：	成功，则返回SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSetNonBlock(SOCKET s32Socket);

/******************************************************************************************
函数名称：Oal_SocketListen
函数说明：设置socket监听
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。
输出参数：
返 回 值：	成功，则返回SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketListen(SOCKET s32Socket);

/******************************************************************************************
函数名称：Oal_SocketAccept
函数说明：接受远端客户端的链接。
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。
输出参数：无。
返 回 值：	成功，则返回一个新的TSocket ID用来和建立连接的客户端进行通信，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API SOCKET Oal_SocketAccept(SOCKET s32Socket, char* ps8Ip, unsigned short* pu16Port);

/******************************************************************************************
函数名称：Oal_SocketAccept
函数说明：接受远端客户端的链接。
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。
输出参数：无。
返 回 值：	成功，则返回SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketConnect(SOCKET s32Socket, char* ps8Ip, unsigned short u16Port);

/******************************************************************************************
函数名称：Oal_SocketSendTo
函数说明： udp发送数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
		  u16DstPort目的端口 s32WaitTime等待时间，对于非阻塞方式使用
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSendTo(SOCKET s32Socket, void* ptBuffer, int s32BufLen, char* ps8DstIp, unsigned short u16DstPort, int s32WaitTime);

/******************************************************************************************
函数名称：Oal_SocketSendToEx
函数说明： udp发送数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
		  u16DstPort目的端口
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSendToEx(SOCKET s32Socket, void* ptBuffer, int s32BufLen, char* ps8DstIp, unsigned short u16DstPort);

/******************************************************************************************
函数名称：Oal_SocketSend
函数说明： tcp发送数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
		  u16DstPort目的端口 s32WaitTime等待时间，对于非阻塞方式使用
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSend(SOCKET s32Socket, void* ptBuffer, int s32BufLen, int s32WaitTime);

/******************************************************************************************
函数名称：Oal_SocketSend
函数说明： tcp发送数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
u16DstPort目的端口 s32WaitTime等待时间，对于非阻塞方式使用
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSendEx(SOCKET s32Socket, void* ptBuffer, int s32BufLen);

/******************************************************************************************
函数名称：Oal_SocketRecvFrom
函数说明： udp接收数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
u16DstPort目的端口 s32WaitTime等待时间，对于非阻塞方式使用
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketRecvFrom(SOCKET s32Socket, void* ptBuffer, int s32BufLen, char* ps8SrcIp, unsigned short* pu16SrcPort, int s32WaitTime);

/******************************************************************************************
函数名称：Oal_SocketRecvFrom
函数说明： udp接收数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
u16DstPort目的端口
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketRecvFromEx(SOCKET s32Socket, void* ptBuffer, int s32BufLen, char* ps8SrcIp, unsigned short* pu16SrcPort);

/******************************************************************************************
函数名称：Oal_SocketRecv
函数说明： tcp接收数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
u16DstPort目的端口 s32WaitTime等待时间，对于非阻塞方式使用
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketRecv(SOCKET s32Socket, void* ptBuffer, int s32BufLen, int s32WaitTime);

/******************************************************************************************
函数名称：Oal_SocketRecv
函数说明： tcp接收数据
输入参数：s32Socket网络通信模块对象标识符ID,ptBuffer数据内存地址 s32BufLen数据长度 ps8DstIp目的ip
u16DstPort目的端口 s32WaitTime等待时间，对于非阻塞方式使用
输出参数：无。
返 回 值：	成功，则返回发送数据的长度，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketRecvEx(SOCKET s32Socket, void* ptBuffer, int s32BufLen);

/******************************************************************************************
函数名称：IsPrivateIp
函数说明：检测ip是否是内网ip
输入参数：ps8Ip 地址
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_IsPrivateAddr(char* ps8Ip);

/******************************************************************************************
函数名称：Oal_SocketSetBuffer
函数说明：设置socket缓冲区大小
输入参数：tSocketID：TSocket网络通信模块对象标识符ID。s32BufLen 缓冲区大小
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSetBuffer(SOCKET s32Socket, int s32BufLen, int s32Direction);

/******************************************************************************************
函数名称：Oal_SocketErrorCode
函数说明：获取错误码
输入参数：
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketErrorCode(void);

/******************************************************************************************
函数名称：Oal_SocketSetBroadcast
函数说明：设置广播方式
输入参数：
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSetBroadcast(int s32Socket);

/******************************************************************************************
函数名称：Oal_SocketSetMemberShip
函数说明：设置组播方式
输入参数：
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSetMemberShip(int s32Socket, char* ps8DstIp);

/******************************************************************************************
函数名称：Oal_SocketSetMemberShip
函数说明：设置组播方式
输入参数：
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketDropMemberShip(int s32Socket, char* ps8DstIp);

/******************************************************************************************
函数名称：Oal_SocketSendBroadcastData
函数说明：发送广播包
输入参数：ps8Buffer:包数据 s32BufLen包长度，ps8DstIp广播ip，u16DstPort广播端口
输出参数：
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSendBroadcastData(int s32Socket, char* ps8Buffer, int s32BufLen, char* ps8DstIp, unsigned short u16DstPort);

/******************************************************************************************
函数名称：Oal_SocketGetLocalIpByDstIp
函数说明：通过目的地址得到本地ip
输入参数：ps8DstIp 目的ip， 目的端口 u16DstPort
输出参数：ps8LocalIp 本地ip s32IpLen本地ip长度
返 回 值：	成功，SUCCESS，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketGetLocalIpByDstIp(const char* ps8DstIp, unsigned short u16DstPort, char* ps8LocalIp, int s32IpLen);

/******************************************************************************************
函数名称：Oal_SocketSetHostAddrInfo
函数说明：设置主机的IP地址相关信息
输入参数：ps8Addr 点分十进制ip地址, ip地址长度, ps8DiviceName网卡的名称(linux物理网卡为"eth0")
输出参数：
返 回 值：	成功，则返回请求信息类型的个数，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSetHostAddrInfo(char* ps8Addr, const char* ps8DiviceName, int s32Type);

/******************************************************************************************
函数名称：Oal_SocketGetHostAddrInfo
函数说明：获取主机的IP地址相关信息
输入参数：ps8Addr 点分十进制ip地址, ip地址长度, ps8DiviceName网卡的名称(linux物理网卡为"eth0")
输出参数：
返 回 值：	成功，则返回请求信息类型的个数，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketGetHostAddrInfo(char* ps8Addr, const char* ps8DiviceName, int s32Type);

DLL_API int Oal_SocketGetDeviceName(char *ps8DiviceName);
/******************************************************************************************
函数名称：SocketPrioritySet
函数说明：设置Socket 优先级和稳定性
输入参数：s32Socket
输出参数：
返 回 值：	成功，则返回请求信息类型的个数，失败,则返回FAILURE。
*******************************************************************************************/
DLL_API int Oal_SocketSetHighPriority(int s32Socket);

#ifdef __cplusplus
}
#endif

#endif
