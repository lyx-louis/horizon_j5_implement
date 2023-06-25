#ifndef _IPC_MANAGER_H_
#define _IPC_MANAGER_H_

#include "ipc_socket.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*端口*/
#ifndef SERVER_PORT
#	define SERVER_PORT  		45678
#endif

/*最大连接数*/
#ifndef MAX_SOCKET_NUM
#define MAX_SOCKET_NUM  		20
#endif

typedef struct TagSocketInfo
{
	int s32Socket;
	int s32Used;
	int s32ErrCnt;
	int s32BeatErrCnt;
	int type;
	int channel;
	double sysTime;
	pthread_mutex_t lock;
} TSocketInfo;

typedef struct TagSocketList
{
	TSocketInfo tSockets[MAX_SOCKET_NUM];
	int s32SndNum;
} TSocketList;

int Ipc_socketListInit(TSocketList *ptSocketList);
int Ipc_socketListPrint(TSocketList *ptSocketList);
int Ipc_socketListAdd(TSocketList *ptSocketList, int s32Socket, int type);
int Ipc_socketListDel(TSocketList *ptSocketList, int s32Socket);
int Ipc_socketListFind(TSocketList *ptSocketList, int s32Socket);
int Ipc_socketListDestroy(TSocketList *ptSocketList);
int Ipc_socketListGetSocket(TSocketList *ptSocketList);

void Ipc_socketDelAndClrSet(TSocketList *ptSocketList, int s32Socket);

int Ipc_serverSocketCreate(void);

#ifdef __cplusplus
}
#endif

#endif
