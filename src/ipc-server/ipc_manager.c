#include "ipc_manager.h"

int Ipc_socketListInit(TSocketList *ptSocketList)
{
	static int init_flag = 0;

	if (init_flag) {
		return 0;
	}

	init_flag = 1;
	
	for (int i = 0; i < MAX_SOCKET_NUM; i++) {
		TSocketInfo *ptSocket = ptSocketList->tSockets + i;
		pthread_mutex_init(&ptSocket->lock, NULL);
	}

	return 0;
}

int Ipc_socketListPrint(TSocketList *ptSocketList)
{
	int i;
	DEBUG_PRT("Socket Number: %d", ptSocketList->s32SndNum);
	for (i = 0; i < MAX_SOCKET_NUM; i++)
	{
		TSocketInfo* info = &ptSocketList->tSockets[i];
		if (info->s32Used == TRUE)
		{
			DEBUG_PRT("Socket chan: %d, fd: %d, type: %d", info->channel, info->s32Socket, info->type);
		}
	}
	return 0;
}

static double Ipc_getSysTms(void)
{
	struct timespec tp;

	clock_gettime(CLOCK_MONOTONIC, &tp);

	return tp.tv_sec * 1000. + tp.tv_nsec / 1000000.;
}

int Ipc_socketListAdd(TSocketList *ptSocketList, int s32Socket, int type)
{
	int i = 0;
	int s32Found = FAILURE;

	// del
	double nowTime = Ipc_getSysTms();
	for (i = 0; i < MAX_SOCKET_NUM; i++) {
		TSocketInfo *ptSocket = ptSocketList->tSockets + i;

		if (ptSocket->s32Used == TRUE && ptSocket->s32Socket > 0) {
			if (nowTime - ptSocket->sysTime > 5000.) {
				Ipc_socketDelAndClrSet(ptSocketList, ptSocket->s32Socket);
			}
		}
	}

	for (i = 0; i < MAX_SOCKET_NUM; i++)
	{
		if (ptSocketList->tSockets[i].s32Used == TRUE
			&& ptSocketList->tSockets[i].s32Socket == s32Socket)
		{
			DEBUG_PRT("SocketListAdd success, socket:%d, snd_num:%d", s32Socket, ptSocketList->s32SndNum);
			s32Found = SUCCESS;
			break;
		}
	}
	if (s32Found != SUCCESS)
	{
		for (i = 0; i < MAX_SOCKET_NUM; i++)
		{
			if (ptSocketList->tSockets[i].s32Used == FALSE)
			{
				ptSocketList->tSockets[i].sysTime = Ipc_getSysTms();
				ptSocketList->tSockets[i].s32Socket = s32Socket;
				ptSocketList->tSockets[i].s32Used = TRUE;
				ptSocketList->tSockets[i].s32ErrCnt = 0;
				ptSocketList->tSockets[i].s32BeatErrCnt = 0;
				ptSocketList->tSockets[i].type = type;
				ptSocketList->tSockets[i].channel = -1;
				s32Found = SUCCESS;
				ptSocketList->s32SndNum++;
				DEBUG_PRT("SocketListAdd success, socket:%d, snd_num:%d", s32Socket, ptSocketList->s32SndNum);
				break;
			}
		}
	}

	if (s32Found == FAILURE) {
		DEBUG_ERR("use ptSocketList->s32SndNum = %d, max = %d", ptSocketList->s32SndNum, MAX_SOCKET_NUM);
	}

	return s32Found;
}

int Ipc_socketListDel(TSocketList *ptSocketList, int s32Socket)
{
	for (int i = 0; i < MAX_SOCKET_NUM; i++)
	{
		if (ptSocketList->tSockets[i].s32Used == TRUE
			&& ptSocketList->tSockets[i].s32Socket == s32Socket)
		{
			ptSocketList->tSockets[i].s32ErrCnt = 0;
			ptSocketList->tSockets[i].s32BeatErrCnt = 0;
			ptSocketList->tSockets[i].s32Used = FALSE;
			ptSocketList->tSockets[i].s32Socket = 0;
			ptSocketList->s32SndNum--;
			DEBUG_PRT("SocketListDel success, socket:%d, snd_num:%d", s32Socket, ptSocketList->s32SndNum);
			break;
		}
	}
	return 0;
}

int Ipc_socketListFind(TSocketList *ptSocketList, int s32Socket)
{
	int i = 0;

	for (i = 0; i < MAX_SOCKET_NUM; i++)
	{
		if (ptSocketList->tSockets[i].s32Used == TRUE
			&& ptSocketList->tSockets[i].s32Socket == s32Socket)
		{
			return i;
		}
	}

	return -1;
}

int Ipc_socketListDestroy(TSocketList *ptSocketList)
{
	for (int i = 0; i < MAX_SOCKET_NUM; i++)
	{
		if (ptSocketList->tSockets[i].s32Used == TRUE)
		{
			ptSocketList->tSockets[i].s32Used = FALSE;
			Oal_SocketClose(ptSocketList->tSockets[i].s32Socket);
			ptSocketList->tSockets[i].s32Socket = 0;
		}
	}
	return 0;
}

int Ipc_socketListGetSocket(TSocketList *ptSocketList)
{
	int socket = -1;

	for (int i = 0; i < MAX_SOCKET_NUM; i++)
	{
		if (ptSocketList->tSockets[i].s32Used == TRUE)
		{
			socket = ptSocketList->tSockets[i].s32Socket;
			break;
		}
	}

	return socket;
}

void Ipc_socketDelAndClrSet(TSocketList *ptSocketList, int s32Socket)
{
	Ipc_socketListDel(ptSocketList, s32Socket);
	Oal_SocketClose(s32Socket);
}

static int Ipc_serverBind(int s32Socket, char *ps8Ip)
{
	int s32Result = FAILURE;
	if (Oal_SocketBind(s32Socket, ps8Ip, SERVER_PORT) != SUCCESS)
	{
		DEBUG_ERR("Oal_SocketBind ip:%s %d failed, errno:%d", ps8Ip, SERVER_PORT, Oal_SocketErrorCode());
		s32Result = FAILURE;
	}
	else
	{
		DEBUG_PRT("Oal_SocketBind ip:%s %d success", ps8Ip, SERVER_PORT);
		s32Result = SUCCESS;
	}

	return s32Result;
}

#include <regex.h>

static int Ipc_checkIpLlegal(char *ptIpAddr)
{
	char IP_PATTERN[256] = "^([0-9]|[1-9][0-9]|1[0-9]{1,2}|2[0-4][0-9]|25[0-5]).([0-9]|[1-9][0-9]|1[0-9]{1,2}|2[0-4][0-9]|25[0-5]).([0-9]|[1-9][0-9]|1[0-9]{1,2}|2[0-4][0-9]|25[0-5]).([0-9]|[1-9][0-9]|1[0-9]{1,2}|2[0-4][0-9]|25[0-5])$";

	//check ip legal
    regex_t reg;
    regmatch_t match[5];
    int ret = regcomp(&reg, IP_PATTERN, REG_EXTENDED | REG_NEWLINE);
    ret = regexec(&reg, ptIpAddr, sizeof match / sizeof match[0], match, 0);
	regfree(&reg);
    
	// DEBUG_INFO("%s is %s", ip, ret == 0 ? "legal" : "illegal");
    // if (retval == 0) {
    //     int i;
    //     for (i = 1; i < sizeof match / sizeof match[0]; i++) {
    //         DEBUG_INFO("ip part[%d]: %.*s", i, match[i].rm_eo - match[i].rm_so,
    //                 ip + match[i].rm_so);
    //     }
    // }
    return ret ? -1 : 0;
}

int Ipc_serverSocketCreate(void)
{
	char as8DeviceName[256] = { 0 };
	char as8DstIp[IP_LEN] = "0.0.0.0";
	int s32ServerSocket = Oal_SocketCreate(SOCKET_TCP);

	if (s32ServerSocket > 0)
	{
		Oal_SocketGetDeviceName(as8DeviceName);
		// Oal_SocketGetHostAddrInfo(as8DstIp, as8DeviceName, IPADDR_TYPE);

		if (Ipc_checkIpLlegal(as8DstIp) < 0)
		{
			DEBUG_ERR("get %s ip addr failed\n", as8DeviceName);
			Oal_SocketClose(s32ServerSocket);
			return -1;
		}

		Oal_SocketSetAddrReuse(s32ServerSocket);
		Oal_SocketSetNonBlock(s32ServerSocket);
		if (Ipc_serverBind(s32ServerSocket, as8DstIp) != SUCCESS)
		{
			DEBUG_ERR("Oal_SocketBind device name:%s ip:%s %d failed, errno:%d", as8DeviceName, as8DstIp, SERVER_PORT, Oal_SocketErrorCode());
			Oal_SocketClose(s32ServerSocket);
			return -1;
		}
		DEBUG_PRT("Oal_SocketBind device name:%s ip:%s %d success", as8DeviceName, as8DstIp, SERVER_PORT);
		Oal_SocketListen(s32ServerSocket);
	}

	return s32ServerSocket;
}
