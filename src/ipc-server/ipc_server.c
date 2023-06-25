#include "ipc_server.h"

static TSocketList g_tSocketList = {0};

typedef enum _EUsbFrameType_
{
	SERIAL_MSG_TYPE_VIDEO = 1,
	SERIAL_MSG_TYPE_PARAM = 2,
	SERIAL_MSG_TYPE_ADAS_INFO = 3,
	SERIAL_MSG_TYPE_TIME = 4,
	SERIAL_MSG_TYPE_CHANNEL = 5,
	SERIAL_MSG_TYPE_REBOOT = 6,
	SERIAL_MSG_TYPE_CANXML_SET = 7,
	SERIAL_MSG_TYPE_CANOUT_SET = 8,
	SERIAL_MSG_TYPE_UPDATE_BIN = 9,
	SERIAL_MSG_TYPE_INSTALL_BIN = 10,
	SERIAL_MSG_TYPE_GET_PARAM = 11,
	SERIAL_MSG_TYPE_SET_PARAM = 12,
	SERIAL_MSG_TYPE_REPORT_PARAM = 13,
	SERIAL_MSG_TYPE_CAM_PARAM = 14,

	SERIAL_MSG_TYPE_JPEG_VIDEO = 0x0e,         // jpeg??
	SERIAL_MSG_TYPE_FPGA_VIDEO_GRAY = 0x0f,			// yyyy..
	SERIAL_MSG_TYPE_FPGA_VIDEO_I420 = 0x10,			// yyyy.. u.. v..
	E_USB_FRAME_3D = 0x11,			// yyyy.. u.. v..  +  trace	
	SERIAL_MSG_TYPE_MCU_CAN = 0x17,
	SERIAL_MSG_TYPE_MCU_APP = 0x18,
	SERIAL_MSG_TYPE_AVM_CALIB = 0x19,	//AVM

	SERIAL_MSG_TYPE_3D_OBJECT = 0x20,  //TrackObject
	SERIAL_MSG_TYPE_CALIB_JPG = 0x30,  // JPG image 

	SERIAL_MSG_TYPE_VIDEOSTOP = 0x40,
	SERIAL_MSG_TYPE_VIDEOPLAY = 0x41,

	SERIAL_MSG_TYPE_PACKS_DATA = 0xFF
} EUsbFrameType;

typedef enum _TProtoUsb
{
	PROTO_NONE = 0,
	PROTO_USB = 1,
	PROTO_NET = 2,
	PROTO_MAX
} TProtoUsb;

typedef struct TagHead
{
	char au8Head[4];
	int s32Length;
} THead;

static int usb_send(int s32Fd, char *pu8Data, int s32DataLen)
{
	int s32SendLen = Oal_SocketSend(s32Fd, pu8Data, s32DataLen, 1000);
	if ( s32SendLen < 0 && Oal_SocketErrorCode() != 0 ) {
		DEBUG_ERR("usb send error:%d s32Fd:%d", Oal_SocketErrorCode(), s32Fd);
	}

	return s32SendLen;
}

static int usb_send_head(int s32Fd, char u8type, int s32Length)
{
	THead tHead;
	int s32SndLen = 0;

	tHead.au8Head[0] = 'J';
	tHead.au8Head[1] = 'M';
	tHead.au8Head[2] = 0;
	tHead.au8Head[3] = u8type;
	tHead.s32Length = s32Length + sizeof(THead);

	return usb_send(s32Fd, (char *)&tHead, sizeof(tHead));
}

static int usb_send_data(int s32Fd, char *pu8Data, int s32DataLen, char u8type)
{
	if (usb_send_head(s32Fd, u8type, s32DataLen) != sizeof(THead)) {
		return FAILURE;
	}

	if (usb_send(s32Fd, pu8Data, s32DataLen) != s32DataLen) {
		return FAILURE;
	}

	return s32DataLen;
}

#if 0
static int __SendParam(int s32Fd)
{
	char buffer[1024];
	snprintf(buffer, sizeof(buffer), "COMPILE_TIME=%s", COMPILE_TIME);
	int len = strlen(buffer);

	if (usb_send_data(s32Fd, (char *)buffer, len, SERIAL_MSG_TYPE_PARAM) != len) {
		return FAILURE;
	}

	return SUCCESS;
}

static void __SendHeartBeat(TSocketList *ptSocketList)
{
	static const uint64_t BEAT_TIME = 1000;
	static uint64_t s_lastTime = 0;

	//每间隔1s发送一次
	uint64_t currTime = (uint64_t)Comm_getSysTms();
	if (currTime - s_lastTime > BEAT_TIME) {
		s_lastTime = currTime;
	} else {
		return;
	}

	for (int i = 0; i < MAX_SOCKET_NUM; i++) {
		TSocketInfo *ptSocket = ptSocketList->tSockets + i;

		if (ptSocket->s32Used == TRUE && ptSocket->type == PROTO_NET && ptSocket->s32Socket > 0) {
			if (__SendParam(ptSocket->s32Socket) != SUCCESS)
				ptSocket->s32BeatErrCnt++;
			else
				ptSocket->s32BeatErrCnt = 0;

			if (ptSocket->s32BeatErrCnt > 3) {
				DEBUG_ERR("__SendParam failed, socket:%d, snd_num:%d",
					ptSocket->s32Socket, ptSocketList->s32SndNum);
				Ipc_socketDelAndClrSet(ptSocketList, ptSocket->s32Socket);
			}
		}
	}
}
#endif

void Comm_setIntTime(int s32Time)  
{
    struct timeval tTv;

    tTv.tv_sec = s32Time;
    tTv.tv_usec = 0;

    settimeofday(&tTv, NULL);

    {
        // debug time
        char buf[64];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", localtime(&tTv.tv_sec));
        DEBUG_INFO("Set time is : %d (%s)", s32Time, buf);
    }
}

double Comm_getSysTms(void)
{
	struct timespec tp;

	clock_gettime(CLOCK_MONOTONIC, &tp);

	return tp.tv_sec * 1000. + tp.tv_nsec / 1000000.;
}

struct timeval Comm_getNetTime(void)
{
	struct timeval tTime;
	gettimeofday(&tTime, NULL);
	return tTime;
}

int Comm_setsockoptSendTimeout(int fd, int sec, int usec)
{
	struct timeval timeout = {0};
	timeout.tv_sec = sec;
	timeout.tv_usec = usec;
	if(fd >= 0)
		setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
	return 0;
}

int Comm_setsockoptRecvTimeout(int fd, int sec, int usec)
{
	struct timeval timeout = {0};
	timeout.tv_sec = sec;
	timeout.tv_usec = usec;
	if(fd >= 0)
		setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
	return 0;
}

static void __Answer(int s32Fd)
{
	char buff[10] = "ANSWER=OK";
	int len = strlen(buff);

	if (usb_send_data(s32Fd, (char *)buff, len, SERIAL_MSG_TYPE_PARAM) != len) {
		DEBUG_ERR("usb_send_data fail");
	}
}

static void __RecvMsg(TSocketList *ptSocketList)
{
	double nowTime = Comm_getSysTms();

	for (int i = 0; i < MAX_SOCKET_NUM; i++) {
		TSocketInfo *ptSocket = ptSocketList->tSockets + i;

		if (ptSocket->s32Used == TRUE && ptSocket->type == PROTO_NET && ptSocket->s32Socket > 0) {
			struct timeval tWaitTime;
			tWaitTime.tv_sec = 0;
			tWaitTime.tv_usec = 1000;

			if (Oal_SocketSelect(ptSocket->s32Socket, &tWaitTime, FD_SET_READ) > 0) {
				THead tHead;

				//获取当前socket buffer里面有多少个字节，<=8个字节等USB客户端全部发完后再收
				int socketLen = 0;
				ioctl(ptSocket->s32Socket, FIONREAD, &socketLen);
				if (socketLen <= sizeof(THead)) {
					return;
				}

				int ret = Oal_SocketRecv(ptSocket->s32Socket, &tHead, sizeof(tHead), 10);
				if (ret != sizeof(tHead)) {
					DEBUG_ERR("Oal_SocketRecv size err");
					ptSocket->s32ErrCnt ++;
					if (ptSocket->s32ErrCnt > 3) {
						Ipc_socketDelAndClrSet(ptSocketList, ptSocket->s32Socket);
					}
					return;
				} else {
					ptSocket->s32ErrCnt = 0;
				}

				if (tHead.au8Head[0] == 'J' && tHead.au8Head[1] == 'M' && tHead.au8Head[2] == 0) {
					char u8Type = tHead.au8Head[3];
					int s32Length = tHead.s32Length - sizeof(tHead); // 剩余接收数据长度
					// printf("u8Type = %d, s32Length = %d\n", u8Type, s32Length);

					if (s32Length > 0) {
						char* buff = (char *)malloc(s32Length + 1);
						ret = Oal_SocketRecv(ptSocket->s32Socket, buff, s32Length, 10);
						if (buff != NULL) {
							buff[ret] = 0;
							if (u8Type == SERIAL_MSG_TYPE_TIME) {
								unsigned int nowTime = *(uint32_t *)buff - 8*60*60; // 客户端多加8小时,此处减去

								struct timeval tv = Comm_getNetTime();
								if (abs(nowTime - tv.tv_sec) > 1) {
									tv.tv_sec = nowTime;
									Comm_setIntTime(nowTime);
								}

								// show time
								char strTime[64] = {0};
								strftime(strTime, sizeof(strTime), "%Y-%m-%d %H:%M:%S", localtime(&tv.tv_sec));
								DEBUG_INFO("now time is : %d (%s), Ipc cali time : %d", (int)tv.tv_sec, strTime, nowTime);
							} else if (u8Type == SERIAL_MSG_TYPE_CHANNEL) {
								ptSocket->channel = *((int*)buff);
								DEBUG_INFO("ptSocket->ptSocket = %d, ptSocket->channel = %d", ptSocket->s32Socket, ptSocket->channel);
							} else if (u8Type == SERIAL_MSG_TYPE_VIDEOSTOP) {
								DEBUG_PRT("SERIAL_MSG_TYPE_VIDEOSTOP !");
							} else if (u8Type == SERIAL_MSG_TYPE_VIDEOPLAY) {
								DEBUG_PRT("SERIAL_MSG_TYPE_VIDEOPLAY !");
							} else {
								// 应答心跳
								pthread_mutex_lock(&ptSocket->lock);
								__Answer(ptSocket->s32Socket);
								pthread_mutex_unlock(&ptSocket->lock);
							}
							free(buff);
						}
					}

					ptSocket->sysTime = nowTime;
				}
			} else {
				if (nowTime - ptSocket->sysTime > 5000.) {
					Ipc_socketDelAndClrSet(ptSocketList, ptSocket->s32Socket);
				}
			}
		}
	}
}

void *Ipc_threadCreat(void* arg)
{
	int s32ServerSocket = Ipc_serverSocketCreate();

	char as8DstIp[IP_LEN];
	unsigned short u16DstPort = 0;

	TSocketList *ptSocketList = &g_tSocketList;
	memset(ptSocketList, 0, sizeof(TSocketList));

	Ipc_socketListInit(ptSocketList);

	for (;;) {
		if (s32ServerSocket < 0) {
			s32ServerSocket = Ipc_serverSocketCreate();
			if (s32ServerSocket < 0) {
				sleep(10);
			}
		}

		struct timeval tWaitTime;
		tWaitTime.tv_sec = 0;
		tWaitTime.tv_usec = 1000;

		if (Oal_SocketSelect(s32ServerSocket, &tWaitTime, FD_SET_READ) > 0) {
			int s32Socket = Oal_SocketAccept(s32ServerSocket, as8DstIp, &u16DstPort);
			if ( Ipc_socketListAdd(ptSocketList, s32Socket, PROTO_NET) != SUCCESS ) {
				DEBUG_ERR("SocketAdd failed, socket:%d", s32Socket);
				Oal_SocketClose(s32Socket);
			} else {
				DEBUG_PRT("SocketAccept success, socket:%d %s %d", s32Socket, as8DstIp, u16DstPort);
				// Oal_SocketSetNonBlock(s32Socket);
				Comm_setsockoptSendTimeout(s32Socket, 1, 300 * 1000); // 300ms
				Comm_setsockoptRecvTimeout(s32Socket, 0, 1000);
			}
		}

		__RecvMsg(ptSocketList);
	}

	return NULL;
}

#pragma pack(push, 1)
typedef struct {
	unsigned char type;
	unsigned char format;
	unsigned short reserved;
	unsigned short width;
	unsigned short height;
	unsigned int frame_index;
	unsigned int timestamp;
} VINF_t;
#pragma pack(pop)

typedef enum
{
	ENCODE_RAW_DATA = 0,
	ENCODE_TYPE_MJPEG = 1,
	ENCODE_TYPE_H264  = 2,
	ENCODE_TYPE_H265  = 3,
	ENCODE_TYPE_NV12  = 4,
} EncodeType;

static int __FillDataNV12(unsigned char* outBuff, int buffSize, int channel, int frame_no, void *data, int datasize, int width, int height, unsigned int pts, int isCalcSize)
{
	THead tHead;
	int ret = SUCCESS;
	int totalSize = 0, fieldSize = 0;
	unsigned char *dataPtr = NULL, *dataStart = NULL, *dataEnd = NULL;

	if (!isCalcSize) {
		if (outBuff == NULL) {
			DEBUG_ERR("outBuff is Null");
			return -1;
		}
		dataPtr = outBuff;
		dataEnd = outBuff + buffSize;
	}

	// VINF
	if (ret == SUCCESS) {
		VINF_t st_vinf = { 0 };
		tHead.au8Head[0] = 'V';
		tHead.au8Head[1] = 'I';
		tHead.au8Head[2] = 'N';
		tHead.au8Head[3] = 'F';
		tHead.s32Length = sizeof(st_vinf);

		fieldSize = sizeof(THead) + tHead.s32Length; 
		totalSize += fieldSize;
		if (!isCalcSize) {
			if(data != NULL) {
				st_vinf.format = ENCODE_TYPE_NV12;
				st_vinf.reserved = channel;
				st_vinf.width = width;
				st_vinf.height = height;
				st_vinf.timestamp = pts;
			}

			st_vinf.frame_index = frame_no;

			dataStart = dataPtr;
			if (dataPtr + fieldSize <= dataEnd) {
				memcpy(dataPtr, (unsigned char *)&tHead, sizeof(tHead));
				dataPtr += sizeof(tHead);
				memcpy(dataPtr, (unsigned char *)&st_vinf, tHead.s32Length);
				dataPtr += tHead.s32Length;
			}

			if (dataPtr - dataStart != fieldSize) {
				DEBUG_ERR("fill data error!");
				ret = FAILURE;
			}
		}
	}

	// VDAT
	if (ret == SUCCESS) {
		void *pVdBuf = NULL;
		if (data != NULL) {
			// VDAT
			tHead.au8Head[0] = 'V';
			tHead.au8Head[1] = 'D';
			tHead.au8Head[2] = 'A';
			tHead.au8Head[3] = 'T';

			pVdBuf = data;
			tHead.s32Length = datasize;

			fieldSize = sizeof(THead) + tHead.s32Length;
			totalSize += fieldSize;
			if (!isCalcSize) {
				dataStart = dataPtr;
				
				if (dataPtr + fieldSize <= dataEnd) {
					memcpy(dataPtr, (unsigned char *)&tHead, sizeof(THead));
					dataPtr += sizeof(tHead);
					memcpy(dataPtr, pVdBuf, tHead.s32Length);
					dataPtr += tHead.s32Length;
				}

				if (dataPtr - dataStart != fieldSize) {
					DEBUG_ERR("fill data error!");
					ret = FAILURE;
				}
			}
		}
	}

	if (ret != FAILURE)
		ret = totalSize;

	return ret;
}

int Ipc_sendNV12(int channel, int frame_no, void *data, int datasize, int width, int height, unsigned int pts)
{
	TSocketList *ptSocketList = &g_tSocketList;

	if (ptSocketList->s32SndNum > 0) {
		int sendflag = 0;

		for (int i = 0; i < MAX_SOCKET_NUM; i++) {
			TSocketInfo *ptSocket = ptSocketList->tSockets + i;

			if (ptSocket->s32Used == TRUE && ptSocket->type == PROTO_NET && ptSocket->s32Socket > 0) {
				if (ptSocket->channel == channel) {
					sendflag = 1;
					break;
				}
			}
		}

		if (sendflag == 0) {
			return 0;
		}

		int fillsize = __FillDataNV12(NULL, 0, channel, frame_no, data, datasize, width, height, pts, TRUE);
		if (fillsize <= 0) {
			DEBUG_ERR("__FillData err, camId:%d", channel);
			return -1;
		}

		unsigned char* buff = (unsigned char *)malloc(fillsize);
		if (buff != NULL) {
			__FillDataNV12(buff, fillsize, channel, frame_no, data, datasize, width, height, pts, FALSE);

			for (int j = 0; j < MAX_SOCKET_NUM; j++) {
				TSocketInfo *ptSocket = ptSocketList->tSockets + j;

				if (ptSocket->s32Used == TRUE && ptSocket->type == PROTO_NET && ptSocket->s32Socket > 0) {
					if (ptSocket->channel != channel) {
						continue;
					}

					pthread_mutex_lock(&ptSocket->lock);
					int ret = usb_send_head(ptSocket->s32Socket, (unsigned char)SERIAL_MSG_TYPE_PACKS_DATA, fillsize);
					if (ret != sizeof(THead)) {
						DEBUG_ERR("send err, s32Socket:%d, send:%d, fillsize:%d, camId:%d", ptSocket->s32Socket, ret, (int)sizeof(THead), channel);
						if (ret <= 0) {
							Ipc_socketDelAndClrSet(ptSocketList, ptSocket->s32Socket);
						}
						pthread_mutex_unlock(&ptSocket->lock);
						continue;
					}

					ret = usb_send(ptSocket->s32Socket, buff, fillsize);
					if (ret != fillsize) {
						DEBUG_ERR("send err, send:%d, fillsize:%d, camId:%d", ret, fillsize, channel);
						if (ret <= 0) {
							Ipc_socketDelAndClrSet(ptSocketList, ptSocket->s32Socket);
						}
						pthread_mutex_unlock(&ptSocket->lock);
						continue;
					}
					pthread_mutex_unlock(&ptSocket->lock);
					// DEBUG_PRT("send ok !");
				}
			}
			
			free(buff);
		}
	}

	return 0;
}

static int __FillData(unsigned char* outBuff, int buffSize, int channel, int frame_no, void *data, int datasize, int width, int height, unsigned int pts, int isCalcSize)
{
	THead tHead;
	int ret = SUCCESS;
	int totalSize = 0, fieldSize = 0;
	unsigned char *dataPtr = NULL, *dataStart = NULL, *dataEnd = NULL;

	if (!isCalcSize) {
		if (outBuff == NULL) {
			DEBUG_ERR("outBuff is Null");
			return -1;
		}
		dataPtr = outBuff;
		dataEnd = outBuff + buffSize;
	}

	// VINF
	if (ret == SUCCESS) {
		VINF_t st_vinf = { 0 };
		tHead.au8Head[0] = 'V';
		tHead.au8Head[1] = 'I';
		tHead.au8Head[2] = 'N';
		tHead.au8Head[3] = 'F';
		tHead.s32Length = sizeof(st_vinf);

		fieldSize = sizeof(THead) + tHead.s32Length; 
		totalSize += fieldSize;
		if (!isCalcSize) {
			if(data != NULL) {
				st_vinf.format = ENCODE_TYPE_MJPEG;
				st_vinf.reserved = channel;
				st_vinf.width = width;
				st_vinf.height = height;
				st_vinf.timestamp = pts;
			}

			st_vinf.frame_index = frame_no;

			dataStart = dataPtr;
			if (dataPtr + fieldSize <= dataEnd) {
				memcpy(dataPtr, (unsigned char *)&tHead, sizeof(tHead));
				dataPtr += sizeof(tHead);
				memcpy(dataPtr, (unsigned char *)&st_vinf, tHead.s32Length);
				dataPtr += tHead.s32Length;
			}

			if (dataPtr - dataStart != fieldSize) {
				DEBUG_ERR("fill data error!");
				ret = FAILURE;
			}
		}
	}

	// VDAT
	if (ret == SUCCESS) {
		void *pVdBuf = NULL;
		if (data != NULL) {
			// VDAT
			tHead.au8Head[0] = 'V';
			tHead.au8Head[1] = 'D';
			tHead.au8Head[2] = 'A';
			tHead.au8Head[3] = 'T';

			pVdBuf = data;
			tHead.s32Length = datasize;

			fieldSize = sizeof(THead) + tHead.s32Length;
			totalSize += fieldSize;
			if (!isCalcSize) {
				dataStart = dataPtr;
				
				if (dataPtr + fieldSize <= dataEnd) {
					memcpy(dataPtr, (unsigned char *)&tHead, sizeof(THead));
					dataPtr += sizeof(tHead);
					memcpy(dataPtr, pVdBuf, tHead.s32Length);
					dataPtr += tHead.s32Length;
				}

				if (dataPtr - dataStart != fieldSize) {
					DEBUG_ERR("fill data error!");
					ret = FAILURE;
				}
			}
		}
	}

	if (ret != FAILURE)
		ret = totalSize;

	return ret;
}

int Ipc_sendJpg(int channel, int frame_no, void *data, int datasize, int width, int height, unsigned int pts)
{
	TSocketList *ptSocketList = &g_tSocketList;

	if (ptSocketList->s32SndNum > 0) {
		int sendflag = 0;

		for (int i = 0; i < MAX_SOCKET_NUM; i++) {
			TSocketInfo *ptSocket = ptSocketList->tSockets + i;

			if (ptSocket->s32Used == TRUE && ptSocket->type == PROTO_NET && ptSocket->s32Socket > 0) {
				if (ptSocket->channel == channel) {
					sendflag = 1;
					break;
				}
			}
		}

		if (sendflag == 0) {
			return 0;
		}

		int fillsize = __FillData(NULL, 0, channel, frame_no, data, datasize, width, height, pts, TRUE);
		if (fillsize <= 0) {
			DEBUG_ERR("__FillData err, camId:%d", channel);
			return -1;
		}

		unsigned char* buff = (unsigned char *)malloc(fillsize);
		if (buff != NULL) {
			__FillData(buff, fillsize, channel, frame_no, data, datasize, width, height, pts, FALSE);

			for (int j = 0; j < MAX_SOCKET_NUM; j++) {
				TSocketInfo *ptSocket = ptSocketList->tSockets + j;

				if (ptSocket->s32Used == TRUE && ptSocket->type == PROTO_NET && ptSocket->s32Socket > 0) {
					if (ptSocket->channel != channel) {
						continue;
					}

					pthread_mutex_lock(&ptSocket->lock);
					int ret = usb_send_head(ptSocket->s32Socket, (unsigned char)SERIAL_MSG_TYPE_PACKS_DATA, fillsize);
					if (ret != sizeof(THead)) {
						DEBUG_ERR("send err, s32Socket:%d, send:%d, fillsize:%d, camId:%d", ptSocket->s32Socket, ret, (int)sizeof(THead), channel);
						if (ret <= 0) {
							Ipc_socketDelAndClrSet(ptSocketList, ptSocket->s32Socket);
						}
						pthread_mutex_unlock(&ptSocket->lock);
						continue;
					}

					ret = usb_send(ptSocket->s32Socket, buff, fillsize);
					if (ret != fillsize) {
						DEBUG_ERR("send err, send:%d, fillsize:%d, camId:%d", ret, fillsize, channel);
						if (ret <= 0) {
							Ipc_socketDelAndClrSet(ptSocketList, ptSocket->s32Socket);
						}
						pthread_mutex_unlock(&ptSocket->lock);
						continue;
					}
					pthread_mutex_unlock(&ptSocket->lock);
					// DEBUG_PRT("send ok !");
				}
			}
			
			free(buff);
		}
	}

	return 0;
}

// 读取点位信息发送到上位机,表示车道线的点与点之间无需连线,通过大量的点叠加即可
typedef struct {
	unsigned char *u8Data;
	unsigned int s32Length;
	unsigned int offset;
} BUFF_t;

/**
 * @func: 不传图像信息(上位机处理数据解析), 将点位信息绘制到上位机右侧的世界坐标系下,表示范围:前后(正负30m)左右(正负15m)
 * 		  文件格式: 前4字节表示点的总数(size),后面每8个字节表示(x,y)的世界坐标系下的坐标位置,xy各占四字节,依次连续
 * 		  如:size(4) + x0(4) + y0(4) + x1(4) + y1(4) ... (均为 float 类型)
 * 		  设备端直接读文件,上位机解析
*/
static BUFF_t __FillBuff(int channel, int frame_no, void *data, int datasize, int width, int height, unsigned int pts)
{
	BUFF_t tBuf = {0};

	// VINF size
	tBuf.s32Length = sizeof(THead) + sizeof(VINF_t);

	// POINT size
	tBuf.s32Length += sizeof(THead) + datasize;

	// malloc size
	tBuf.u8Data = (unsigned char *)malloc(tBuf.s32Length);
	if (tBuf.u8Data == NULL) {
		DEBUG_ERR("malloc size:%d fail", tBuf.s32Length);
		memset(&tBuf, 0, sizeof(tBuf));
		return tBuf;
	}

	// VINF buff
	{
		THead tHead = {0};
		tHead.au8Head[0] = 'V';
		tHead.au8Head[1] = 'I';
		tHead.au8Head[2] = 'N';
		tHead.au8Head[3] = 'F';
		tHead.s32Length = sizeof(VINF_t);

		memcpy(tBuf.u8Data + tBuf.offset, (unsigned char *)&tHead, sizeof(tHead));
		tBuf.offset += sizeof(tHead);

		VINF_t tVinf = {0};
		tVinf.format = 99; //ENCODE_TYPE_MJPEG, 自定义序号,仅用于上位机右侧展示信息
		tVinf.reserved = channel;
		tVinf.frame_index = frame_no;
		tVinf.width = width;
		tVinf.height = height;
		tVinf.timestamp = pts;

		memcpy(tBuf.u8Data + tBuf.offset, (unsigned char *)&tVinf, sizeof(tVinf));
		tBuf.offset += sizeof(tVinf);
	}

	// POINT buff
	{
		THead tHead = {0};
		tHead.au8Head[0] = 'P';
		tHead.au8Head[1] = 'O';
		tHead.au8Head[2] = 'N';
		tHead.au8Head[3] = 'T';
		tHead.s32Length = datasize;

		memcpy(tBuf.u8Data + tBuf.offset, (unsigned char *)&tHead, sizeof(tHead));
		tBuf.offset += sizeof(tHead);
		memcpy(tBuf.u8Data + tBuf.offset, (unsigned char *)data, datasize);
		tBuf.offset += datasize;
	}

	return tBuf;
}

// 单独一个通道号用于传输该点位信息,不传图像信息,上位机自行解析数据
int Ipc_sendPoints(int channel, int frame_no, void *data, int datasize, int width, int height, unsigned int pts)
{
	TSocketList *ptSocketList = &g_tSocketList;

	if (ptSocketList->s32SndNum > 0) {
		int sendflag = 0;

		for (int i = 0; i < MAX_SOCKET_NUM; i++) {
			TSocketInfo *ptSocket = ptSocketList->tSockets + i;

			if (ptSocket->s32Used == TRUE && ptSocket->type == PROTO_NET && ptSocket->s32Socket > 0) {
				if (ptSocket->channel == channel) {
					sendflag = 1;
					break;
				}
			}
		}

		if (sendflag == 0) {
			return 0;
		}

		BUFF_t tBuff = __FillBuff(channel, frame_no, data, datasize, width, height, pts);
		if (tBuff.u8Data != NULL) {
			for (int j = 0; j < MAX_SOCKET_NUM; j++) {
				TSocketInfo *ptSocket = ptSocketList->tSockets + j;

				if (ptSocket->s32Used == TRUE && ptSocket->type == PROTO_NET && ptSocket->s32Socket > 0) {
					if (ptSocket->channel != channel) {
						continue;
					}

					pthread_mutex_lock(&ptSocket->lock);
					int ret = usb_send_head(ptSocket->s32Socket, (unsigned char)SERIAL_MSG_TYPE_PACKS_DATA, tBuff.s32Length);
					if (ret != sizeof(THead)) {
						DEBUG_ERR("send err, s32Socket:%d, send:%d, fillsize:%d, camId:%d", ptSocket->s32Socket, ret, (int)sizeof(THead), channel);
						if (ret <= 0) {
							Ipc_socketDelAndClrSet(ptSocketList, ptSocket->s32Socket);
						}
						pthread_mutex_unlock(&ptSocket->lock);
						continue;
					}

					ret = usb_send(ptSocket->s32Socket, tBuff.u8Data, tBuff.s32Length);
					if (ret != tBuff.s32Length) {
						DEBUG_ERR("send err, send:%d, fillsize:%d, camId:%d", ret, tBuff.s32Length, channel);
						if (ret <= 0) {
							Ipc_socketDelAndClrSet(ptSocketList, ptSocket->s32Socket);
						}
						pthread_mutex_unlock(&ptSocket->lock);
						continue;
					}
					pthread_mutex_unlock(&ptSocket->lock);
					// DEBUG_PRT("send ok !");
				}
			}
			
			free(tBuff.u8Data);
		}
	}

	return 0;
}

#if 0
int Pthread_creat(void *(*start_routine)(void*), void *arg)
{
	pthread_t pid;

	int ret = pthread_create(&pid, NULL, start_routine, arg);
	if(ret != 0) {
		DEBUG_ERR("error[%s,%d]: pthread_create error!!!!!!", __func__, __LINE__);
	}
	pthread_detach(pid);

	return ret;
}

int Read_file2buff(char* ptPath, char* ptData)
{
    if (ptPath == NULL || ptData == NULL)
        return -1;

    if (access(ptPath, F_OK) != 0) {
        return -1;
    }

    struct stat statbuf;
    stat((char*)ptPath, &statbuf);
    if(statbuf.st_size > 0) {
        FILE* fp = fopen(ptPath, "rb");
        if (NULL == fp) {
            DEBUG_ERR("fopen file %s err !\n", ptPath);
            return -1;
        }

        fread(ptData, 1, statbuf.st_size, fp);
        fclose(fp);
    }

    return statbuf.st_size;
}

void Ipc_init(void)
{
	Pthread_creat(Ipc_threadCreat, NULL);
}


int main(int argc, char **argv)
{
	Pthread_creat(Ipc_threadCreat, NULL);
	
	char path0[] = "0.jpg";
	char path1[] = "1.jpg";
	int channel = 0;
	int frame_no = 0;
	unsigned char data0[1024 *1024 * 1];
	unsigned char data1[1024 *1024 * 1];
	int datasize0 = Read_file2buff(path0, data0);
	int datasize1 = Read_file2buff(path1, data1);
	int width = 1600;
	int height = 900;
	unsigned int pts = 0;

	DEBUG_PRT ("path0:%s, datasize0:%d", path0, datasize0);
	DEBUG_PRT ("path1:%s, datasize1:%d", path1, datasize1);

	while(1) {
		if (pts % 2 == 0) {
			for (int i=0; i<8; i++) {
				Ipc_sendJpg(i, frame_no++, data0, datasize0, width, height, pts);
			}
		} else {
			for (int i=0; i<8; i++) {
				Ipc_sendJpg(i, frame_no++, data1, datasize1, width, height, pts);
			}
		}
		pts ++;
		usleep(50*1000);
	}

	return 0;
}
#endif
