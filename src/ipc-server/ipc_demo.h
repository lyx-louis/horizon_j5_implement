#ifndef _IPC_DEMO_H_
#define _IPC_DEMO_H_

#include "ipc_server.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_CHANNEL		16

typedef struct {
	int cam_num;
	int cam_channel[MAX_CHANNEL];
} ipc_set_channel_t;

int Ipc_init(ipc_set_channel_t t_set_channel);

typedef enum {
	FORMAT_JPG
,	FORMAT_BIN
,	FORMAT_NV12
} ipc_format_n;

void Ipc_sendData(int channel, int frame_no, unsigned char *data, int datasize, int width, int height, unsigned int pts, ipc_format_n format);

#ifdef __cplusplus
}
#endif

#endif
