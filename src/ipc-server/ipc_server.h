#ifndef _IPC_SERVER_H_
#define _IPC_SERVER_H_

#include "ipc_manager.h"

#ifdef __cplusplus
extern "C"
{
#endif

void *Ipc_threadCreat(void* arg);
int Ipc_sendJpg(int channel, int frame_no, void *data, int datasize, int width, int height, unsigned int pts);
int Ipc_sendPoints(int channel, int frame_no, void *data, int datasize, int width, int height, unsigned int pts);
int Ipc_sendNV12(int channel, int frame_no, void *data, int datasize, int width, int height, unsigned int pts);

#ifdef __cplusplus
}
#endif

#endif
