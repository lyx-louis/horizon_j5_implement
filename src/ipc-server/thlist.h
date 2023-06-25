#pragma once

#ifndef __THLIST_H__
#define __THLIST_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define THLOG_LEVEL_NONE			0
#define THLOG_LEVEL_TRACE			1
#define THLOG_LEVEL_DEBUG			2

#define thname(x) 	(char *)(#x)
typedef void *(*thfunc)(void *arg);

void* thlist_creat(void);
int thlist_insert(int canvsa_id, char* name_p, void** handle_pp, thfunc func_p, void* arg_p, int que_size, int log_level);
int thlist_run(void* handle_pp);
int thlist_destory(void* handle_pp, void (*release)(void *arg));
void* thlist_pullQue(void *handle_pp);
int thlist_pushQue(void *handle_pp, void *arg_p, void (*release)(void *arg));
int thlist_extransmit(int thread_id, void *handle_pp, void *arg_p, void (*release)(void *arg));

int thlist_demo(void);

#ifdef __cplusplus
}
#endif

#endif
