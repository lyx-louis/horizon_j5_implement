#include "thlist.h"
#include "blockingconcurrentqueue.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/syscall.h>

#define gettid() syscall(SYS_gettid)

#ifdef USE_ZLOG
#include "base_define.h"
#define log_trace	DEBUG_TRACE
#define log_prt		DEBUG_PRT
#define log_info	DEBUG_INFO
#define log_warn	DEBUG_WARN
#define log_err		DEBUG_ERR

#else

#ifndef nodir
#define nodir(x)  strrchr(x, '/') ? (strrchr(x, '/') + 1) : x
#endif

#define log_trace(fmt, ...) printf("\033[1;30m[TRACE] [%s|%s|%d]: " fmt "\033[0m" "\n", nodir(__FILE__), __func__, __LINE__, ##__VA_ARGS__);

#define log_prt(fmt, ...) 	printf("\033[1;32m[DEBUG] [%s|%s|%d]: " fmt "\033[0m" "\n", nodir(__FILE__), __func__, __LINE__, ##__VA_ARGS__);

#define log_info(fmt, ...) 	printf("\033[1;34m[INFO] [%s|%s|%d]: " fmt "\033[0m" "\n", nodir(__FILE__), __func__, __LINE__, ##__VA_ARGS__);

#define log_warn(fmt, ...) 	printf("\033[1;33m[WARN] [%s|%s|%d]: " fmt "\033[0m" "\n", nodir(__FILE__), __func__, __LINE__, ##__VA_ARGS__);

#define log_err(fmt, ...) 	printf("\033[1;31m[ERROR] [%s|%s|%d]: " fmt "\033[0m" "\n", nodir(__FILE__), __func__, __LINE__, ##__VA_ARGS__);
#endif

#if 0
/*=============================== 队列 ===============================*/

#define FAILURE				-1
#define SUCCESS				0

#define TIMEOUT_NONE      	0
#define TIMEOUT_FOREVER   	-1

typedef struct TagQueHndl
{
  int s32CurRd;      /* 当前队列读索引值   */
  int s32CurWr;      /* 当前队列写索引值   */
  int s32Maxlen;     /* 当前队列最大节点数 */
  int s32Count;      /* 当前队列节点数量   */

  void **pptQueue;     /* 当前队列节点列表   */

  pthread_mutex_t tLock;   /* 当前队列互斥锁   */
  pthread_cond_t  tCondRd; /* 当前队列读信号量 */
  pthread_cond_t  tCondWr; /* 当前队列写信号量 */
} TQueHndl;

static TQueHndl* QueCreate(int s32MaxLen)
{
    pthread_mutexattr_t tMutexAttr;
    pthread_condattr_t tCondAttr;
    int s32Result = SUCCESS;
	TQueHndl *ptHndl = NULL;

    ptHndl = (TQueHndl *)malloc(sizeof(TQueHndl));
    if (ptHndl == NULL) {
	    return NULL;
    }

	memset(ptHndl, 0, sizeof(TQueHndl));
	ptHndl->s32Maxlen = s32MaxLen;
	ptHndl->pptQueue = (void **)malloc(sizeof(void *)*s32MaxLen);
	if(ptHndl->pptQueue==NULL) {
		free(ptHndl);
		return NULL;
	}

	memset(ptHndl->pptQueue, 0, sizeof(void *)*s32MaxLen);

	s32Result |= pthread_mutexattr_init(&tMutexAttr);
	s32Result |= pthread_condattr_init(&tCondAttr);  

	s32Result |= pthread_mutex_init(&ptHndl->tLock, &tMutexAttr);
	s32Result |= pthread_cond_init(&ptHndl->tCondRd, &tCondAttr);    
	s32Result |= pthread_cond_init(&ptHndl->tCondWr, &tCondAttr);  


	pthread_condattr_destroy(&tCondAttr);
	pthread_mutexattr_destroy(&tMutexAttr);
	return ptHndl;
}

static int QueDelete(TQueHndl *ptHndl)
{
	if(ptHndl == NULL)
	{
		return FAILURE;
	}
	if (ptHndl->pptQueue != NULL)
	{
		free(ptHndl->pptQueue);
	}

	pthread_cond_destroy(&ptHndl->tCondRd);
	pthread_cond_destroy(&ptHndl->tCondWr);
	pthread_mutex_destroy(&ptHndl->tLock); 
	free(ptHndl);
	return SUCCESS;
}

static int QuePut(TQueHndl *ptHndl, void * ptData, int s32TimeOut)
{
	int s32Result = FAILURE;

	if(ptHndl == NULL)
	{
		return FAILURE;
	}
	pthread_mutex_lock(&ptHndl->tLock);

	while(1) 
	{
	    if( ptHndl->s32Count < ptHndl->s32Maxlen ) 
		{
			ptHndl->pptQueue[ptHndl->s32CurWr] = ptData;
			ptHndl->s32CurWr = (ptHndl->s32CurWr+1)%ptHndl->s32Maxlen;
			ptHndl->s32Count++;
			s32Result = SUCCESS;
			pthread_cond_signal(&ptHndl->tCondRd);
			break;
		} 
		else 
		{
			if(s32TimeOut == TIMEOUT_NONE)
			{
				break;
			}
			else if (s32TimeOut == TIMEOUT_FOREVER)
			{
				s32Result = pthread_cond_wait(&ptHndl->tCondWr, &ptHndl->tLock);
			}
			else
			{
				struct timeval tCurTime;
				struct timespec tWaitTime;

				gettimeofday(&tCurTime, NULL);
				tWaitTime.tv_sec = tCurTime.tv_sec + s32TimeOut/1000;
				tWaitTime.tv_nsec = (tCurTime.tv_usec  + 1000*(s32TimeOut%1000))*1000;
				s32Result = pthread_cond_timedwait(&ptHndl->tCondWr, &ptHndl->tLock, &tWaitTime);
				if(s32Result == ETIMEDOUT)
				{
					s32Result = FAILURE;
					break;
				}
			}
		}
	}

	pthread_mutex_unlock(&ptHndl->tLock);
	return s32Result;
}

static void* QueGet(TQueHndl *ptHndl, int s32TimeOut)
{
	int s32Result = FAILURE;
	void *ptData = NULL;

	if(ptHndl == NULL)
	{
		return NULL;
	}
	pthread_mutex_lock(&ptHndl->tLock);
	while(1) 
	{
		if(ptHndl->s32Count > 0 ) 
		{
			ptData = (void *)ptHndl->pptQueue[ptHndl->s32CurRd];
			ptHndl->s32CurRd = (ptHndl->s32CurRd+1)%ptHndl->s32Maxlen;
			ptHndl->s32Count--;
			pthread_cond_signal(&ptHndl->tCondWr);
			break;
		}
		else 
		{
			if(s32TimeOut == TIMEOUT_NONE)
			{
				break;
			}
			else if (s32TimeOut == TIMEOUT_FOREVER)
			{
				s32Result = pthread_cond_wait(&ptHndl->tCondRd, &ptHndl->tLock);
			}
			else
			{
				struct timeval tCurTime;
				struct timespec tWaitTime;

				gettimeofday(&tCurTime, NULL);
				tWaitTime.tv_sec = tCurTime.tv_sec + s32TimeOut/1000;
				tWaitTime.tv_nsec = (tCurTime.tv_usec  + 1000*(s32TimeOut%1000))*1000;

				s32Result = pthread_cond_timedwait(&ptHndl->tCondRd, &ptHndl->tLock, &tWaitTime);
				if(s32Result == ETIMEDOUT)
				{
					break;
				}
			}
		}
	}
	pthread_mutex_unlock(&ptHndl->tLock);
	return ptData;
}

static int QueSizeGet(TQueHndl *ptHndl)
{
	if(ptHndl == NULL) {
		return FAILURE;
	}

	int s32Count = 0;
	pthread_mutex_lock(&ptHndl->tLock);
	s32Count = ptHndl->s32Count;
	pthread_mutex_unlock(&ptHndl->tLock);

	return s32Count;
}

/*=============================== 线程串 ===============================*/
#endif

double get_systimems(void)
{
	struct timespec tp;

	clock_gettime(CLOCK_MONOTONIC, &tp);

	return tp.tv_sec * 1000. + tp.tv_nsec / 1000000.;
}

struct threadlist
{
	int tid;
	int canvsa_id;
	int	thread_id;
	std::string name;
	int que_size;
	thfunc func_p;
	void* arg_p;
	moodycamel::BlockingConcurrentQueue<void *> queue;
	threadlist* next_p;
	// log
	int log_level;
	int log_fps;
	double log_time;
	double pull_time;
	double push_time;
	double avm_time;
};

void* thlist_creat(void)
{
	return NULL;
}

// 插入到最后
static void insert_back(threadlist** thlist_pp, threadlist* node_p)
{
	if (*thlist_pp == NULL) {
		*thlist_pp = node_p;
	} else {
		threadlist* tail = *thlist_pp;

		while (tail->next_p != NULL) {
			tail = tail->next_p;
		}

		tail->next_p = node_p;
	}
}

// 获取当前链表节点个数
static int get_node_number(threadlist* thlist_pp)
{
	int depth = 0;
	threadlist* curt_p = thlist_pp;

	if (curt_p == NULL) {
		return depth;
	} else {
		threadlist* tail = curt_p;

		depth ++;
		while (tail->next_p != NULL) {
			depth ++;
			tail = tail->next_p;
		}
	}

	return depth;
}

int thlist_insert(int canvsa_id, char* name_p, void** handle_pp, thfunc func_p, void* arg_p, int que_size, int log_level)
{
	threadlist** thlist_pp = (threadlist**) handle_pp;
	if (thlist_pp == NULL || func_p == NULL || que_size < 1) {
		log_err("input is error");
		return -1;
	}

	threadlist *node_p = new threadlist(); // (threadlist *)malloc(sizeof(threadlist));
	if (node_p == NULL) {
		log_err("malloc threadlist failed");
		return -1;
	}

	if (name_p != NULL) {
		node_p->name = name_p;
	} else {
		node_p->name = "null";
	}
	node_p->tid = gettid();
	node_p->canvsa_id = canvsa_id;
	node_p->thread_id = get_node_number(*thlist_pp);
	node_p->que_size = que_size;
	node_p->func_p = func_p;
	node_p->arg_p = arg_p;
	node_p->next_p = NULL;
	// log
	node_p->log_level = log_level;
	node_p->log_fps = 0;
	node_p->log_time = 0;
	node_p->pull_time = 0;
	node_p->push_time = 0;
	node_p->avm_time = 0;

	log_info("insert node, tid:%d, canvsa_id:%d, thread_id:%d, name:%s, depth:%d !", node_p->tid, node_p->canvsa_id, node_p->thread_id, node_p->name.c_str(), node_p->que_size);
	insert_back(thlist_pp, node_p);

	return 0;
}

int thlist_run(void* handle_pp)
{
	threadlist* thlist_pp = (threadlist*) handle_pp;
	if (thlist_pp == NULL) {
		log_err("input is null");
		return -1;
	}

	threadlist* curt_p = thlist_pp;

	while(curt_p != NULL) {
		pthread_t pid;
		if (pthread_create(&pid, NULL, curt_p->func_p, curt_p) != 0) {
			log_err("pthread_create failed, tid:%d, canvsa_id:%d, thread_id:%d, name:%s, depth:%d !", curt_p->tid, curt_p->canvsa_id, curt_p->thread_id, curt_p->name.c_str(), curt_p->que_size);
			delete curt_p; // free(curt_p);
		} else {
			pthread_detach(pid);
		}
		log_info("thlist run, tid:%d, canvsa_id:%d, thread_id:%d, name:%s, depth:%d !", curt_p->tid, curt_p->canvsa_id, curt_p->thread_id, curt_p->name.c_str(), curt_p->que_size);

		curt_p = curt_p->next_p;
	}

	return 0;
}

int thlist_destory(void* handle_pp, void (*release)(void *arg))
{
	threadlist* thlist_pp = (threadlist*) handle_pp;
	if (thlist_pp == NULL) {
		log_err("input is null");
		return -1;
	}

	threadlist* curt_p = thlist_pp;

	while (curt_p != NULL) {
		threadlist* next_p = curt_p->next_p;
		while(curt_p->queue.size_approx() > 0) {
			void *tmp = NULL;
			bool ret = curt_p->queue.try_dequeue(tmp);
			if (ret) {
				// 释放
				if (release != NULL) {
					release(tmp);
				}
			}
		}
		delete curt_p; // free(curt_p);
		curt_p = next_p;
	}

	return 0;
}

void* thlist_pullQue(void *handle_pp)
{
	threadlist* curt_p = (threadlist*) handle_pp;
	if (curt_p == NULL) {
		log_err("input is null");
		return NULL;
	}

	void* data = NULL;
	curt_p->queue.wait_dequeue(data);
	// curt_p->queue.wait_dequeue_timed(data, 1000); // 1 ms timeout

	if (curt_p->log_level)
		curt_p->pull_time = get_systimems();

	return data;
}

// 注意:返回 -1 时需手动释放 arg_p
int thlist_pushQue(void *handle_pp, void *arg_p, void (*release)(void *arg))
{
	threadlist* curt_p = (threadlist*) handle_pp;
	if (curt_p == NULL) {
		log_err("input is null");
		return -1;
	}

	if (curt_p->log_level) {
		curt_p->push_time = get_systimems();
		curt_p->log_fps ++;
		curt_p->avm_time += (curt_p->push_time - curt_p->pull_time);
		if (curt_p->log_time < 1) {
			curt_p->log_time = curt_p->push_time + 5000;
		} else {
			if (curt_p->push_time > curt_p->log_time) {
				if (curt_p->log_level == THLOG_LEVEL_TRACE) {
					log_trace("tid:%d, canvsa_id:%d, thread_id:%d, name:%s, all:%d, fps:%0.1f, avetime:%0.1f ms", curt_p->tid, curt_p->canvsa_id, curt_p->thread_id, curt_p->name.c_str(), curt_p->log_fps, (float)curt_p->log_fps/5.0, curt_p->avm_time/curt_p->log_fps);
				} else {
					log_prt("tid:%d, canvsa_id:%d, thread_id:%d, name:%s, all:%d, fps:%0.1f, avetime:%0.1f ms", curt_p->tid, curt_p->canvsa_id, curt_p->thread_id, curt_p->name.c_str(), curt_p->log_fps, (float)curt_p->log_fps/5.0, curt_p->avm_time/curt_p->log_fps);
				}
				curt_p->log_fps = 0;
				curt_p->avm_time = 0;
				curt_p->log_time = 0;
			}
		}
	}

	threadlist* next_p = curt_p->next_p;

	if (next_p == NULL) {
		if (release == NULL) {
			// printf("[WARN] next_p is null \n");
			return -1;
		} else {
			// log_prt("no next node, release arg !");
			release(arg_p);
			return 0;
		}
	}

	int size_approx = next_p->queue.size_approx();
	if (size_approx >= next_p->que_size) {
		if (release == NULL) {
			log_warn("tid:%d, canvsa_id:%d, thread_id:%d, name:%s, size_approx:%d >= que_size:%d", next_p->tid, next_p->canvsa_id, next_p->thread_id, next_p->name.c_str(), size_approx, next_p->que_size);
			return -1;
		} else {
			next_p->queue.enqueue(arg_p);
			bool ret = next_p->queue.try_dequeue(arg_p);
			if (ret) {
				release(arg_p);
			}
		}
	} else {
		next_p->queue.enqueue(arg_p);
	}

	return 0;
}

// 注意:返回 -1 时需手动释放 arg_p
int thlist_extransmit(int thread_id, void *handle_pp, void *arg_p, void (*release)(void *arg))
{
	threadlist* thlist_pp = (threadlist*) handle_pp;
	if (thlist_pp == NULL) {
		log_err("input is null");
		return -1;
	}

	threadlist* curt_p = thlist_pp;

	while (curt_p != NULL) {
		if (curt_p->thread_id == thread_id) {
			int size_approx = curt_p->queue.size_approx();
			if (size_approx >= curt_p->que_size) {
				if (release == NULL) {
					log_warn("tid:%d, canvsa_id:%d, thread_id:%d, name:%s, size_approx:%d >= que_size:%d", curt_p->tid, curt_p->canvsa_id, curt_p->thread_id, curt_p->name.c_str(), size_approx, curt_p->que_size);
					return -1;
				} else {
					curt_p->queue.enqueue(arg_p);
					bool ret = curt_p->queue.try_dequeue(arg_p);
					if (ret) {
						release(arg_p);
					}
				}
			} else {
				curt_p->queue.enqueue(arg_p);
			}
			return 0;
		}
		curt_p = curt_p->next_p;
	}

	return -1;
}


// demo
#if 1
static void* thlist = NULL;

typedef struct {
	int a;
} test_t;

static test_t* test_creat(void)
{
	return (test_t *)malloc(sizeof(test_t));
}

static void test_release(void *arg)
{
	test_t* test_p = (test_t*) arg;

	if (test_p != NULL) {
		log_prt("release value:%d", test_p->a);
		free(test_p);
		test_p = NULL;
	}
}

static void* func1(void* handle)
{
	for (;;) {
		test_t* tmp = (test_t*) thlist_pullQue(handle);
		if(tmp == NULL) {
			log_warn("exit !");
			break;
		}

		log_prt("run value:%d !", tmp->a);
		if (thlist_pushQue(handle, tmp, test_release) < 0) {
			test_release(tmp);
		}
	}
	return NULL;
}

static void* func2(void* handle)
{
	for (;;) {
		test_t* tmp = (test_t*) thlist_pullQue(handle);
		if(tmp == NULL) {
			log_warn("exit !");
			break;
		}

		log_prt("run value:%d !", tmp->a);
		if (thlist_pushQue(handle, tmp, test_release) < 0) {
			test_release(tmp);
		}
	}
	return NULL;
}

static void* func3(void* handle)
{
	for (;;) {
		test_t* tmp = (test_t*) thlist_pullQue(handle);
		if(tmp == NULL) {
			log_warn("exit !");
			break;
		}

		log_prt("run value:%d !", tmp->a);
		if (thlist_pushQue(handle, tmp, test_release) < 0) {
			test_release(tmp);
		}
	}
	return NULL;
}

int thlist_demo(void)
{
	int count = 0;
	thlist = thlist_creat();

	thlist_insert(count, thname(func1), &thlist, func1, NULL, 10, THLOG_LEVEL_NONE);
	thlist_insert(count, thname(func2), &thlist, func2, NULL, 10, THLOG_LEVEL_NONE);
	thlist_insert(count, thname(func3), &thlist, func3, NULL, 10, THLOG_LEVEL_NONE);

	thlist_run(thlist);

	for(;;){
		test_t* tmp = test_creat();
		if (tmp != NULL) {
			tmp->a = count ++;
			if (thlist_extransmit(0, thlist, tmp, test_release) < 0) {
				test_release(tmp);
			}
		}

		usleep(10);

		if (count > 100) {
			break;
		}
	}

	sleep(1);
	thlist_destory(thlist, test_release);

	return 0;
}

#ifdef THLIST_MAIN
int main (int argc, char** argv)
{
	thlist_demo();

	return 0;
}
#endif

#endif
