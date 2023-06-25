#include "ipc_demo.h"
#include "thlist.h"

typedef struct {
	ipc_format_n format;
	int channel;
	int frame_no;
	unsigned char *data;
	int datasize;
	int width;
	int height;
	unsigned int pts;
} ipc_send_t;

static ipc_send_t* Ipc_malloc(int channel, int frame_no, unsigned char *data, int datasize, int width, int height, unsigned int pts, ipc_format_n format)
{
	if (data == NULL || datasize <= 0) {
		return NULL;
	}

	ipc_send_t* tmp = (ipc_send_t *)malloc(sizeof(ipc_send_t));
	if (tmp != NULL) {
		tmp->data = (unsigned char *)malloc(datasize);
		if (tmp->data != NULL) {
			memcpy(tmp->data, data, datasize);
		} else {
			free(tmp);
			return NULL;
		}
		tmp->channel = channel;
		tmp->frame_no = frame_no;
		tmp->datasize = datasize;
		tmp->width = width;
		tmp->height = height;
		tmp->pts = pts;
		tmp->format = format;
	}

	return tmp;
}

static void Ipc_release(void *arg)
{
	ipc_send_t* data = (ipc_send_t*) arg;

	if (data != NULL && data->data != NULL) {
		free(data->data);
		data->data = NULL;
		free(data);
		data = NULL;
	}
}

static void* Ipc_send(void* handle)
{
	for (;;) {
		ipc_send_t* data = (ipc_send_t*) thlist_pullQue(handle);
		if(data == NULL) {
			DEBUG_ERR("exit !");
			break;
		}

		if (data->format == FORMAT_JPG) {
			Ipc_sendJpg(data->channel, data->frame_no, data->data, data->datasize, data->width, data->height, data->pts);
		} else if (data->format == FORMAT_BIN) {
			Ipc_sendPoints(data->channel, data->frame_no, data->data, data->datasize, data->width, data->height, data->pts);
		} else if (data->format == FORMAT_NV12) {
			Ipc_sendNV12(data->channel, data->frame_no, data->data, data->datasize, data->width, data->height, data->pts);
		}

		if (thlist_pushQue(handle, data, Ipc_release) < 0) {
			Ipc_release(data);
		}
	}
	return NULL;
}

static void *Ipc_show(void* arg)
{
	int camId = (uintptr_t)arg;
	void* thlist = thlist_creat();
	thlist_insert(camId, thname(Ipc_send), &thlist, Ipc_send, NULL, 2, THLOG_LEVEL_TRACE);
	thlist_run(thlist);
	return thlist;
}

int Ipc_pthread(void *(*start_routine)(void*), void *arg)
{
	pthread_t pid;

	int ret = pthread_create(&pid, NULL, start_routine, arg);
	if(ret != 0) {
		DEBUG_ERR("error[%s,%d]: pthread_create error!!!!!!", __func__, __LINE__);
	}
	pthread_detach(pid);

	return ret;
}

ipc_set_channel_t g_real_channel = {0};
void* g_handle[MAX_CHANNEL] = {0};

int Ipc_init(ipc_set_channel_t t_set_channel)
{
	if (t_set_channel.cam_num <= MAX_CHANNEL) {
		g_real_channel = t_set_channel;
	} else {
		DEBUG_WARN("Err, cam_num:%d > MAX_CHANNEL:%d !", t_set_channel.cam_num, MAX_CHANNEL);
		return -1;
	}

	Ipc_pthread(Ipc_threadCreat, NULL);

	for (int i=0; i<g_real_channel.cam_num; i++) {
		g_handle[i] = Ipc_show((void *)(uintptr_t)i);
	}

	return 0;
}

double get_sys_time(void)
{
	struct timespec tp;

	clock_gettime(CLOCK_MONOTONIC, &tp);

	return tp.tv_sec * 1000. + tp.tv_nsec / 1000000.;
}

void Ipc_sendData(int channel, int frame_no, unsigned char *data, int datasize, int width, int height, unsigned int pts, ipc_format_n format)
{
	int num = -1;

	for (int i=0; i<g_real_channel.cam_num; i++) {
		if (g_real_channel.cam_channel[i] == channel) {
			num = i;
			break;
		}
	}

	if (num < 0) {
		DEBUG_ERR("no find channel:%d", channel);
		return;
	}

	ipc_send_t *buff = Ipc_malloc(channel, frame_no, data, datasize, width, height, pts, format);
	if (buff != NULL) {
		if (thlist_extransmit(0, g_handle[num], buff, Ipc_release) < 0) {
			Ipc_release(buff);
		}
	}
	// DEBUG_PRT("channel:%d, frame_no:%d, datasize:%d, width:%d, height:%d, pts:%u, tv:%lf ms", channel, frame_no, datasize, width, height, pts, get_sys_time());
}


// 1. 车道线点位映射的文件在前,文件格式:size(4) + x0(4) + y0(4) + x1(4) + y1(4) ... (均为 float 类型), 总共 size 个点
// 2. BEV点位映射的文件在后,文件格式:obj num(4) + x0(4) + y0(4) + x1(4) + y1(4) ... (均为 float 类型), 每个目标对应4个点位,总共4 * (num)个点


