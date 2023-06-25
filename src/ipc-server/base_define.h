#ifndef __BESE_DEFINE_H__
#define __BESE_DEFINE_H__

#ifdef USE_ZLOG
#include "zlog_user.h"
#endif

/*C标准库头文件*/
#include <assert.h>		/*断言*/
#include <complex.h>	/*复数*/
#include <ctype.h>		/*字符处理*/
#include <errno.h>		/*错误类型*/
#include <fenv.h>		/*浮点环境*/
#include <float.h>		/*浮点类型*/
#include <inttypes.h>	/*整数类型格式转换*/
#include <iso646.h>		/*拼写替换*/
#include <limits.h>		/*整数类型大小*/
#include <locale.h>		/*本地化*/
#include <math.h>		/*数学运算*/
#include <setjmp.h>		/*非局部跳转*/
#include <signal.h>		/*信号处理*/
#include <stdarg.h>		/*可变参数*/
#include <stdbool.h>	/*布尔类型和值*/
#include <stddef.h>		/*通用定义*/
#include <stdint.h>		/*整数类型*/
#include <stdio.h>		/*输入输出*/
#include <stdlib.h>		/*常用工具*/
#include <string.h>		/*字符串处理*/
#include <time.h>		/*时间和日期*/
#include <wchar.h>		/*扩展的多字节/宽字符工具*/
#include <wctype.h>		/*宽字符分类和映射工具*/

/*系统相关*/
#if (defined(_WIN32) || defined(_WIN64))
#	include <winsock2.h>
#	include <windows.h>
#	include <direct.h>
#	include <synchapi.h>
#	include <io.h>
#	include <locale.h>
#	include <Shlwapi.h>
#	include <Windows.h>
#	include <Mmsystem.h>
#	include <winioctl.h>
#	include <assert.h>
#	include <process.h>
#	include <memory.h>
#	include <Iphlpapi.h>
#	include <ws2tcpip.h>
#	include <wtypes.h>
#	include <sys/timeb.h>
#elif defined(__linux__)
#	include <pthread.h>
#	include <unistd.h>
#	include <fcntl.h>
#	include <signal.h>
#	include <sys/ioctl.h>
#	include <sys/param.h>
#	include <sys/types.h>
#	include <sys/stat.h>
#	include <sys/time.h>
#	include <sys/timerfd.h>
#	include <sys/wait.h>
#	include <sys/prctl.h>
#	include <sys/syscall.h>
#	include <sys/select.h>
#	include <sys/socket.h>
#	include <sys/epoll.h>
#	include <sys/mman.h>
#	include <sys/un.h>
#	include <sys/mount.h>

#	include <semaphore.h>
#	include <dirent.h>
#	include <termios.h>
#	include <memory.h>
#	include <limits.h>
#	include <errno.h>

#	include <linux/fs.h>
#	include <linux/spi/spidev.h>
#	include <linux/rtc.h>
#	include <linux/netlink.h>  
#	include <linux/rtnetlink.h>

#	include <linux/can.h>
#	include <linux/can/raw.h>

#	include <ifaddrs.h>
#	include <arpa/inet.h>
#	include <netinet/in.h>
#	include <netinet/ip.h>
#	include <netinet/tcp.h>
#	include <net/if.h>
#	include <net/route.h>
#	include <netdb.h>

/*Android 头文件是Linux子集*/
#	if defined(__ANDROID__)
#		include <android/log.h>
#		include <pthread.h>
#		include <fcntl.h>
#		include <termios.h>
#	endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef NODIR
#define NODIR(x)  strrchr(x, '/') ? (strrchr(x, '/') + 1) : x
#endif

#ifndef DEBUG_TRACE
#define DEBUG_TRACE(fmt, ...) printf("\033[1;30m[TRACE] [%s|%s|%d]: " fmt "\033[0m" "\n", NODIR(__FILE__), __func__, __LINE__, ##__VA_ARGS__);
#endif

#ifndef DEBUG_PRT
#define DEBUG_PRT(fmt, ...) printf("\033[1;32m[DEBUG] [%s|%s|%d]: " fmt "\033[0m" "\n", NODIR(__FILE__), __func__, __LINE__, ##__VA_ARGS__);
#endif

#ifndef DEBUG_INFO
#define DEBUG_INFO(fmt, ...) printf("\033[1;34m[INFO] [%s|%s|%d]: " fmt "\033[0m" "\n", NODIR(__FILE__), __func__, __LINE__, ##__VA_ARGS__);
#endif

#ifndef DEBUG_WARN
#define DEBUG_WARN(fmt, ...) printf("\033[1;33m[WARN] [%s|%s|%d]: " fmt "\033[0m" "\n", NODIR(__FILE__), __func__, __LINE__, ##__VA_ARGS__);
#endif

#ifndef DEBUG_ERR
#define DEBUG_ERR(fmt, ...) printf("\033[1;31m[ERROR] [%s|%s|%d]: " fmt "\033[0m" "\n", NODIR(__FILE__), __func__, __LINE__, ##__VA_ARGS__);
#endif

#ifdef __cplusplus
}
#endif

#endif
