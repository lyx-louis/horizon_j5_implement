/***************************************************************
模块名	     ：类型定义
文件名	     ：oal_type.h
相关文件	 ：
文件实现功能 ：
作者		 ：fengquanyou
版本		 ：V1.0
-----------------------------------------------------------------
修改记录:
日  期		    版本		修改人		修改内容
2022/01/20		1.0         fengquanyou 初始版本
*****************************************************************/

#ifndef __OAL_TYPE__
#define __OAL_TYPE__

/*基本类型定义*/
typedef char 					s8;
typedef unsigned char 			u8;
typedef short 					s16;
typedef unsigned short 			u16;
typedef int						s32;
typedef unsigned int 			u32;
typedef long long 				s64;
typedef unsigned long long 		u64;

typedef float					f32;
typedef double 					f64;

typedef struct tm				tm;

/*枚举类型定义*/
enum EEndianMode
{
	EndianMode_Big,       /*大端模式*/
	EndianMode_Little,    /*小端模式*/
};

#endif

