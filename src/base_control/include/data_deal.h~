#ifndef DATA_DEAL_H_   
#define DATA_DEAL_H_
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#define u8 unsigned char 

/*
要点：
1.float和unsigned long具有相同的数据结构长度
2.union类型里的数据存放在相同的物理空间
*/
typedef union
{
	float fdata;
	unsigned long ldata;
}FloatLongType;

//double双精度数据处理
typedef union
{
	double ddata;
	unsigned long long ldata;
}DoubleLongType;

void Float_to_Byte(float f,u8 byte[]);
void Byte_to_Float(float *f,u8 byte[]);
void sendfloat(u8 sendbuff[],float data[],int num);
void recvfloat(u8 recvbuff[],float data[],int num);

void Double_to_Byte(double d,u8 byte[]);
void Byte_to_Double(double *d,u8 byte[]);
void senddouble(u8 sendbuff[],double data[],int num);
void recvdouble(u8 recvbuff[],double data[],int num);


#endif
