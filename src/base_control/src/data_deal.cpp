#include "data_deal.h"

//将浮点数f转化为4个字节数据存放在byte[4]中
void Float_to_Byte(float f,u8 byte[])
{
	FloatLongType fl;
	fl.fdata=f;
	byte[0]=(unsigned char)fl.ldata;
	byte[1]=(unsigned char)(fl.ldata>>8);
	byte[2]=(unsigned char)(fl.ldata>>16);
	byte[3]=(unsigned char)(fl.ldata>>24);
}

//发送数据处理函数，将数据转化为字节存储到发送缓存区
//sendbuff:发送数据缓存区
//data:发送数组
//num:数组长度
void sendfloat(u8 sendbuff[],float data[],int num)
{
	int i,j;
	u8 byte[4];
	//数据开头
	sendbuff[0]=0xff;
	sendbuff[1]=0xff;
	for(i=0;i<num;i++)
	{
		Float_to_Byte(data[i],byte);   //拆解数据
		for(j=0;j<4;j++)       //拆解之后的字节存入发送缓存区
		{
			sendbuff[i*4+j+2] = byte[j];
		}	
	}
	sendbuff[4*num+2]='\r';
	sendbuff[4*num+3]='\n';	
}
