
#ifndef _DATASCOPE_H__
#define _DATASCOPE_H__

//导入Arduino核心头文件
#include"Arduino.h"  
extern unsigned char DataScope_OutPut_Buffer[42];	   


class DATASCOPE
{
public:
DATASCOPE();
void  Float2Byte(float *target,unsigned char *buf,unsigned char beg);
void DataScope_Get_Channel_Data(float Data,unsigned char Channel);
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);
};

#endif