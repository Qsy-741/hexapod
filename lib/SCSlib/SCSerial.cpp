/*
 * SCSerial.h
 * hardware interface layer for waveshare serial bus servo
 * date: 2023.6.28
 */


#include "SCSerial.h"

SCSerial::SCSerial()
{
	IOTimeOut = 100;
	pSerial = NULL;
	pSerial2 = NULL;
}

SCSerial::SCSerial(u8 End):SCS(End)
{
	IOTimeOut = 100;
	pSerial = NULL;
	pSerial2 = NULL;
}

SCSerial::SCSerial(u8 End, u8 Level):SCS(End, Level)
{
	IOTimeOut = 1000;
	pSerial = NULL;
	pSerial2 = NULL;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen,uint8_t id)
{
	int Size = 0;
	int ComData;
	unsigned long t_begin = millis();
	unsigned long t_user;

	while(1){
		if(id<10){
			ComData = pSerial->read();//左右两个串口控制
		}else{
			ComData = pSerial2->read();
		}
		if(ComData!=-1){
			if(nDat){
				nDat[Size] = ComData;
			}
			Size++;
			t_begin = millis();
		}
		if(Size>=nLen){
			break;
		}
		t_user = millis() - t_begin;
		if(t_user>IOTimeOut){
			break;
		}
	}
	return Size;
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return pSerial->write(nDat, nLen);
}
int SCSerial::writeSCS2(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return pSerial2->write(nDat, nLen);
}

int SCSerial::writeSCS(unsigned char bDat)
{
	return pSerial->write(&bDat, 1);
}
int SCSerial::writeSCS2(unsigned char bDat)
{
	return pSerial2->write(&bDat, 1);
}
void SCSerial::rFlushSCS()
{
	while(pSerial->read()!=-1);
}

void SCSerial::wFlushSCS()
{
}