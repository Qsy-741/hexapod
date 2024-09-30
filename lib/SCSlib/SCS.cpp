/*
 * SCS.cpp
 * 串行总线舵机的通信层
 * 日期: 2023.6.28
 */

#include <stddef.h>
#include "SCS.h"

// 构造函数，初始化Level和Error
SCS::SCS()
{
	Level = 1; // 除广播命令外，所有命令都返回响应
	Error = 0;
}

// 带End参数的构造函数，初始化Level、End和Error
SCS::SCS(u8 End)
{
	Level = 1;
	this->End = End;
	Error = 0;
}

// 带End和Level参数的构造函数，初始化Level、End和Error
SCS::SCS(u8 End, u8 Level)
{
	this->Level = Level;
	this->End = End;
	Error = 0;
}

// 将一个16位数分成两个8位数
// DataL是低位，DataH是高位
void SCS::Host2SCS(u8 *DataL, u8* DataH, u16 Data)
{
	if(End){
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	}else{
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}

// 将两个8位数组合成一个16位数
// DataL是低位，DataH是高位
u16 SCS::SCS2Host(u8 DataL, u8 DataH)
{
	u16 Data;
	if(End){
		Data = DataL;
		Data<<=8;
		Data |= DataH;
	}else{
		Data = DataH;
		Data<<=8;
		Data |= DataL;
	}
	return Data;
}

// 写入缓冲区
// ID是舵机ID，MemAddr是内存地址，nDat是数据，nLen是数据长度，Fun是功能码
void SCS::writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun)
{
	u8 msgLen = 2;
	u8 bBuf[6];
	u8 CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;
	if(nDat){
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		ID<10?writeSCS(bBuf, 6):writeSCS2(bBuf, 6);
		
	}else{
		bBuf[3] = msgLen;
		ID<10?writeSCS(bBuf, 5):writeSCS2(bBuf, 5);
	}
	CheckSum = ID + msgLen + Fun + MemAddr;
	u8 i = 0;
	if(nDat){
		for(i=0; i<nLen; i++){
			CheckSum += nDat[i];
		}
		ID<10?writeSCS(nDat, nLen):writeSCS2(nDat, nLen);
	}
	ID<10?writeSCS(~CheckSum):writeSCS2(~CheckSum);
}

// 通用写命令
// ID是舵机ID，MemAddr是内存地址，nDat是数据，nLen是数据长度
int SCS::genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
	// rFlushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
	// wFlushSCS();
	return Ack(ID);
}

// 异步写入
// ID是舵机ID，MemAddr是内存地址，nDat是数据，nLen是数据长度
int SCS::regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
	wFlushSCS();
	return Ack(ID);
}

// regWrite()的触发命令
// 调用此函数以启动regWrite()命令
// ID是舵机ID
int SCS::RegWriteAction(u8 ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_REG_ACTION);
	wFlushSCS();
	return Ack(ID);
}

// 同步写入
// ID是舵机ID列表，IDN是ID列表的长度，MemAddr是内存地址，nDat是数据，nLen是数据长度
void SCS::syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	u8 mesLen = ((nLen+1)*IDN+4);
	u8 Sum = 0;
	u8 bBuf[7];
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	writeSCS(bBuf, 7);

	Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
	u8 i, j;
	for(i=0; i<IDN; i++){
		writeSCS(ID[i]);
		writeSCS(nDat+i*nLen, nLen);
		Sum += ID[i];
		for(j=0; j<nLen; j++){
			Sum += nDat[i*nLen+j];
		}
	}
	writeSCS(~Sum);
	wFlushSCS();
}

// 写入一个字节
// ID是舵机ID，MemAddr是内存地址，bDat是数据
int SCS::writeByte(u8 ID, u8 MemAddr, u8 bDat)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

// 写入两个字节
// ID是舵机ID，MemAddr是内存地址，wDat是数据
int SCS::writeWord(u8 ID, u8 MemAddr, u16 wDat)
{
	u8 bBuf[2];
	Host2SCS(bBuf+0, bBuf+1, wDat);
	rFlushSCS();
	writeBuf(ID, MemAddr, bBuf, 2, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

// 读取命令
// ID是舵机ID，MemAddr是内存地址，nData是返回数据，nLen是数据长度
int SCS::Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
	wFlushSCS();
	if(!checkHead(ID)){
		return 0;
	}
	u8 bBuf[4];
	Error = 0;
	if(readSCS(bBuf, 3,ID)!=3){
		return 0;
	}
	int Size = readSCS(nData, nLen,ID);
	if(Size!=nLen){
		return 0;
	}
	if(readSCS(bBuf+3, 1,ID)!=1){
		return 0;
	}
	u8 calSum = bBuf[0]+bBuf[1]+bBuf[2];
	u8 i;
	for(i=0; i<Size; i++){
		calSum += nData[i];
	}
	calSum = ~calSum;
	if(calSum!=bBuf[3]){
		return 0;
	}
	Error = bBuf[2];
	return Size;
}

// 从舵机读取1个字节，超时时返回-1
int SCS::readByte(u8 ID, u8 MemAddr)
{
	u8 bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if(Size!=1){
		return -1;
	}else{
		return bDat;
	}
}

// 从舵机读取2个字节，超时时返回-1
int SCS::readWord(u8 ID, u8 MemAddr)
{	
	u8 nDat[2];
	int Size;
	u16 wDat;
	Size = Read(ID, MemAddr, nDat, 2);
	if(Size!=2)
		return -1;
	wDat = SCS2Host(nDat[0], nDat[1]);
	return wDat;
}

// Ping命令，返回舵机ID，超时时返回-1
int	SCS::Ping(u8 ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_PING);
	wFlushSCS();
	Error = 0;
	if(!checkHead(ID)){
		return -1;
	}
	u8 bBuf[4];
	if(readSCS(bBuf, 4,ID)!=4){
		return -1;
	}
	if(bBuf[0]!=ID && ID!=0xfe){
		return -1;
	}
	if(bBuf[1]!=2){
		return -1;
	}
	u8 calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if(calSum!=bBuf[3]){
		return -1;			
	}
	Error = bBuf[2];
	return bBuf[0];
}

// 检查头部
int SCS::checkHead(uint8_t ID)
{
	u8 bDat;
	u8 bBuf[2] = {0, 0};
	u8 Cnt = 0;
	while(1){
		if(!readSCS(&bDat, 1,ID)){
			return 0;
		}
		bBuf[1] = bBuf[0];
		bBuf[0] = bDat;
		if(bBuf[0]==0xff && bBuf[1]==0xff){
			break;
		}
		Cnt++;
		if(Cnt>10){
			return 0;
		}
	}
	return 1;
}

// 确认命令
int	SCS::Ack(u8 ID)
{
	Error = 0;
	// if(ID!=0xfe && Level){
	// 	if(!checkHead(ID)){
	// 		return 0;
	// 	}
	// 	u8 bBuf[4];
	// 	if(readSCS(bBuf, 4,ID)!=4){
	// 		return 0;
	// 	}
	// 	if(bBuf[0]!=ID){
	// 		return 0;
	// 	}
	// 	if(bBuf[1]!=2){
	// 		return 0;
	// 	}
	// 	u8 calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	// 	if(calSum!=bBuf[3]){
	// 		return 0;			
	// 	}
	// 	Error = bBuf[2];
	// }
	return 1;
}

// 同步读取命令发送
// ID是舵机ID列表，IDN是ID列表的长度，MemAddr是内存地址，nLen是数据长度
int	SCS::syncReadPacketTx(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen)
{
	syncReadRxPacketLen = nLen;
	u8 checkSum = (4+0xfe)+IDN+MemAddr+nLen+INST_SYNC_READ;
	u8 i;
	writeSCS(0xff);
	writeSCS(0xff);
	writeSCS(0xfe);
	writeSCS(IDN+4);
	writeSCS(INST_SYNC_READ);
	writeSCS(MemAddr);
	writeSCS(nLen);
	for(i=0; i<IDN; i++){
		writeSCS(ID[i]);
		checkSum += ID[i];
	}
	checkSum = ~checkSum;
	writeSCS(checkSum);
	return nLen;
}

// 同步读取命令接收
// ID是舵机ID，nDat是返回数据
int SCS::syncReadPacketRx(u8 ID, u8 *nDat)
{
	syncReadRxPacket = nDat;
	syncReadRxPacketIndex = 0;
	u8 bBuf[4];
	if(!checkHead(ID)){
		return 0;
	}
	if(readSCS(bBuf, 3,ID)!=3){
		return 0;
	}
	if(bBuf[0]!=ID){
		return 0;
	}
	if(bBuf[1]!=(syncReadRxPacketLen+2)){
		return 0;
	}
	Error = bBuf[2];
	if(readSCS(nDat, syncReadRxPacketLen,ID)!=syncReadRxPacketLen){
		return 0;
	}
	return syncReadRxPacketLen;
}

// 将同步读取的数据包转换为字节
int SCS::syncReadRxPacketToByte()
{
	if(syncReadRxPacketIndex>=syncReadRxPacketLen){
		return -1;
	}
	return syncReadRxPacket[syncReadRxPacketIndex++];
}

// 将同步读取的数据包转换为字
// negBit是负位
int SCS::syncReadRxPacketToWrod(u8 negBit)
{
	if((syncReadRxPacketIndex+1)>=syncReadRxPacketLen){
		return -1;
	}
	int Word = SCS2Host(syncReadRxPacket[syncReadRxPacketIndex], syncReadRxPacket[syncReadRxPacketIndex+1]);
	syncReadRxPacketIndex += 2;
	if(negBit){
		if(Word&(1<<negBit)){
			Word = -(Word & ~(1<<negBit));
		}
	}
	return Word;
}
