#include "PDS_protocol/pdsPalletRequestClass.h"
#include "Function/function.h"
//#include <cstring>
//#include <iostream>
//#include <QDebug>

pdsPalletRequestClass::pdsPalletRequestClass()
{

};

pdsPalletRequestClass::~pdsPalletRequestClass()
{

};

pdsPalletRequestClass::pdsPalletRequestClass(uint32_t commandID, uint16_t palletType)
{
    strncpy(palletRequestStruct.startSequnce,seqStart,4);
    palletRequestStruct.commandID = commandID;
    palletRequestStruct.argsLen = 7;
    palletRequestStruct.palletType = palletType;
    palletRequestStruct.depthHint = 0;
    palletRequestStruct.filterMask = 0;
    strncpy(palletRequestStruct.stopSequence,seqEnd,6);
};

pdsPalletRequestClass::pdsPalletRequestClass(uint32_t commandID, uint16_t palletType,float depthHint)
{
    strncpy(palletRequestStruct.startSequnce,seqStart,4);
    palletRequestStruct.commandID = commandID;
    palletRequestStruct.argsLen = 7;
    palletRequestStruct.palletType = palletType;
    palletRequestStruct.depthHint = depthHint;
    palletRequestStruct.filterMask = 0;
    strncpy(palletRequestStruct.stopSequence,seqEnd,6);
};


QByteArray pdsPalletRequestClass::ToArray()
{
    palletRequestStruct.commandID = swapUInt32(palletRequestStruct.commandID);
    palletRequestStruct.argsLen = swapUInt32(palletRequestStruct.argsLen);
    palletRequestStruct.palletType = swapUInt16(palletRequestStruct.palletType);
    //palletRequestStruct.depthHint = swapInt32(palletRequestStruct.depthHint);

    QByteArray array;
    array.clear();
    array.append((char*)&palletRequestStruct,sizeof(palletRequestStruct));
    reverseByte(array,14,17);

    return array;
}

pdsPalletRequestClass::pdsPalletRequestClass(QByteArray array)
{
    getPalletRequest* data = (getPalletRequest*)array.data();
    data->commandID=swapUInt32(data->commandID);
    data->argsLen=swapUInt32(data->argsLen);
    data->palletType =swapUInt16(data->palletType);
//    data->depthHint = swapInt32(data->depthHint);

    strncpy(palletRequestStruct.startSequnce,data->startSequnce,4);
    palletRequestStruct.commandID =data->commandID;
    palletRequestStruct.argsLen   =data->argsLen;
    palletRequestStruct.palletType =data->palletType;
    palletRequestStruct.depthHint = byte2Float(array,14);
//    palletRequestStruct.depthHint = data->depthHint;
    palletRequestStruct.filterMask =data->filterMask;
    strncpy(palletRequestStruct.stopSequence,data->stopSequence,6);
}

void pdsPalletRequestClass::ToString()
{

};
