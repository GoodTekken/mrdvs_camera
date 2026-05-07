#include "pdsSafeZoneCheckResponseClass.h"
#include "Function/function.h" 

pdsSafeZoneCheckResponseClass::pdsSafeZoneCheckResponseClass()
{

}

pdsSafeZoneCheckResponseClass::~pdsSafeZoneCheckResponseClass()
{

}

pdsSafeZoneCheckResponseClass::pdsSafeZoneCheckResponseClass(int errorCode,bool safezoneExist,float safezone_max_left,float safezone_max_right)    //Number of pixels in the VOI
{
    strncpy(safeZoneCheckResponseStruct.startSequnce,seqStart,4);
    safeZoneCheckResponseStruct.commandID=6;
    safeZoneCheckResponseStruct.errorCode=errorCode;
    safeZoneCheckResponseStruct.len=9;
    safeZoneCheckResponseStruct.safezoneExist = safezoneExist;
    safeZoneCheckResponseStruct.safezone_max_left = safezone_max_left;
    safeZoneCheckResponseStruct.safezone_max_right = safezone_max_right;
    strncpy(safeZoneCheckResponseStruct.stopSequence,seqEnd,6);
}
QByteArray pdsSafeZoneCheckResponseClass::ToArray()
{
    safeZoneCheckResponseStruct.commandID = swapUInt32(safeZoneCheckResponseStruct.commandID);
    safeZoneCheckResponseStruct.errorCode = swapInt32(safeZoneCheckResponseStruct.errorCode);
    safeZoneCheckResponseStruct.len = swapUInt32(safeZoneCheckResponseStruct.len);
    safeZoneCheckResponseStruct.safezoneExist = safeZoneCheckResponseStruct.safezoneExist;
    QByteArray array;
    array.clear();
    array.append((char*)&safeZoneCheckResponseStruct,sizeof(safeZoneCheckResponseStruct));
    for(uint32_t i =0;i<2;i++)      //float transform
    {
        uint32_t index = 17 + 4*i;    //
        reverseByte(array,index,index+3);
    }

    return array;
}


pdsSafeZoneCheckResponseClass::pdsSafeZoneCheckResponseClass(QByteArray array)
{
    safeZoneCheckResponse* data = (safeZoneCheckResponse*)array.data();
    data->commandID=swapUInt32(data->commandID);
    data->errorCode=swapInt32(data->errorCode);
    data->len=swapUInt32(data->len);
    data->safezoneExist= data->safezoneExist;

    strncpy(safeZoneCheckResponseStruct.startSequnce,data->startSequnce,4);
    safeZoneCheckResponseStruct.commandID =data->commandID;
    safeZoneCheckResponseStruct.errorCode   =data->errorCode;
    safeZoneCheckResponseStruct.len   =data->len;
    safeZoneCheckResponseStruct.safezoneExist = data->safezoneExist;
    safeZoneCheckResponseStruct.safezone_max_left = byte2Float(array,17);
    safeZoneCheckResponseStruct.safezone_max_right = byte2Float(array,21);
    strncpy(safeZoneCheckResponseStruct.stopSequence,data->stopSequence,6);
}
