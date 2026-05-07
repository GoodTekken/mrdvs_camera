#include "pdsSafeZoneCheckRequestClass.h"
#include "Function/function.h"

pdsSafeZoneCheckRequestClass::pdsSafeZoneCheckRequestClass()
{
    strncpy(safeZoneCheckRequestStruct.startSequnce,seqStart,4);
    safeZoneCheckRequestStruct.commandID = 6;
    safeZoneCheckRequestStruct.argsLen = 0;
    strncpy(safeZoneCheckRequestStruct.stopSequence,seqEnd,6);
}

pdsSafeZoneCheckRequestClass::~pdsSafeZoneCheckRequestClass()
{
    ;
}

QByteArray pdsSafeZoneCheckRequestClass::ToArray()
{
    safeZoneCheckRequestStruct.commandID = swapUInt32(safeZoneCheckRequestStruct.commandID);
    safeZoneCheckRequestStruct.argsLen = swapUInt32(safeZoneCheckRequestStruct.argsLen);

    QByteArray array;
    array.clear();
    array.append((char*)&safeZoneCheckRequestStruct,sizeof(safeZoneCheckRequestStruct));
    return array;
}
pdsSafeZoneCheckRequestClass::pdsSafeZoneCheckRequestClass(QByteArray array)
{
    safeZoneCheckRequest* data = (safeZoneCheckRequest*)array.data();
    data->commandID=swapUInt32(data->commandID);
    data->argsLen=swapUInt32(data->argsLen);
    strncpy(safeZoneCheckRequestStruct.startSequnce,data->startSequnce,4);
    safeZoneCheckRequestStruct.commandID =data->commandID;
    safeZoneCheckRequestStruct.argsLen   =data->argsLen;
    strncpy(safeZoneCheckRequestStruct.stopSequence,data->stopSequence,6);
}
