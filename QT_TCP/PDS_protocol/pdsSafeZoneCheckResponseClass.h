#ifndef PDSSAFEZONECHECKRESPONSECLASS_H
#define PDSSAFEZONECHECKRESPONSECLASS_H
#include <QObject>
#include "PDS_protocol/pdsStruct.h"

class pdsSafeZoneCheckResponseClass
{
public:

    safeZoneCheckResponse safeZoneCheckResponseStruct;

    pdsSafeZoneCheckResponseClass();
    ~pdsSafeZoneCheckResponseClass();
    pdsSafeZoneCheckResponseClass(int errorCode,bool safezoneExist,float safezone_max_left,float safezone_max_right);
    QByteArray ToArray();
    pdsSafeZoneCheckResponseClass(QByteArray array);
};

#endif // PDSSAFEZONECHECKRESPONSECLASS_H
