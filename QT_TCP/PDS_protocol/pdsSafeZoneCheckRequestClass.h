#ifndef PDSSAFEZONECHECKREQUESTCLASS_H
#define PDSSAFEZONECHECKREQUESTCLASS_H

#include <QObject>
#include "PDS_protocol/pdsStruct.h"

class pdsSafeZoneCheckRequestClass
{
public:
    safeZoneCheckRequest safeZoneCheckRequestStruct;

    pdsSafeZoneCheckRequestClass();
    ~pdsSafeZoneCheckRequestClass();
    QByteArray ToArray();
    pdsSafeZoneCheckRequestClass(QByteArray array);

};

#endif // PDSSAFEZONECHECKREQUESTCLASS_H
