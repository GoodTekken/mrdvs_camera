#ifndef MAIN_H
#define MAIN_H

#include "AGVMessage/AgvExternalPara.h"
#include "Function/function.h"
#include "PDS_protocol/pdsStruct.h"
#include "PDS_protocol/pdsErrorCode.h"

extern QSharedPointer<AGVExternalPara> g_AGV_ptr;

extern bool g_detectFlag;
extern int g_errorCode;
extern float g_angleOffset;
extern pds_point g_leftPoint;
extern pds_point g_certenPoint;
extern pds_point g_rightPoint;

extern float g_elapsedTime;
extern float g_confidence;
extern float g_roll;
extern float g_pitch;
extern float g_yaw;

extern std::size_t g_existpoint;
extern float g_safezone_min_x;
extern float g_safezone_max_x;
extern float g_safezone_min_y;
extern float g_safezone_max_y;
extern float g_safezone_min_z;
extern float g_safezone_max_z;

extern int g_command_id;
extern bool g_safezoneExist;
extern float g_safezone_max_left;
extern float g_safezone_max_right;

extern float g_volCheck_min_x;
extern float g_volCheck_max_x;
extern float g_volCheck_min_y;
extern float g_volCheck_max_y;
extern float g_volCheck_min_z;
extern float g_volCheck_max_z;

#endif // MAIN_H
