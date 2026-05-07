#ifndef PALLET_STRUCT_H
#define PALLET_STRUCT_H

#pragma once
#include <cstdint>

typedef struct
{
    uint32_t ElapsedTime;
    uint32_t PalletX;
    uint32_t PalletY;
    uint32_t PalletZ;

    uint32_t LeftPocketX;
    uint32_t LeftPocketY;
    uint32_t LeftPocketZ;
    uint32_t RightPocketX;
    uint32_t RightPocketY;
    uint32_t RightPocketZ;
    uint32_t Roll;
    uint32_t Pitch;
    uint32_t Yaw;
}pallet_pose;

struct zone_struct
{
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    int   color_red;
    int color_green;
    int color_blue;
    float cloudDensity;
};

#endif // PALLET_STRUCT_H
