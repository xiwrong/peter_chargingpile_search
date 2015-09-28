
#pragma once

#include <ros/ros.h>

namespace peter_chargingpile_search {


struct PointInfo
{
    double x;
    double y;
    size_t index;
    double angle;
    double range;
};

struct XYPoint
{
    double x;
    double y;
};


struct PointWithTimeStamp
{
    double x;
    double y;
    double z;
    double theta;   //angle with x-axis
    ros::Time timestamp;

};

struct UpdateDataPacket
{
    bool chargeOrderFlag;
    bool powerStatusFlag;
    PointWithTimeStamp objPosition;
    bool outOfTimeFlag;
};


}
