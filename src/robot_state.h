#pragma once

#include <iostream>

#include "global_data_structure.h"
#include "search_chargingpile_FSM.h"


namespace peter_chargingpile_search {



class SearchChargingPileFSM;

class RobotState
{
public:
    void begin() {}
    void end() {}
    virtual void update(UpdateDataPacket& pack, tf::Transform& transform) {}

protected:
    SearchChargingPileFSM* _fsm;

};

// =================
class InitialState : public RobotState
{
public:

    InitialState(SearchChargingPileFSM* fsm);
//    void begin();
//    void end();
    void update(UpdateDataPacket& pack, tf::Transform& transform);

};

// =================
class ScanLandmarkState : public RobotState
{
public:
    ScanLandmarkState(SearchChargingPileFSM* fsm);

//    void begin();
//    void end();
    void update(UpdateDataPacket& pack, tf::Transform& transform);
};

// ==================
class ApproachingState : public RobotState
{
public:
    ApproachingState(SearchChargingPileFSM* fsm);
    void update(UpdateDataPacket& pack, tf::Transform& transform);

private:
    PointWithTimeStamp _oldPoint;
    bool _isFoundObjectivePoint;
    double distanceBetweenTwoPoints(PointWithTimeStamp a, PointWithTimeStamp b);
};

// ==================
class FinishState : public RobotState
{
public:
    FinishState(SearchChargingPileFSM* fsm);
    void update(UpdateDataPacket& pack, tf::Transform& transform);
};


}
