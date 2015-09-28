
#include "robot_state.h"
#include <ros/ros.h>


namespace peter_chargingpile_search {


// ====================================

InitialState::InitialState(SearchChargingPileFSM* fsm)
{
    _fsm = fsm;
}

//void InitialState::begin()
//{
////    ROS_INFO("SearchChargingPileFSM--[State Initial Begin]");

//}

//void InitialState::end()
//{

//}

void InitialState::update(UpdateDataPacket& pack, tf::Transform& transform)
{
    if (pack.chargeOrderFlag)
    {
        _fsm->changeState(_fsm->_scanLandmarkState);
    }
}




// =============================

ScanLandmarkState::ScanLandmarkState(SearchChargingPileFSM* fsm)
{
    _fsm = fsm;
}

//void ScanLandmarkState::begin()
//{
////    ROS_INFO("SearchChargingPileFSM--[State ScanLandmark Begin]");
//}
//void ScanLandmarkState::end()
//{
////    std::cout << "call ScanLandmarkState::end()" << std::endl;
//}

void ScanLandmarkState::update(UpdateDataPacket& pack, tf::Transform& transform)
{

    if (ros::Time::now() - pack.objPosition.timestamp < ros::Duration(0.5))
    {
        _fsm->changeState(_fsm->_approachingState);
    }
}

// ==============================

ApproachingState::ApproachingState(SearchChargingPileFSM* fsm)
{
    _fsm = fsm;
    _isFoundObjectivePoint = false;
}

void ApproachingState::update(UpdateDataPacket& pack, tf::Transform& transform)
{
    ROS_INFO_STREAM("ApproachingState- " << pack.objPosition.timestamp << "-->(x, y, z, theta) = " <<
                    "(" <<
                    pack.objPosition.x << " " <<
                    pack.objPosition.y << " " <<
                    pack.objPosition.z << " " <<
                    pack.objPosition.theta << " " <<
                    ")");


    //xiwrong-->todo    timestamp 3s

    if (!_isFoundObjectivePoint)
    {
        _isFoundObjectivePoint  = true;
        _oldPoint = pack.objPosition;
    }
    //    if (distanceBetweenTwoPoints(_oldPoint, pack.objPosition) < 0.1)    //saltus step filter
    //    {
    transform.setOrigin(tf::Vector3(pack.objPosition.x, pack.objPosition.y, pack.objPosition.z));
    tf::Quaternion q;
    q.setRPY(0, 0, pack.objPosition.theta);
    transform.setRotation(q);
    _oldPoint = pack.objPosition;
    //    }

    if (pack.powerStatusFlag)
    {
        _fsm->changeState(_fsm->_finishState);
    }
}

double ApproachingState::distanceBetweenTwoPoints(PointWithTimeStamp a, PointWithTimeStamp b)
{
    return sqrt(pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0));
}

// ================================
FinishState::FinishState(SearchChargingPileFSM* fsm)
{
    _fsm = fsm;
}

void FinishState::update(UpdateDataPacket& pack, tf::Transform& transform)
{
    //    ROS_INFO("FinishState...");
}

// ===============================



}
