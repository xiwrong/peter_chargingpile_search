#include "search_chargingpile_FSM.h"

#include "robot_state.h"

namespace peter_chargingpile_search {

SearchChargingPileFSM::SearchChargingPileFSM()
{
    _initialState      = boost::make_shared<InitialState>(this);
    _scanLandmarkState = boost::make_shared<ScanLandmarkState>(this);
    _approachingState  = boost::make_shared<ApproachingState>(this);
    _finishState       = boost::make_shared<FinishState>(this);

    _currentState = _initialState;
}

SearchChargingPileFSM::~SearchChargingPileFSM()
{
    std::cout << "call ~SearchChargingPileFSM()" << std::endl;
}

void SearchChargingPileFSM::begin()
{

}

void SearchChargingPileFSM::update(UpdateDataPacket& pack, tf::Transform& transform)
{
    if (_currentState != NULL)
    {

        _currentState->update(pack, transform);
//        std::cout << transform.getOrigin().getX() << " " <<
//                     transform.getOrigin().getY() << " " <<
//                     transform.getOrigin().getZ() << " " <<
//                     transform.getRotation().getAngle() << std::endl;

//        _expectedPoseTFbr.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot", "charging_pile"));
    }

}

void SearchChargingPileFSM::end(){}

void SearchChargingPileFSM::changeState(boost::shared_ptr<RobotState> state)
{
    _currentState->end();
    _currentState = state;
    _currentState->begin();
}

}
