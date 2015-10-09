

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include "search_chargingpile_manager.h"

#include <dynamic_reconfigure/server.h>
#include "peter_chargingpile_search/peterChargingNodeParamsConfig.h"


using namespace peter_chargingpile_search;

boost::shared_ptr<SearchChargingPileManager> searchChargingManager;

void recvLaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    //xiwrong-->todo   //if(msg->time > lastFrameTime) return;
    //    if(msg->time > lastFrameTime) return;

    if(searchChargingManager != NULL)
        searchChargingManager->addLaserScanMsg(msg);
}

void dynConfCb(peterChargingNodeParamsConfig& config, uint32_t level)
{
    searchChargingManager->_param_break_distance             = config.break_distance;
    searchChargingManager->_param_ignore_point_num           = config.ignore_point_num;
    searchChargingManager->_param_max_salient_tolerance      = config.max_salient_tolerance;
    searchChargingManager->_param_max_variance_tolerance     = config.max_variance_tolerance;
    searchChargingManager->_param_max_festureangle_tolerance = config.max_festureangle_tolerance;
    searchChargingManager->_param_linear_vel                 = config.linear_vel;
    searchChargingManager->_param_angular_vel                = config.angular_vel;



    ROS_INFO_STREAM("Reconfigure: " << "break_distance = " << config.break_distance << "; " <<
                    "ignore_point_num = " << config.ignore_point_num << "; " << std::endl <<
                    "max_salient_tolerance = " << config.max_salient_tolerance << "; " <<
                    "max_variance_tolerance = " << config.max_variance_tolerance << "; " <<
                    "max_festureangle_tolerance = " << config.max_festureangle_tolerance << "; "
                    );
}


void rosThreadLoop(int argc, char** argv, ros::NodeHandle& nh)
{


    //    ros::Subscriber powerStatusSub = nh.subscribe(nh.resolveName("lsd_slam/liveframes"), 1, frameCb);
    //    ros::Subscriber chargeOrderSub = nh.subscribe(nh.resolveName("lsd_slam/keyframes"), 20, frameCb);

    //xiwrong-->todo    //resolveName
    ros::Subscriber laserScanSub = nh.subscribe(nh.resolveName("/scan"), 1000, recvLaserScanCallback);
    //ros::Subscriber laserScanSub = nh.subscribe("sensor_msgs/LaserScan", 1000, recvLaserScan);



    ros::spin();
    ros::shutdown();
    ROS_INFO("Exiting search_chargingpile_node rosThreadLoop\n");
    //    exit(1);

}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "charge_pile");
    ROS_INFO("Search_Chargingpile_Node Started ...");

    ros::NodeHandle nh;

    searchChargingManager.reset();
    searchChargingManager = boost::make_shared<SearchChargingPileManager>(nh);

    //dynamic_reconfigure::Server
    dynamic_reconfigure::Server<peterChargingNodeParamsConfig> srv;
    srv.setCallback(dynConfCb);

    boost::thread rosThread;

    rosThread = boost::thread(rosThreadLoop, argc, argv, nh);

    rosThread.join();

}
