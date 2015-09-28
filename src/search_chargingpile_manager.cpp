#include "search_chargingpile_manager.h"

#include <boost/bind.hpp>
#include <sensor_msgs/PointCloud.h>
#include <algorithm>
#include <boost/make_shared.hpp>

#include "search_chargingpile_FSM.h"
#include "weighted_fit.h"


namespace peter_chargingpile_search {

#define PAI 3.1415927


const int SearchChargingPileManager::Const_Queue_Length = 1500;             //sourceNum = 1080
const double SearchChargingPileManager::Recognization_Angle = 160.0;        //unit:angle


//dynamic_reconfigure
const double SearchChargingPileManager::Break_Distance = 0.02;              //unit:m
const int SearchChargingPileManager::Ignore_Point_Num = 25;              //unit:none
const double SearchChargingPileManager::Max_Salient_Tolerance = 0.005;      //unit:m
const double SearchChargingPileManager::Max_FestureAngle_Tolerance = 1.5;   //unit:angle
const double SearchChargingPileManager::Max_Variance_Tolerance = 0.01;      //unit:Standard Deviation


//SearchChargingPileManager::
SearchChargingPileManager::SearchChargingPileManager(ros::NodeHandle nh) : _nh(nh)
{

    _param_break_distance               = _nh.param("peter_break_distance", Break_Distance);
    _param_ignore_point_num             = _nh.param("peter_ignore_point_num", Ignore_Point_Num);
    _param_max_salient_tolerance        = _nh.param("peter_max_salient_tolerance", Max_Salient_Tolerance);
    _param_max_variance_tolerance       = _nh.param("peter_max_variance_tolerance", Max_Variance_Tolerance);
    _param_max_festureangle_tolerance   = _nh.param("peter_max_festureangle_tolerance", Max_FestureAngle_Tolerance);

    // xiwrong-->todo temp _chargeOrderFlag
    _chargeOrderFlag = true;
    _powerStatusFlag = false;
    _searchFSM = boost::make_shared<SearchChargingPileFSM>();

    _pointsPub = _nh.advertise<sensor_msgs::PointCloud>("debug/cloud", 50);
    _ctrlCmdVelPub = _nh.advertise<geometry_msgs::Twist>("ctrl_cmd_vel", 10);

    ros::Duration period = ros::Duration(1./10);
    _ctrlCmdVelTimer = _nh.createTimer(period, &SearchChargingPileManager::onTimerCtrlCmdVel, this);

}

void SearchChargingPileManager::addLaserScanMsg(const sensor_msgs::LaserScanConstPtr& msg)
{
    size_t index_sum = (msg->angle_max - msg->angle_min)/ msg->angle_increment;
    _sourcePoints.clear();
    double angle, range;
    for (size_t index = 0; index <= index_sum; index++)
    {
        PointInfo point;
        angle = msg->angle_min + index* msg->angle_increment;
        range = msg->ranges[index];
        point.x =  range* cos(angle);
        point.y = -range* sin(angle);
        point.index = index;
        point.angle = angle;
        point.range = range;

        _sourcePoints.push_back(point);
    }


    splitLaserWithRange();                                      //calculate _breakedLaser...
    filterSplitLaser(_breakedLaserAngle, _breakedLaserRange);   //calculate _filterLaser...
    changeRangetoXY(_filterLaserAngle, _filterLaserRange);

    PointWithTimeStamp keyPoint;
    bool flag = findKeyPointFromLines(keyPoint);

    // ===============show pointcloud============
    publishPointCloud(_filterLaserAngle, _filterLaserRange, keyPoint, flag);

    // ===============send packet================
    if (flag)
    {
        UpdateDataPacket packet;
        packet.objPosition = keyPoint;
        packet.chargeOrderFlag = _chargeOrderFlag;
        packet.powerStatusFlag = _powerStatusFlag;

        tf::Transform transform;
        _searchFSM->update(packet, transform);
        std::cout << transform.getOrigin().x() << " " << transform.getOrigin().y() << std::endl;

        _vel_msg.angular.z = 1.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
        _vel_msg.linear.x  = 0.5 * sqrt(pow(transform.getOrigin().x(), 2.0) + pow(transform.getOrigin().y(), 2.0));

    }

}

void SearchChargingPileManager::onTimerCtrlCmdVel(const ros::TimerEvent& t)
{
    _ctrlCmdVelPub.publish(_vel_msg);

}


void SearchChargingPileManager::publishPointCloud(std::vector<double> angles,
                                                  std::vector<double> ranges,
                                                  PointWithTimeStamp keyPoint,
                                                  bool isShowKeyPoint)
{
    if (angles.size() != ranges.size())
        return;

    int CirclePointNum = isShowKeyPoint? 50: 0;
    int ArrowPointNum = isShowKeyPoint? 30: 0;
    double CircleRadius = 0.05;
    double ArrowLength = 0.15;

    int num_points = angles.size();
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "laser";
    cloud.points.resize(num_points + CirclePointNum + ArrowPointNum);

    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points + CirclePointNum + ArrowPointNum);

    double x, y;
    double color = 2.0;
    for(int i = 0; i < num_points; i++)
    {
        if (ranges.at(i) < -0.1)
        {
            color += 1.0;
            continue;
        }

        assert(ranges.at(i) > -0.1);
        x = ranges.at(i)* cos(angles.at(i));
        y = ranges.at(i)* sin(angles.at(i));
        cloud.points[i].x = x;
        cloud.points[i].y = y;
        cloud.points[i].z = 0;
        cloud.channels[0].values[i] = color;
    }

    //draw  feature point using circle
    for (int i=0; i < CirclePointNum; i++ )
    {
        cloud.points[i+num_points].x = keyPoint.x + CircleRadius*cos(2*PAI*i/CirclePointNum);
        cloud.points[i+num_points].y = keyPoint.y + CircleRadius*sin(2*PAI*i/CirclePointNum);
        cloud.points[i+num_points].z = 0;
        cloud.channels[0].values[i+num_points] = 0;
    }
    //draw orientation using line
    for (int i=0; i < ArrowPointNum; i++)
    {
        cloud.points[CirclePointNum + num_points + i].x = keyPoint.x + cos(keyPoint.theta)*i*ArrowLength/ArrowPointNum;
        cloud.points[CirclePointNum + num_points + i].y = keyPoint.y + sin(keyPoint.theta)*i*ArrowLength/ArrowPointNum;
        cloud.points[CirclePointNum + num_points + i].z = 0;
        cloud.channels[0].values[CirclePointNum + num_points + i] = 0;
    }

    _pointsPub.publish(cloud);
}


int  SearchChargingPileManager::splitLaserWithRange()
{
    int breakCount = 0;
    double distance = 0;
    int breakPointNum = 0;

    double range = 0;
    double lastRange = _sourcePoints.at(0).range;
    double angle = _sourcePoints.at(0).angle;


    _breakedLaserRange.clear();
    _breakedLaserAngle.clear();

    _breakedLaserRange.push_back(lastRange);
    _breakedLaserAngle.push_back(angle);

    for (int i = 1; i< _sourcePoints.size(); i++)
    {
        breakPointNum++;
        range = _sourcePoints.at(i).range;
        angle = _sourcePoints.at(i).angle;
        distance = fabs(range - lastRange);

        if (distance > _param_break_distance)
        {
            _breakedLaserRange.push_back(-1.0*breakPointNum);
            _breakedLaserAngle.push_back(1000.0);
            breakCount++;
            breakPointNum = 0;
        }
        _breakedLaserRange.push_back(range);
        _breakedLaserAngle.push_back(angle);
        lastRange = range;
    }
    _breakedLaserRange.push_back(-1.0*(breakPointNum+1));
    _breakedLaserAngle.push_back(1000.0);
    //cout<<"breakCnt: "<<breakCnt<<endl;
    return breakCount;
}

void SearchChargingPileManager::filterSplitLaser(std::vector<double> angles, std::vector<double> ranges)
{
    if (angles.size() != ranges.size())
    {
        return;
    }

    _filterLaserAngle.clear();
    _filterLaserRange.clear();

    int num_points = angles.size();

    double x, y;
    int pointNum;

    for(int i = num_points - 1; i >= 0; i--)
    {
        if (ranges.at(i) < -0.1)
        {
            //xiwrong-->todo  //risk come from double --> int
            pointNum = (int)fabs(ranges.at(i)); //calculate number of points in one split area
        }
        if (pointNum > _param_ignore_point_num)
        {
            _filterLaserAngle.push_back(angles.at(i));
            _filterLaserRange.push_back(ranges.at(i));
        }
    }
    std::reverse(_filterLaserAngle.begin(), _filterLaserAngle.end());
    std::reverse(_filterLaserRange.begin(), _filterLaserRange.end());
}

void SearchChargingPileManager::changeRangetoXY(std::vector<double> angles, std::vector<double> ranges)
{
    // =========================== change double-angles&ranges to vector(x, y)

    if (angles.size() != ranges.size())
        return;
    _splitLines.clear();
    int num_points = angles.size();

    std::vector<XYPoint> oneLine;

    for (int i = num_points - 1; i >=0; i--)
    {
        if (ranges.at(i) < 0)
        {
            _splitLines.push_back(oneLine);
            oneLine.clear();
            continue;
        }
        XYPoint onePoint;
        onePoint.x = ranges.at(i)* cos(angles.at(i));
        onePoint.y = ranges.at(i)* sin(angles.at(i));
        oneLine.push_back(onePoint);
    }

    //    int returnValue;
    //    for (int i = 0; i < _splitLines.size(); i++)
    //    {
    //        returnValue = findSalientIndexByVector(_splitLines.at(i), Max_Salient_Tolerance);
    //        //        std::cout << returnValue << "/" << _splitLines.at(i).size() << std::endl;
    //    }
}

int  SearchChargingPileManager::findSalientIndexByVector(std::vector<XYPoint> line, double eps)
{
    int n = line.size();
    if (n < _param_ignore_point_num)
        return 0;
    double dis = sqrt(pow(line.at(0).x - line.at(n-1).x, 2.0) + pow(line.at(0).y - line.at(n-1).y, 2.0));
    double cosTheta =  (line.at(n-1).x - line.at(0).x)/ dis;
    double sinTheta = -(line.at(n-1).y - line.at(0).y)/ dis;
    double maxDis = 0;
    int maxDisIndex = 0;
    double dbDis;

    for(int i = 1 ; i < n-1 ; i++)
    {
        dbDis = fabs((line.at(i).y - line.at(0).y)*cosTheta + (line.at(i).x - line.at(0).x)*sinTheta);
        if( dbDis > maxDis)
        {
            maxDis = dbDis;
            maxDisIndex = i;
        }
    }
    if(maxDis > eps)
    {
        return maxDisIndex;
    }
    return 0;
}

int  SearchChargingPileManager::findSalientIndexByArray(double* x, double* y, int n, double eps)
{
    double dis = sqrt(pow((x[0]-x[n-1]), 2.0) + pow((y[0]-y[n-1]), 2.0));
    double cosTheta =  (x[n-1] - x[0])/ dis;
    double sinTheta = -(y[n-1] - y[0])/ dis;
    double maxDis = 0;
    int maxDisIndex = 0;
    double dbDis;

    for(int i = 1 ; i < n-1 ; i++)
    {
        dbDis = fabs((y[i]-y[0])*cosTheta + (x[i]-x[0])*sinTheta);
        if( dbDis > maxDis)
        {
            maxDis = dbDis;
            maxDisIndex = i;
        }
    }
    if(maxDis > eps)
    {
        return maxDisIndex;
    }
    return 0;
}

bool SearchChargingPileManager::findKeyPointFromLines(PointWithTimeStamp& keyPoint)
{
    bool findResultFlag = false;
    int returnValue;
    double tempx[Const_Queue_Length];
    double tempy[Const_Queue_Length];


    weighted_fit::LinePara tmpLinePara1;
    weighted_fit::LinePara tmpLinePara2;


    double tempVariance = 0.0;
    double minVariance = 100.0;
    double changedAngle1 = 0.0;
    double changedAngle2 = 0.0;

    for (int i = 0; i < _splitLines.size(); i++)
    {
        std::vector<XYPoint> tempLine = _splitLines.at(i);
        for (int j = 0; j < tempLine.size(); j++)
        {
            tempx[j] = tempLine.at(j).x;
            tempy[j] = tempLine.at(j).y;
        }

        returnValue = findSalientIndexByVector(_splitLines.at(i), _param_max_salient_tolerance);
        if (returnValue == 0 || returnValue < _param_ignore_point_num ||
                tempLine.size() - returnValue < _param_ignore_point_num)
            continue;
        //xiwrong-->todo      //Temporary processing--split one line into two lines -- should consider recursion
        double variance1 = weighted_fit::weightedFit(tempx, tempy, returnValue, &tmpLinePara1);
        double variance2 = weighted_fit::weightedFit(tempx + returnValue, tempy + returnValue, tempLine.size() - returnValue, &tmpLinePara2);


        double theta = (tmpLinePara1.Rho - tmpLinePara2.Rho)*180/PAI;

        if ((fabs( theta - 180 + Recognization_Angle) < _param_max_festureangle_tolerance ||
             fabs( theta + Recognization_Angle ) < _param_max_festureangle_tolerance) &&
                variance1 < _param_max_variance_tolerance && variance2 < _param_max_variance_tolerance)    //filter [angle=160]
        {
            tempVariance = std::max(variance1, variance2);
            if (tempVariance < minVariance)
            {
                minVariance = tempVariance;

                keyPoint.timestamp = ros::Time::now();
                //xiwrong-->todo  should select the most suitable one from N keypoints, not select the first we find.
                keyPoint.x = (tmpLinePara2.b - tmpLinePara1.b)/ (tmpLinePara1.a - tmpLinePara2.a);
                keyPoint.y = tmpLinePara1.a* keyPoint.x + tmpLinePara1.b;
                keyPoint.z = 0;
                //               std::cout << " tmpLinePara1 = "  << tmpLinePara1.Rho*180/PAI <<
                //                            " tmpLinePara2 = "  << tmpLinePara2.Rho*180/PAI << std::endl;

                // xiwrong-->todo      problem: x=0
                changedAngle1 = (tmpLinePara1.Rho > 0)? tmpLinePara1.Rho: PAI+tmpLinePara1.Rho;
                changedAngle2 = (tmpLinePara2.Rho > 0)? tmpLinePara2.Rho: PAI+tmpLinePara2.Rho;
                double average = (changedAngle1 + changedAngle2)/2 - PAI/2;
                keyPoint.theta = average;



                //                double average = (changedAngle1 + changedAngle2)/2;
                //                double sign = (keyPoint.x*average > 0) ? -1.0: 1.0;
                //                double signAngle = (tmpLinePara1.Rho + tmpLinePara2.Rho > 0) ? 1.0: -1.0;

                //                keyPoint.theta = average* signAngle + sign*PAI/2;
                //                std::cout << keyPoint.theta*180/PAI << std::endl;
                findResultFlag = true;
            }
        }
    }

    return findResultFlag;

}


}