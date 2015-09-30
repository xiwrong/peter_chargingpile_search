



#include "ros/ros.h"
#include "std_msgs/String.h"

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include "testweakptr.hpp"
#include "../src/search_chargingpile_FSM.h"
#include "../src/robot_state.h"

#include "../src/weighted_fit.h"

using namespace peter_chargingpile_search;
using namespace std;
using namespace boost;
using namespace weighted_fit;

template<typename T> void printVec(std::vector<T>& vecs)
{
    for (int i = 0; i < vecs.size(); i++)
    {
        std::cout << vecs[i];
    }
    std::cout << std::endl;
}

void testVector()
{
    std::vector<int> vec;
    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);
    vec.push_back(4);
    vec.push_back(5);
    vec.push_back(6);

    int maxDisIndex = 2;

    std::vector<int> splitLine1(vec.begin(), vec.begin() + maxDisIndex);
    std::vector<int> splitLine2(vec.begin() + maxDisIndex, vec.end());

    printVec<int>(splitLine1);
    printVec<int>(splitLine2);
    //    std::cout << splitLine1 << std::endl;
}



void testMath()
{
    double value[9] = {1, 2, 3, 4, 8, 1, 2, 9, 4};
    double res = med(value, 3);
    cout << res << endl;

    double x[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    double y[9] = {0, 0.5, 1, 0.5, 0, -0.5, -1, -0.5, 0};

    LinePara lPara;
    weightedFit(x, y, 9, &lPara);
    double variance = lPara.standardDeviation;
    cout << "Variance = " << variance << endl;
    // fitPara(x, y, 3, &lPara, w);
    cout << lPara.a << "X" << "+" << lPara.b << endl;

    //    double xx[4] = {1, 2, 3, 0};
    //    double yy[4] = {3, 3, 4, 0};
    //    weightedFit(xx, yy, 4, &lPara);
    //    cout << lPara.a << "X" << "+" << lPara.b << endl;
}

void test1()
{
    shared_ptr<SearchChargingPileFSM> p = make_shared<SearchChargingPileFSM>();


    for (double i = 0.0f; i < 10.0f; i++)
    {
        UpdateDataPacket data;
        tf::Transform transform;
        data.objPosition.x = i;
        data.objPosition.y = i;
        data.objPosition.z = i;

        data.chargeOrderFlag = (i < 5.0) ? false : true;

        p->update(data, transform);
    }
    p->end();
}

void test2()
{
    Foo* pFoo = new Foo();
    pFoo->index = 11;
    TestClassA a;

    TestClassB b;

    a.SetFooPtr(FooPtr(pFoo));
    b.SetFooPtr(a.GetFooPtr());

}

void test3()
{
    boost::shared_ptr<ObjectA> a(new ObjectA);
    boost::shared_ptr<ObjectB> b(new ObjectB);

    a->setObjectB(b);
    b->setObjectA(a);
}

int main()
{
    //    test1();
    // test2();
    //    testMath();
    testVector();
}
