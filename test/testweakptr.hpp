
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

using namespace std;

class ObjectB;

class ObjectA
{
public:
    ~ObjectA()
    {
        std::cout<<"dctor ~ObjectA"<<std::endl;
    }

    void setObjectB(boost::shared_ptr<ObjectB> b)
    {
        m_objB = b;
    }
private:
    boost::shared_ptr<ObjectB> m_objB;
};

class ObjectB
{
public:
    ~ObjectB()
    {
        cout<<"dctor ~ObjectB"<<endl;
    }

    void setObjectA(boost::shared_ptr<ObjectA> a)
    {
        m_objA = a;
    }
private:
    boost::weak_ptr<ObjectA> m_objA;
};



class Foo
{
public:
    Foo()
    {
        cout<<"create a Foo object"<<endl;
    }

    ~Foo()
    {
        cout<<"delete a Foo Object"<<endl;
    }
public:
    int index;
};
typedef boost::shared_ptr<Foo> FooPtr;

class TestClassA
{
public:
    void SetFooPtr(const FooPtr& pi)
    {
        p = pi;
        cout << "object count = " << p.use_count() << endl;
        cout << pi->index << endl;
        pi->index = 22;
    }

    /*inline*/ FooPtr& GetFooPtr()
    {
        return p;
    }


private:
    FooPtr p;
};

class TestClassB
{
public:
    void SetFooPtr(const FooPtr& pi )
    {
        p = pi;
        cout << "object count = " << p.use_count() << endl;
        cout << pi->index << endl;
    }

private:
    FooPtr p;
};

//int _tmain(int argc, _TCHAR* argv[])
//{
//        test();
//        printf("test ");
//        getchar();
//        return 0;
//}
