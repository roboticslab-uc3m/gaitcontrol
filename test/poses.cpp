#include <iostream>
#include <fstream>

#include "GaitSupportPoligon.h"
#include "MiddlewareInterface.h"
#include "SensorIntegration.h"

using namespace std;

using namespace teo;


int main()
{

    std::vector<double> rot;

    kin::Pose pose1(1,1,1);
    pose1.ChangeRotation(0,1,0,-M_PI/2);
    pose1.ChangeRotation(1,0,0,M_PI/2);
    std::cout << "rotation pose1: " << pose1.Ux() << "," << pose1.Uy() << "," << pose1.Uz() << "," << pose1.Angle()*180/M_PI << std::endl;

    kin::Pose pose2(pose1);
    pose2.ChangeRotation(0,0,1,-M_PI/20);
    pose2.GetRotation(rot);
    std::cout << "rotation pose2: " << rot << " deg " << rot[3]*180/M_PI << std::endl;
    pose2.ChangePosition(0,0,1);


    kin::Pose segment(pose1,pose2);

    segment.GetRotation(rot);
    std::cout << "rotation segment: " << rot << std::endl;
    double a,b,c,d;
    pose1.Inverse().GetPosition(a,b,c);
    std::cout << "pose1.Inverse().GetPosition: " << a << "," << b << "," << c << std::endl;
    pose1.Inverse().GetRotation(a,b,c,d);
    std::cout << "pose1.Inverse().GetRotation: " << a << "," << b << "," << c << "," << d << std::endl;

    kin::Pose ext= segment.WatchFromOriginOf(pose1);
    ext.GetRotation(rot);
    std::cout << "ext.GetRotation: " << rot << std::endl;
    ext.GetPosition(a,b,c);
    std::cout << "ext.GetPosition: " << a << "," << b << "," << c << std::endl;

    std::cout << " " << std::endl;




    ext = pose1.ExtrinsicMoveTo(pose2);

    ext.GetRotation(rot);
    std::cout << "ext.GetRotation: " << rot << std::endl;
    ext.GetPosition(a,b,c);
    std::cout << "ext.GetPosition: " << a << "," << b << "," << c << std::endl;
    std::cout << " " << std::endl;


    kin::Pose rootcom(0,0.3,0.3);

    //footcom.ChangePosition(0,0.35,1);

    kin::Pose initialRightFoot(0,-0.1285,-0.845);
    initialRightFoot.ChangeRotation(0,1,0,M_PI/2);
    initialRightFoot.ChangeRotation(1,0,0,M_PI);
    initialRightFoot.GetRotation(rot);
    std::cout << "initialRightFoot.GetRotation: " << rot << std::endl;
    initialRightFoot.GetPosition(a,b,c);
    std::cout << "initialRightFoot.GetPosition: " << a << "," << b << "," << c << std::endl;

    kin::Pose footcom(initialRightFoot,rootcom);
    footcom.GetRotation(rot);
    std::cout << "footcom.GetRotation: " << rot << std::endl;
    footcom.GetPosition(a,b,c);
    std::cout << "footcom.GetPosition: " << a << "," << b << "," << c << std::endl;


    kin::Pose footfinal(rootcom.Inverse(),footcom.Inverse());
    footfinal.GetRotation(rot);
    std::cout << "footfinal.GetRotation: " << rot << std::endl;
    footfinal.GetPosition(a,b,c);
    std::cout << "footfinal.GetPosition: " << a << "," << b << "," << c << std::endl;
    std::cout << " " << std::endl;

//    std::cout << " After move : footcom.CircularMotion(0,0,1,0.15)" << std::endl;
//    footcom.CircularMotion(0,0,1,0.15);
//    footcom.GetRotation(rot);
//    std::cout << "footcom.GetRotation: " << rot << std::endl;
//    footcom.GetPosition(a,b,c);
//    std::cout << "footcom.GetPosition: " << a << "," << b << "," << c << std::endl;

    footcom.ChangePosition(0,0.1,0);

    footfinal = kin::Pose(rootcom.Inverse(),footcom.Inverse());
    footfinal.GetRotation(rot);
    std::cout << "footfinal.GetRotation: " << rot << std::endl;
    footfinal.GetPosition(a,b,c);
    std::cout << "footfinal.GetPosition: " << a << "," << b << "," << c << std::endl;
    std::cout << " " << std::endl;


    return 0;
}

