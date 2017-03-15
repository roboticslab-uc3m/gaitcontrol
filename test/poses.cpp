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

    kin::Pose extrinsic= segment.WatchFromOriginOf(pose1);
    extrinsic.GetRotation(rot);
    std::cout << "extrinsic.GetRotation: " << rot << std::endl;
    extrinsic.GetPosition(a,b,c);
    std::cout << "extrinsic.GetPosition: " << a << "," << b << "," << c << std::endl;

    std::cout << " " << std::endl;




    extrinsic = pose1.ExtrinsicMoveTo(pose2);

    extrinsic.GetRotation(rot);
    std::cout << "extrinsic.GetRotation: " << rot << std::endl;
    extrinsic.GetPosition(a,b,c);
    std::cout << "extrinsic.GetPosition: " << a << "," << b << "," << c << std::endl;
    std::cout << " " << std::endl;



    return 0;
}

