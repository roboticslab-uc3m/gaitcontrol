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

    kin::Pose pose1;
    pose1.ChangeRotation(0,1,0,-M_PI/2);
    pose1.ChangeRotation(1,0,0,M_PI);
    std::cout << "rotation pose1: " << pose1.Ux() << "," << pose1.Uy() << "," << pose1.Uz() << "," << pose1.Angle() << std::endl;

    kin::Pose pose2(pose1);
    pose2.ChangeRotation(0,0,1,M_PI/20);
    pose2.GetRotation(rot);
    std::cout << "rotation pose2: " << rot << std::endl;

    kin::Pose segment(pose1,pose2);

    segment.GetRotation(rot);
    std::cout << "rotation segment: " << rot << std::endl;

    kin::Pose extrinsic= segment.ExtrinsicTransform(pose1);
    extrinsic.GetRotation(rot);
    std::cout << "rotation extrinsic: " << rot << std::endl;

    std::cout << " " << std::endl;








    return 0;
}

