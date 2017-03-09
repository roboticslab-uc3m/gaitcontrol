#include <iostream>
#include <fstream>

#include "GaitSupportPoligon.h"
#include "MiddlewareInterface.h"
#include "SensorIntegration.h"

using namespace std;

using namespace teo;


int main()
{

    kin::Pose pose1(0,-0.1285,-0.85);
    pose1.ChangeRotation(0,1,0,-M_PI/2);
    pose1.ChangeRotation(1,0,0,M_PI);
    std::cout << "rotation: " << pose1.GetUx() << "," << pose1.GetUy() << "," << pose1.GetUz() << "," << pose1.GetAngle() << std::endl;
    pose1.ChangeRotation(0,0,1,M_PI);
    std::cout << "rotation: " << pose1.GetUx() << "," << pose1.GetUy() << "," << pose1.GetUz() << "," << pose1.GetAngle() << std::endl;
    pose1.ChangeRotation(0,1,0,-M_PI/2);
    std::cout << "final rotation: " << pose1.GetUx() << "," << pose1.GetUy() << "," << pose1.GetUz() << "," << pose1.GetAngle() << std::endl;

    kin::Pose pose2(0,0,0);
    pose2.ChangeRotation(1,0,0,M_PI);
    std::vector<double> rot;
    pose2.GetRotation(rot);
    std::cout << "rotation " << rot << std::endl;
    pose2.ChangeRotation(1,0,0,M_PI/3);
    pose2.GetRotation(rot);
    std::cout << "rotation " << rot << std::endl;
    pose2.ChangeRotation(1,0,0,M_PI);
    pose2.GetRotation(rot);
    std::cout << "rotation " << rot << std::endl;
    pose2.ChangeRotation(1,0,0,M_PI);
    pose2.GetRotation(rot);
    std::cout << "rotation " << rot << std::endl;
    pose2.ChangeRotation(1,0,0,M_PI);
    pose2.GetRotation(rot);
    std::cout << "rotation " << rot << std::endl;
    return 0;
}

