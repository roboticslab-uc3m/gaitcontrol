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

    kin::Pose pose1(0,0,0);
    pose1.ChangeRotation(0,1,0,-M_PI/2);
    pose1.GetRotation(rot);
    std::cout << "rotation (0,1,0,-M_PI/2) " << rot << std::endl;
    pose1.ChangeRotation(1,0,0,M_PI);
    std::cout << "rotation:(1,0,0,M_PI) " << pose1.GetUx() << "," << pose1.GetUy() << "," << pose1.GetUz() << "," << pose1.GetAngle() << std::endl;
    pose1.ChangeRotation(0,0,1,M_PI);
    std::cout << "rotation:(0,0,1,M_PI) " << pose1.GetUx() << "," << pose1.GetUy() << "," << pose1.GetUz() << "," << pose1.GetAngle() << std::endl;
    pose1.ChangeRotation(0,1,0,-M_PI/2);
    std::cout << "final rotation:(0,1,0,-M_PI/2) " << pose1.GetUx() << "," << pose1.GetUy() << "," << pose1.GetUz() << "," << pose1.GetAngle() << std::endl;
    std::cout << " " << std::endl;

    //several rotations in one axis
    kin::Pose pose2(0,0,0);
    pose2.ChangeRotation(1,0,0,M_PI);
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
    std::cout << " " << std::endl;


    //Relative rotations
    pose1.SetRotation(0,1,0,M_PI/2);
    pose1.ChangeRotation(1,0,0,M_PI);
    pose1.GetRotation(rot);
    std::cout << "pose1 rotation " << rot << std::endl;
    pose2=pose1;
    pose2.ChangeRotation(0,0,1,-M_PI/20);
    pose2.GetRotation(rot);
    std::cout << "pose2 rotation " << rot << std::endl;
    kin::Pose segment(pose1,pose2);
    segment.GetRotation(rot);
    std::cout << "Relative rotation " << rot << std::endl;

//    kin::Pose segmentfrom0(pose1.Invert(),);
//    pose3.GetRotation(rot);
//    std::cout << "Relative rotation " << rot << std::endl;





    return 0;
}

