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
    std::cout << "rotation:(1,0,0,M_PI) " << pose1.Ux() << "," << pose1.Uy() << "," << pose1.Uz() << "," << pose1.Angle() << std::endl;
    pose1.ChangeRotation(0,0,1,M_PI);
    std::cout << "rotation:(0,0,1,M_PI) " << pose1.Ux() << "," << pose1.Uy() << "," << pose1.Uz() << "," << pose1.Angle() << std::endl;
    pose1.ChangeRotation(0,1,0,-M_PI/2);
    std::cout << "final rotation:(0,1,0,-M_PI/2) " << pose1.Ux() << "," << pose1.Uy() << "," << pose1.Uz() << "," << pose1.Angle() << std::endl;
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


    kin::Rotation r1(1,0,0,M_PI/2);

    double x1=0,y1=1,z1=0;
    std::cout << "X1: " << x1 << ", " << y1 << ", " << z1 << std::endl;
    r1.RotatePoint(x1,y1,z1);
    std::cout << "X1: " << x1 << ", " << y1 << ", " << z1 << std::endl;




    return 0;
}

