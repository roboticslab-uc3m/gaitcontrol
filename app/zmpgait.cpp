#include <iostream>
#include <fstream>

#include "GaitLipm.h"
#include "MiddlewareInterface.h"
#include "SensorIntegration.h"

using namespace std;

using namespace teo;


int main()
{

    std::vector<double> rot;
    double a,b,c,d;


    //Define the initial feet poses from root (robot origin).
    kin::Pose initialRightFoot(0,-0.1285,-0.845);
    initialRightFoot.ChangeRotation(0,1,0,M_PI/2);
    initialRightFoot.ChangeRotation(1,0,0,M_PI);
    kin::Pose initialLefttFoot(0,+0.1285,-0.845);
    initialLefttFoot.ChangeRotation(0,1,0,-M_PI/2);

    //define the pose of com from root.
    kin::Pose rootcom(0,0.3,0.3);

    //define the pose of com from the foot.
    kin::Pose footcom(initialRightFoot,rootcom);


    //computation of foot position based on com position
    kin::Pose footfinal(rootcom.Inverse(),footcom.Inverse());
    //check it
    footfinal.GetRotation(rot);
    std::cout << "footfinal.GetRotation: " << rot << std::endl;
    footfinal.GetPosition(a,b,c);
    std::cout << "footfinal.GetPosition: " << a << "," << b << "," << c << std::endl;
    std::cout << " " << std::endl;


    //create a lipm object and compute trajectory
    teo::GaitLipm lipmStep(initialRightFoot,initialLefttFoot,80.0);
    //set initial swing velocity for y
    double yvel = 0.5;

    lipmStep.BeforeStep();


    lipmStep.AddStepForward(1);




    footfinal = kin::Pose(rootcom.Inverse(),footcom.Inverse());
    footfinal.GetRotation(rot);
    std::cout << "footfinal.GetRotation: " << rot << std::endl;
    footfinal.GetPosition(a,b,c);
    std::cout << "footfinal.GetPosition: " << a << "," << b << "," << c << std::endl;
    std::cout << " " << std::endl;


    return 0;
}

