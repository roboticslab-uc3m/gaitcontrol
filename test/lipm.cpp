#include <iostream>
#include <fstream>

#include "GaitLipm.h"
#include "MiddlewareInterface.h"
#include "SensorIntegration.h"

using namespace std;

using namespace teo;


int main()
{


    kin::Pose initialRightFoot(0,-0.1285,-0.85);
    initialRightFoot.ChangeRotation(0,1,0,-M_PI/2);
    initialRightFoot.ChangeRotation(1,0,0,M_PI);
    kin::Pose initialLeftFoot(0,+0.15,-0.85);
    initialLeftFoot.ChangeRotation(0,1,0,-M_PI/2);
    //every gait operation start with instantiation and initialization of a Gait child class
    GaitLipm walk01(initialRightFoot,initialLeftFoot,80.);
    walk01.SetKickParameters(0.05,0.05); //(swing distance, swing height). revisar valores
    walk01.SetHipParameters(0.25, 0.10, 0.15); //(hip sideshift, hip lowering). revisar estos valores

    //The Gait objects can do the following tasks.

    double Ts = 0.01;


    //get the initial speed
    double ySpeed = walk01.GetSwingYInitialSpeed(0.3,1);
    std::cout << "ySpeed : " << ySpeed << std::endl;

    double y1=0.3;
    double y2=0.299;//y1-ySpeed*Ts*0.8;



    std::vector<double> x={0,0};
    std::vector<double> y={y1,y2};
    std::vector<double> z={1,1};
    std::cout << "initial x: " << x << std::endl;
    std::cout << "initial y: " << y << std::endl;
    std::cout << "initial z: " << z << std::endl;

    //zmp based trajectory.
    double time01 = walk01.LipInitAndGetZmpTrajectory(x,y,z,Ts);
    std::cout << "time01: " << time01 << std::endl;

    physics::StateVariable sx(0,0,0);
    physics::StateVariable sy(0.3,-0.8,0);
    physics::StateVariable sz(1,0,0);
    std::vector<double> xs(0);
    std::vector<double> ys(0);
    std::vector<double> zs(0);

    //zmp based trajectory.
    walk01.LipmInitialState(sx,sy,sz);
    time01 = walk01.LipZmpTrajectory(xs,ys,zs,Ts);
    std::cout << "time01: " << time01 << std::endl;

//    for (double y2test=y1-0.001;y2test>0.28;y2test-=0.001)
//    {
//        std::vector<double> y={y1,y2test};
//        double time01 = walk01.LipZmpTrajectory(x,y,z,Ts);
//        std::cout << "y1: " << y1<< " ,y2test: " << y2test<< ", time01: " << time01 << std::endl;
//    }

    //Angular response of an inverted pendulum
    //define the first two points in the trajectory
    std::vector<double> lipm1={0.15,0.1465};
    //define the initial trajectory with the first two points,
    walk01.LipmAngularResponse(lipm1,0.01,1);
    std::cout << "lipm1: " << lipm1 << std::endl;


    return 0;
}
