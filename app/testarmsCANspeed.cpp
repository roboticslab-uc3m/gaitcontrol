#include <time.h>
#include <chrono>

#include "IKinematics.hpp"
#include "MiddlewareInterface.h"
#include "GaitSupportPoligon.h"
#include "tools.h"

#define ROBOT "teo"

using namespace roboticslab;
using namespace std;

double max_accel=(60.0/60.0);//rad/s^2
std::vector<double> DqRightArm(6,0), DqLeftArm(6,0);
std::vector<double> qRightArm(6,0), qLeftArm(6,0);

MWI::Limb teoRightArm(ROBOT,"rightArm"), teoLeftArm(ROBOT,"leftArm");

int main()
{
    double dts= 0.01;
    double st;

    teoRightArm.SetControlMode(1);
    teoLeftArm.SetControlMode(1);

    teoRightArm.SetJointPositions(qRightArm);
    teoLeftArm.SetJointPositions(qLeftArm);

    yarp::os::Time::delay(4);


    for (double t = dts; t < 5; t=dts+t)
    //for (int i=0; i< traLeftArm.Size(); i++)
    {

        //store old values
        //oldpos=pos;
        //oldvel=vel;

        DqRightArm[3]=DqRightArm[3]+2*dts;
        teoRightArm.SetJointPositions(DqRightArm);
        std::cout << "RightFrontalElbow position:"  << DqRightArm[3] << std::endl;
        yarp::os::Time::delay(dts);
    }
    return 0;
}
