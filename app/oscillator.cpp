#include <iostream>
#include <fstream>
#include <math.h>

#include "MiddlewareInterface.h"
#include "SensorIntegration.h"



int main()
{


    //Middleware
    //Robot teo right arm
    std::stringstream robConfig;
    //YARP device
    robConfig << "device remote_controlboard" << " ";
    //To what will be connected
    robConfig << "remote /teo/rightArm" << " ";
    //How will be called on YARP network
    robConfig << "local /local/rightArm/" << " ";
    MWI::Robot rightArm(robConfig);


     int axis = 3;
    //Amplitude in degrees
    double A = 10;
    double acc = 0.1; //accurracy to reach amplitude
    //Period in secods
    double T = 10;
    //Number of repetitions
    int loops = 2;
    double w = 2*M_PI/T;
    double waitYarp = 0.01;

    double actualPos=0;
    double actualVel=0;
    int direction=+1; // +1 forward, -1 backward

    for (int i=0; i<loops; i++)
    {
        while ( actualPos+acc < A )
        {

            actualPos=rightArm.GetJoint(axis);
            actualVel=direction*w*sqrt(A*A-actualPos*actualPos);
            rightArm.SetJointVel(axis,actualVel);
            yarp::os::Time::delay(waitYarp);

        }
        direction=direction*-1;
    }
    rightArm.SetJointVel(axis,0);


    return 0;
}

