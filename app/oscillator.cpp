#include <iostream>
#include <fstream>
#include <math.h>

#include "MiddlewareInterface.h"
#include "SensorIntegration.h"



int main()
{


    //Middleware

/*    std::cout << "[Initial]";
    //MWI::Port imuPort;
    //INITIALISE AND CHECK YARP
    yarp::os::Network yarpNet;

    if ( !yarpNet.checkNetwork(2) )
    {
        std::cout << "[error] %s found no YARP network (try running \"yarp detect --write\")." << std::endl;
        return -1;
    }
    else
    {
        std::cout << "[success] YARP network found." << std::endl;
    }
*/
    //Robot teo right arm
    std::stringstream robConfig;
    //YARP device
    robConfig << "device remote_controlboard" << " ";
    //To what will be connected
    robConfig << "remote /teo/rightArm" << " ";
    //How will be called on YARP network
    robConfig << "local /local/rightArm/" << " ";
    MWI::Robot rightArm(robConfig);


     int axis = 0;
    //Amplitude in degrees
    double A = 10;
    double acc = 0.1; //accurracy to reach amplitude
    //Period in secods
    double T = 10;
    //Number of repetitions
    int loops = 4;
    double w = 2*M_PI/T;
    double waitYarp = 0.01;

    double actualPos=0;
    double actualVel=0;
    int direction=+1; // +1 forward, -1 backward
    int edge=0; //when position is A, and vel = 0.

    rightArm.SetJointPos(axis,0);
    yarp::os::Time::delay(2);

    for (int i=0; i<loops; i++)
    {

        actualPos=rightArm.GetJoint(axis);
        actualVel=A*w*cos(asin(actualPos/A));
        rightArm.SetJointVel(axis,actualVel);

        yarp::os::Time::delay(waitYarp);
        std::cout << T << ", v "
                  << actualVel << ", p "
                  << actualPos << ", edge "
                  << edge << ","
                  << std::endl;


    }


    rightArm.SetJointVel(axis,0);


    return 0;
}

