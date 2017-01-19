#include <iostream>
#include <fstream>
#include <math.h>

#include "MiddlewareInterface.h"
#include "SensorIntegration.h"
#include "Oscillator.hpp"



int main()
{


    //Middleware

    std::cout << "[Initial]";
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

    //Robot teo right arm
    std::stringstream robConfig;
    //YARP device
    robConfig << "device remote_controlboard" << " ";
    //To what will be connected
    robConfig << "remote /teoSim/rightArm" << " ";
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
    int loops = 2;
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

        while (pow(actualPos,2)<pow(A,2))
        {
            //important!! Set actualVel before get actualPos, so position can be less than Amplitude
            actualVel=direction*A*w*cos(asin(actualPos/A));

            actualPos=rightArm.GetJoint(axis);
            rightArm.SetJointVel(axis,actualVel);

            yarp::os::Time::delay(waitYarp);
            std::cout << T << ", v "
                      << actualVel << ", p "
                      << actualPos << ", direction "
                      << direction << ","
                      << std::endl;
        }
        rightArm.SetJointVel(axis,-actualVel);

        std::cout << " Edge------------------------------- ";

        while (pow(actualPos,2)>=pow(A,2))
        {
            //actualVel=(-actualPos/A)*actualVel;
            actualPos=rightArm.GetJoint(axis);

            yarp::os::Time::delay(waitYarp);
            std::cout << T << ", v "
                      << actualVel << ", p "
                      << actualPos << ", direction "
                      << direction << ","
                      << std::endl;
        }
        direction=direction*-1;

        std::cout << " Direction------------------------------- ";


    }


    rightArm.SetJointVel(axis,0);


    Oscillator o1(T,+A,-A);

    rightArm.SetJointPos(axis,0);
    yarp::os::Time::delay(2);

    for (int i=0; i<loops; i++)
    {

        while (pow(actualPos,2)<pow(A,2))
        {
            //important!! Set actualVel before get actualPos, so position can be less than Amplitude
            actualVel=o1.GetVelocity(actualPos);

            actualPos=rightArm.GetJoint(axis);
            rightArm.SetJointVel(axis,actualVel);

            yarp::os::Time::delay(waitYarp);
            std::cout << T << ", v "
                      << actualVel << ", p "
                      << actualPos << ", direction "
                      << direction << ","
                      << std::endl;
        }
        rightArm.SetJointVel(axis,-actualVel);

        std::cout << " Edge------------------------------- ";

        while (pow(actualPos,2)>=pow(A,2))
        {
            //actualVel=(-actualPos/A)*actualVel;
            actualPos=rightArm.GetJoint(axis);

            yarp::os::Time::delay(waitYarp);
            std::cout << T << ", v "
                      << actualVel << ", p "
                      << actualPos << ", direction "
                      << direction << ","
                      << std::endl;
        }
        direction=direction*-1;

        std::cout << " Direction------------------------------- ";


    }


    return 0;
}

