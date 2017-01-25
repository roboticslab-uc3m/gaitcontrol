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

    //Robot teo left Leg
    std::stringstream leftLegConfig;
    //YARP device
    leftLegConfig << "device remote_controlboard" << " ";
    //To what will be connected
    leftLegConfig << "remote /teo/leftLeg" << " ";
    //How will be called on YARP network
    leftLegConfig << "local /local/leftLeg/" << " ";
    MWI::Robot leftLeg(leftLegConfig);


    //Robot teo right leg
    std::stringstream rightLegConfig;
    //YARP device
    rightLegConfig << "device remote_controlboard" << " ";
    //To what will be connected
    rightLegConfig << "remote /teo/rightLeg" << " ";
    //How will be called on YARP network
    rightLegConfig << "local /local/rightLeg/" << " ";
    MWI::Robot rightLeg(rightLegConfig);

    int rHip = 1;
    int lHip = 1;
    int rKnee = 5;
    int lKnee = 5;
    //Amplitude in degrees
    double A = 1;
    //double acc = 0.1; //accurracy to reach amplitude
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

    rightLeg.DefaultPosition();
    leftLeg.DefaultPosition();
    yarp::os::Time::delay(2);

    rightLeg.SetJointVel(rHip,0);
    leftLeg.SetJointVel(lHip,0);
    rightLeg.SetJointVel(rKnee,0);
    leftLeg.SetJointVel(lKnee,0);


    Oscillator o1(T/2,T/2,A,-A);

    rightLeg.SetJointPos(rHip,0);
    yarp::os::Time::delay(4);

    double Ts=T/100;

    //hip shake
    /*double t=0;

    for (int i=0; i<loops; i++)
    {

        for (t=0; t<T; t=t+Ts)
        {
            actualVel=o1.GetVelocity(t);

            actualPos=rightLeg.GetJoint(rHip);
            rightLeg.SetJointVel(rHip,actualVel);

            leftLeg.SetJointVel(lHip,-actualVel);

            rightLeg.SetJointVel(rKnee,actualVel);

            leftLeg.SetJointVel(lKnee,-actualVel);

            yarp::os::Time::delay(Ts);
            //std::cout << t << ", v " << actualVel << ", p " << actualPos << std::endl;
        }


    }
    */
    //squat
    double t=0;

    for (int i=0; i<loops; i++)
    {

        for (t=0; t<T; t=t+Ts)
        {
            actualVel=o1.GetVelocity(t);

            actualPos=rightLeg.GetJoint(rHip);
            rightLeg.SetJointVel(2,actualVel);
            rightLeg.SetJointVel(3,actualVel);
            rightLeg.SetJointVel(4,actualVel);


            leftLeg.SetJointVel(2,actualVel);
            leftLeg.SetJointVel(3,actualVel);
            leftLeg.SetJointVel(4,actualVel);


            yarp::os::Time::delay(Ts);
            //std::cout << t << ", v " << actualVel << ", p " << actualPos << std::endl;
        }


    }




    /*
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

    */

    //rightArm.SetJointVel(axis,-actualVel);


 /*   std::cout << " Edge------------------------------- ";

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

    std::cout << " Direction------------------------------- ";*/



    rightLeg.Stop();
    leftLeg.Stop();

    return 0;


}

