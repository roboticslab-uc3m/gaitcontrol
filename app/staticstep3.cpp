#include <time.h>
#include <chrono>

#include "IKinematics.hpp"
#include "MiddlewareInterface.h"
#include "GaitSupportPoligon.h"
#include "tools.h"

#define ROBOT "teo"

using namespace roboticslab;
using namespace std;

double max_accel=(100.0/60.0);//rad/s²
std::vector<double> DqRightLeg(6,0), DqLeftLeg(6,0);
std::vector<double> qRightLeg(6,0), qLeftLeg(6,0);
double accelSmoother(valarray<double> &pos, const valarray<double> &dvels, const double dts);
double peak_acc = 0;
double maxAcceleration=10; //deg/s²
bool checkAcceleration(std::vector<double> accRight,std::vector<double> accLeft, double maxAcc);

int main()
{
    MWI::Limb teoRightLeg(ROBOT,"rightLeg"), teoLeftLeg(ROBOT,"leftLeg");

    teoRightLeg.SetControlMode(1);
    teoLeftLeg.SetControlMode(1);

    IKinematics teokin;

    kin::Pose initialRightFoot(0,-0.1285,-0.845);

    kin::Pose initialLeftFoot(0,+0.1285,-0.845);

    //every gait operation start with instantiation and initialization of a Gait child class
    roboticslab::GaitSupportPoligon step(initialRightFoot,initialLeftFoot);


    tra::SpaceTrajectory traRightLeg, traLeftLeg;


    step.SetDefaultSpeeds(0.03,0.02);//rad/s
    step.SetHipParameters(0.065,0.01,0.1);
    step.SetKickParameters(0.06,0.03);
    step.BeforeStep();

    step.AddStepForward(1);
    step.AfterStep();
    step.GetTrajectories(traRightLeg, traLeftLeg);


    kin::Pose kinposeRightLeg, kinposeLeftLeg;
    std::vector<double> poseRightLeg(12,0), poseLeftLeg(12,0);
    std::vector<double> angsRightLeg(6,0), angsLeftLeg(6,0);//Actual angular values
    std::vector<double> q1RightLeg(6,0), q1LeftLeg(6,0);//q-1 vector of angular values
    std::vector<double> q2RightLeg(6,0), q2LeftLeg(6,0);//q-2 vector of angular values
    std::vector<double> v1RightLeg(6,0), v1LeftLeg(6,0);
    std::vector<double> v2RightLeg(6,0), v2LeftLeg(6,0);
    std::vector<double> accRightLeg(6,0), accLeftLeg(6,0);
    //std:vector<double> currRightLeg(6,0), currLeftLeg(6,0);


    double dtLeftLeg, dtRightLeg;

    std::ofstream exportData;
    exportData.open("exportData.csv");
    if(exportData.is_open())
    {
        exportData << "t,angsLeftLeg,angsRightLeg,accLeftLeg,accRightLeg" << std::endl;
    }else{
        std::cout << "Unable to open file" << std::endl;
    }

    double dts= 0.01;
    for (double t = 0.01; t < traRightLeg.GetTotalDuration(); t=dts+t)
    //for (int i=0; i< traLeftLeg.Size(); i++)
    {
        

        traLeftLeg.GetSample(t,kinposeLeftLeg);



        //traLeftLeg.GetWaypoint(i,kinposeLeftLeg,t);
        //dt=t-dt;
        kinposeLeftLeg.GetPoseMatrix(poseLeftLeg);

        traRightLeg.GetSample(t, kinposeRightLeg);
        //traRightLeg.GetWaypoint(i,kinposeRightLeg);
        kinposeRightLeg.GetPoseMatrix(poseRightLeg);

        teokin.LeftLegInvKin(poseLeftLeg, angsLeftLeg);
        teokin.RightLegInvKin(poseRightLeg, angsRightLeg);

        angsRightLeg[1]= -1*angsRightLeg[1];
        angsLeftLeg[5]= -1*angsLeftLeg[6];

        //to degrees



        std::transform(angsLeftLeg.begin(), angsLeftLeg.end(), angsLeftLeg.begin(),
                                     std::bind1st(std::multiplies<double>(), 180/M_PI));
        std::transform(angsRightLeg.begin(), angsRightLeg.end(), angsRightLeg.begin(),
                                     std::bind1st(std::multiplies<double>(), 180/M_PI));

        //1st step, calculate the angular velocities
        



        for(int i=0; i<6; i++)
        {
            v1RightLeg[i] = (q1RightLeg[i]-angsRightLeg[i])/ dts;
            v1LeftLeg[i] = (q1LeftLeg[i]-angsLeftLeg[i] ) / dts;

            accRightLeg[i] = ( v2RightLeg[i]-v1RightLeg[i]) / dts;
            accLeftLeg[i] = ( v2LeftLeg[i] - v1LeftLeg[i]) / dts;
        }



        std::cout << "Angular acceleration of the right leg:" << accRightLeg << std::endl;
        std::cout << "Angular acceleration of the left leg:" << accLeftLeg << std::endl;
        
        bool checkAcc= checkAcceleration(accRightLeg, accRightLeg, maxAcceleration);


        teoLeftLeg.SetJointPositions(angsLeftLeg);
        teoRightLeg.SetJointPositions(angsRightLeg);

        //Get currents
        /*
        for(int i=0; i<5; i++)
        {
            currRightLeg[i]=teoRightLeg.GetCurrent(i);
            currLeftLeg[i]=teoLeftLeg.GetCurrent(i);
        }
        */
        q2RightLeg = q1RightLeg;
        q2LeftLeg = q1LeftLeg;
        q1RightLeg = angsRightLeg;
        q1LeftLeg = angsLeftLeg;

        v2RightLeg = v1RightLeg;
        v2LeftLeg = v1LeftLeg;

        std::cout << "new waypoint: " << t << " will take " << dts << " seconds " << std::endl;
        std::cout << "leftLeg" << angsLeftLeg << std::endl;
        std::cout << "rightLeg" << angsRightLeg << std::endl;


//        std::cout << "Currents (A) Right Leg:" << currRightLeg << std::endl;
//        std::cout << "Currents (A) Left Leg:" << currLeftLeg << std::endl;

        if(exportData.is_open())
        {
            exportData << t << " " <<angsLeftLeg << " " << angsRightLeg << " " << accLeftLeg << " " << accRightLeg << std::endl;

        }else{

            std::cout << "Unable to open file" << std::endl;
        }


        yarp::os::Time::delay(dts);


    }

    exportData.close();
    return 0;
}

bool checkAcceleration(std::vector<double> accRight,std::vector<double> accLeft, double maxAcc)
{

    for(int i=0; i<6; i++)
    {
        if(accRight[i]>maxAcc)
        {
            std::cout << "Max acceleration reached in the joint: " << i << "of the Right Leg" << std::endl;
            return true;
        }
    }
    for(int i=0; i<6; i++)
    {
        if(accLeft[i]>maxAcc)
        {
            std::cout << "Max acceleration reached in the joint: " << i << "of the Left Leg" << std::endl;
            return true;
        }
    }
    return false;
}
