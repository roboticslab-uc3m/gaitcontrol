#include <time.h>
#include <chrono>

#include "IKinematics.hpp"
#include "MiddlewareInterface.h"
#include "GaitSupportPoligon.h"
#include "tools.h"

#define ROBOT "teoSim"

using namespace roboticslab;
using namespace std;

double max_accel=(100.0/60.0);//rad/s²
std::vector<double> DqRightLeg(6,0), DqLeftLeg(6,0);
std::vector<double> qRightLeg(6,0), qLeftLeg(6,0);
double accelSmoother(valarray<double> &pos, const valarray<double> &dvels, const double dts);
double peak_acc = 0;
double maxAcceleration=10; //deg/s²
bool checkAcceleration(std::vector<double> accRight,std::vector<double> accLeft, double maxAcc);
int checkLimitantAcceleration(std::vector<double> accLeft, std::vector<double> accRight);
int calculateSmothSteps(double topVelocity, double bottomVelocity, double maxACC);


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

    step.Squat(0.01);

    //step.AddStepForward(1);
    //step.AfterStep();
    step.GetTrajectories(traRightLeg, traLeftLeg);


    kin::Pose kinposeRightLeg, kinposeLeftLeg;
    std::vector<double> poseRightLeg(12,0), poseLeftLeg(12,0);
    std::vector<double> angsRightLeg(6,0), angsLeftLeg(6,0);//Actual angular values
    std::vector<double> q1RightLeg(6,0), q1LeftLeg(6,0);//q-1 vector of angular values
    std::vector<double> q2RightLeg(6,0), q2LeftLeg(6,0);//q-2 vector of angular values
    std::vector<double> v1RightLeg(6,0), v1LeftLeg(6,0);
    std::vector<double> v2RightLeg(6,0), v2LeftLeg(6,0);
    std::vector<double> accRightLeg(6,0), accLeftLeg(6,0);
    std::vector<double> incrementRight(6,0), incrementLeft(6,0);
    std::vector<double> currRightLeg(6,0), currLeftLeg(6,0);


    double dtLeftLeg, dtRightLeg;

    std::ofstream exportData;
    exportData.open("exportData.csv");
/*
    if(exportData.is_open())
    {
        exportData << "t,angsLeftLeg,angsRightLeg,accLeftLeg,accRightLeg" << std::endl;
    }else{
        std::cout << "Unable to open file" << std::endl;
    }

    /*
    for(int i=0; i<traLeftLeg.GetTimeTotalsSize(); i++)
    {
        std::cout << "LeftLegDuration: " << traLeftLeg.GetIndividualDuration(i) << std::endl;
        std::cout << "RightLegDuration: " << traRightLeg.GetIndividualDuration(i) << std::endl;
    }
    std::cout << "Size of time totals Left: " << traLeftLeg.GetTimeTotalsSize() << std::endl;
    std::cout << "Size of time totals Right: " << traRightLeg.GetTimeTotalsSize() << std::endl;
    */

    double elapsedTime=traRightLeg.GetTotalDuration();

    std::cout << "Total duration: " << traRightLeg.GetTotalDuration() << std::endl;

    double dts= 0.01;

    for (double t = 0.01; t < traRightLeg.GetTotalDuration(); t=dts+t)
    {

        std::cout << "t: " << t << std::endl;

        traLeftLeg.GetSample(t,kinposeLeftLeg);



        //traLeftLeg.GetWaypoint(i,kinposeLeftLeg,t);
        //dt=t-dt;
        kinposeLeftLeg.GetPoseMatrix(poseLeftLeg);

        traRightLeg.GetSample(t, kinposeRightLeg);
        //traRightLeg.GetWaypoint(i,kinposeRightLeg);
        kinposeRightLeg.GetPoseMatrix(poseRightLeg);

        teokin.LeftLegInvKin(poseLeftLeg, angsLeftLeg);
        teokin.RightLegInvKin(poseRightLeg, angsRightLeg);

        angsLeftLeg[1]= -1*angsLeftLeg[1];
        angsRightLeg[5]= -1*angsRightLeg[5];

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


        std::cout << std::endl;



        std::cout << "Angular acceleration of the right leg:" << accRightLeg << std::endl;
        std::cout << "Angular acceleration of the left leg:" << accLeftLeg << std::endl;

        bool checkAcc = checkAcceleration(accRightLeg, accRightLeg, maxAcceleration);
        std::vector<double> viRightLeg(6,0), viLeftLeg(6,0);


        std::cout << "Acceleration check: " << checkAcc << std::endl;
/*
        if((checkAcc == 1 )&&(t>2))
        {
            std::cout << "Acceleration too high, must smoth. "<<std::endl;
            int objectiveAccelerationJoint = checkLimitantAcceleration(accLeftLeg, accRightLeg);
            double objectiveVelocity, baseVelocity;
            int nSteps=0;

            if(objectiveAccelerationJoint>=6)
            {
                objectiveVelocity = v1RightLeg[objectiveAccelerationJoint-6];
                baseVelocity = v2RightLeg[objectiveAccelerationJoint-6];
            }
            else
            {
                objectiveVelocity = v1LeftLeg[objectiveAccelerationJoint];
                baseVelocity = v2LeftLeg[objectiveAccelerationJoint];
            }

            nSteps = calculateSmothSteps(objectiveVelocity, baseVelocity, maxAcceleration);

            bool limitingLeg=0;


            if(objectiveAccelerationJoint>=6)
            {
                //Limit in the Right Leg
                limitingLeg=0;
                objectiveAccelerationJoint=objectiveAccelerationJoint-6;
            }
            else
            {
                //Limit in the Left Leg
                limitingLeg=1;
            }

            std::cout << "Numer of steps required: " << nSteps << std::endl;

            //calculate necesary increments

            for(int i=0; i<6; i++)
            {
                incrementLeft[i]=(angsLeftLeg[i]-q2LeftLeg[i])/nSteps;
                incrementRight[i]=(angsRightLeg[i]-q2RightLeg[i])/nSteps;
            }


            for(int i=0; i<nSteps-1; i++)
            {
                for(int j=6; j<6; j++)
                {
                    q2RightLeg[i]+=incrementRight[i];
                    q2LeftLeg[i]+=incrementLeft[i];
                }
                //solo la maxima
                /*if(limitingLeg==0)
                {
                    //Limit in the Right Leg
                    for(int j=0; j++; j<6)
                    {
                        if(j==objectiveAccelerationJoint)
                        {
                            viRightLeg[objectiveAccelerationJoint]   =   viRightLeg[objectiveAccelerationJoint]    + (maxAcceleration*dts);
                            q2RightLeg[objectiveAccelerationJoint]   =   q2RightLeg[objectiveAccelerationJoint]    + (viRightLeg[i]*dts);
                        }
                        else
                        {
                            q2RightLeg[i]   =   q2RightLeg[i]    + (incrementRight[i]*dts);
                            q2LeftLeg[i]    =   q2LeftLeg[i]     + (incrementLeft[i]*dts);
                        }
                    }

                }

                /*for(int i=0;i<6;i++)
                {
                    /*
                    viRightLeg[i]   =   viRightLeg[i]    + (maxAcceleration*dts);
                    viLeftLeg[i]    =   viLeftLeg[i]     + (maxAcceleration*dts);
                    q2RightLeg[i]   =   q2RightLeg[i]    + (viRightLeg[i]*dts);
                    q2LeftLeg[i]    =   q2LeftLeg[i]     + (viLeftLeg[i]*dts);


                }

                teoLeftLeg.SetJointPositions(q2RightLeg);
                teoRightLeg.SetJointPositions(q2LeftLeg);

                yarp::os::Time::delay(dts);
            }

            //Update time_totals
            std::cout << "Acceleration smooth finished "<< std::endl;

            //elapsedTime+=(nSteps-2)*dts;

        }*/


        teoLeftLeg.SetJointPositions(q2RightLeg);
        teoRightLeg.SetJointPositions(q2LeftLeg);

        //Get currents

        for(int i=0; i<6; i++)
        {
            currRightLeg[i]=teoRightLeg.GetCurrent(i);
            currLeftLeg[i]=teoLeftLeg.GetCurrent(i);
        }

        q2RightLeg = q1RightLeg;
        q2LeftLeg = q1LeftLeg;
        q1RightLeg = angsRightLeg;
        q1LeftLeg = angsLeftLeg;

        v2RightLeg = v1RightLeg;
        v2LeftLeg = v1LeftLeg;

        std::cout << "new waypoint: " << t << " will take " << dts << " seconds " << std::endl;
        std::cout << "leftLeg" << q2LeftLeg << std::endl;
        std::cout << "rightLeg" << q2RightLeg << std::endl;

/*
        std::cout << "Currents (A) Right Leg:" << currRightLeg << std::endl;
        std::cout << "Currents (A) Left Leg:" << currLeftLeg << std::endl;

        if(exportData.is_open())
        {
            //exportData << t << " " <<q2LeftLeg << " " << q2RightLeg << " " << accLeftLeg << " " << accRightLeg << std::endl;
            exportData << t << " " <<currLeftLeg << " " << q2RightLeg << " " << currRightLeg << std::endl;
        }
        else
        {
            std::cout << "Unable to open file" << std::endl;
        }

*/
        yarp::os::Time::delay(dts);


    }

    exportData.close();
    return 0;
}

bool checkAcceleration(std::vector<double> accRight,std::vector<double> accLeft, double maxAcc)
{

    for(int i=0; i<6; i++)
    {
        accRight[i]=abs(accRight[i]);

        if(accRight[i]>maxAcc)
        {
            std::cout << "Max acceleration reached in the joint: " << i << " of the Right Leg" << std::endl;
            return true;
        }
    }
    for(int i=0; i<6; i++)
    {
        accLeft[i]= abs(accLeft[i]);

        if(accLeft[i]>maxAcc)
        {
            std::cout << "Max acceleration reached in the joint: " << i << " of the Left Leg" << std::endl;
            return true;
        }
    }
    return false;
}
int checkLimitantAcceleration(std::vector<double> accLeft, std::vector<double> accRight)
{
    int limitAcc=0;
    double auxAcc=0;

    for(int i=0; i<6; i++)
    {
        if(abs(accLeft[i])>auxAcc)
        {
            auxAcc=accLeft[i];
            limitAcc=i;
        }
    }

    for(int i=0;i<6; i++)
    {
        if(abs(accRight[i])>auxAcc)
        {
            auxAcc=accRight[i];
            limitAcc=i+6;
        }
    }

    return limitAcc;
}
int calculateSmothSteps(double topVelocity, double bottomVelocity, double maxACC)
{
    double steps;
    steps = (topVelocity-bottomVelocity)/maxACC;
    steps = abs(ceil(100*steps));
    return steps;
}
