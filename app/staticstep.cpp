
#include <time.h>
#include <chrono>

#include "IKinematics.hpp"
#include "MiddlewareInterface.h"
#include "GaitSupportPoligon.h"
#include "tools.h"

#define ROBOT "teoSim"

using namespace roboticslab;
using namespace std;

double max_accel=1/60;//rad/s^2
std::vector<double> DqRightLeg(6,0), DqLeftLeg(6,0);
std::vector<double> qRightLeg(6,0), qLeftLeg(6,0);
long accelSmoother (const valarray<double> &pos, const valarray<double> &dvels, const double dts);
MWI::Limb teoRightLeg(ROBOT,"rightLeg"), teoLeftLeg(ROBOT,"leftLeg");


int main()
{
    teoRightLeg.SetControlMode(1);
    teoLeftLeg.SetControlMode(1);

    IKinematics teokin;

    kin::Pose initialRightFoot(0,-0.1285,-0.845);

    kin::Pose initialLeftFoot(0,+0.1285,-0.845);

    //every gait operation start with instantiation and initialization of a Gait child class
    roboticslab::GaitSupportPoligon step(initialRightFoot,initialLeftFoot);


    tra::SpaceTrajectory traRightLeg, traLeftLeg;


    step.SetDefaultSpeeds(0.04,0.04);
    step.SetHipParameters(0.06,0.01,0.1);
    step.SetKickParameters(0.07,0.03);
    step.BeforeStep();
    step.AddStepForward(1);
    step.AfterStep();
    step.GetTrajectories(traRightLeg, traLeftLeg);


    kin::Pose kinposeRightLeg, kinposeLeftLeg;
    std::vector<double> poseRightLeg(12,0), poseLeftLeg(12,0);
    std::vector<double> angsRightLeg(6,0), angsLeftLeg(6,0);
    //std::vector<double> qRightLeg(6,0), qLeftLeg(6,0);

    valarray<double> oldpos(12, 0);
    valarray<double> dpos(12, 0);
    valarray<double> pos(12, 0); //if the first position is all zeros!!
    valarray<double> vel(12, 0); //if the first position has all zero vels!!
    valarray<double> oldvel(12, 0);

    double dtLeftLeg, dtRightLeg;

    double dts= 0.01;
    double t;
    for (double t = 0.01; t < traRightLeg.GetTotalDuration(); t=dts+t)
    //for (int i=0; i< traLeftLeg.Size(); i++)
    {

        //store old values
        oldpos=pos;
        oldvel=vel;

        traLeftLeg.GetSample(t,kinposeLeftLeg);
        //traLeftLeg.GetWaypoint(i,kinposeLeftLeg,t);
        //dt=t-dt;
        kinposeLeftLeg.GetPoseMatrix(poseLeftLeg);

        traRightLeg.GetSample(t, kinposeRightLeg);
        //traRightLeg.GetWaypoint(i,kinposeRightLeg);
        kinposeRightLeg.GetPoseMatrix(poseRightLeg);

        teokin.LeftLegInvKin(poseLeftLeg, angsLeftLeg);
        teokin.RightLegInvKin(poseRightLeg, angsRightLeg);

        //compute pos
        for (int i=0; i<pos.size()/2; i++)
        {
            pos[i]=angsRightLeg[i];
            pos[i+6]=angsLeftLeg[i];
        }

        vel=(pos-oldpos)/dts;
        accelSmoother((pos-oldpos), (vel-oldvel), dts);


        //to degrees
        std::transform(angsLeftLeg.begin(), angsLeftLeg.end(), angsLeftLeg.begin(),
                                     std::bind1st(std::multiplies<double>(), 180/M_PI));
        std::transform(angsRightLeg.begin(), angsRightLeg.end(), angsRightLeg.begin(),
                                     std::bind1st(std::multiplies<double>(), 180/M_PI));

        teoLeftLeg.SetJointPositions(angsLeftLeg);
        teoRightLeg.SetJointPositions(angsRightLeg);


//        std::cout << "new waypoint: " << t << " will take " << dt << " seconds " << std::endl;
//        std::cout << "leftLeg" << angsLeftLeg << std::endl;
//        std::cout << "rightLeg" << angsRightLeg << std::endl;

        yarp::os::Time::delay(dts);

//        bool done=false;
//        double prec = 1;
//        long maxRetries = 15;
//        for (int retries=0; retries <maxRetries; retries++)
//        {
//            yarp::os::Time::delay(dt/maxRetries);
//            done=true;
//            teoLeftLeg.GetJoints(qLeftLeg);
//            teoRightLeg.GetJoints(qRightLeg);
//            for (int i=0; i<qLeftLeg.size(); i++)
//            {
//                if ( (qLeftLeg[i]-angsLeftLeg[i] > prec) ||
//                     (qRightLeg[i]-angsRightLeg[i] > prec)
//                     )
//                {
//                    done=false;
//                }

//            }

//            if (done) break;
//        }
        //while (done==false);

        //dt=t;

    }

    return 0;



}


long accelSmoother (const valarray<double>& pos,const valarray<double>& dvel, const double dts)
{
//    valarray<double> dpos(angs, oldangs);
//    valarray<double> vels(angs, oldangs);

//    dvel=dpos/dts;
//    valarray<double> dvel
//            (vels, oldvels);


    double acc=dvel.max()/dts;
    if(acc>max_accel)
    {
        valarray<double> q(12,0);
        valarray<double> vels(12,0);
        vector<double> qRight(6,0),qLeft(6,0);
        double times=(long)(acc/max_accel);
        vels=dvel/(times*(times+1)/2);
        q=pos;
        for (long i=0; i<times; i++)
        {
            for (long j=0; j<q.size(); j++)
            {

                q[j]=q[j]+i*vels[j]*dts;
                qRight[j]=q[j];
                qLeft[j]=q[j+6];
            }
            teoRightLeg.SetJointPositions(qRight);
            teoLeftLeg.SetJointPositions(qLeft);
        }
    }

return 0;

}
