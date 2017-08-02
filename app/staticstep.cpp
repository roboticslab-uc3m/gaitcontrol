
#include <time.h>
#include <chrono>

#include "IKinematics.hpp"
#include "MiddlewareInterface.h"
#include "GaitSupportPoligon.h"
#include "tools.h"

#define ROBOT "teoSim"

using namespace roboticslab;
using namespace std;

double max_accel=(100.0/60.0);//rad/s^2
std::vector<double> DqRightLeg(6,0), DqLeftLeg(6,0);
std::vector<double> qRightLeg(6,0), qLeftLeg(6,0);
double accelSmoother(const valarray<double> &pos, const valarray<double> &dvels, const double dts);
MWI::Limb teoRightLeg(ROBOT,"rightLeg"), teoLeftLeg(ROBOT,"leftLeg");
double peak_acc = 0;

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

    valarray<double> oldpos(0.0, 12);
    valarray<double> dpos(0.0, 12);
    valarray<double> pos(0.0, 12); //if the first position is all zeros!!
    valarray<double> vel(0.0, 12); //if the first position has all zero vels!!
    valarray<double> acc(0.0, 12); //if the first position is all zeros!!

    valarray<double> oldvel(0.0, 12);

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

        //store pos
        for (int i=0; i<6; i++)
        {
            pos[i]=angsRightLeg[i];
            pos[i+6]=angsLeftLeg[i];
            std::cout << "pos[i]: " << pos[i] << " angsRightLeg[i]: " << angsRightLeg[i] << "pos[i+6]: " << pos[i+6] << std::endl;
            std::cout << "(vel-oldvel)[i]: " << (vel-oldvel)[i] << " (vel-oldvel)[i+6]: " << (vel-oldvel)[i+6] << std::endl;

        }
        std::cout << "pos.max(): " << pos.max() << " vel.max(): " << vel.max() << " oldvel.max(): " << oldvel.max() << std::endl;

        vel=(pos-oldpos)/dts;
        acc=(vel-oldvel)/dts;

        peak_acc = max(acc.max(),-acc.min());

        if (  peak_acc>max_accel  )
        {


        }

        accelSmoother(pos, (vel-oldvel), dts);





        //to degrees
        std::transform(angsLeftLeg.begin(), angsLeftLeg.end(), angsLeftLeg.begin(),
                                     std::bind1st(std::multiplies<double>(), 180/M_PI));
        std::transform(angsRightLeg.begin(), angsRightLeg.end(), angsRightLeg.begin(),
                                     std::bind1st(std::multiplies<double>(), 180/M_PI));

        teoLeftLeg.SetJointPositions(angsLeftLeg);
        teoRightLeg.SetJointPositions(angsRightLeg);


        std::cout << "new waypoint: " << t << " will take " << dts << " seconds " << std::endl;
        std::cout << "leftLeg" << angsLeftLeg << std::endl;
        std::cout << "rightLeg" << angsRightLeg << std::endl;

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


double accelSmoother (const valarray<double>& pos,const valarray<double>& dvel, const double dts)
{
//    valarray<double> dpos(angs, oldangs);
//    valarray<double> vels(angs, oldangs);

//    dvel=dpos/dts;
//    valarray<double> dvel
//            (vels, oldvels);

    for (int i=0; i<12; i++)
    {
        std::cout << i << " pos[i]: " << pos[i] << " dvel[i]: " << dvel[i] << std::endl;


    }

    double acc=max(dvel.max(),-dvel.min())/dts;
    std::cout << "acc: " << acc << " dvel.max(): " << dvel.max() << "  max_accel " << max_accel << std::endl;

    if(acc>max_accel)
    {
        valarray<double> q(0.0,12);
        valarray<double> vels(0.0,12);
        vector<double> qRight(6,0),qLeft(6,0);
        //how many times acc is bigger than max
        double times=(long)(acc/max_accel);
        std::cout << "times: " << times << " acc " << acc << " max_accel " << max_accel << std::endl;

        //will use times velocity slices
        // but the ending position (sum of all qs) must be the same
        //so vels are like vels*times*dts=dpos
        vels=dvel/times;//(times*(times+1)/2);
        q=pos;
        for (long i=0; i<times; i++)
        {
            for (long j=0; j<q.size()/2; j++)
            {

                q[j]=q[j]+vels[j]*dts;//*i;
                qRight[j]=q[j];
                qLeft[j]=q[j+6];
                //std::cout << "j: " << j << " qRight " << qRight[j] << " qLeft " << qLeft[j] << std::endl;

            }

            //to degrees
            std::transform(qLeft.begin(), qLeft.end(), qLeft.begin(),
                                         std::bind1st(std::multiplies<double>(), 180/M_PI));
            std::transform(qRight.begin(), qRight.end(), qRight.begin(),
                                         std::bind1st(std::multiplies<double>(), 180/M_PI));
            teoRightLeg.SetJointPositions(qRight);
            teoLeftLeg.SetJointPositions(qLeft);
            yarp::os::Time::delay(10*dts/times);
        }
        std::cout << "smooth: " << times << " by " << dts << " seconds " << std::endl;

        //need to compute the positon difference for the main traj
        //while main traj should have moved dts*times*dvel (average dvel) in position,
        //in the same time soft moved with average vel dvel*(times+1)/2);
        //so positions will be the same at dts =


        return 0.01;// dts*((int)(times+1)/2);

    }
    //at return, it will be dts*times later, and vels=dvel, but the calling function
    //wont notice, so traj time will increase but can keep the plan.

return 0.0;

}
