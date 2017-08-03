
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
std::vector<double> DqRightLeg(6,0), DqLeftLeg(6,0);
std::vector<double> qRightLeg(6,0), qLeftLeg(6,0);
double accelSmoother(valarray<double> &pos, const valarray<double> &dvels, const double dts);
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


    step.SetDefaultSpeeds(0.03,0.02);
    step.SetHipParameters(0.065,0.009,0.1);
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

    std::vector<valarray<double> > traj;

    double dtLeftLeg, dtRightLeg;

    double dts= 0.01;
    double st;
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

        if (peak_acc>max_accel)

        traj.push_back(pos);

        st=accelSmoother(pos, (vel-oldvel), dts);
        t+=st;
        if (  st==0  )
        {




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
        }


    }

    return 0;



}


double accelSmoother (valarray<double>& pos,const valarray<double>& dvel, const double dts)
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
        double n=min((long)(acc/max_accel),(long)20);//std::min((long)(acc/max_accel),(long)100);
        std::cout << "times: " << n << " acc " << acc << " max_accel " << max_accel << std::endl;

        //will use times velocity slices
        // but the ending position (sum of all qs) must be the same
        //so vels are like vels*times*dts=dpos
        vels=dvel/n;//(times*(times+1)/2);
        q=pos;
        for (long i=0; i<n; i++)
        {
            for (long j=0; j<q.size(); j++)
            {

                q[j]=q[j]+dvel[j]*dts*(i/n);

                std::cout << "j: " << j << " q[j] " << q[j] << " vels[j] " << vels[j] << std::endl;

            }
            for (long j=0; j<qRight.size(); j++)
            {

                qRight[j]=q[j];
                qLeft[j]=q[j+6];

                std::cout << "j: " << j << " qRight " << qRight[j] << " qLeft " << qLeft[j] << std::endl;

            }


            //to degrees
            std::transform(qLeft.begin(), qLeft.end(), qLeft.begin(),
                                         std::bind1st(std::multiplies<double>(), 180/M_PI));
            std::transform(qRight.begin(), qRight.end(), qRight.begin(),
                                         std::bind1st(std::multiplies<double>(), 180/M_PI));
            teoRightLeg.SetJointPositions(qRight);
            teoLeftLeg.SetJointPositions(qLeft);
            yarp::os::Time::delay(dts);
        }
        std::cout << "smooth: " << n << " by " << dts << " seconds " << std::endl;

        //need to compute the positon difference for the main traj
        //while main traj should have moved dts*times*dvel (average dvel) in position,
        //in the same time soft moved with average vel dvel*(times+1)/2);
        //so positions will be the same at dts =


        return dts*(long)0.99*((n+1)/2);

    }
    //at return, it will be dts*times later, and vels=dvel, but the calling function
    //wont notice, so traj time will increase but can keep the plan.

return 0.0;

}
