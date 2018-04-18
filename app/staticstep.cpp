
#include <time.h>
#include <chrono>

#include "IKinematics.hpp"
#include "MiddlewareInterface.h"
#include "GaitSupportPoligon.h"
#include "tools.h"

#define ROBOT "teo"

using namespace roboticslab;
using namespace std;

double max_accel=(100.0/60.0);//rad/s^2
std::vector<double> DqRightLeg(6,0), DqLeftLeg(6,0);
std::vector<double> qRightLeg(6,0), qLeftLeg(6,0);
double accelSmoother(valarray<double> &pos, const valarray<double> &dvels, const double dts);
double peak_acc = 0;

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


    step.SetDefaultSpeeds(0.06,0.04);
    step.SetHipParameters(0.065,0.01,0.1);
    step.SetKickParameters(0.06,0.03);
    step.BeforeStep();

    step.AddStepForward(1);
    step.AfterStep();
    step.GetTrajectories(traRightLeg, traLeftLeg);


    kin::Pose kinposeRightLeg, kinposeLeftLeg;
    std::vector<double> poseRightLeg(12,0), poseLeftLeg(12,0);
    std::vector<double> angsRightLeg(6,0), angsLeftLeg(6,0);
    //std::vector<double> qRightLeg(6,0), qLeftLeg(6,0);


    double dtLeftLeg, dtRightLeg;

    // Opening file:

    //std::ofstream exportData;
    //exportData.open("exportData.csv");

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

        //angsRightLeg[1]= -1*angsRightLeg[1];
        //angsLeftLeg[5]= -1*angsLeftLeg[6];

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

        //if(exportData.is_open()){
        //    exportData << t << " " <<angsLeftLeg << " " << angsRightLeg << std::endl;
        //}
        //else std::cout << "Unable to open file" << std::endl;


        yarp::os::Time::delay(dts);

        //This section will export the data to a file in order to review the results.

    }
    //exportData.close();

    return 0;



}


