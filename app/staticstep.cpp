
#include <time.h>
#include <chrono>

#include "IKinematics.hpp"
#include "MiddlewareInterface.h"
#include "GaitSupportPoligon.h"
#include "tools.h"

#define ROBOT "teoSim"

using namespace teo;

int main()
{

    MWI::Robot teoRightLeg(ROBOT,"rightLeg"), teoLeftLeg(ROBOT,"leftLeg");
    IKinematics teokin;

    kin::Pose initialRightFoot(0,-0.1285,-0.845);

    kin::Pose initialLeftFoot(0,+0.1285,-0.845);

    //every gait operation start with instantiation and initialization of a Gait child class
    teo::GaitSupportPoligon step(initialRightFoot,initialLeftFoot);


    tra::SpaceTrajectory traRightLeg, traLeftLeg;

    step.SetHipParameters(0.08,0.02,0.1);
    step.SetKickParameters(0.07,0.04);
    step.BeforeStep();
    step.AddStepForward(1);
    step.AfterStep();
    step.GetTrajectories(traRightLeg, traLeftLeg);


    kin::Pose kinposeRightLeg, kinposeLeftLeg;
    std::vector<double> poseRightLeg(12,0), poseLeftLeg(12,0);
    std::vector<double> angsRightLeg(6,0), angsLeftLeg(6,0);
    double dtLeftLeg, dtRightLeg;

    double dt= 0.1;
    for (double t = 0.1; t < traRightLeg.GetTotalDuration(); t=dt+t)
    {
        traLeftLeg.GetSample(t,kinposeLeftLeg);
        kinposeLeftLeg.GetPoseMatrix(poseLeftLeg);

        traRightLeg.GetSample(t, kinposeRightLeg);
        kinposeRightLeg.GetPoseMatrix(poseRightLeg);

        teokin.LeftLegInvKin(poseLeftLeg, angsLeftLeg);
        teokin.RightLegInvKin(poseRightLeg, angsRightLeg);

        std::transform(angsLeftLeg.begin(), angsLeftLeg.end(), angsLeftLeg.begin(),
                                     std::bind1st(std::multiplies<double>(), 180/M_PI));
        std::transform(angsRightLeg.begin(), angsRightLeg.end(), angsRightLeg.begin(),
                                     std::bind1st(std::multiplies<double>(), 180/M_PI));

        teoLeftLeg.SetJointPositions(angsLeftLeg);
        teoRightLeg.SetJointPositions(angsRightLeg);


        std::cout << "new waypoint: " << t << " will take " << dt << " seconds " << std::endl;
        std::cout << "leftLeg" << angsLeftLeg << std::endl;
        std::cout << "rightLeg" << angsRightLeg << std::endl;

        yarp::os::Time::delay(dt);

    }

    return 0;



}
