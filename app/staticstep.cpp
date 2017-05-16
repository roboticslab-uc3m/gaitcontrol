
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

    kin::Pose initialRightFoot(0,-0.1285,-0.85);
    initialRightFoot.ChangeRotation(0,1,0,-M_PI/2);
    initialRightFoot.ChangeRotation(1,0,0,M_PI);
    kin::Pose initialLeftFoot(0,+0.15,-0.85);
    initialLeftFoot.ChangeRotation(0,1,0,-M_PI/2);
    //every gait operation start with instantiation and initialization of a Gait child class
    teo::GaitSupportPoligon step(initialRightFoot,initialLeftFoot);


    tra::SpaceTrajectory traRightLeg, traLeftLeg;

    step.BeforeStep();
    step.AddStepForward(1);
    step.GetTrajectories(traRightLeg, traLeftLeg);


    kin::Pose kinposeRightLeg, kinposeLeftLeg;
    std::vector<double> poseRightLeg(12,0), poseLeftLeg(12,0);
    std::vector<double> angsRightLeg(6,0), angsLeftLeg(6,0);
    double dtLeftLeg, dtRightLeg;

    for (int i = 0; i < traRightLeg.Size(); i++)
    {
        traLeftLeg.GetWaypoint(i, kinposeLeftLeg, dtLeftLeg);
        kinposeLeftLeg.GetPose(poseLeftLeg);

        traRightLeg.GetWaypoint(i, kinposeRightLeg, dtRightLeg);
        kinposeRightLeg.GetPose(poseRightLeg);

        teokin.LeftLegInvKin(poseLeftLeg, angsLeftLeg);
        teokin.RightLegInvKin(poseRightLeg, angsRightLeg);

        teoLeftLeg.SetJointPositions(angsLeftLeg);
        teoRightLeg.SetJointPositions(angsRightLeg);


        yarp::os::Time::delay(dtLeftLeg);

    }

    return 0;



}
