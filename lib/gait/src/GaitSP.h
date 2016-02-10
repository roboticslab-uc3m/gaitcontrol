#include "Gait.h"
#include "tools.h"
#include <stdio.h>

using namespace teo::tra;

class GaitSP : public Gait
{
public:

    GaitSP(Pose initialRightFoot, Pose initialLeftFoot);
    bool AddStepForward(int stepNumber);
    bool SaveSpaceTrajectories(std::ofstream &fileLeftFoot, std::ofstream &fileRightFoot);
    //parameters in meters, seconds

    bool SetStepParameters(double swingFootDistance, double swingFootElevation);
private:
    //swing foot parameters
    double swingElevation;
    double swingDistance;

    //step parameters
    double t;
    int stepPhase;  //from 1 to .. step phases
    long stepTotalPhases; //total number of phases in a step

    //initial foot poses
    Pose poseRf0,poseLf0;

    //trajectories based on root
    SpaceTrajectory trajRightFoot, trajLeftFoot;
    JointTrajectory jointRootTraj[3];
};
