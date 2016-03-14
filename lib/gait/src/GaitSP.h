#include "Gait.h"
#include "tools.h"
#include <stdio.h>

using namespace teo::tra;

class GaitSP : public Gait
{
public:

    /**
     * @brief spGait::spGait = Constructor with the initial feet poses.
     * @param initialRightFoot = Initial right foot pose.
     * @param initialLeftFoot = Initial left foot pose.
     */
    GaitSP(Pose initialRightFoot, Pose initialLeftFoot);


    /**
     * @brief This function will add "stepNumber" steps in forward direction, using
     * support poligon stability criteria. Results are stored in the Space trajectory variable.
     * @param stepNumber = Number of steps to add.
     * @return true at success.
     */
    bool AddStepForward(int stepNumber);


    /**
     * @brief GaitSP::SaveSpaceTrajectories: Will save existing trajectories in the class (all the waypoints) in two csv files,
     * one for each foot.
     * @param fileLeftFoot: File (std::ofstream) for saving left foot trajectory.
     * @param fileRightFoot: File (std::ofstream) for saving right foot trajectory.
     * @return true on success.
     */
    bool SaveSpaceTrajectories(std::ofstream &fileLeftFoot, std::ofstream &fileRightFoot);


    /**
     * @brief spGait::SetStepParameters = Set the step parameters for the gait functions.
     * @param swingFootDistance = The distance the floating foot will move forward on every step.
     * @param swingFootElevation = The distance the floating foot will raise from ground on every step.
     * @return
     */
    bool SetStepParameters(double swingFootDistance, double swingFootElevation);


private:

    //parameters in meters, seconds

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
