#include "Gait.h"
#include "tools.h"
#include <stdio.h>

using namespace teo::tra;

/**
 * @brief The GaitSupportPoligon class encapsulates cartesian foot trajectory generation for biped gaits.
 *
 * How to use:
 * 1- Instance by calling the constructor wiht feet Poses as parameters.
 * 2- Set the gait parameters with SetStepParameters function.
 * 3- Call the Add functions to add steps to trajectory.
 *
 * The object main properties are:
 * 1- SpaceTrajectory: All the steps will be stored in a cartesian trajectory object. Retrieve data with get functions.
 *
 */

class GaitSupportPoligon : public Gait
{
public:

    /**
     * @brief spGait::spGait = Constructor with the initial feet poses.
     * Frame of reference for feet poses must be the center of mass (com) at robot default configuration.
     * @param initialRightFoot = Initial right foot pose from com.
     * @param initialLeftFoot = Initial left foot pose from com.
     */
    GaitSupportPoligon(Pose initialRightFoot, Pose initialLeftFoot);


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

    bool GetTrajectories(tra::SpaceTrajectory& getRightFoot, tra::SpaceTrajectory& getLeftFoot);


private:

    //parameters in meters, seconds

    //swing foot parameters
    double swingElevation;
    double swingDistance;
    bool startOnRightFootSupport;

    //step parameters
    double t;
    int stepPhase;  //from 1 to .. step phases
    long stepTotalPhases; //total number of phases in a step

    //initial foot poses
    Pose poseRf0,poseLf0;

    //trajectories based on root
    SpaceTrajectory trajRightFoot, trajLeftFoot;

    //private functions
    bool HalfStepForwardRS();
    bool HalfStepForwardLS();
};
