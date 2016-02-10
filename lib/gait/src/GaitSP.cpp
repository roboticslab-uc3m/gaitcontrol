#include <iostream>
#include <vector>

using namespace std;

#include "GaitSP.h"

//kdl
//#include <frames.hpp>

/**
 * @brief This function will add "stepNumber" steps in forward direction, using
 * support poligon stability criteria. Results are stored in the Space trajectory variable.
 * @param stepNumber = Number of steps to add.
 * @return true at success.
 */
bool GaitSP::AddStepForward(int stepNumber)
{

    double x,y,z;
    double dx,dy,dz;
    Pose actualRightFoot, actualLeftFoot;
    Pose desiredRightFoot,desiredLeftFoot;


    //strategy:
    //-1-move root over right foot (or right foot under root (0,0,z), z is actual foot elevation)
    trajRightFoot.GetLastWaypoint(actualRightFoot);
    actualRightFoot.GetPosition(x,y,z);
    //origin (x,y,z) destination (0,0,z)
    dx=0-x;
    dy=0-y;
    dz=z-z;
    desiredRightFoot.ChangePosition(dx,dy,dz);
    desiredLeftFoot.ChangePosition(dx,dy,dz);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //-2-balance over right foot
    //TODO


    //-3-left foot forward
    //trajRightFoot.GetCurrentPose(desiredRightFoot);
    desiredLeftFoot.ChangePosition(swingDistance/2, 0, swingElevation);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //trajRightFoot.GetCurrentPose(desiredRightFoot);
    desiredLeftFoot.ChangePosition(swingDistance/2, 0, -swingElevation);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //-4-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (or feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-swingDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-swingDistance/2,0,0);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //-5-move root over left foot (or left foot under root (0,0,z), z is actual foot elevation)
    trajLeftFoot.GetLastWaypoint(actualLeftFoot);
    actualLeftFoot.GetPosition(x,y,z);
    //origin (x,y,z) destination (0,0,z)
    dx=0-x;
    dy=0-y;
    dz=z-z;
    desiredRightFoot.ChangePosition(dx,dy,dz);
    desiredLeftFoot.ChangePosition(dx,dy,dz);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //-6-balance over left foot.
    //TODO


    //-7-right foot forward
    //trajLeftFoot.GetCurrentPose(desiredLeftFoot);
    desiredRightFoot.ChangePosition(swingDistance/2, 0, swingElevation);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //trajLeftFoot.GetCurrentPose(desiredLeftFoot);
    desiredRightFoot.ChangePosition(swingDistance/2, 0, -swingElevation);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //-8-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (or feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-swingDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-swingDistance/2,0,0);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //one step finished


    return true;
}

/**
 * @brief GaitSP::SaveSpaceTrajectories: Will save existing trajectories in the class (all the waypoints) in two csv files,
 * one for each foot.
 * @param fileLeftFoot: File (std::ofstream) for saving left foot trajectory.
 * @param fileRightFoot: File (std::ofstream) for saving right foot trajectory.
 * @return true on success.
 */

bool GaitSP::SaveSpaceTrajectories(ofstream &fileLeftFoot, ofstream &fileRightFoot)
{

    trajLeftFoot.SaveToFile(fileLeftFoot);
    trajRightFoot.SaveToFile(fileRightFoot);
    return true;
}


/**
 * @brief spGait::spGait = Constructor with the initial feet poses.
 * @param initialRightFoot = Initial right foot pose.
 * @param initialLeftFoot = Initial left foot pose.
 */
GaitSP::GaitSP(Pose initialRightFoot, Pose initialLeftFoot)
{
    trajRightFoot.AddTimedWaypoint(-1, initialRightFoot);
    trajLeftFoot.AddTimedWaypoint(-1, initialLeftFoot);

    swingDistance = 0.0;
    swingElevation = 0.0;

}

/**
 * @brief spGait::SetStepParameters = Set the step parameters for the gait functions.
 * @param swingFootDistance = The distance the floating foot will move forward on every step.
 * @param swingFootElevation = The distance the floating foot will raise from ground on every step.
 * @return
 */
bool GaitSP::SetStepParameters( double swingFootDistance, double swingFootElevation )
{
    swingDistance = swingFootDistance;
    swingElevation = swingFootElevation;

    return true;
}

