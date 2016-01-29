#include <iostream>
#include <vector>

using namespace std;

#include "spgait.h"

//kdl
//#include <frames.hpp>

/**
 * @brief This function will add "stepNumber" steps in forward direction, using
 * support poligon stability criteria. Results are stored in the Space trajectory variable.
 * @param stepNumber = Number of steps to add.
 * @return true at success.
 */
bool spGait::AddStepForward(int stepNumber)
{

    double x,y,z;
    double dx,dy,dz;
    Pose actualRightFoot, actualLeftFoot;
    Pose desiredRightFoot,desiredLeftFoot;


    //strategy
    //-1-move root over right foot (or right foot under root (0,0,z), z is actual foot elevation)
    trajRightFoot.GetCurrentPose(actualRightFoot);
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
    trajLeftFoot.GetCurrentPose(actualLeftFoot);
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
 * @brief spGait::spGait = Constructor with the initial feet poses.
 * @param initialRightFoot = Initial right foot pose.
 * @param initialLeftFoot = Initial left foot pose.
 */
spGait::spGait(Pose initialRightFoot, Pose initialLeftFoot)
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
bool spGait::SetStepParameters( double swingFootDistance, double swingFootElevation )
{
    swingDistance = swingFootDistance;
    swingElevation = swingFootElevation;

    return true;
}

/*
bool StepStageUpdatePoses(spGaitParameters params, std::vector<Transform>& effectorPosesResult)
{

    double dx, dy, dz; //root translation
    switch(params.stepPhase)
    {

    //--Double support. Moving root inside poligon to left foot.--

    case 1: //--Move root over left foot while in double support--
        dx = 0;
        dy = effectorPosesResult[6].trans.y - effectorPosesResult[0].trans.y;
        dz = 0;
        RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);

        break;

        //--Single support on left foot. Move root and right floating foot.--

    case 2: //First half swing of right foot

        //righ foot swing
        effectorPosesResult[5].trans.z += params.swingElevation;
        effectorPosesResult[5].trans.x += params.swingDistance/2;

        //root translation
        dx = params.swingDistance/4;
        dy = 0;
        dz = 0;
        RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);
        break;

    case 3: //Second half swing of right foot

        //righ foot swing
        effectorPosesResult[5].trans.z -= params.swingElevation;
        effectorPosesResult[5].trans.x += params.swingDistance/2;

        //root translation
        dx = params.swingDistance/4;
        dy = 0;
        dz = 0;
        RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);
        break;

        //--Double support. Moving root inside poligon.--

    case 4: //--Move root over right foot while in double support--
        dx = 0;
        dy = effectorPosesResult[5].trans.y - effectorPosesResult[0].trans.y;
        dz = 0;
        RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);
        break;

        //--Single support on right foot. Move root and left floating foot.--

    case 5: //First half swing of left foot

        //left foot swing
        effectorPosesResult[6].trans.z += params.swingElevation;
        effectorPosesResult[6].trans.x += params.swingDistance/2;

        //root translation
        dx = params.swingDistance/4;
        dy = 0;
        dz = 0;
        RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);
        break;

    case 6: //Second half swing of left foot

        //left foot swing
        effectorPosesResult[6].trans.z -= params.swingElevation;
        effectorPosesResult[6].trans.x += params.swingDistance/2;

        //root translation
        dx = params.swingDistance/4;
        dy = 0;
        dz = 0;
        RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);
        break;

    case 7:
        //move root from right foot to double support center


    default:
        //If this is no step phase stop and return error.
        return false;
    };


    return true;
}
*/
