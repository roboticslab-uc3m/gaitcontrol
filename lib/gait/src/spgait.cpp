#include <iostream>
#include <vector>

using namespace std;

#include "spgait.h"

//kdl
//#include <frames.hpp>

/**
 * @brief This function will add "stepNumber" steps in forward direction, using
 * support poligon stability criteria. Results are stored in the trajectory source variable.
 * @param stepNumber = Number of steps to add.
 * @return true at success.
 */
bool spGait::AddStepForward(int stepNumber)
{

    SpaceTrajectory next;
    //strategy
    //move root over right foot
    //balance over right foot
    //left foot forward
    //move root over center
    //move root over left foot
    //balance over left foot
    //right foot forward
    //move root over center


    return true;
}

spGait::spGait(Pose initialRightFoot, Pose initialLeftFoot)
{
    rfTraj.waypoints[0] = initialRightFoot;
    lfTraj.waypoints[0] = initialLeftFoot;

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
