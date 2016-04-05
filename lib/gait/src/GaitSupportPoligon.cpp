#include <iostream>
#include <vector>

using namespace std;

#include "GaitSP.h"

//kdl
//#include <frames.hpp>




bool GaitSupportPoligon::SaveSpaceTrajectories(ofstream &fileLeftFoot, ofstream &fileRightFoot)
{

    trajLeftFoot.SaveToFile(fileLeftFoot);
    trajRightFoot.SaveToFile(fileRightFoot);
    return true;
}



GaitSupportPoligon::GaitSupportPoligon(Pose initialRightFoot, Pose initialLeftFoot)
{
    trajRightFoot.AddTimedWaypoint(-1, initialRightFoot);
    trajLeftFoot.AddTimedWaypoint(-1, initialLeftFoot);

    swingDistance = 0.0;
    swingElevation = 0.0;
    startOnRightFootSupport = true;

}


bool GaitSupportPoligon::SetStepParameters( double swingFootDistance, double swingFootElevation )
{
    swingDistance = swingFootDistance;
    swingElevation = swingFootElevation;

    return true;
}

bool GaitSupportPoligon::GetTrajectories(SpaceTrajectory& getRightFoot, SpaceTrajectory& getLeftFoot)
{

    getRightFoot = trajRightFoot;
    getLeftFoot = trajLeftFoot;

}


bool GaitSupportPoligon::AddStepForward(int stepNumber)
{

    for (int i=0; i<stepNumber; i++)
    {
        if (startOnRightFootSupport)
        {
            HalfStepForwardRS();
            HalfStepForwardLS();
        }
        else
        {
            HalfStepForwardLS();
            HalfStepForwardRS();

        }
    }
    //double x,y,z; actualRightFoot.GetPosition(x,y,z);
   /* double dx,dy,dz;
    Pose actualRightFoot, actualLeftFoot;
    Pose desiredRightFoot,desiredLeftFoot;

    trajRightFoot.GetLastWaypoint(actualRightFoot);
    trajLeftFoot.GetLastWaypoint(actualLeftFoot);


    //strategy:
    //-1-move root over right foot (right foot under root (0,0,z), z is actual foot elevation)
    //calculate right foot

    //origin (x,y,z) destination (0,0,z)
    dx=0-actualRightFoot.GetX();
    dy=0-actualRightFoot.GetY();
    dz=0;
    desiredRightFoot=actualRightFoot;
    desiredRightFoot.ChangePosition(dx,dy,dz);

    //left foot moves parallel to right foot
    desiredLeftFoot=actualLeftFoot;
    desiredLeftFoot.ChangePosition(dx,dy,dz);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //-2-balance over right foot
    //TODO


    //-3-left foot forward
    //forward up
    desiredLeftFoot.ChangePosition(swingDistance/2, 0, swingElevation);
    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //forward down
    desiredLeftFoot.ChangePosition(swingDistance/2, 0, -swingElevation);
    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //-4-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-swingDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-swingDistance/2,0,0);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //-5-move root over left foot (left foot under root (0,0,z), z is actual foot elevation)
    trajRightFoot.GetLastWaypoint(actualRightFoot);
    trajLeftFoot.GetLastWaypoint(actualLeftFoot);

    //origin (x,y,z) destination (0,0,z)
    dx=0-actualLeftFoot.GetX();
    dy=0-actualLeftFoot.GetY();
    dz=0;
    desiredLeftFoot=actualLeftFoot;
    desiredLeftFoot.ChangePosition(dx,dy,dz);

    //right foot moves parallel to left foot
    desiredRightFoot=actualRightFoot;
    desiredRightFoot.ChangePosition(dx,dy,dz);

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
    trajLeftFoot.AddWaypoint(desiredLeftFoot);*/

    //one step finished


    return true;
}

bool GaitSupportPoligon::HalfStepForwardRS()
{

    //double x,y,z; actualRightFoot.GetPosition(x,y,z);
    double dx,dy,dz;
    Pose actualRightFoot, actualLeftFoot;
    Pose desiredRightFoot,desiredLeftFoot;

    trajRightFoot.GetLastWaypoint(actualRightFoot);
    trajLeftFoot.GetLastWaypoint(actualLeftFoot);


    //strategy:
    //-1-move root over right foot (right foot under root (0,0,z), z is actual foot elevation)
    //calculate right foot

    //origin (x,y,z) destination (0,0,z)
    dx=0-actualRightFoot.GetX();
    dy=0-actualRightFoot.GetY();
    dz=0;
    desiredRightFoot=actualRightFoot;
    desiredRightFoot.ChangePosition(dx,dy,dz);

    //left foot moves parallel to right foot
    desiredLeftFoot=actualLeftFoot;
    desiredLeftFoot.ChangePosition(dx,dy,dz);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //-2-balance over right foot
    //TODO


    //-3-left foot forward
    //forward up
    desiredLeftFoot.ChangePosition(swingDistance/2, 0, swingElevation);
    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //forward down
    desiredLeftFoot.ChangePosition(swingDistance/2, 0, -swingElevation);
    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //-4-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-swingDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-swingDistance/2,0,0);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //half step finished


    return true;
}

bool GaitSupportPoligon::HalfStepForwardLS()
{

    //double x,y,z; actualRightFoot.GetPosition(x,y,z);
    double dx,dy,dz;
    Pose actualRightFoot, actualLeftFoot;
    Pose desiredRightFoot,desiredLeftFoot;

    trajRightFoot.GetLastWaypoint(actualRightFoot);
    trajLeftFoot.GetLastWaypoint(actualLeftFoot);



    //-5-move root over left foot (left foot under root (0,0,z), z is actual foot elevation)
    trajRightFoot.GetLastWaypoint(actualRightFoot);
    trajLeftFoot.GetLastWaypoint(actualLeftFoot);

    //origin (x,y,z) destination (0,0,z)
    dx=0-actualLeftFoot.GetX();
    dy=0-actualLeftFoot.GetY();
    dz=0;
    desiredLeftFoot=actualLeftFoot;
    desiredLeftFoot.ChangePosition(dx,dy,dz);

    //right foot moves parallel to left foot
    desiredRightFoot=actualRightFoot;
    desiredRightFoot.ChangePosition(dx,dy,dz);

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
