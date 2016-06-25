#include <iostream>
#include <fstream>

#include "GaitSupportPoligon.h"
#include "MiddlewareInterface.h"
#include "SensorIntegration.h"

using namespace std;



int main()
{


    //define robots
    kin::Robot leftLeg;


    GaitSupportPoligon walk01(kin::Pose(0,-0.3,-1),kin::Pose(0,+0.3,-1));
    walk01.SetStepParameters(0.01,0.01);

    //add steps
    walk01.AddStepForward(2);


    //save the trayectories in files
    ofstream saveRF, saveLF;
    saveLF.open("rf.csv");
    saveRF.open("lf.csv");
    saveLF << std::setprecision(6) << std::fixed;
    saveRF << std::setprecision(6) << std::fixed;
    //here we go!
    walk01.SaveSpaceTrajectories(saveRF, saveLF);

    //get and print the trajectories
    tra::SpaceTrajectory righFootTraj, leftFootTraj;
    walk01.GetTrajectories(righFootTraj, leftFootTraj);

    kin::Pose waypoint;
    for(int i=0;i<righFootTraj.Size();i++)
    {
        righFootTraj.GetWaypoint(i,waypoint);
        std::cout << waypoint.GetX() << "," << waypoint.GetY() << "," << waypoint.GetZ() << "," << std::endl;
    }

    for(int i=0;i<leftFootTraj.Size();i++)
    {
        leftFootTraj.GetWaypoint(i,waypoint);
        std::cout << waypoint.GetX() << "," << waypoint.GetY() << "," << waypoint.GetZ() << "," << std::endl;
    }

    cout << "Finished!" << endl;

    return 0;
}

