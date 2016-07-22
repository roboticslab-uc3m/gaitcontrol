#include <iostream>
#include <fstream>

#include "GaitSupportPoligon.h"
#include "MiddlewareInterface.h"
#include "SensorIntegration.h"

using namespace std;

using namespace teo;



int main()
{


    //define robots
/*    kin::Robot leftLeg;
    kin::Link base;
*/


    //every gait operation start with instantiation and initialization of a Gait child class
    GaitSupportPoligon walk01(kin::Pose(0,-0.3,-1),kin::Pose(0,+0.3,-1));
    walk01.SetSwingParameters(0.01,0.01); //(swing distance, swing height). revisar valores
    walk01.SetSupportParameters(0.25); //(hip sideshift). revisar estos valores

    //The Gait objects can do the following tasks.

    //Add steps forward
    //this will generate trajectory objects for both feet according to gait strategy
    walk01.AddStepForward(2);


    //Uncomment to save the trayectories in files (in working directory)
/*    ofstream saveRF, saveLF;
    saveLF.open("rf.csv");
    saveRF.open("lf.csv");
    saveLF << std::setprecision(6) << std::fixed;
    saveRF << std::setprecision(6) << std::fixed;
    //here we go!
    walk01.SaveSpaceTrajectories(saveRF, saveLF);
*/
    //get and print the trajectories in cout
    tra::SpaceTrajectory rightFootTraj, leftFootTraj;
    walk01.GetTrajectories(rightFootTraj, leftFootTraj);

    kin::Pose waypoint;
    double px,py,pz;
    double rx,ry,rz,ang;
    double wpt;
/*
    cout << "--------------waypoints test-------------------------------!" << endl;


    for(int i=0;i<rightFootTraj.Size();i++)
    {
        rightFootTraj.GetWaypoint(i,waypoint,wpt);
        waypoint.GetRotation(rx,ry,rz,ang);
        std::cout << i << ":" << waypoint.GetX() << "," << waypoint.GetY() << "," << waypoint.GetZ() << ","
                  << rx << "," << ry << "," << rz << "," << ang << ","<<  wpt << std::endl;

        leftFootTraj.GetWaypoint(i,waypoint,wpt);
        waypoint.GetRotation(rx,ry,rz,ang);
        std::cout << i << ":" << waypoint.GetX() << "," << waypoint.GetY() << "," << waypoint.GetZ() << ","
                  << rx << "," << ry << "," << rz << "," << ang << ","<<  wpt << std::endl;
    }

*/
    cout << "--------------GetSample test-------------------------------!" << endl;
    kin::Pose sample;

    for (double t=0.1;t<10;t+=0.1)
    {
        rightFootTraj.GetSample(t,sample);
        sample.GetRotation(rx,ry,rz,ang);
        std::cout << t << ": " << sample.GetX() << "," << sample.GetY() << "," << sample.GetZ() << ","
                  << rx << "," << ry << "," << rz << "," << ang << std::endl;
    }
    cout << "Finished!" << endl;
/*

    cout << "-----------------GetSampleVelocity test----------------------------!" << endl;
    kin::Pose sampleVel;

    for (double t=0.1;t<10;t+=0.1)
    {
        rightFootTraj.GetSampleVelocity(t,sampleVel);
        sampleVel.GetPosition(px,py,pz);
        sampleVel.GetRotation(rx,ry,rz,ang);
        std::cout << t << ": " << px << "," << py << "," << pz << ","
                  << rx << "," << ry << "," << rz << "," << ang << std::endl;
    }
    cout << "Finished!" << endl;
*/

    rightFootTraj.ShowData();
    return 0;
}

