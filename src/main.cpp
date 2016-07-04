#include <iostream>
#include <fstream>

#include "GaitSupportPoligon.h"
#include "MiddlewareInterface.h"
#include "SensorIntegration.h"

using namespace std;



int main()
{


    //define robots
/*    kin::Robot leftLeg;
    kin::Link base;
*/


    //every gait operation start with instantiation and initialization of a Gait child class
    GaitSupportPoligon walk01(kin::Pose(0,-0.3,-1),kin::Pose(0,+0.3,-1));
    walk01.SetSwingParameters(0.01,0.01);
    walk01.SetSupportParameters(0.2); //revisar estos valores

    //Having the object we can:

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
    tra::SpaceTrajectory righFootTraj, leftFootTraj;
    walk01.GetTrajectories(righFootTraj, leftFootTraj);

    kin::Pose waypoint;
    double rx,ry,rz,ang;

    for(int i=0;i<righFootTraj.Size();i++)
    {
        righFootTraj.GetWaypoint(i,waypoint);
        waypoint.GetRotation(rx,ry,rz,ang);
        std::cout << waypoint.GetX() << "," << waypoint.GetY() << "," << waypoint.GetZ() << ","
                  << rx << "," << ry << "," << rz << "," << ang << std::endl;

        leftFootTraj.GetWaypoint(i,waypoint);
        waypoint.GetRotation(rx,ry,rz,ang);
        std::cout << waypoint.GetX() << "," << waypoint.GetY() << "," << waypoint.GetZ() << ","
                  << rx << "," << ry << "," << rz << "," << ang << std::endl;
    }

    cout << "Finished!" << endl;

    return 0;
}

