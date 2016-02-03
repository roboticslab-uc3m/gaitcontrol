#include "tools.h"


using namespace std;
using namespace teo::tra;

//Pose definitions

Pose::Pose()
{

}

Pose::Pose(double x0, double y0, double z0)
{
    x=x0;
    y=y0;
    z=z0;
}

bool Pose::GetPosition(double &new_x, double &new_y, double &new_z)
{
    new_x=x;
    new_y=y;
    new_z=z;
    return true;
}

bool Pose::SetPosition(double new_x, double new_y, double new_z)
{
    x=new_x;
    y=new_y;
    z=new_z;
    return true;
}

bool Pose::ChangePosition(double dx, double dy, double dz)
{
    x+=dx;
    y+=dy;
    z+=dz;
    return true;

}
/*
Pose Pose::TransformTo(Pose anotherPose)
{
    Pose transform;
    return transform;
}
*/

//SpaceTrajectory definitions

bool SpaceTrajectory::AddTimedWaypoint(double dt, Pose waypoint)
{

    waypoints.push_back(waypoint);
    delta_t.push_back(dt);
    /*
    error = waypoints.insert(std::pair<double,Pose>(t,waypoint));
    if (error.second == false)
    {
        std::cout << "Trying to insert existing values" << std::endl;
        return -1;
    }*/
    return 0;
}

bool SpaceTrajectory::AddWaypoint(Pose waypoint)
{

    waypoints.push_back(waypoint);

    return 0;
}

bool SpaceTrajectory::GetCurrentPose(Pose &current)
{
    current = waypoints.back();
    return true;
}
