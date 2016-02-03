#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include <map>
#include <iostream>

namespace teo
{

namespace tra
{

class Pose
{
public:
    Pose();
    Pose(double x0, double y0, double z0);
    bool SetPosition(double new_x, double new_y, double new_z);
    bool ChangePosition(double dx, double dy, double dz);
    bool GetPosition(double &new_x, double &new_y, double &new_z);
private:
    double x,y,z;
    double i,j,k;
    double angle;

};

class SpaceTrajectory
{
public:
    bool AddTimedWaypoint(double t, Pose waypoint);
    bool AddWaypoint(Pose waypoint);
    bool GetCurrentPose(Pose & current);
private:
    std::vector<Pose> waypoints;
    std::vector<double> delta_t;
    /*
    std::map<double,Pose> waypoints;
    std::pair<double,Pose> wp; //Temporary storage. Use as local only. It can change.
    std::map<double,Pose>::iterator it;
    std::pair<std::map<double,Pose>::iterator,bool> error;*/

};

struct JointTrajectory
{
    std::vector<double> x;
};

}//end namespace tra

}//end namespace teo

#endif // TOOLS_H

