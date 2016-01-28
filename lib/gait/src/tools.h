#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include <map>
#include <iostream>

class Pose
{
public:
    Pose();
    Pose(double x0, double y0, double z0);

private:
    double x,y,z;
    double i,j,k;
    double angle;

};

class SpaceTrajectory
{
public:
    bool AddTimedWaypoint(double t, Pose waypoint);



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


#endif // TOOLS_H

