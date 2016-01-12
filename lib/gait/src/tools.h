#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include <map>

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

struct SpaceTrajectory
{
    std::map<double,Pose> waypoints;

};

struct JointTrajectory
{
    std::vector<double> x;
};


#endif // TOOLS_H

