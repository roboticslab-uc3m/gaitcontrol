#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include <map>
#include <iostream>
#include <fstream>      // std::ofstream
#include <iomanip>      // std::setprecision

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
    bool GetPosition(double &pos_x, double &pose_y, double &pose_z);
    bool GetRotation(double &axis_i, double &axis_j, double &axis_k, double &pose_angle);
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
    bool GetLastWaypoint(Pose & waypoint);
    bool SaveToFile(std::ofstream &csvFile);
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

