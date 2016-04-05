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

    /**
     * @brief Pose: Create a Pose object with rotation 0 and translation xyz.
     * @param x0
     * @param y0
     * @param z0
     */
    Pose(double x0, double y0, double z0);


    /**
     * @brief SetPosition: Sets a new position. Overwrites old position.
     * @param new_x
     * @param new_y
     * @param new_z
     * @return
     */
    bool SetPosition(double new_x, double new_y, double new_z);

    /**
     * @brief ChangePosition: Add an offset to actual position.
     * @param dx
     * @param dy
     * @param dz
     * @return
     */
    bool ChangePosition(double dx, double dy, double dz);

    /**
     * @brief GetPosition: Copies actual pose coordinates on pose_ variables.
     * @param pose_x
     * @param pose_y
     * @param pose_z
     * @return
     */
    bool GetPosition(double &pose_x, double &pose_y, double &pose_z);

    /**
     * @brief GetX: Return the pose x coordinate.
     * @return
     */
    double GetX();
    /**
     * @brief GetY: Return the pose y coordinate.
     * @return
     */
    double GetY();
    /**
     * @brief GetZ: Return the pose z coordinate.
     * @return
     */
    double GetZ();

    /**
     * @brief GetRotation: Copies the axis-angle rotation on axis_ and pose_angle variables,
     * @param axis_i
     * @param axis_j
     * @param axis_k
     * @param pose_angle
     * @return
     */
    bool GetRotation(double &axis_i, double &axis_j, double &axis_k, double &pose_angle);

    /**
     * @brief SetRotation: Set a new pose rotation given an axis angle. Overwrites old rotation.
     * @param axis_i
     * @param axis_j
     * @param axis_k
     * @param pose_angle
     * @return
     */
    bool SetRotation(double &axis_i, double &axis_j, double &axis_k, double &pose_angle);

private:
    double x,y,z;
    double i,j,k; //axis
    double angle; //angle

};

class SpaceTrajectory
{
public:
    bool AddTimedWaypoint(double t, Pose waypoint);
    bool AddWaypoint(Pose waypoint);
    bool GetLastWaypoint(Pose & waypoint);
    bool SaveToFile(std::ofstream &csvFile);
    bool GetWaypoint(int index, Pose &getWaypoint);
    int Size();
private:
    std::vector<Pose> waypoints;
    std::vector<double> delta_t;
    /*
    std::map<double,Pose> waypoints;
    std::pair<double,Pose> wp; //Temporary storage. Use as local only. It can change.
    std::map<double,Pose>::iterator it;
    std::pair<std::map<double,Pose>::iterator,bool> error;*/

};


}//end namespace tra


}//end namespace teo

#endif // TOOLS_H

