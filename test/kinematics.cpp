
#include "IKinematics.hpp"

int main()
{

    IKinematics teokin;

    std::vector<double> q1 {0,0,0,0,0,M_PI/2};
    std::vector<double> pose(12);
    std::vector<double> q2(6);


    teokin.LeftLegFwdKin(q1, pose);

    std::cout << "LeftLegFwdKin: ";
    for (long i = 0; i< pose.size(); i++)
    {
        std::cout << pose[i] << ",";
    }
    std::cout << std::endl;


    teokin.LeftLegInvKin(pose,q2);

    std::cout << "LeftLegInvKin: ";
    for (long i = 0; i< q2.size(); i++)
    {
        std::cout << q2[i] << ",";
    }
    std::cout << std::endl;


    teokin.RightLegFwdKin(q1, pose);

    std::cout << "RightLegFwdKin: ";
    for (long i = 0; i< pose.size(); i++)
    {
        std::cout << pose[i] << ",";
    }
    std::cout << std::endl;


    teokin.RightLegInvKin(pose,q2);

    std::cout << "RightLegInvKin: ";
    for (long i = 0; i< q2.size(); i++)
    {
        std::cout << q2[i] << ",";
    }
    std::cout << std::endl;

}
