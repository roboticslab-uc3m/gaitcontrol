#include <iostream>
#include <fstream>

#include "GaitSupportPoligon.h"
#include "MiddlewareInterface.h"
#include "SensorIntegration.h"

using namespace std;

using namespace teo;


int main()
{






    //rotation vector is computed as intrinsic vector rotated.
    kin::Quaternion q1,q2;
    q1.FromAxisAngle(1,0,0,M_PI);
    std::cout << "q1: " << q1.Qw() << "," << q1.Qi() << "," << q1.Qj() << "," << q1.Qk() << std::endl;

    q2.FromAxisAngle(0,1,0,M_PI);
    std::cout << "q2: " << q2.Qw() << "," << q2.Qi() << "," << q2.Qj() << "," << q2.Qk() << std::endl;


    kin::Quaternion newP;

    newP.FromProduct(q1,q2);
    std::cout << "q1*q2: " << newP.Qw() << "," << newP.Qi() << "," << newP.Qj() << "," << newP.Qk() << std::endl;


    double ux,uy,uz,angle;
    newP.ToAxisAngle(ux,uy,uz,angle);
    std::cout << "ux,uy,uz,angle: " << ux << "," << uy << "," << uz << "," << angle << std::endl;


    std::cout << std::endl;


    //Rotation computed as p'=q0*p*q0^-1
    //double px=0,py=0,pz=1;
    kin::Quaternion oldP(0,1,0,0);//,px,py,pz);
    q1.FromAxisAngle(0,1,0,M_PI/2);
    newP.FromProduct(q1,oldP);
    newP.FromProduct(newP,q1.Conjugate());

    std::cout << "oldP: " << oldP.Qw() << "," << oldP.Qi() << "," << oldP.Qj() << "," << oldP.Qk() << std::endl;
    std::cout << "q1: " << q1.Qw() << "," << q1.Qi() << "," << q1.Qj() << "," << q1.Qk() << std::endl;
    std::cout << "q1.Conjugate(): " << q1.Conjugate().Qw() << "," << q1.Conjugate().Qi() << "," << q1.Conjugate().Qj() << "," << q1.Conjugate().Qk() << std::endl;
    std::cout << "newP1: " << newP.Qw() << "," << newP.Qi() << "," << newP.Qj() << "," << newP.Qk() << std::endl;
    std::cout << std::endl;

    std::cout << "newP: " << newP.Qw() << "," << newP.Qi() << "," << newP.Qj() << "," << newP.Qk() << std::endl;


    //remember that newP is not a Quaternion, but the new rotated point.

    return 0;
}
