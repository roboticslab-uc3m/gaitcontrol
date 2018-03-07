
#include <time.h>
#include <chrono>

#include "IKinematics.hpp"
#include "MiddlewareInterface.h"
#include "GaitSupportPoligon.h"
#include "tools.h"

#define ROBOT "teo"

using namespace roboticslab;
using namespace std;

double max_accel=(60.0/60.0);//rad/s^2
std::vector<double> DqRightLeg(6,0), DqLeftLeg(6,0);
std::vector<double> qRightLeg(6,0), qLeftLeg(6,0);
double accelSmoother(valarray<double> &pos, const valarray<double> &dvels, const double dts);
MWI::Limb teoRightLeg(ROBOT,"rightLeg"), teoLeftLeg(ROBOT,"leftLeg");
double peak_acc = 0;

int main()
{
    teoRightLeg.SetControlMode(1);
    teoLeftLeg.SetControlMode(1);

    IKinematics teokin;

    kin::Pose initialRightFoot(0,-0.1285,-0.845);

    kin::Pose initialLeftFoot(0,+0.1285,-0.845);



    kin::Pose kinposeRightFoot(initialRightFoot), kinposeLeftFoot(initialLeftFoot);
    std::vector<double> poseRightLeg(12,0), poseLeftLeg(12,0);
    std::vector<double> angsRightLeg(6,0), angsLeftLeg(6,0);
    //std::vector<double> qRightLeg(6,0), qLeftLeg(6,0);




        //set left leg pose (initial). Mind the origin (at teo hip)
        initialLeftFoot.GetPoseMatrix(poseLeftLeg);
        //get left ik
        teokin.LeftLegInvKin(poseLeftLeg, angsLeftLeg);
        //to degrees
        std::transform(angsLeftLeg.begin(), angsLeftLeg.end(), angsLeftLeg.begin(),
                                     std::bind1st(std::multiplies<double>(), 180/M_PI));
        //Send angles to ipos
        teoLeftLeg.SetJointPositions(angsLeftLeg);



        //set right leg pose (initial). Mind the origin (at teo hip)
        initialRightFoot.GetPoseMatrix(poseRightLeg);
        //get right ik
        teokin.RightLegInvKin(poseRightLeg, angsRightLeg);
        //to degrees
        std::transform(angsRightLeg.begin(), angsRightLeg.end(), angsRightLeg.begin(),
                                     std::bind1st(std::multiplies<double>(), 180/M_PI));
        //Send angles to ipos
        teoRightLeg.SetJointPositions(angsRightLeg);

        std::cout << "leftLeg" << angsLeftLeg << std::endl;
        std::cout << "rightLeg" << angsRightLeg << std::endl;

        yarp::os::Time::delay(3);

        //Now for a specific pose
        kinposeLeftFoot.ChangePosition(0,0,0.01);
        //set left leg pose (initial). Mind the origin (at teo hip)
        kinposeLeftFoot.GetPoseMatrix(poseLeftLeg);
        //get left ik
        teokin.LeftLegInvKin(poseLeftLeg, angsLeftLeg);
        //to degrees
        std::transform(angsLeftLeg.begin(), angsLeftLeg.end(), angsLeftLeg.begin(),
                                     std::bind1st(std::multiplies<double>(), 180/M_PI));
        //Send angles to ipos
        //angsLeftLeg=std::vector<double>{0, -1.78111e-13, -9.74135, 20.4678, -10.7265, -1.78111e-13};
        teoLeftLeg.SetJointPositions(angsLeftLeg);



        kinposeRightFoot.ChangePosition(0,0,0.01);
        //set right leg pose (initial). Mind the origin (at teo hip)
        kinposeRightFoot.GetPoseMatrix(poseRightLeg);
        //get right ik
        teokin.RightLegInvKin(poseRightLeg, angsRightLeg);
        //to degrees
        std::transform(angsRightLeg.begin(), angsRightLeg.end(), angsRightLeg.begin(),
                                     std::bind1st(std::multiplies<double>(), 180/M_PI));
        //Send angles to ipos
        teoRightLeg.SetJointPositions(angsRightLeg);

        std::cout << "leftLeg" << angsLeftLeg << std::endl;
        std::cout << "rightLeg" << angsRightLeg << std::endl;


    return 0;



}

