#include <iostream>
#include <fstream>

#include "GaitSP.h"

using namespace std;



int main()
{


    GaitSupportPoligon walkPattern(Pose(0,-0.3,-1),Pose(0,+0.3,-1));
    walkPattern.SetStepParameters(0.01,0.01);

    //add steps
    walkPattern.AddStepForward(1);


    //save the trayectories in files
    ofstream saveRF, saveLF;
    saveLF.open("rf.csv");
    saveRF.open("lf.csv");
    saveLF << std::setprecision(6) << std::fixed;
    saveRF << std::setprecision(6) << std::fixed;
    //here we go!
    walkPattern.SaveSpaceTrajectories(saveRF, saveLF);

    cout << "Finished!" << endl;

    return 0;
}

