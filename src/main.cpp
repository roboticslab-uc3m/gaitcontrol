#include <iostream>
#include <fstream>

#include "GaitSP.h"

using namespace std;



int main()
{

    ofstream saveRF, saveLF;
    saveLF.open("rf.csv");
    saveRF.open("lf.csv");
    saveLF << std::setprecision(6) << std::fixed;
    saveRF << std::setprecision(6) << std::fixed;


    GaitSP walkPat(Pose(0,-0.3,-1),Pose(0,+0.3,-1));
    walkPat.SetStepParameters(0.01,0.01);

    walkPat.AddStepForward(1);
    walkPat.SaveSpaceTrajectories(saveRF, saveLF);

    cout << "Hello Worls!" << endl;

    return 0;
}

