#include <iostream>

#include "spgait.h"

using namespace std;



int main()
{

    spGait walkPat(Pose(0,-0.3,-1),Pose(0,+0.3,-1));
    walkPat.SetStepParameters(0.01,0.01);

    walkPat.AddStepForward(1);

    cout << "Hello Words!" << endl;

    return 0;
}

