#ifndef GAIT_H
#define GAIT_H



class Gait
{
public:
    Gait();
    virtual bool AddStepForward(int stepNumber)=0;
};

#endif // GAIT_H
