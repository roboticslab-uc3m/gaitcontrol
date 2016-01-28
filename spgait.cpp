#include <string>
#include <sstream>
#include <iostream>

template <class T>
std::string to_string(T t, std::ios_base & (*f)(std::ios_base&))
{
  std::ostringstream oss;
  oss << f << t;
  return oss.str();
}

#include <openrave/plugin.h>
#include <openrave/planningutils.h>
#include <openrave/plannerparameters.h>


#include <boost/bind.hpp>
using namespace OpenRAVE;


struct spGaitParameters
{
    //parameters in meters, seconds

    //swing foot parameters
    double swingElevation;
    double swingDistance;

    //step parameters.
    double t;
    int stepPhase;  //from 1 to .. step phases
    uint stepTotalPhases; //total number of phases in a step
    double zOffset; //z variation of the root point while walking
};


class spGait : public ModuleBase
{
public:
    spGait(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&spGait::MyCommand,this,_1,_2),
                        "This is an example command");
        RegisterCommand("SetRobot",boost::bind(&spGait::SetRobot,this,_1,_2),
                        "Choose the robot that will gait");
        RegisterCommand("ChangeRootZ",boost::bind(&spGait::ChangeRootZ,this,_1,_2),
                        "Change root height. Arg1 = delta z in meters");
        RegisterCommand("StepsForward",boost::bind(&spGait::StepsForward,this,_1,_2),
                        "The robot will walk -Arg1- steps forward with -Arg2(m)- foot elevation and -Arg3(m)- step distance");
        RegisterCommand("ViewWorkspaceTrajectories",boost::bind(&spGait::ViewWorkspaceTrajectories,this,_1,_2),
                        "The Wiewer shows all stored trajectories");
        RegisterCommand("PrintTrajectories",boost::bind(&spGait::PrintTrajectories,this,_1,_2),
                        "Print the trajectory. Arg1 = Frame of reference (root or floor). Arg2 = manipulator");
        RegisterCommand("SaveTrajectories",boost::bind(&spGait::SaveTrajectories,this,_1,_2),
                        "Save current Trajectories in 3 files");
        env=penv;

        physics = env->GetPhysicsEngine();

        //initialize trajectory vectors
        wsTrajectory.clear();
        wsRootFrameTrajectory.clear();
        csTrajectory.clear();
        //csRootFrameTrajectory.clear();
        //add 7 trajectories to each vector
        for (uint i=0; i<7; i++)
        {
            wsTrajectory.push_back( RaveCreateTrajectory(env, "") );
            wsRootFrameTrajectory.push_back( RaveCreateTrajectory(env, "") );
            csTrajectory.push_back( RaveCreateTrajectory(env, "") );
            //csRootFrameTrajectory.push_back( RaveCreateTrajectory(env, "") );

        }

        mergedTrajectory = RaveCreateTrajectory(env);

        //wsTracker = RaveCreatePlanner(env,"workspacetrajectorytracker");


        //interpolation for workspace trajectories
        workspaceInterpolation = "quadratic";


        //initialize robot
        SetRobot(std::cout, ss);
        rootLink = "cintura";


        //default step parameters in meters
        stepParams.swingElevation = 0.04;
        stepParams.swingDistance = 0.2;
        //time for a complete step
        stepParams.t = 10;
        //z decrement of root point
        stepParams.zOffset = -0.05;

        // Change the initial foot swing setting stepPhase accordingly
        // 0 for left foot initial, (see StepStageUpdatePoses function)
        stepParams.stepPhase = 0;

        std::cout << "module spgait loaded";



    }
    virtual ~spGait() {}
    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        sout << "output";
        return true;
    }

    bool SetRobot(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        robot = env->GetRobot("teo");

        //std::string trajectoryGroup;
        //ConfigurationSpecification::Group trajectoryGroup;

        if (!robot)
        {
            sout << "sp gait module error. No robot called : " << input << std::endl;
        }
        else
        {
            sout << "Loaded sp gait module for robot : " << robot->GetName() << std::endl;
        }


        //store and sort up to six manipulators with names like  "m(n)"  ("m1", "m2" ...)
        manips.push_back( robot->GetManipulator("m1") );
        manips.push_back( robot->GetManipulator("m2") );
        manips.push_back( robot->GetManipulator("m3") );
        manips.push_back( robot->GetManipulator("m4") );
        manips.push_back( robot->GetManipulator("m5") );
        manips.push_back( robot->GetManipulator("m6") );


        //and initialize trajectories
        deltaTime = 0.05;
        for (uint i=0; i<wsTrajectory.size(); i++)
        {
            ConfigurationSpecification spec6D = IkParameterization::GetConfigurationSpecification(IKP_Transform6D, workspaceInterpolation);
            spec6D.AddDeltaTimeGroup();
            //spec6D.AddDeltaTimeGroup();
            wsTrajectory[i]->Init(spec6D);
            wsRootFrameTrajectory[i]->Init(spec6D);

        }


/* 
        //add every manipulator trajectory group
        workspaceTrajConfig.AddGroup("affine_transform root", 7, workspaceInterpolation);
        workspaceTrajConfig.AddGroup("affine_transform head", 7, workspaceInterpolation);
        workspaceTrajConfig.AddGroup("affine_transform rarm", 7, workspaceInterpolation);
        workspaceTrajConfig.AddGroup("affine_transform larm", 7, workspaceInterpolation);
        workspaceTrajConfig.AddGroup("affine_transform trunk", 7, workspaceInterpolation);
        workspaceTrajConfig.AddGroup("affine_transform rfoot", 7, workspaceInterpolation);
        workspaceTrajConfig.AddGroup("affine_transform lfoot", 7, workspaceInterpolation);

       //add every manipulator trajectory group
        for (uint i=0; i<robot->GetManipulators().size(); i++)
        {
            trajectoryGroup = "affine_transform "+robot->GetManipulators()[i]->GetName();
            //add every manipulator trajectory group as a 7 values vector (cuat,x,y,z)
            workspaceTrajConfig.AddGroup(trajectoryGroup, 7, workspaceInterpolation);
        }*/
        //time for each waypoint


        return true;
    }


    bool ChangeRootZ(std::ostream& sout, std::istream& sinput)
    {
        double dz = 0;
        sinput >> dz;

        sout << "Changing z for " << dz << " meters";

        std::vector<dReal> initValues(28,0);
        std::vector<Transform> effectorPoses;




        //set 0 as the initial trajectory point
        //robot->SetDOFValues(initValues);
        CurrentWorkspacePose(effectorPoses);
        AddWorkspaceWaypoint(effectorPoses);

        //change dz root point height
        RootTranslationWithAttachments(0, 0, dz, effectorPoses);
        AddWorkspaceWaypoint(effectorPoses);

        WorkspaceTrajectoryToJoints(wsTrajectory, csTrajectory);
        //robot->SetDOFValues();


        std::list<TrajectoryBaseConstPtr> listToMerge;
        for (uint i=5; i<csTrajectory.size(); i++)
        {
            if (csTrajectory[i]->GetNumWaypoints() > 0)
            {
                listToMerge.push_back(csTrajectory[i]);
            }
        }

        if (listToMerge.size() > 0)
        {
            std::vector<dReal> maxvelocities(12,0.05);
            std::vector<dReal> maxaccelerations(12,0.05);
            mergedTrajectory = planningutils::MergeTrajectories(listToMerge);

            planningutils::RetimeAffineTrajectory(mergedTrajectory,maxvelocities,maxaccelerations);

            robot->GetController()->SetPath(mergedTrajectory);
            //robot->GetController()->SetPath(csTraj[6]);
        }

        return true;
    }


    bool StepsForward(std::ostream& sout, std::istream& sinput)
    {
        int stepsLeft = 0;
        sinput >> stepsLeft;
        sinput >> stepParams.swingElevation;
        sinput >> stepParams.swingDistance;

        std::vector<Transform> effectorPoses;
        //suppose trajectory already has intial waypoint

        //Before steps
        //Get the actual pose
        CurrentWorkspacePose( effectorPoses );
        //Store as initial waypoint for the trajectory
        AddWorkspaceWaypoint( effectorPoses );

        //Every step will take 6 phases
        stepParams.stepTotalPhases = 8;

        //--phases of first step--

        //phase 1: Get the pose over current phase support poligon
        stepParams.stepPhase++;
        StepStageUpdatePoses(stepParams, effectorPoses);
        AddWorkspaceWaypoint( effectorPoses );

        //phase 2: 1st half swing
        stepParams.stepPhase++;
        //first step need only to elevate the floating foot before swing
        double swapZero = 0;
        std::swap(swapZero, stepParams.swingDistance);
        StepStageUpdatePoses(stepParams, effectorPoses);
        std::swap(stepParams.swingDistance, swapZero);
        AddWorkspaceWaypoint( effectorPoses );

        //phase 3: 2nd half swing
        stepParams.stepPhase++;
        //and makes one normal half swing (like starting at the half of a swing).
        StepStageUpdatePoses(stepParams, effectorPoses);
        AddWorkspaceWaypoint( effectorPoses );

        //phase 4: Change foot support trough (y) displacement (to center)
        stepParams.stepPhase++;
        StepStageUpdatePoses(stepParams, effectorPoses);
        AddWorkspaceWaypoint( effectorPoses );

        //phase 5: Change foot support trough (y) displacement (to foot)
        stepParams.stepPhase++;
        StepStageUpdatePoses(stepParams, effectorPoses);
        AddWorkspaceWaypoint( effectorPoses );

        //phase 6: 1st half swing
        stepParams.stepPhase++;
        StepStageUpdatePoses(stepParams, effectorPoses);
        AddWorkspaceWaypoint( effectorPoses );

        //phase 7: 2nd half swing
        stepParams.stepPhase++;
        StepStageUpdatePoses(stepParams, effectorPoses);
        AddWorkspaceWaypoint( effectorPoses );

        //phase 8: Change foot support trough (y) displacement (to center)
        stepParams.stepPhase++;
        StepStageUpdatePoses(stepParams, effectorPoses);
        AddWorkspaceWaypoint( effectorPoses );


        //restart stepPhase to initial value
        stepParams.stepPhase -= stepParams.stepTotalPhases;
        //decrease the steps left counter
        stepsLeft--;

        //repeat loop until steps left reach 0
        while(stepsLeft>0)
        {
            //make all 6 phases in a loop
            for (uint i=0; i<stepParams.stepTotalPhases ; i++)
            {
                //phase stepPhase
                stepParams.stepPhase++;
                StepStageUpdatePoses(stepParams, effectorPoses);
                AddWorkspaceWaypoint( effectorPoses );
            }

            //restart stepPhase to initial value
            stepParams.stepPhase -= stepParams.stepTotalPhases;
            //decrease the steps left counter
            stepsLeft--;
        }

        WorkspaceTrajectoryToJoints(wsTrajectory, csTrajectory);

        //FormatWorkspaceTrajectories(wsRootFrameTrajectory);

        //ExportWorkspaceTrajectories(wsRootFrameTrajectory);


        return true;
    }

    bool ViewWorkspaceTrajectories(std::ostream& sout, std::istream& sinput)
    {

       /* std::vector<dReal> initValues(28,0);
        initValues[18]=-0.4;
        initValues[19]=0.8;
        initValues[20]=-0.4;
        initValues[24]=-0.4;
        initValues[25]=0.8;
        initValues[26]=-0.4;
        sout << "output";
        std::vector<dReal> initialPose;
        robot->SetDOFValues(initValues);

        Transform goInitial;
        goInitial.trans = OpenRAVE::RaveVector<dReal>(0,0,1);
        robot->SetTransform(goInitial);*/


        std::list<TrajectoryBaseConstPtr> listToMerge;
        for (uint i=5; i<csTrajectory.size(); i++)
        {
            if (csTrajectory[i]->GetNumWaypoints() > 0)
            {
                listToMerge.push_back(csTrajectory[i]);
            }
        }

        if (listToMerge.size() > 0)
        {
            mergedTrajectory = planningutils::MergeTrajectories(listToMerge);

            //TODO: set velocicty and acceleration by parameters.
            std::vector<dReal> maxvelocities(12,0.05);
            std::vector<dReal> maxaccelerations(12,0.05);
            planningutils::RetimeAffineTrajectory(mergedTrajectory,maxvelocities,maxaccelerations,false,"ParabolicTrajectoryRetimer");

            robot->GetController()->SetPath(mergedTrajectory);
            //robot->GetController()->SetPath(csTraj[6]);
        }

        //physics->SetGravity(Vector(0,0,-4));

        return true;
    }

    bool PrintTrajectories(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        uint manipNumber;

        if (input == "root")
        {
            sinput >> manipNumber;
            if(manipNumber<=7)
            {

                wsRootFrameTrajectory[manipNumber]->serialize(std::cout);
                csTrajectory[manipNumber]->serialize(std::cout);

                //csRootFrameTrajectory[manipNumber]->serialize(std::cout);
            }
            else
            {
                sout << "There is no manipulator with that number " << manipNumber;

            }
        }

        if (input == "floor")
        {
            sinput >> manipNumber;
            if(manipNumber<=7)
            {

                wsTrajectory[manipNumber]->serialize(std::cout);

            }
            else
            {
                sout << "There is no manipulator with that number " << manipNumber;

            }
        }

        return true;
    }

    bool SaveTrajectories(std::ostream& sout, std::istream& sinput)
    {

        ExportJointTrajectories(csTrajectory);
        return true;
    }

private:
    //--robot--
    EnvironmentBasePtr env;
    PhysicsEngineBasePtr physics;
    RobotBasePtr robot;
    spGaitParameters stepParams;
    std::string rootLink;

    //--Manipulators--
    std::vector<RobotBase::ManipulatorPtr> manips;
    //RobotBase::ManipulatorPtr manip; //Temporal data. For operations over manips.

    //--Trajectory--
    ConfigurationSpecification workspaceTrajConfig;
    //ConfigurationSpecificationPtr wsTrajConfig;
    //TrajectoryBasePtr workspaceTrajectory, wsRootFrameTrajectory;
    std::string workspaceInterpolation;

    std::vector<TrajectoryBasePtr> wsTrajectory, wsRootFrameTrajectory;
    std::vector<TrajectoryBasePtr> csTrajectory;//, csRootFrameTrajectory;
    TrajectoryBasePtr mergedTrajectory;

    std::vector<TrajectoryBasePtr> csRetimedTrajectory;



    //PlannerBasePtr wsTracker;
    //std::vector<std::string> wsTrajectoryGroups;
    //IkParameterization ikparamPose;

    //std::vector<dReal>::iterator deltaOffset;
    double deltaTime;




    bool StepStageUpdatePoses(spGaitParameters params, std::vector<Transform>& effectorPosesResult)
    {

        double dx, dy, dz; //root translation
        switch(params.stepPhase)
        {

        //--Double support. Moving root inside poligon to left foot.--

        case 1: //--Move root over left foot while in double support--
            dx = effectorPosesResult[6].trans.x - effectorPosesResult[0].trans.x;
            dy = effectorPosesResult[6].trans.y - effectorPosesResult[0].trans.y;
            dz = 0;
            RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);

            break;

            //--Single support on left foot. Move root and right floating foot.--

        case 2: //First half swing of right foot

            //righ foot swing
            effectorPosesResult[5].trans.z += params.swingElevation;
            effectorPosesResult[5].trans.x += params.swingDistance/2;

            //root translation
            dx = params.swingDistance/4;
            dy = 0;
            dz = 0;
            RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);
            break;

        case 3: //Second half swing of right foot

            //righ foot swing
            effectorPosesResult[5].trans.z -= params.swingElevation;
            effectorPosesResult[5].trans.x += params.swingDistance/2;

            //root translation
            dx = params.swingDistance/4;
            dy = 0;
            dz = 0;
            RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);
            break;

            //--Double support. Moving root inside poligon.-- (to center)

        case 4: //--Move root over right foot while in double support--
            dx = (effectorPosesResult[5].trans.x+effectorPosesResult[6].trans.x)/2 - effectorPosesResult[0].trans.x;
            dy = (effectorPosesResult[5].trans.y+effectorPosesResult[6].trans.y)/2 - effectorPosesResult[0].trans.y;
            dz = 0;
            RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);
            break;

            //--Double support. Moving root inside poligon.-- (to right foot)

        case 5: //--Move root over right foot while in double support--
            dx = effectorPosesResult[5].trans.x - effectorPosesResult[0].trans.x;
            dy = effectorPosesResult[5].trans.y - effectorPosesResult[0].trans.y;
            dz = 0;
            RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);
            break;

            //--Single support on right foot. Move root and left floating foot.--

        case 6: //First half swing of left foot

            //left foot swing
            effectorPosesResult[6].trans.z += params.swingElevation;
            effectorPosesResult[6].trans.x += params.swingDistance/2;

            //root translation
            dx = params.swingDistance/4;
            dy = 0;
            dz = 0;
            RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);
            break;

        case 7: //Second half swing of left foot

            //left foot swing
            effectorPosesResult[6].trans.z -= params.swingElevation;
            effectorPosesResult[6].trans.x += params.swingDistance/2;

            //root translation
            dx = params.swingDistance/4;
            dy = 0;
            dz = 0;
            RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);
            break;

            //--Double support. Moving root inside poligon.-- (to center)

        case 8: //--Move root over right foot while in double support--
            dx = (effectorPosesResult[5].trans.x+effectorPosesResult[6].trans.x)/2 - effectorPosesResult[0].trans.x;
            dy = (effectorPosesResult[5].trans.y+effectorPosesResult[6].trans.y)/2 - effectorPosesResult[0].trans.y;
            dz = 0;
            RootTranslationWithAttachments(dx, dy, dz, effectorPosesResult);
            break;


        default:
            //If this is no step phase stop and return error.
            return false;
        };


        return true;
    }

    bool RootTranslationWithAttachments (double dx, double dy, double dz, std::vector<Transform>& effectorPosesResult)
    {

        //Change positions to root
        effectorPosesResult[0].trans.x += dx;
        effectorPosesResult[0].trans.y += dy;
        effectorPosesResult[0].trans.z += dz;

        //Update all bodies attached to root
        for (int i=1; i<=4; i++)
        {
            effectorPosesResult[i].trans.x += dx;
            effectorPosesResult[i].trans.y += dy;
            effectorPosesResult[i].trans.z += dz;
        }


        return true;

    }

    //function for adding workspace trajectory waypoints to wsTrajectory and wsRootFrameTrajectory vectors
    bool AddWorkspaceWaypoint(const std::vector<Transform>& wpPoses)
    {
        OpenRAVE::RaveVector<dReal> trans;
        OpenRAVE::RaveVector<dReal> rot;
        Transform T_0_root = wpPoses[0];
        Transform T_0_ee, T_root_ee;

        std::vector<dReal> waypoints, wpRootFrame;


        ConfigurationSpecification trajConfig;

        //No need to check the trajectory vector size if both vectors are sized same before
        //as root (0) and manipulator number.
        for (uint i=0; i<wpPoses.size(); i++)
        {
            std::vector<dReal> values;
            IkParameterization ikparamPose(wpPoses[i],IKP_Transform6D);
            trajConfig = wsTrajectory[i]->GetConfigurationSpecification();
            values.resize(trajConfig.GetDOF());

            ikparamPose.GetValues(values.begin());
            //values.push_back(deltaTime);


            trajConfig.InsertDeltaTime(values.begin(),stepParams.t);
            wsTrajectory[i]->Insert(wsTrajectory[i]->GetNumWaypoints(),values);
        }

        for (uint i=0; i<wpPoses.size(); i++)
        {
            T_0_ee = wpPoses[i];
            T_root_ee = T_0_root.inverse()*T_0_ee;

            std::vector<dReal> values;
            IkParameterization ikparamPose(T_root_ee,IKP_Transform6D);

            //wsTrajectoryGroups.push_back( ikparamPose.GetConfigurationSpecification().GetGroupFromName("").name );
            //std::cout << "ikparam name: " << wsTrajectoryGroups[i] << std::endl;
            trajConfig = wsRootFrameTrajectory[i]->GetConfigurationSpecification();
            values.resize(trajConfig.GetDOF());

            ikparamPose.GetValues(values.begin());
            //values.push_back(deltaTime);
            trajConfig.InsertDeltaTime(values.begin(),stepParams.t);
            wsRootFrameTrajectory[i]->Insert(wsRootFrameTrajectory[i]->GetNumWaypoints(),values);
        }

        return true;
    }
//TODO: write this function
    /*
    bool GetWorkspaceWaypoint(uint wp, std::vector<Transform>& wpPoses)
    {
        OpenRAVE::RaveVector<dReal> trans;
        OpenRAVE::RaveVector<dReal> rot;
        Transform T_0_root = wpPoses[0];
        Transform T_0_ee, T_root_ee;

        std::vector<dReal> waypoints, wpRootFrame;


        ConfigurationSpecification trajConfig;

        //No need to check the trajectory vector size if both vectors are sized same before
        //as root (0) and manipulator number.
        for (uint i=0; i<wpPoses.size(); i++)
        {
            std::vector<dReal> values;
            IkParameterization ikparamPose(wpPoses[i],IKP_Transform6D);
            trajConfig = wsTrajectory[i]->GetConfigurationSpecification();
            values.resize(trajConfig.GetDOF());

            ikparamPose.GetValues(values.begin());
            //values.push_back(deltaTime);


            trajConfig.InsertDeltaTime(values.begin(),stepParams.t);
            wsTrajectory[i]->Insert(wsTrajectory[i]->GetNumWaypoints(),values);
        }

        for (uint i=0; i<wpPoses.size(); i++)
        {
            T_0_ee = wpPoses[i];
            T_root_ee = T_0_root.inverse()*T_0_ee;

            std::vector<dReal> values;
            IkParameterization ikparamPose(T_root_ee,IKP_Transform6D);

            //wsTrajectoryGroups.push_back( ikparamPose.GetConfigurationSpecification().GetGroupFromName("").name );
            //std::cout << "ikparam name: " << wsTrajectoryGroups[i] << std::endl;
            trajConfig = wsRootFrameTrajectory[i]->GetConfigurationSpecification();
            values.resize(trajConfig.GetDOF());

            ikparamPose.GetValues(values.begin());
            //values.push_back(deltaTime);
            trajConfig.InsertDeltaTime(values.begin(),stepParams.t);
            wsRootFrameTrajectory[i]->Insert(wsRootFrameTrajectory[i]->GetNumWaypoints(),values);
        }

        return true;
    }
    */
    //Function: Fill rootAndEffector_poses with the current workspace state of robot
    bool CurrentWorkspacePose (std::vector<Transform>& rootAndEffector_poses)
    {

        //--Manipulators--
        //std::vector<RobotBase::ManipulatorPtr> manips;
        //RobotBase::ManipulatorPtr manip; //Temporal data. For operations over manips.

        rootAndEffector_poses.push_back( robot->GetLink(rootLink)->GetTransform() );

       for (uint i=0; i<manips.size(); i++)
        {
           //manip=manips[i];
           rootAndEffector_poses.push_back( manips[i]->GetTransform() );
        }
        return true;
    }


    bool FormatWorkspaceTrajectories(std::vector<TrajectoryBasePtr> trajectories)
    {



        std::vector<dReal> maxvelocities(7,0.05);
        std::vector<dReal> maxaccelerations(7,0.05);

        //TrajectoryBasePtr trajectory;

        //retime trajectories
        planningutils::RetimeAffineTrajectory(trajectories[0],maxvelocities,maxaccelerations);
        planningutils::RetimeAffineTrajectory(trajectories[1],maxvelocities,maxaccelerations);
        planningutils::RetimeAffineTrajectory(trajectories[2],maxvelocities,maxaccelerations);
        planningutils::RetimeAffineTrajectory(trajectories[3],maxvelocities,maxaccelerations);
        planningutils::RetimeAffineTrajectory(trajectories[4],maxvelocities,maxaccelerations);
        planningutils::RetimeAffineTrajectory(trajectories[5],maxvelocities,maxaccelerations,true);
        planningutils::RetimeAffineTrajectory(trajectories[6],maxvelocities,maxaccelerations,true);

        for (uint i=0; i<trajectories.size(); i++)
        {
            RAVELOG_INFO(str(boost::format("duration=%f, points=%d")%trajectories[i]->GetDuration()%trajectories[i]->GetNumWaypoints()));

        }

        return true;
    }

    bool FormatJointTrajectories(std::vector<TrajectoryBasePtr> trajectories)
    {

        TrajectoryBasePtr trajectory;
        ConfigurationSpecification trajConfig;
        TrajectoryBasePtr formattedTrajectory;

        formattedTrajectory = RaveCreateTrajectory(env);

        for (uint i=0; i<trajectories.size(); i++)
        {
            trajectory = trajectories[i];

            //Prepare FormattedTraj to get data
            trajConfig = trajectory->GetConfigurationSpecification();
            formattedTrajectory->Init(trajConfig);

            //copy data


            RAVELOG_INFO(str(boost::format("duration=%f, points=%d")%trajectory->GetDuration()%trajectories[i]->GetNumWaypoints()));

        }

        return true;
    }


    bool ExportWorkspaceTrajectories(std::vector<TrajectoryBasePtr> trajectories)
    {
        //compute trajectory total time
        double totalTime =0;
        for (uint i=0; i<trajectories.size(); i++)
        {
            //put the biggest trajectory time in totalTime
            if ( totalTime < trajectories[i]->GetDuration() )
            {
                totalTime = trajectories[i]->GetDuration();
            }
        }

        std::ofstream froot, fmanip, flocom;
        froot.open("root.csv");
        flocom.open("locom.csv");
        fmanip.open("manip.csv");

        std::vector<Transform> poses;

        //get all the trajectory values and write the files
        for (double time=0; time<totalTime; time += deltaTime)
        {
            //get the poses of the robot for a time
            poses = WorkspaceTrajectoryPose(time, trajectories);

            froot << poses[0].trans << std::endl;
            fmanip << poses[1].trans;
            fmanip << poses[2].trans;
            fmanip << poses[3].trans << std::endl;
            flocom << poses[4].trans;
            flocom << poses[5].trans;
            flocom << poses[6].trans << std::endl;


        }


        froot.close();
        flocom.close();
        fmanip.close();

        return true;
    }

    bool ExportJointTrajectories(std::vector<TrajectoryBasePtr> trajectories)
    {
        //compute trajectory total time
        double totalTime =0;
        /*for (uint i=5; i<trajectories.size(); i++)
        {
            //put the biggest trajectory time in totalTime
            if ( totalTime < trajectories[i]->GetDuration() )
            {
                totalTime = trajectories[i]->GetDuration();
            }
        }*/

        std::ofstream froot;
        froot.open("locomotionRecording.txt");


        std::vector<double> JointValuesVector;


        //Call viewTrajectories and fill merged trajectories
        ViewWorkspaceTrajectories(std::cout, std::cin);
        totalTime = mergedTrajectory->GetDuration();


        //get all the trajectory values and write the files
        for (double time=0; time<totalTime; time += deltaTime)
        {
            //get the poses of the robot for a time
            JointValuesVector = JointTrajectoryValues(time, mergedTrajectory);

            froot << std::setprecision(6) << std::fixed;

            if (JointValuesVector.size()==12)
            {
                //can14
                froot << 0.0 << " ";
                //can6
                froot << JointValuesVector[0]*57.296 << " ";
                //can5
                froot << JointValuesVector[1]*57.296 << " ";
                //can4
                froot << JointValuesVector[2]*57.296 << " ";
                //can3
                froot << JointValuesVector[3]*57.296 << " ";
                //can2
                froot << JointValuesVector[4]*57.296 << " ";
                //can1
                froot << JointValuesVector[5]*-57.296 << " ";
                //can13
                froot << 0.0 << " ";
                //can7
                froot << JointValuesVector[6]*57.296 << " ";
                //can8
                froot << JointValuesVector[7]*-57.296 << " ";
                //can9
                froot << JointValuesVector[8]*57.296 << " ";
                //can10
                froot << JointValuesVector[9]*57.296 << " ";
                //can11
                froot << JointValuesVector[10]*57.296 << " ";
                //can12
                froot << JointValuesVector[11]*57.296 << " ";

            }
            else {std::cout << "write file size error";}

            froot << std::endl;

            /*
            froot << JointValuesVector[0] << std::endl;
            fmanip << JointValuesVector[1].trans;
            fmanip << JointValuesVector[2].trans;
            fmanip << JointValuesVector[3].trans << std::endl;
            flocom << JointValuesVector[4].trans;
            flocom << JointValuesVector[5].trans;
            flocom << JointValuesVector[6].trans << std::endl;
*/

        }


        froot.close();


        return true;
    }

    std::vector<Transform> WorkspaceTrajectoryPose(double time, std::vector<TrajectoryBasePtr> traj)
    {
        ConfigurationSpecification trajConfig;
        std::vector<dReal> trajPoint;
        IkParameterization wsPose;
        std::vector<Transform> transforms;

        for (uint i = 0; i<traj.size();i++)
        {
            trajConfig = traj[i]->GetConfigurationSpecification();

            traj[i]->Sample(trajPoint, time);



            trajConfig.ExtractIkParameterization(wsPose, trajPoint.begin());
            transforms.push_back(wsPose.GetTransform6D());
        }
        return transforms;
    }

    std::vector<std::vector<double> > JointTrajectoriesValues(double time, std::vector<TrajectoryBasePtr> traj)
    {
        //needed for ExtractJointValues
        ConfigurationSpecification trajConfig;
        std::vector<dReal> trajPoint;
        std::vector<KinBodyPtr> bodies;
        std::vector<int> indices, dofindices;
        //return values
        std::vector<std::vector<double> > jointVector;
        std::vector<double> jointValues;

        for (uint i = 5; i<traj.size();i++)
        {
            //get the traj info
            trajConfig = traj[i]->GetConfigurationSpecification();
            //prepare the vector to copy
            trajConfig.ExtractUsedBodies(env, bodies);
            trajConfig.ExtractUsedIndices(bodies[0],dofindices,indices);
            jointValues.resize( dofindices.size() );

            traj[i]->Sample(trajPoint, time);

            trajConfig.ExtractJointValues(jointValues.begin(), trajPoint.begin(),
                                          bodies[0], dofindices);
            jointVector.push_back(jointValues);

            /*std::cout << bodies[0]->GetName() << ", " << time << ", ";
            for (uint i=0; i<jointValues.size();i++)
            {

                std::cout << indices[i] << ", ";
                std::cout << jointValues[i] << ", ";
            }
            std::cout << std::endl;*/

        }
        return jointVector;
    }

    std::vector<double> JointTrajectoryValues(double time, TrajectoryBasePtr traj)
    {
        //needed for ExtractJointValues
        ConfigurationSpecification trajConfig;
        std::vector<dReal> trajPoint;
        std::vector<KinBodyPtr> bodies;
        std::vector<int> indices, dofindices;
        //return values
        //std::vector<std::vector<double> > jointVector;
        std::vector<double> jointValues;

            //get the traj info
            trajConfig = traj->GetConfigurationSpecification();
            //prepare the vector to copy
            trajConfig.ExtractUsedBodies(env, bodies);
            trajConfig.ExtractUsedIndices(bodies[0],dofindices,indices);
            jointValues.resize( dofindices.size() );

            traj->Sample(trajPoint, time);

            trajConfig.ExtractJointValues(jointValues.begin(), trajPoint.begin(),
                                          bodies[0], dofindices);
            //jointVector.push_back(jointValues);

            /*std::cout << bodies[0]->GetName() << ", " << time << ", ";
            for (uint i=0; i<jointValues.size();i++)
            {

                std::cout << indices[i] << ", ";
                std::cout << jointValues[i] << ", ";
            }
            std::cout << std::endl;*/


        return jointValues;
    }

    bool WorkspaceTrajectoryToJoints( const std::vector<TrajectoryBasePtr>& wsTraj, std::vector<TrajectoryBasePtr>& csTraj)
    {
        //TODO: set velocicty and acceleration by parameters.
        //std::vector<dReal> maxvelocities(6,0.1);
        //std::vector<dReal> maxaccelerations(6,0.1);

        for (uint i=5; i<csTraj.size(); i++)
        {
            if (GetTrajectoryIK(manips[i-1], wsTraj[i], csTraj[i]))
            {
                //planningutils::RetimeAffineTrajectory(csTraj[i],maxvelocities,maxaccelerations);

            }
            else
            {
                std::cout << "Inverse Kinematics number " << i << " failed. " ;
                std::cout << "CSpace trajectory will not be available" << std::endl;

            }

        }



//TODO: Try to use : wsTracker = RaveCreatePlanner(env,"workspacetrajectorytracker");
  /*       //RAVELOG_INFO("starting to plan\n");
        if( !wsTracker->InitPlan(robot,params) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("plan init failed",ORE_Assert);
        }


        //wsTracker->GetParameters()->Serialize(std::cout);
        // create a new output trajectory
        //TrajectoryBasePtr outputtraj = RaveCreateTrajectory(penv,"");
        if( !wsTracker->PlanPath(csTraj[5]) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("plan failed",ORE_Assert);
        }
  */     return true;
    }

    bool GetTrajectoryIK(RobotBase::ManipulatorPtr manip, TrajectoryBasePtr wsTraj,TrajectoryBasePtr csTraj)
    {

        ConfigurationSpecification trajConfig, csTrajConfig;
        std::vector<dReal> trajPoint;
        std::vector< std::vector<dReal> >ikSols;
        std::vector<dReal> deltaIK(6,0),prevIK(6,0);
        std::vector<dReal> ikSol;
        IkParameterization wsPose;
        double module, lowestModule;
        uint slimVector;
        double timePoint;




        csTrajConfig = manip->GetArmConfigurationSpecification();
        csTrajConfig.AddDeltaTimeGroup();
        csTraj->Init(csTrajConfig);


        for (uint i=0; i<wsTraj->GetNumWaypoints(); i++)
        {
            //find the best IK for each point of ws trajectory

            //FindIKSolutions works inside the environment (env) Frame of reference.
            //Then, wsPose has to be framed in env.
            //If manipulator base moves, it must be updated before the call.
            trajConfig = wsTrajectory[0]->GetConfigurationSpecification();
            wsTrajectory[0]->GetWaypoint(i,trajPoint);
            trajConfig.ExtractIkParameterization(wsPose, trajPoint.begin());
            robot->SetTransform(wsPose.GetTransform6D());

            trajConfig = wsTraj->GetConfigurationSpecification();
            wsTraj->GetWaypoint(i,trajPoint);
            trajConfig.ExtractIkParameterization(wsPose, trajPoint.begin());
            trajConfig.ExtractDeltaTime(timePoint, trajPoint.begin());
            //std::cout << "cuat : " << wsPose.GetTransform6D().rot << std::endl;
            //std::cout << "trans : " << wsPose.GetTransform6D().trans << std::endl;

            manip->FindIKSolutions(wsPose,ikSols,IKFO_IgnoreSelfCollisions);
            if(ikSols.size() < 1)
            {
                std::cout << "IK solutions = " << ikSols.size() << ". Unreachable?" << std::endl;
                return false;
            }
            slimVector = 0;
            lowestModule = 100;

            for (uint j=0; j<ikSols.size(); j++)
            {
                module = 0;
                for (uint i=0; i<ikSols[j].size(); i++)
                {
                    //std::cout << ikSols[j][i] << ",";
                    deltaIK[i] = ikSols[j][i] - prevIK[i];
                    module += deltaIK[i]*deltaIK[i];

                }
                module = std::sqrt(module);
                if (module < lowestModule && ikSols[j][4]<0)
                {
                    slimVector = j;
                    lowestModule = module;
                }
               // std::cout << "vector : " << std::endl;

            }
            //std::cout << "Lowest : " << slimVector << std::endl;

            ikSol = ikSols[slimVector];
            //TODO: check for a better way to insert delta times
            ikSol.push_back(0);
            csTrajConfig.InsertDeltaTime(ikSol.begin(),timePoint);
            csTraj->Insert(csTraj->GetNumWaypoints(),ikSol,csTrajConfig);

        }
        //planningutils::RetimeTrajectory(csTraj, false, 0.05, 0.05);

        return true;
    }

  /*  LastTrajectoryVector(std::vector<double> jointVector, std::vector<TrajectoryBasePtr> trajectories)
    {

        TrajectoryBasePtr actualtraj;

        for (int i=0; i<trajectories.size(); i++)
        {
            actualtraj=trajectories[i];
            for (int j=0; j<actualtraj->GetDOF(); j++)
            {


            }
        }

    }
*/

};

class SPGaitPlanner : public PlannerBase
{
public:
    SPGaitPlanner(EnvironmentBasePtr penv, std::istream& ss) : PlannerBase(penv)
    {

    }
    virtual ~SPGaitPlanner() {}

private:


};


class spGaitTrajectory : public TrajectoryBase
{
public:
    spGaitTrajectory(EnvironmentBasePtr penv, std::istream& ss) : TrajectoryBase(penv)
    {

    }
    virtual ~spGaitTrajectory() {}

private:


};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "spgait" ) {
        return InterfaceBasePtr(new spGait(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("spgait");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

