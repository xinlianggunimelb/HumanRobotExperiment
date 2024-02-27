#include "M2AssistRobot.h"


bool EndCalib(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    return sm.state<M2Calib>("calibState")->isCalibDone();
}


bool GoToNextState(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==1) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "GTNS") { //Go To Next State command received
            //Acknowledge
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool GoToPrevState(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==2) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "GTPS") { //Go To Previous State command received
            //Acknowledge
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool StartRecording(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==3) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "RECD") { //Start Recording command received
            //Acknowledge
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool EndRecording(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    return sm.state<M2Recording>("recordingState")->isRecordingDone();
}


bool FailRecording(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    return sm.state<M2Recording>("recordingState")->isRecordingError();
}


bool StartTesting(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==4) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "TEST") { //Start Testing command received
            //Acknowledge
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool EndTesting(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    return sm.state<M2CircleTest>("testingState")->isTestingDone();
}


bool FailTesting(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    return sm.state<M2CircleTest>("testingState")->isTestingError();
}


bool StartTrial(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==5) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "TRIA") { //Start Trial command received
            //Acknowledge
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool StartNextVel(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    return sm.state<M2MinJerkPosition>("minJerkState")->GoToNextVel();
}



bool StartReturn(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==5) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "TRIA") { //Start Trial command received
            //Acknowledge
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool EndTrial(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    return sm.state<M2MinJerkPosition>("minJerkState")->isTrialDone();
}


bool MaxForceReturn(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==8) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "MFRT") { //Max Force Return command received
            //Acknowledge
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool GoToTransparent(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==9))
        return true;

    if (sm.STest->goToTransparentFlag)
    {
        sm.STest->goToTransparentFlag = false;
        return true;
    }

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "REST") { //Go To Transparent command received
            //Acknowledge
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(string("OK"));
            sm.STest->StateIndex = 9.;
            return true;
        }
    }

    //Otherwise false
    return false;
}


/******
Newly defined for EMD
******/
bool GoToEMDtest1(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==5) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "EMDS") { //Start Trial command received
            //Acknowledge
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool GoToEMDtest2(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==6) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "EMDV") { //Start Trial command received
            //Acknowledge
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool GoToEMDtest3(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==7) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "EMDP") { //Start Trial command received
            //Acknowledge
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool EndExtention(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    return sm.state<M2EMDtest3EXT>("EMDpassiveExtState")->isExtentionDone();
}


bool EndFlexion(StateMachine & SM) {
    M2AssistRobot & sm = static_cast<M2AssistRobot&>(SM); //Cast to specific StateMachine type
    return sm.state<M2EMDtest3FLX>("EMDpassiveFlxState")->isFlexionDone();
}




M2AssistRobot::M2AssistRobot() {
    //Create an M2 Robot and set it to generic state machine
    setRobot(std::make_unique<RobotM2>("M2_MELB"));

    //Shared data structure
    STest = new SpasticityTest();

    //Create state instances and add to the State Machine
    addState("calibState", std::make_shared<M2Calib>(robot(), STest));
    addState("standbyState", std::make_shared<M2Transparent>(robot(), STest));
    addState("minJerkState", std::make_shared<M2MinJerkPosition>(robot(), STest));
    addState("recordingState", std::make_shared<M2Recording>(robot(), STest));
    addState("testingState", std::make_shared<M2CircleTest>(robot(), STest));
    addState("experimentState", std::make_shared<M2ArcCircle>(robot(), STest));
    addState("experimentReturnState", std::make_shared<M2ArcCircleReturn>(robot(), STest));
    //Newly defined
    addState("EMDstaticState", std::make_shared<M2EMDtest1>(robot(), STest));
    addState("EMDvoluntaryState", std::make_shared<M2EMDtest2>(robot(), STest));
    addState("EMDpassiveExtState", std::make_shared<M2EMDtest3EXT>(robot(), STest));
    addState("EMDpassiveFlxState", std::make_shared<M2EMDtest3FLX>(robot(), STest));


    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    addTransition("calibState", &EndCalib, "standbyState");
    addTransition("standbyState", &StartRecording, "recordingState");
    addTransition("recordingState", &EndRecording, "minJerkState");
    addTransition("recordingState", &FailRecording, "standbyState");
    addTransition("minJerkState", &StartTesting, "testingState");
    addTransition("testingState", &EndTesting, "minJerkState");
    addTransition("testingState", &FailTesting, "standbyState");
    //addTransition("minJerkState", &startTrial, "experimentState");
    //addTransition("minJerkState", &startNextVel, "experimentState");
    //addTransition("experimentState", &startReturn, "minJerkState");
    //addTransition("minJerkState", &endTrial, "standbyState");
    //addTransition("experimentState", &goToNextState, "experimentReturnState");
    //addTransition("experimentReturnState", &goToPrevState, "experimentState");
    addTransition("standbyState", &MaxForceReturn, "minJerkState");
    addTransition("standbyState", &GoToTransparent, "standbyState");
    addTransition("recordingState", &GoToTransparent, "standbyState");
    addTransition("testingState", &GoToTransparent, "standbyState");
    addTransition("experimentState", &GoToTransparent, "standbyState");
    addTransition("minJerkState", &GoToTransparent, "standbyState");
    //Newly defined
    addTransition("minJerkState", &GoToEMDtest1, "EMDstaticState");
    addTransition("minJerkState", &GoToEMDtest2, "EMDvoluntaryState");
    addTransition("minJerkState", &GoToEMDtest3, "EMDpassiveExtState");
    addTransition("EMDpassiveExtState", &EndExtention, "EMDpassiveFlxState");
    addTransition("EMDpassiveFlxState", &EndFlexion, "EMDpassiveExtState");
    addTransition("EMDstaticState", &MaxForceReturn, "minJerkState");
    addTransition("EMDvoluntaryState", &MaxForceReturn, "minJerkState");
    addTransition("EMDpassiveExtState", &MaxForceReturn, "minJerkState");
    addTransition("EMDpassiveFlxState", &MaxForceReturn, "minJerkState");
    addTransition("EMDstaticState", &GoToTransparent, "standbyState");
    addTransition("EMDvoluntaryState", &GoToTransparent, "standbyState");
    addTransition("EMDpassiveExtState", &GoToTransparent, "standbyState");
    addTransition("EMDpassiveFlxState", &GoToTransparent, "standbyState");
}
M2AssistRobot::~M2AssistRobot() {
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void M2AssistRobot::init() {
    spdlog::debug("M2AssistRobot::init()");
    if(robot()->initialise()) {
        logHelper.initLogger("M2AssistRobotLog", "logs/M2AssistRobot.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getEndEffPosition(), "Position");
        logHelper.add(robot()->getEndEffVelocity(), "Velocity");
        logHelper.add(robot()->getInteractionForce(), "Force");
        logHelper.startLogger();
        UIserver = std::make_shared<FLNLHelper>(*robot(), "127.0.0.1"); //Locally
        //UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.6.2");  //Linux
        //UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.7.2");  //Windows
        UIserver->registerState(STest->StateIndex);
        UIserver->registerState(STest->AngularVelocity);
        UIserver->registerState(STest->global_radius);
        UIserver->registerState(STest->global_center_point[0]);
        UIserver->registerState(STest->global_center_point[1]);
        UIserver->registerState(STest->global_start_angle);
    }
    else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
}

void M2AssistRobot::end() {
    if(running())
        UIserver->closeConnection();
    StateMachine::end();
}

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void M2AssistRobot::hwStateUpdate(void) {
    StateMachine::hwStateUpdate();
    //Also send robot state over network
    UIserver->sendState();
}




