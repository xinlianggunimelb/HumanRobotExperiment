#include "M2EmgRobotCtrl.h"


bool EndCalib(StateMachine & SM) {
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
    return sm.state<M2Calib>("calibState")->isCalibDone();
}


bool GoToNextState(StateMachine & SM) {
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
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
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
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
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
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
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
    return sm.state<M2Recording>("recordingState")->isRecordingDone();
}


bool FailRecording(StateMachine & SM) {
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
    return sm.state<M2Recording>("recordingState")->isRecordingError();
}


bool StartTesting(StateMachine & SM) {
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
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
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
    return sm.state<M2ArcCircle>("circleTestState")->isTestingDone();
}


bool FailTesting(StateMachine & SM) {
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
    return sm.state<M2ArcCircle>("circleTestState")->isTestingError();
}

bool EndTestReturn(StateMachine & SM) {
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
    return sm.state<M2ArcCircleReturn>("circleReturnState")->isTestReturnDone();
}

bool StartConstForce(StateMachine & SM) {
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==5) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "CSTF") { //Start Constant Force command received
            //Acknowledge
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool EndConstForce(StateMachine & SM) {
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
    return sm.state<M2ConstForce>("constForceState")->isConstFDone();
}


bool EndPert(StateMachine & SM) {
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
    return sm.state<M2StochPert>("stochPertState")->isPertDone();
}


bool EndTrial(StateMachine & SM) {
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
    return sm.state<M2MinJerkPosition>("minJerkState")->isTrialDone();
}


bool MaxForceReturn(StateMachine & SM) {
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
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
    M2EmgRobotCtrl & sm = static_cast<M2EmgRobotCtrl&>(SM); //Cast to specific StateMachine type
    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==9))
        return true;

    if (sm.EmgCtrl->goToTransparentFlag)
    {
        sm.EmgCtrl->goToTransparentFlag = false;
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
            sm.EmgCtrl->StateIndex = 10.;
            return true;
        }
    }

    //Otherwise false
    return false;
}



M2EmgRobotCtrl::M2EmgRobotCtrl() {
    //Create an M2 Robot and set it to generic state machine
    setRobot(std::make_unique<RobotM2>("M2_MELB"));

    //Shared data structure
    EmgCtrl = new EMGbaseRobotCtrl();

    //Create state instances and add to the State Machine
    addState("calibState", std::make_shared<M2Calib>(robot(), EmgCtrl));
    addState("standbyState", std::make_shared<M2Transparent>(robot(), EmgCtrl));
    addState("minJerkState", std::make_shared<M2MinJerkPosition>(robot(), EmgCtrl));
    addState("recordingState", std::make_shared<M2Recording>(robot(), EmgCtrl));
    addState("circleTestState", std::make_shared<M2ArcCircle>(robot(), EmgCtrl));
    addState("circleReturnState", std::make_shared<M2ArcCircleReturn>(robot(), EmgCtrl));
    addState("constForceState", std::make_shared<M2ConstForce>(robot(), EmgCtrl));
    addState("stochPertState", std::make_shared<M2StochPert>(robot(), EmgCtrl));
    //Newly defined:
    addState("identifyState", std::make_shared<M2Identify>(robot(), EmgCtrl));
    addState("identify2State", std::make_shared<M2Identify2>(robot(), EmgCtrl));
    addState("stabilityState", std::make_shared<M2Stability>(robot(), EmgCtrl));
    addState("adaptVEState", std::make_shared<M2AdaptVE>(robot(), EmgCtrl));

    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    addTransition("calibState", &EndCalib, "standbyState");

    //addTransition("standbyState", &StartRecording, "recordingState");
    //addTransition("recordingState", &FailRecording, "standbyState");
    //addTransition("recordingState", &EndRecording, "minJerkState");
    //addTransition("minJerkState", &StartTesting, "circleTestState");
    //addTransition("circleTestState", &FailTesting, "standbyState");
    //addTransition("circleTestState", &EndTesting, "circleReturnState");
    //addTransition("circleReturnState", &EndTestReturn, "minJerkState");
    //addTransition("minJerkState", &StartConstForce, "constForceState");
    //addTransition("constForceState", &EndConstForce, "minJerkState");
    //addTransition("minJerkState", &GoToNextState, "stochPertState");
    //addTransition("stochPertState", &EndPert, "minJerkState");

    //addTransition("standbyState", &MaxForceReturn, "minJerkState");
    addTransition("standbyState", &GoToTransparent, "standbyState");
    //addTransition("recordingState", &GoToTransparent, "standbyState");
    //addTransition("circleTestState", &GoToTransparent, "standbyState");
    //addTransition("circleReturnState", &GoToTransparent, "standbyState");
    //addTransition("minJerkState", &GoToTransparent, "standbyState");
    //addTransition("stochPertState", &GoToTransparent, "standbyState");

    //M2 Identification
    addTransition("standbyState", &GoToNextState, "identifyState");
    addTransition("standbyState", &GoToPrevState, "identify2State");
    addTransition("identifyState", &GoToTransparent, "standbyState");
    addTransition("identify2State", &GoToTransparent, "standbyState");

    //M2 Stability
    addTransition("standbyState", &StartRecording, "stabilityState");
    addTransition("stabilityState", &GoToTransparent, "standbyState");

    //M2 Adaptive Control
    addTransition("standbyState", &StartTesting, "adaptVEState");
    addTransition("adaptVEState", &GoToTransparent, "standbyState");

}
M2EmgRobotCtrl::~M2EmgRobotCtrl() {
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void M2EmgRobotCtrl::init() {
    spdlog::debug("M2EmgRobotCtrl::init()");
    if(robot()->initialise()) {
        logHelper.initLogger("M2EmgRobotCtrlLog", "logs/M2EmgRobotCtrl.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getEndEffPosition(), "Position");
        logHelper.add(robot()->getEndEffVelocity(), "Velocity");
        logHelper.add(robot()->getInteractionForce(), "Force");
        //logHelper.add(robot()->getTorque(), "MotorTorque");
        //logHelper.add(robot()->getEndEffForce(), "MotorForce");
        //Added
        logHelper.add(EmgCtrl->StateIndex, "State");
        //logHelper.add(EmgCtrl->constF_number, "ForceNum");
        //logHelper.add(EmgCtrl->perturbation_number, "PertNum");
        //logHelper.add(EmgCtrl->global_radius, "Radius");
        //logHelper.add(EmgCtrl->global_center_point, "Center");
        //logHelper.add(EmgCtrl->global_start_angle, "Angle");
        //M2 Identification
        //logHelper.add(EmgCtrl->sine_A_record, "A");
        //logHelper.add(EmgCtrl->sine_f_record, "f");
        //logHelper.add(EmgCtrl->sine_w_record, "w");
        //logHelper.add(EmgCtrl->const_Vd_record, "constVd");
        //logHelper.add(EmgCtrl->const_VelPhase_record, "constVelPhase");
        //Stability
        logHelper.add(EmgCtrl->B_ve, "Bve");
        logHelper.add(EmgCtrl->M_ve, "Mve");
        logHelper.add(EmgCtrl->Osci_detect, "OsciDetect");
        logHelper.startLogger();
        //UIserver = std::make_shared<FLNLHelper>(*robot(), "127.0.0.1"); //Locally
        //UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.6.2");  //Linux
        UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.7.2");  //Windows

        UIserver->registerState(EmgCtrl->StateIndex);
        //UIserver->registerState(EmgCtrl->constF_number);
        //UIserver->registerState(EmgCtrl->perturbation_number);
        //UIserver->registerState(EmgCtrl->global_radius);
        //UIserver->registerState(EmgCtrl->global_center_point[0]);
        //UIserver->registerState(EmgCtrl->global_center_point[1]);
        //UIserver->registerState(EmgCtrl->global_start_angle);
        //UIserver->registerState(EmgCtrl->Feedback_F);
        //UIserver->registerState(EmgCtrl->Feedback_K);
        //Stability
        UIserver->registerState(EmgCtrl->B_ve);
        UIserver->registerState(EmgCtrl->M_ve);
        UIserver->registerState(EmgCtrl->Osci_detect);
    }
    else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
}

void M2EmgRobotCtrl::end() {
    if(running())
        UIserver->closeConnection();
        StateMachine::end();
}

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void M2EmgRobotCtrl::hwStateUpdate(void) {
    StateMachine::hwStateUpdate();
    //Also send robot state over network
    UIserver->sendState();

    // =====================================================
    // Receive continuous values from Unity
    // =====================================================
    if(UIserver->isValues()) {
        UIserver->getValues(unityValues);

        int n = unityValues.size(); // number of values received
        for(int i=0; i<n; i++) {
            EmgCtrl->K_h = unityValues[0];
            //std::cout << "Unity value " << i << " = " << unityValues[i] << std::endl;
        }
    }

}




