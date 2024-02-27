#include "M2Admittance.h"

bool endCalib(StateMachine & sm) {
    return (sm.state<M2Calib>("CalibState"))->isCalibDone();
}


bool goToNextState(StateMachine & SM) {
    M2Admittance & sm = static_cast<M2Admittance &>(SM); //Cast to specific StateMachine type

    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(5) || sm.robot()->keyboard->getS()) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "GTNS") { //Go To Next State command received
            //Acknowledge
            sm.UIserver->sendCmd(string("OK"));

            return true;
        }
    }

    //Otherwise false
    return false;
}

bool goToPrevState(StateMachine & SM) {
    M2Admittance & sm = static_cast<M2Admittance &>(SM); //Cast to specific StateMachine type

    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(6) || sm.robot()->keyboard->getNb()==2) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "GTPS") { //Go To Next State command received
            //Acknowledge
            sm.UIserver->sendCmd(string("OK"));

            return true;
        }
    }

    //Otherwise false
    return false;
}

bool goToTransparent(StateMachine & SM) {
    M2Admittance & sm = static_cast<M2Admittance &>(SM); //Cast to specific StateMachine type

    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==9) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "GTTS") { //Go To Next State command received
            //Acknowledge
            sm.UIserver->sendCmd(string("OK"));

            return true;
        }
    }

    if(sm.goToTransparentFlag)
    {
        sm.goToTransparentFlag = false;
        return true;
    }

    //Otherwise false
    return false;
}


bool goToOne(StateMachine & SM) {
    M2Admittance & sm = static_cast<M2Admittance &>(SM); //Cast to specific StateMachine type

    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(1) || sm.robot()->keyboard->getNb()==1) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "GT1S") { //Go To Next State command received
            //Acknowledge
            sm.UIserver->sendCmd(string("OK"));

            return true;
        }
    }

    //Otherwise false
    return false;
}

bool goToTwo(StateMachine & SM) {
    M2Admittance & sm = static_cast<M2Admittance &>(SM); //Cast to specific StateMachine type

    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(2) || sm.robot()->keyboard->getNb()==2) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "GT2S") { //Go To Next State command received
            //Acknowledge
            sm.UIserver->sendCmd(string("OK"));

            return true;
        }
    }

    //Otherwise false
    return false;
}

bool goToThree(StateMachine & SM) {
    M2Admittance & sm = static_cast<M2Admittance &>(SM); //Cast to specific StateMachine type

    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(3) || sm.robot()->keyboard->getNb()==3) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "GT3S") { //Go To Next State command received
            //Acknowledge
            sm.UIserver->sendCmd(string("OK"));

            return true;
        }
    }

    //Otherwise false
    return false;
}

bool goToFour(StateMachine & SM) {
    M2Admittance & sm = static_cast<M2Admittance &>(SM); //Cast to specific StateMachine type

    //keyboard or joystick press
    if ( (sm.robot()->joystick->isButtonPressed(4) || sm.robot()->keyboard->getNb()==4) )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        sm.UIserver->getCmd(cmd, v);
        if (cmd == "GT4S") { //Go To Next State command received
            //Acknowledge
            sm.UIserver->sendCmd(string("OK"));

            return true;
        }
    }

    //Otherwise false
    return false;
}


M2Admittance::M2Admittance() {
    //Create an M2 Robot and set it to generic state machine
    setRobot(std::make_unique<RobotM2>("M2_MELB"));

    //Create state instances and add to the State Machine
    addState("CalibState", std::make_shared<M2Calib>(robot()));
    addState("StandbyState", std::make_shared<M2Transparent>(robot()));
    addState("GoToP0State", std::make_shared<M2MinJerkPosition>(robot()));
    addState("Admittance1State", std::make_shared<M2Admittance1>(robot()));
    addState("Admittance2State", std::make_shared<M2Admittance2>(robot()));
    addState("Admittance3State", std::make_shared<M2Admittance3>(robot()));
    addState("Admittance4State", std::make_shared<M2Admittance3>(robot()));


    //Define transitions between states
    addTransition("CalibState", &endCalib, "GoToP0State");
    addTransitionFromLast(&goToOne, "Admittance1State");
    addTransitionFromLast(&goToNextState, "GoToP0State");
    addTransitionFromLast(&goToTwo, "Admittance2State");
    addTransitionFromLast(&goToNextState, "GoToP0State");
    addTransitionFromLast(&goToThree, "Admittance3State");
    addTransitionFromLast(&goToNextState, "GoToP0State");
    addTransitionFromLast(&goToFour, "Admittance4State");
    addTransitionFromLast(&goToNextState, "GoToP0State");


    //Initialize the state machine with first state of the designed state machine
    setInitState("CalibState");

    //Data structure
    STest = new AdmittanceTest();
}
M2Admittance::~M2Admittance() {
}

/**
 * \brief start function for running any designed statemachine specific functions
 *
 */
void M2Admittance::init() {
    spdlog::debug("M2Admittance::init()");
    if(robot()->initialise()) {
        logHelper.initLogger("M2AdmittanceLog", "logs/M2Admittance.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getEndEffPosition(), "Position");
        logHelper.add(robot()->getEndEffVelocity(), "Velocity");
        logHelper.add(robot()->getEndEffForce(), "Force");
        logHelper.startLogger();
        UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.7.2");
    }
    else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
}

void M2Admittance::end() {
    if(running())
        UIserver->closeConnection();
    StateMachine::end();
}



/**
 * \brief Statemachine to hardware interface method.
 *
 */
void M2Admittance::hwStateUpdate(void) {
    StateMachine::hwStateUpdate();
    //Also send robot state over network
    UIserver->sendState();
}
