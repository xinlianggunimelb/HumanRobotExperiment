/**
 * \file M2EmgRobotCtrl.h
 * \author Xinliang Guo
 * /brief The <code>M2EmgRobotCtrl</code> class represents an example implementation of an M2 state machine.
 * \version 0.2
 * \date 2024-02-27
 *
 * \copyright Copyright (c) 2020-2024
 *
 */
#ifndef M2_SM_H
#define M2_SM_H


#include "RobotM2.h"
#include "StateMachine.h"
#include "FLNLHelper.h"

// State Classes
#include "M2EmgRobotCtrlStates.h"


/**
 * @brief Example implementation of a StateMachine for the M2Robot class. States should implemented M2DemoState
 *
 */
class M2EmgRobotCtrl : public StateMachine {
   public:
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M2EmgRobotCtrl();
    ~M2EmgRobotCtrl();
    void init();
    void end();

    void hwStateUpdate();

    EMGbaseRobotCtrl *EmgCtrl;

    RobotM2 *robot() { return static_cast<RobotM2*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)
    std::shared_ptr<FLNLHelper> UIserver = nullptr;     //!< Pointer to communication server
    std::vector<double> unityValues;  //!NEW: Values received from Unity
};

#endif /*M2_SM_H*/
