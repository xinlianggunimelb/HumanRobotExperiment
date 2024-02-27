/**
 * \file M2AssistRobot.h
 * \author Xinliang Guo
 * /brief The <code>M2AssistRobot</code> class represents an example implementation of an M2 state machine.
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
#include "M2AssistRobotStates.h"


/**
 * @brief Example implementation of a StateMachine for the M2Robot class. States should implemented M2DemoState
 *
 */
class M2AssistRobot : public StateMachine {
   public:
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M2AssistRobot();
    ~M2AssistRobot();
    void init();
    void end();

    void hwStateUpdate();

    SpasticityTest *STest;

    RobotM2 *robot() { return static_cast<RobotM2*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)
    std::shared_ptr<FLNLHelper> UIserver = nullptr;     //!< Pointer to communication server
};

#endif /*M2_SM_H*/
