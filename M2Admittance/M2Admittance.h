/**
 * \file M2Admittance.h
 * \author Xinliang Guo, Vincent Crocher
 * /brief The M2Admittance class represents an example implementation of an M2 state machine.
 * \version 0.2
 * \date 2024-02-23
 *
 * \copyright Copyright (c) 2020 - 2024
 *
 */
#ifndef M2_SM_H
#define M2_SM_H

#include "RobotM2.h"
#include "StateMachine.h"
#include "FLNLHelper.h"

// State Classes
#include "M2AdmittanceStates.h"


//Admittance test variables
struct AdmittanceTest
{
};


/**
 * @brief Example implementation of a StateMachine for the M2Robot class. States should implemented M2DemoState
 *
 */
class M2Admittance : public StateMachine {

   public:
    M2Admittance();
    ~M2Admittance();
    void init();
    void end();

    void hwStateUpdate();

    RobotM2 *robot() { return static_cast<RobotM2*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)

    std::shared_ptr<FLNLHelper> UIserver = nullptr;     //!< Pointer to communication server


    AdmittanceTest *STest;
    bool goToTransparentFlag = false;
};

#endif /*M2_SM_H*/
