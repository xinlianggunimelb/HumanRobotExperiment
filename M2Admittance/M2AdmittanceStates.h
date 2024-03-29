/**
 * /file M2AdmittanceStates.h
 * \author Xinliang Guo, Vincent Crocher
 * \version 0.1
 * \date 2024-202-23
 *
 * \copyright Copyright (c) 2024
 *
 */

#ifndef M2AdmittanceStates_H
#define M2AdmittanceStates_H

#include "State.h"
#include "RobotM2.h"
#include "LogHelper.h"

using namespace std;


class M2State : public State {
   protected:
    RobotM2 * robot;                               //!< Pointer to state machines robot object

    M2State(RobotM2* M2, const char *name = NULL): State(name), robot(M2){spdlog::debug("Created M2State {}", name);};
};

class M2DemoState : public M2State {

   public:
    M2DemoState(RobotM2 *M2, const char *name = "M2 Test State"):M2State(M2, name){};

    void entry(void);
    void during(void);
    void exit(void);

    VM2 qi, Xi, tau;
};



/**
 * \brief Position calibration of M2. Go to the bottom left stops of robot at constant torque for absolute position calibration.
 *
 */
class M2Calib : public M2State {

   public:
    M2Calib(RobotM2 *M2, const char *name = "M2 Calib State"):M2State(M2, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isCalibDone() {return calibDone;}

   private:
     VM2 stop_reached_time;
     bool at_stop[2];
     bool calibDone=false;
};


/**
 * \brief Provide end-effector mass compensation on M2. Mass is controllable through keyboard inputs.
 *
 */
class M2Transparent : public M2State {

   public:
    M2Transparent(RobotM2 *M2, const char *name = "M2 Transparent"):M2State(M2, name){};

    void entry(void);
    void during(void);
    void exit(void);

   private:
    LogHelper stateLogger;
    VM2 dXd;
    VM2 Fd;
    double amplitude = 0;
};



/**
 * \brief Point to tpoint position control with min jerk trajectory interpolation
 *
 */
class M2MinJerkPosition: public M2State {

   public:
    M2MinJerkPosition(RobotM2 *M2, const char *name = "M2 Demo Minimum Jerk Position"):M2State(M2, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool GoToNextVel() {return goToNextVel;}
    bool isTrialDone() {return trialDone;}

   private:
    bool goToNextVel=false;
    bool trialDone=false;
    double startTime;
    VM2 Xi, Xf, Verror;
    double T;
    float k_i=1.; //Integral gain
};


/**
 * \brief Admittance Lawrence algo 1
 *
 */
class M2Admittance1: public M2State {

   public:
    M2Admittance1(RobotM2 *M2, const char *name = "M2 Admittance 1"):M2State(M2, name){};

    void entry(void);
    void during(void);
    void exit(void);

   private:
    LogHelper stateLogger;

    VM2 E_obs, E_obs_ls, E_obs_prediction;
    Eigen::Matrix2d B;
    Eigen::Matrix2d M;
    Eigen::Matrix2d Operator;
    //Eigen::Matrix2d B1, B2;
    Eigen::Matrix2d C_diss, C_diss_ls;

    VM2 X, dX;
    VM2 Fs, Fs_ls;
    VM2 Vd, V_ve, V_diss;
    VM2 E_class, E_class_dyn, E_diss;
    int Obsv_T, i;

    double limit;
};


/**
 * \brief Admittance Lawrence algo 2
 *
 */
class M2Admittance2: public M2State {

   public:
    M2Admittance2(RobotM2 *M2, const char *name = "M2 Admittance 2"):M2State(M2, name){};

    void entry(void);
    void during(void);
    void exit(void);

   private:
    LogHelper stateLogger;

    VM2 E_obs, E_obs_ls, E_obs_prediction;
    Eigen::Matrix2d B;
    Eigen::Matrix2d M;
    Eigen::Matrix2d Operator;
    //Eigen::Matrix2d B1, B2;
    Eigen::Matrix2d C_diss, C_diss_ls;

    VM2 X, dX;
    VM2 Fs, Fs_ls;
    VM2 Vd, V_ve, V_diss;
    VM2 E_class, E_class_dyn, E_diss;
    int Obsv_T, i;

    double limit;
};


/**
 * \brief Admittance Lawrence algo 3
 *
 */
class M2Admittance3: public M2State {

   public:
    M2Admittance3(RobotM2 *M2, const char *name = "M2 Admittance 3"):M2State(M2, name){};

    void entry(void);
    void during(void);
    void exit(void);

   private:
    LogHelper stateLogger;

    VM2 E_obs, E_obs_ls, E_obs_prediction;
    Eigen::Matrix2d B;
    Eigen::Matrix2d M;
    Eigen::Matrix2d Operator;
    Eigen::Matrix2d B1, B2;
    Eigen::Matrix2d C_diss, C_diss_ls;

    VM2 X, dX;
    //VM2 V_error;
    VM2 Fs, Fs_ls;
    VM2 Vd, V_ve, V_diss;
    VM2 E_upper, E_lower;
    int Obsv_T, i;

    double limit;
};


/**
 * \brief Admittance 4 CORC-PID
 *
 */
class M2Admittance4: public M2State {

   public:
    M2Admittance4(RobotM2 *M2, const char *name = "M2 Admittance CORC-PID"):M2State(M2, name){};

    void entry(void);
    void during(void);
    void exit(void);

   private:
    LogHelper stateLogger;

    VM2 E_obs, E_obs_ls, E_obs_prediction;
    Eigen::Matrix2d B;
    Eigen::Matrix2d M;
    Eigen::Matrix2d Operator;
    //Eigen::Matrix2d B1, B2;
    Eigen::Matrix2d C_diss, C_diss_ls;

    VM2 X, dX;
    VM2 Fs, Fs_ls;
    VM2 Vd, V_ve, V_diss;
    VM2 E_class, E_class_dyn, E_diss;
    int Obsv_T, i;

    VM2 Fd;
    VM2 V_error;
    double k_p;
    Eigen::Matrix2d Kp;
    double amplitude;

    VM2 tau_fc;
    double alpha, beta, threshold;
    double dq;
};

#endif /* M2AdmittanceStates_H */
