/**
 * /file M2AssistRobotStates.h
 * \author Vincent Crocher
 * \version 0.2
 * \date 2024-02-27
 *
 * \copyright Copyright (c) 2020-2024
 *
 */

#ifndef M2DemoSTATE_H_DEF
#define M2DemoSTATE_H_DEF

#include <csignal> //For raise()
#include "RobotM2.h"
#include "State.h"
#include "LogHelper.h"


using namespace std;


//Spasticity test variables
struct StiffnessTest
{
    VM2 global_center_point;
    VM2 global_start_point;
    double global_radius;
    double global_start_angle;
    int total_constF = 28;
    int constF_number = 0;
    int total_perturbation = 40;
    int perturbation_number = 0;
    double Feedback_F = 0.;
    double Feedback_K = 0.;
    double StateIndex = 0.;
    bool goToTransparentFlag = false;
};



/**
 * \brief Generic state type for used with M2AssistRobot.
 *
 */
class M2State : public State {
   protected:
    RobotM2 *robot;                //!< Pointer to state machines robot object
    StiffnessTest * KTest;

    M2State(RobotM2 *M2, StiffnessTest * _kt, const char *name = NULL): State(name), robot(M2), KTest(_kt){};
};


class M2DemoState : public M2State {

   public:
    M2DemoState(RobotM2 *M2, StiffnessTest *_kt, const char *name = "M2 Demo State"):M2State(M2, _kt, name){};

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
    M2Calib(RobotM2 *M2, StiffnessTest *_kt, const char *name = "M2 Calib State"):M2State(M2, _kt, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isCalibDone() {return calibDone;}

   private:
     VM2 stop_reached_time;
     bool at_stop[2];
     bool calibDone=false;
     int calibAttempts=0;
};


/**
 * \brief Basic impedance control on a static point.
 *
 */
class M2DemoImpedanceState : public M2State {

   public:
    M2DemoImpedanceState(RobotM2 *M2, StiffnessTest *_kt, const char *name = "M2 Demo Impedance State"):M2State(M2, _kt, name){};

    void entry(void);
    void during(void);
    void exit(void);

   private:
    VM2 Xi;
    double k = 700;     //! Impedance proportional gain (spring)
    double d = 2;       //! Impedance derivative gain (damper)
    bool init=false;

    unsigned int nb_samples=10000;
    double dts[10000];
    double dX[10000];
    int new_value;
};


/**
 * \brief Provide end-effector mass compensation on M2.
 *
 */
class M2Transparent : public M2State {

   public:
    M2Transparent(RobotM2 *M2, StiffnessTest *_kt, const char *name = "M2 Transparent"):M2State(M2, _kt, name){};

    void entry(void);
    void during(void);
    void exit(void);

   private:
    Eigen::Matrix2d ForceP;

    Eigen::Matrix2d B;
    Eigen::Matrix2d M;
    Eigen::Matrix2d Operator;
    VM2 X;
    VM2 dX;
    VM2 Fm;
    VM2 Vd;
};


/**
 * \brief Point to tpoint position control with min jerk trajectory interpolation.
 *
 */
class M2MinJerkPosition: public M2State {

   public:
    M2MinJerkPosition(RobotM2 *M2, StiffnessTest *_kt, const char *name = "M2 Minimum Jerk Position"):M2State(M2, _kt, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isTrialDone() {return trialDone;}

   private:
    bool trialDone=false;
    double startTime;
    VM2 Xi, Xf;
    double T;
    float k_i=1.; //Integral gain
};


/**
 * \brief Movement recording
 *
 */
class M2Recording : public M2State {

   public:
    M2Recording(RobotM2 *M2, StiffnessTest *_kt, const char *name = "M2 Recording State"):M2State(M2, _kt, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isRecordingDone() {return recordingDone;}
    bool isRecordingError() {return recordingError;}

   private:
    Eigen::Matrix2d ForceP;

    Eigen::Matrix2d B;
    Eigen::Matrix2d M;
    Eigen::Matrix2d Operator;
    VM2 X;
    VM2 dX;
    VM2 Fm;
    VM2 Vd;

    bool recordingDone=false;
    bool recordingError=false;

    int RecordingPoint;
    static const int MaxRecordingPts = 10100;
    VM2 PositionNow;
    VM2 PositionRecorded[MaxRecordingPts];

    int n;
    VM2 centroid;
    double Mxx, Myy, Mxy, Mxz, Myz, Mzz;
    double Xi, Yi, Zi;
    double Mz, Cov_xy, Mxz2, Myz2;
    double A2, A1, A0, A22;
    double epsilon;
    double ynew, yold, xnew, xold;
    int IterMax;
    double Dy, DET;
    VM2 Center;
    double radius;
    double start_angle;
    VM2 StartPt;
    //VM2 testing;
};


/**
 * \brief Movement testing
 *
 */
class M2ArcCircle : public M2State {

   public:
    M2ArcCircle(RobotM2 *M2, StiffnessTest *_kt, const char *name = "M2 Circle Test"):M2State(M2, _kt, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isTestingDone() {return testingDone;}
    bool isTestingError() {return testingError;}

   private:
    bool testingDone = false;
    bool testingError = false;
    bool movement_finished;
    double radius;
    double theta_s;
    double thetaRange;
    double theta;
    int sign;
    double dTheta_t; //Movement target velocity (max of profile) in deg.s-1
    double ddTheta=200; //in deg.s-2
    VM2 centerPt;
    VM2 startingPt;
    double t_init, t_end_accel, t_end_cstt, t_end_decel;
};


/**
 * \brief End-effector arc circle trajectory (position over velocity) back to starting point
 *
 */
class M2ArcCircleReturn : public M2State {

   public:
    M2ArcCircleReturn(RobotM2 *M2, StiffnessTest *_kt, const char *name = "M2 Circle Return"):M2State(M2, _kt, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isTestReturnDone() {return testReturnDone;}

   private:
    bool testReturnDone = false;
    bool movement_finished = true;
    double radius;
    double theta_s;
    double startReturnAngle;
    double thetaReturnRange;
    double thetaReturn;
    int sign;
    double dTheta_t; //Movement target velocity (max of profile) in deg.s-1
    double ddTheta=200; //in deg.s-2
    VM2 centerPt;
    VM2 startingReturnPt;
    double t_init, t_end_accel, t_end_cstt, t_end_decel;
};


/**
 * \brief Point to tpoint position control with min jerk trajectory interpolation.
 *
 */
class M2ConstForce: public M2State {

   public:
    M2ConstForce(RobotM2 *M2, StiffnessTest *_kt, const char *name = "M2 Constant Force Testing"):M2State(M2, _kt, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isConstFDone() {return constFDone;}

   private:
    bool constFDone=false;
    double elapsedT, duration;
    VM2 Fs, Vd;
    int i=0;
    double mvtDirAngle, mvtDirForce, mvtDirForceSum;
};


/**
 * \brief Stochastic Perturbation
 *
 */
class M2StochPert : public M2State {

   public:
    M2StochPert(RobotM2 *M2, StiffnessTest *_kt, const char *name = "Stochastic Perturbation"):M2State(M2, _kt, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isPertDone() {return pertDone;}

   private:
    LogHelper stateLogger;

    bool pertDone = false;

    double wait, duration, fs, fs2, fc, fc2;
    Eigen::Vector3d b, a, b2, a2;
    int filt_order, num_samples, order_samples, round;
    std::vector<double> white_noise, perturbation;
    double DocWhiteNoise, DocPerturbation;
    Eigen::Vector3d x_dX0, y_dX0, x_dX1, y_dX1;
    Eigen::Vector3d x_Fs0, y_Fs0, x_Fs1, y_Fs1;
    Eigen::Vector3d x_MDF, y_MDF, x_MDV, y_MDV;

    VM2 X;
    VM2 dX, dX_filt;
    VM2 Fs, Fs_filt;
    VM2 PertAmp, PertDest, Xi, X_orgn, stepDistance;
    double elapsedT=0, deltaT=0;
    int i=0, j=1;
    int step;

    VM2 Xd, dXd, Vd;
    double Theta;

    std::vector<double> mvtDirForceAbsVec;
    int mvtDirForceAbsSize;
    double mvtDirAngle, mvtDirForce, mvtDirForce_filt, mvtDirVelocity, mvtDirVelocity_filt, mvtDirVelocity_filt_ls, mvtDirAcc;
    double mvtDirForceRmI, mvtDirForceAbs, mvtDirForceAbsSum;
    double fixedI = 2.0;

};


#endif
