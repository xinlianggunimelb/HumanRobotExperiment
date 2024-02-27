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


using namespace std;


//Spasticity test variables
struct SpasticityTest
{
    VM2 global_center_point;
    VM2 global_start_point;
    double global_radius;
    double global_start_angle;
    int movement_loop = 0;
    int vel_sequence[9];
    double StateIndex = 0.;
    double AngularVelocity = 0.;
    bool goToTransparentFlag = false;
};



/**
 * \brief Generic state type for used with M2AssistRobot, providing running time and iterations number.
 *
 */
class M2State : public State {
   protected:
    RobotM2 *robot;                //!< Pointer to state machines robot object
    SpasticityTest * STest;

    M2State(RobotM2 *M2, SpasticityTest * _st, const char *name = NULL): State(name), robot(M2), STest(_st){};
};


class M2DemoState : public M2State {

   public:
    M2DemoState(RobotM2 *M2, SpasticityTest *_st, const char *name = "M2 Test State"):M2State(M2, _st, name){};

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
    M2Calib(RobotM2 *M2, SpasticityTest *_st, const char *name = "M2 Calib State"):M2State(M2, _st, name){};

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
    M2DemoImpedanceState(RobotM2 *M2, SpasticityTest *_st, const char *name = "M2 Demo Impedance State"):M2State(M2, _st, name){};

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
 * \brief Provide end-effector mass compensation on M2. Mass is controllable through keyboard inputs.
 *
 */
class M2Transparent : public M2State {

   public:
    M2Transparent(RobotM2 *M2, SpasticityTest *_st, const char *name = "M2 Transparent"):M2State(M2, _st, name){};

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
 * \brief Point to tpoint position control with min jerk trajectory interpolation
 *
 */
class M2MinJerkPosition: public M2State {

   public:
    M2MinJerkPosition(RobotM2 *M2, SpasticityTest *_st, const char *name = "M2 Demo Minimum Jerk Position"):M2State(M2, _st, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool GoToNextVel() {return goToNextVel;}
    bool isTrialDone() {return trialDone;}

   private:
    bool goToNextVel=false;
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
    M2Recording(RobotM2 *M2, SpasticityTest *_st, const char *name = "M2 Recording State"):M2State(M2, _st, name){};

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
class M2CircleTest : public M2State {

   public:
    M2CircleTest(RobotM2 *M2, SpasticityTest *_st, const char *name = "M2 Circle Test"):M2State(M2, _st, name){};

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
 * \brief End-effector arc circle trajectory (position over velocity)
 *
 */
class M2ArcCircle : public M2State {

   public:
    M2ArcCircle(RobotM2 *M2, SpasticityTest *_st, const char *name = "M2 Arc Circle"):M2State(M2, _st, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool GoToStartPt() {return goToStartPt;}

   private:
    bool movement_finished;
    bool goToStartPt=false;

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
    double ang_vel[9] = {10, 20, 30, 40, 50, 60, 70, 80, 90};

    Eigen::Matrix2d B;
    Eigen::Matrix2d M;
    Eigen::Matrix2d Operator;
    VM2 X;
    VM2 dX;
    VM2 Fm;
    VM2 Vd;
};


/**
 * \brief End-effector arc circle trajectory (position over velocity) back to starting point
 *
 */
class M2ArcCircleReturn : public M2State {

   public:
    M2ArcCircleReturn(RobotM2 *M2, SpasticityTest *_st, const char *name = "M2 Arc Circle Return"):M2State(M2, _st, name){};

    void entry(void);
    void during(void);
    void exit(void);

   private:
    bool finished;
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
 * \brief EMD Test Static
 *
 */
class M2EMDtest1 : public M2State {

   public:
    M2EMDtest1(RobotM2 *M2, SpasticityTest *_st, const char *name = "M2 EMD Test Static"):M2State(M2, _st, name){};

    void entry(void);
    void during(void);
    void exit(void);

   private:
    double startTime;
    double radius;
    double theta_s;
    double theta_d;
    double theta;
    int sign;
    VM2 centerPt;
    VM2 startingPt;
    VM2 Xi, Xf;
    double T;
    float k_i=1.; //Integral gain
};


/**
 * \brief EMD Test Voluntary
 */
class M2EMDtest2 : public M2State {

   public:
    M2EMDtest2(RobotM2 *M2, SpasticityTest *_st, const char *name = "EMD Test Voluntary"):M2State(M2, _st, name){};

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
 * \brief EMD Test Passive Extention
 *
 */
class M2EMDtest3EXT : public M2State {

   public:
    M2EMDtest3EXT(RobotM2 *M2, SpasticityTest *_st, const char *name = "EMD Test Passive Extention"):M2State(M2, _st, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isExtentionDone() {return extentionDone;}

   private:
    bool extentionDone = false;
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
 * \brief EMD Test Passive Flexion
 *
 */
class M2EMDtest3FLX : public M2State {

   public:
    M2EMDtest3FLX(RobotM2 *M2, SpasticityTest *_st, const char *name = "EMD Test Passive Flexion"):M2State(M2, _st, name){};

    void entry(void);
    void during(void);
    void exit(void);

    bool isFlexionDone() {return flexionDone;}

   private:
    bool flexionDone = false;
    bool finished;
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


#endif
