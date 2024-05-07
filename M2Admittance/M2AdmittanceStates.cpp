#include "M2AdmittanceStates.h"
using namespace Eigen;

int EveryN=1;//Process every N samples: reduce frequency

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

VM2 myVE(VM2 X, VM2 dX, VM2 Fm, Eigen::Matrix2d realB, Eigen::Matrix2d realM, double dt)
{
    Eigen::Matrix2d Operator;
    Operator(0,0) = 1/(realM(0,0) + realB(0,0)*dt);
    Operator(1,1) = 1/(realM(1,1) + realB(1,1)*dt);

    return Operator*(Fm*dt + realM*dX);
}

VM2 PsvObsv(VM2 E_obs_ls, VM2 Fm, VM2 Fm_ls, VM2 V_ve, Eigen::Matrix2d C_diss_ls, double dt)
{
    VM2 E_obs;
    E_obs = E_obs_ls + V_ve.cwiseProduct(Fm)*dt + C_diss_ls*Fm_ls.cwiseProduct(Fm_ls)*dt;
    return E_obs;
}

VM2 PsvObsvSimple(VM2 E_obs_ls, VM2 Fm, VM2 dX, double dt)
{
    VM2 E_obs;
    E_obs = E_obs_ls + dX.cwiseProduct(Fm)*dt;
    return E_obs;
}

VM2 PsvObsvST(VM2 E_obs_ls, VM2 dX, VM2 Fm, double dt)
{
    VM2 E_obs;
    E_obs = E_obs_ls + dX.cwiseProduct(Fm)*dt;
    return E_obs;
}

VM2 PsvObsvSTprediction(VM2 E_obs, VM2 Fm, VM2 V_ve, double dt)
{
    VM2 E_obs_prediction;
    E_obs_prediction = E_obs + V_ve.cwiseProduct(Fm)*dt;
    return E_obs_prediction;
}

VM2 EffDamp(Eigen::Matrix2d M, VM2 dX, VM2 Fm, VM2 Vd, double dt)
{
    //Effective damping calculation
    VM2 Acc, B_e;
    B_e(0) = (Fm(0) - M(0,0)*((Vd(0)-dX(0))/dt)) / Vd(0);
    B_e(1) = (Fm(1) - M(1,1)*((Vd(1)-dX(1))/dt)) / Vd(1);
    return B_e;
}

VM2 impedance(Eigen::Matrix2d K, Eigen::Matrix2d D, VM2 X0, VM2 X, VM2 dX, VM2 dXd=VM2::Zero()) {
    return K*(X0-X) + D*(dXd-dX); //K is equivlent to the P gain in position control, D is equivlent to the D gain in position control the reference point is the messured velocity
}

//minJerk(X0, Xf, T, t, &X, &dX)
double JerkIt(VM2 X0, VM2 Xf, double T, double t, VM2 &Xd, VM2 &dXd) {
    t = std::max(std::min(t, T), .0); //Bound time
    double tn=std::max(std::min(t/T, 1.0), .0);//Normalised time bounded 0-1
    double tn3=pow(tn,3.);
    double tn4=tn*tn3;
    double tn5=tn*tn4;
    Xd = X0 + ( (X0-Xf) * (15.*tn4-6.*tn5-10.*tn3) );
    dXd = (X0-Xf) * (4.*15.*tn4-5.*6.*tn5-10.*3*tn3)/t;
    return tn;
}


void M2Calib::entry(void) {
    calibDone=false;
    for(unsigned int i=0; i<2; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    robot->decalibrate();
    robot->initTorqueControl();
   // robot -> printStatus();
    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}
//Move slowly on each joint until max force detected
void M2Calib::during(void) {
    VM2 tau(0, 0);

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    VM2 vel=robot->getVelocity();
    double b = 3;
    for(int i=0; i<vel.size(); i++) {
        tau(i) = -std::min(std::max(20 - b * vel(i), .0), 20.);
        if(stop_reached_time(i)>1) {
            at_stop[i]=true;
        }
        if(abs(vel(i))<0.005) {
            stop_reached_time(i) += dt();
        }
    }

    //Switch to gravity control when done
    if(robot->isCalibrated()) {
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        calibDone=true; //Trigger event
    }
    else {
        //If all joints are calibrated
        if(at_stop[0] && at_stop[1]) {
            robot->applyCalibration();
            std::cout << "OK." << std::endl;
        }
        else {
            robot->setJointTorque(tau);
            if(iterations()%100==1) {
                std::cout << "." << std::flush;
            }
        }
    }
}
void M2Calib::exit(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}


//M2 Velocity Loop Testing
void M2Transparent::entry(void) {
    robot->initTorqueControl();
    /*ForceP(0,0) = 1.3;
    ForceP(1,1) = 1.4;*/
    //robot->initVelocityControl();
    std::cout << "Test Loop" << std::endl;

    Fd = VM2(0,0);
    //dXd = VM2(0,0);
    amplitude = 1.0;


    stateLogger.initLogger("M2Transparent", "logs/M2VelocityLoop.csv", LogFormat::CSV, true);
    stateLogger.add(running(), "%Time (s)");
    stateLogger.add(robot->getEndEffPosition(), "Position");
    stateLogger.add(robot->getEndEffVelocity(), "Velocity");
    stateLogger.add(robot->getInteractionForce(), "Force");
    //stateLogger.add(dXd(0), "Desired_velocity_X");
    stateLogger.add(Fd(0), "Desired_force_X");
    stateLogger.startLogger();
}
void M2Transparent::during(void) {

    //Smooth transition in case a mass is set at startup
    //double settling_time = 3.0;
    //double t=running()>settling_time?1.0:running()/settling_time;

    /*//Apply corresponding force
    VM2 f_m = robot->getInteractionForce();
    robot->setEndEffForce(ForceP*f_m);
    */

    if(iterations()%100==1) {
        robot->printStatus();
    }
    Fd(0) = amplitude*sin(100.*running());
    //dXd(0) = amplitude*sin(15.*running());

    //Apply force
    //robot->setEndEffForce(Fd);
    robot->setEndEffForceWithCompensation(Fd, false);
    //robot->setEndEffVelocity(dXd);

    stateLogger.recordLogData();

    //Read keyboard inputs to change gain
    if(robot->keyboard->getS()) {
        amplitude-=1.0;
        std::cout << amplitude <<std::endl;
    }
    if(robot->keyboard->getW()) {
        amplitude+=1.0;
        std::cout << amplitude <<std::endl;
    }
}
void M2Transparent::exit(void) {
    robot->setEndEffForce(VM2::Zero());
}


void M2MinJerkPosition::entry(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    goToNextVel=false;
    trialDone=false;

    startTime=running();
    Xi = robot->getEndEffPosition();
    VM2 destination;
    destination(0)=0.1;
    destination(1)=0.2;
    Xf = destination;
    T=3; //Trajectory Time
    k_i=1.;
}
void M2MinJerkPosition::during(void) {
    VM2 Xd, dXd;
    //Compute current desired interpolated point
    double status=JerkIt(Xi, Xf, T, running()-startTime, Xd, dXd);
    //Apply position control
    robot->setEndEffVelocity(dXd+k_i*(Xd-robot->getEndEffPosition()));

    if(iterations()%10==1) {
        robot->printStatus();
    }
}
void M2MinJerkPosition::exit(void) {
    robot->setJointVelocity(VM2::Zero());
}


// M2Admittance1: PO Only
void M2Admittance1::entry(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    //Virtual Environment
    M(0,0) = M(1,1) = 0.24;
    B(0,0) = B(1,1) = 0.08;
    //Damper for dissipation
    C_diss(0,0) = C_diss(1,1) = 0.0;
    C_diss_ls(0,0) = C_diss_ls(1,1) = 0.0;

    Obsv_T = 10000000; //an arbitrary large value
    i = 0;

    E_class(0) = E_class(1) = 0.0;
    E_diss(0) = E_diss(1) = 0.0;

    //E_obs(0) = E_obs(1) = 0.0;
    E_obs(0) = E_obs(1) = E_class(0) + 0.1;
    E_obs_ls(0) = E_obs_ls(1) = E_class(0) + 0.1;

    X(VM2::Zero());
    dX(VM2::Zero());
    Fs(VM2::Zero());
    Fs_ls(VM2::Zero());
    Vd(VM2::Zero());
    V_ve(VM2::Zero());
    V_diss(VM2::Zero());

    stateLogger.initLogger("M2Admittance1State", "logs/M2Admittance1State.csv", LogFormat::CSV, true);
    stateLogger.add(elapsedT, "%Time (s)");
    stateLogger.add(X, "Position");
    stateLogger.add(dX, "Velocity");
    stateLogger.add(Fs, "Force");
    stateLogger.add(B(0,0), "Virtual_damping_X");
    //stateLogger.add(B(1,1), "Virtual_damping_Y");
    stateLogger.add(M(0,0), "Virtual_mass_X");
    stateLogger.add(E_class, "Energy_threshold");
    stateLogger.add(E_obs, "Observed_Energy");
    stateLogger.add(E_diss, "Dissipation_Energy");
    stateLogger.add(V_ve(0), "VE_velocity_X");
    stateLogger.add(V_diss(0), "Diss_velocity_X");
    stateLogger.add(Vd(0), "Desired_velocity_X");
    stateLogger.add(C_diss(0,0), "1/b_X");
    stateLogger.add(E_obs_prediction, "Predicted_Energy");
    stateLogger.add(B_eff(0), "Effective_damping_X");
    stateLogger.startLogger();
}
void M2Admittance1::during(void) {
    //VM2 X, dX, Fm, Vd;
    //get robot position and velocity and force mesaure
    X = robot->getEndEffPosition();
    dX = robot->getEndEffVelocity();
    Fs = robot->getInteractionForce();
    elapsedT = running();
    deltaT = dt();

    //Change Virtual Damping
    if(robot->keyboard->getQ()) {
        B(0,0)+=0.01;
        B(1,1)+=0.01;
        std::cout << B <<std::endl;
    }
    if(robot->keyboard->getA()) {
        B(0,0)-=0.01;
        B(1,1)-=0.01;
        std::cout << B <<std::endl;
    }
    //Change Virtual Mass
    if(robot->keyboard->getW()) {
        M(0,0)+=0.01;
        M(1,1)+=0.01;
        std::cout << M <<std::endl;
    }
    if(robot->keyboard->getS()) {
        M(0,0)-=0.01;
        M(1,1)-=0.01;
        std::cout << M <<std::endl;
    }


    V_ve = myVE(X, dX, Fs, B, M, deltaT);


    // Passivity Observer
    if (i < Obsv_T) {
        E_obs = PsvObsvST(E_obs_ls, dX, Fs, deltaT);
        E_obs_prediction = PsvObsvSTprediction(E_obs, Fs, V_ve, deltaT);
        //E_obs = PsvObsv(E_obs_ls, Fs, Fs_ls, V_ve, C_diss_ls, deltaT);
        //E_obs = PsvObsvSimple(E_obs_ls, Fs, dX, deltaT);
        i += 1;
    }
    if (i >= Obsv_T) {
        i = 0;
        //E_obs(VM2::Zero());
        E_obs(0) = E_obs(1) = 0;
        std::cout << "PO is reseted" << std::endl;
    }


    Vd = V_ve; //PO Only - Testing PO
    Vd(1) = 0.0; //lock y axis


    B_eff = EffDamp(M, dX, Fs, Vd, deltaT);


    E_obs_ls = E_obs;
    Fs_ls = Fs;
    C_diss_ls = C_diss;


    stateLogger.recordLogData();
    if(iterations()%10==1) {
        //robot->printStatus();
        //std::cout << "i=[ " << i << " ]\t" ;
        std::cout << "Mass_X=[ " << M(0,0) << " ]\t" ;
        std::cout << "Damping_X=[ " << B(0,0) << " ]\t" ;
        //std::cout << "Damping_Y=[ " << B(1,1) << " ]\t" <<std::endl;
        //std::cout << "E_class=[ " << E_class.transpose() << " ]\t" ;
        //std::cout << "Energy=[ " << E_obs.transpose() << " ]\t" ;
        std::cout << "Energy=[ " << E_obs_prediction.transpose() << " ]\t" ;
        //std::cout << "E_diss=[ " << E_diss.transpose() << " ]\t" ;
        //std::cout << "V_ve=[ " << V_ve.transpose() << " ]\t" ;
        //std::cout << "V_diss=[ " << V_diss.transpose() << " ]\t" ;
        std::cout << "C_X=[ " << C_diss(0,0) << " ]\t" <<std::endl;
    }

    //apply velocity
    robot->setEndEffVelocity(Vd);
}
void M2Admittance1::exit(void) {
    robot->setJointVelocity(VM2::Zero());
}


// M2Admittance2: Classic PO-PC
void M2Admittance2::entry(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    //Virtual Environment
    M(0,0) = M(1,1) = 0.24;
    B(0,0) = B(1,1) = 0.08;
    //Damper for dissipation
    C_diss(0,0) = C_diss(1,1) = 0.0;
    C_diss_ls(0,0) = C_diss_ls(1,1) = 0.0;

    Obsv_T = 10000000; //an arbitrary large value
    i = 0;

    E_class(0) = E_class(1) = 0.0;
    E_diss(0) = E_diss(1) = 0.0;

    //E_obs(0) = E_obs(1) = 0.0;
    E_obs(0) = E_obs(1) = E_class(0) + 0.1;
    E_obs_ls(0) = E_obs_ls(1) = E_class(0) + 0.1;

    X(VM2::Zero());
    dX(VM2::Zero());
    Fs(VM2::Zero());
    Fs_ls(VM2::Zero());
    Vd(VM2::Zero());
    V_ve(VM2::Zero());
    V_diss(VM2::Zero());

    limit = 0.1;

    stateLogger.initLogger("M2Admittance2State", "logs/M2Admittance2State.csv", LogFormat::CSV, true);
    stateLogger.add(elapsedT, "%Time (s)");
    stateLogger.add(X, "Position");
    stateLogger.add(dX, "Velocity");
    stateLogger.add(Fs, "Force");
    stateLogger.add(B(0,0), "Virtual_damping_X");
    //stateLogger.add(B(1,1), "Virtual_damping_Y");
    stateLogger.add(M(0,0), "Virtual_mass_X");
    stateLogger.add(E_class, "Energy_threshold");
    stateLogger.add(E_obs, "Observed_Energy");
    stateLogger.add(E_diss, "Dissipation_Energy");
    stateLogger.add(V_ve(0), "VE_velocity_X");
    stateLogger.add(V_diss(0), "Diss_velocity_X");
    stateLogger.add(Vd(0), "Desired_velocity_X");
    stateLogger.add(C_diss(0,0), "1/b_X");
    stateLogger.add(E_obs_prediction, "Predicted_Energy");
    stateLogger.add(B_eff(0), "Effective_damping_X");
    stateLogger.startLogger();
}
void M2Admittance2::during(void) {
    //VM2 X, dX, Fm, Vd;
    //get robot position and velocity and force mesaure
    X = robot->getEndEffPosition();
    dX = robot->getEndEffVelocity();
    Fs = robot->getInteractionForce();
    elapsedT = running();
    deltaT = dt();

    //Change Virtual Damping
    if(robot->keyboard->getQ()) {
        B(0,0)+=0.01;
        B(1,1)+=0.01;
        std::cout << B <<std::endl;
    }
    if(robot->keyboard->getA()) {
        B(0,0)-=0.01;
        B(1,1)-=0.01;
        std::cout << B <<std::endl;
    }
    //Change Virtual Mass
    if(robot->keyboard->getW()) {
        M(0,0)+=0.01;
        M(1,1)+=0.01;
        std::cout << M <<std::endl;
    }
    if(robot->keyboard->getS()) {
        M(0,0)-=0.01;
        M(1,1)-=0.01;
        std::cout << M <<std::endl;
    }


    V_ve = myVE(X, dX, Fs, B, M, deltaT);


    // Passivity Observer
    if (i < Obsv_T) {
        E_obs = PsvObsvST(E_obs_ls, dX, Fs, deltaT);
        E_obs_prediction = PsvObsvSTprediction(E_obs, Fs, V_ve, deltaT);
        //E_obs = PsvObsv(E_obs_ls, Fs, Fs_ls, V_ve, C_diss_ls, deltaT);
        //E_obs = PsvObsvSimple(E_obs_ls, Fs, dX, deltaT);
        i += 1;
    }
    if (i >= Obsv_T) {
        i = 0;
        //E_obs(VM2::Zero());
        E_obs(0) = E_obs(1) = 0;
        std::cout << "PO is reseted" << std::endl;
    }


    //E_class triggered
    /*
    if (E_obs(0) < E_class(0)) {
        E_diss(0) = E_class(0) - E_obs(0); //energy to be dissipated
        C_diss(0,0) = E_diss(0) / (Fs(0)*Fs(0)*dt());
    }
    else {
        E_diss(0) = 0.0;
        C_diss(0,0) = 0.0;
    }
    if (E_obs(1) < E_class(1)) {
        E_diss(1) = E_class(1) - E_obs(1); //energy to be dissipated
        C_diss(1,1) = E_diss(1) / (Fs(1)*Fs(1)*dt());
    }
    else {
        E_diss(1) = 0.0;
        C_diss(1,1) = 0.0;
    }
    */
    //update for the new PO
    if (E_obs_prediction(0) < E_class(0)) {
        E_diss(0) = E_class(0) - E_obs_prediction(0); //energy to be dissipated
        C_diss(0,0) = E_diss(0) / (Fs(0)*Fs(0)*dt());
    }
    else {
        E_diss(0) = 0.0;
        C_diss(0,0) = 0.0;
    }

    V_diss(0) = C_diss(0,0)*Fs(0);
    V_diss(1) = C_diss(1,1)*Fs(1);

    Vd = V_ve + V_diss;
    Vd(1) = 0.0; //lock y axis


    //Saturate velocity when E threshold triggered
    if (V_diss(0) != 0.0){
        Vd(0) = fmax(fmin(Vd(0), limit), -limit);
    }


    B_eff = EffDamp(M, dX, Fs, Vd, deltaT);


    E_obs_ls = E_obs;
    Fs_ls = Fs;
    C_diss_ls = C_diss;


    stateLogger.recordLogData();
    if(iterations()%10==1) {
        //robot->printStatus();
        //std::cout << "i=[ " << i << " ]\t" ;
        std::cout << "Mass_X=[ " << M(0,0) << " ]\t" ;
        std::cout << "Damping_X=[ " << B(0,0) << " ]\t" ;
        //std::cout << "Damping_Y=[ " << B(1,1) << " ]\t" <<std::endl;
        //std::cout << "E_class=[ " << E_class.transpose() << " ]\t" ;
        //std::cout << "Energy=[ " << E_obs.transpose() << " ]\t" ;
        std::cout << "Energy=[ " << E_obs_prediction.transpose() << " ]\t" ;
        //std::cout << "E_diss=[ " << E_diss.transpose() << " ]\t" ;
        //std::cout << "V_ve=[ " << V_ve.transpose() << " ]\t" ;
        //std::cout << "V_diss=[ " << V_diss.transpose() << " ]\t" ;
        std::cout << "C_X=[ " << C_diss(0,0) << " ]\t" <<std::endl;
    }

    //apply velocity
    robot->setEndEffVelocity(Vd);
}
void M2Admittance2::exit(void) {
    robot->setJointVelocity(VM2::Zero());
}


// M2Admittance3: Ultimate Passivity
void M2Admittance3::entry(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    //Virtual Environment
    M(0,0) = M(1,1) = 0.24;
    B(0,0) = B(1,1) = 0.08;

    //B1(0,0) = B1(1,1) = 1.2;
    //B1(0,0) = B1(1,1) = 0.0;
    B2(0,0) = B2(1,1) = 50.0;

    //Damper for dissipation
    C_diss(0,0) = C_diss(1,1) = 0.0;
    C_diss_ls(0,0) = C_diss_ls(1,1) = 0.0;

    E_obs(0) = E_obs(1) = 0.0 + 0.1;
    E_obs_ls(0) = E_obs_ls(1) = 0.0 + 0.1;

    Obsv_T = 10000000;
    i = 0;

    X(VM2::Zero());
    dX(VM2::Zero());
    Fs(VM2::Zero());
    Fs_ls(VM2::Zero());
    Vd(VM2::Zero());
    V_ve(VM2::Zero());
    V_diss(VM2::Zero());

    E_lower(0) = E_lower(1) = -2.0;
    E_upper(0) = E_upper(1) = 0.1;

    limit = 0.1;

    stateLogger.initLogger("M2Admittance3State", "logs/M2Admittance3State.csv", LogFormat::CSV, true);
    stateLogger.add(elapsedT, "%Time (s)");
    stateLogger.add(X, "Position");
    stateLogger.add(dX, "Velocity");
    stateLogger.add(Fs, "Force");
    stateLogger.add(B(0,0), "Virtual_damping_X");
    //stateLogger.add(B(1,1), "Virtual_damping_Y");
    stateLogger.add(M(0,0), "Virtual_mass_X");
    stateLogger.add(E_upper, "Energy_threshold_upper");
    stateLogger.add(E_lower, "Energy_threshold_lower");
    stateLogger.add(E_obs, "Observed_Energy");
    stateLogger.add(V_ve(0), "VE_velocity_X");
    stateLogger.add(V_diss(0), "Diss_velocity_X");
    stateLogger.add(Vd(0), "Desired_velocity_X");
    stateLogger.add(C_diss(0,0), "1/b_X");
    stateLogger.add(E_obs_prediction, "Predicted_Energy");
    stateLogger.add(B_eff(0), "Effective_damping_X");
    stateLogger.startLogger();
}
void M2Admittance3::during(void) {
    //get robot position and velocity and force mesaure
    X = robot->getEndEffPosition();
    dX = robot->getEndEffVelocity();
    Fs = robot->getInteractionForce();
    elapsedT = running();
    deltaT = dt();

    //Change E_UltPsv upper threshold
    if(robot->keyboard->getQ()) {
        E_upper(0)+=0.1;
        E_upper(1)+=0.1;
        std::cout << E_upper <<std::endl;
    }
    if(robot->keyboard->getA()) {
        E_upper(0)-=0.1;
        E_upper(1)-=0.1;
        std::cout << E_upper <<std::endl;
    }
    //Change E_UltPsv lower threshold
    if(robot->keyboard->getW()) {
        E_lower(0)+=0.1;
        E_lower(1)+=0.1;
        std::cout << E_lower <<std::endl;
    }
    if(robot->keyboard->getS()) {
        E_lower(0)-=0.1;
        E_lower(1)-=0.1;
        std::cout << E_lower <<std::endl;
    }


    V_ve = myVE(X, dX, Fs, B, M, deltaT);


    // Passivity Observer
    if (i < Obsv_T) {
        E_obs = PsvObsvST(E_obs_ls, dX, Fs, deltaT);
        E_obs_prediction = PsvObsvSTprediction(E_obs, Fs, V_ve, deltaT);
        //E_obs = PsvObsv(E_obs_ls, Fs, Fs_ls, V_ve, C_diss_ls, deltaT);
        //E_obs = PsvObsvSimple(E_obs_ls, Fs, dX, deltaT);
        i += 1;
    }
    if (i >= Obsv_T) {
        i = 0;
        //E_obs(VM2::Zero());
        E_obs(0) = E_obs(1) = 0;
        std::cout << "PO is reseted" << std::endl;
    }


    //Switching Law
    /*
    if (E_obs(0) <= E_lower(0)) {
        C_diss(0,0) = 1.0 / B2(0,0);
    }
    else if (E_obs(0) >= E_upper(0)) {
        C_diss(0,0) = 0.0;
    }
    else {
        C_diss(0,0) = C_diss(0,0);
    }

    if (E_obs(1) <= E_lower(1)) {
        C_diss(1,1) = 1.0 / B2(1,1);
    }
    else if (E_obs(1) >= E_upper(1)) {
        C_diss(1,1) = 0.0;
    }
    else {
        C_diss(1,1) = C_diss(1,1);
    }
    */
    //update for the new PO
    if (E_obs_prediction(0) <= E_lower(0)) {
        C_diss(0,0) = 1.0 / B2(0,0);
    }
    else if (E_obs_prediction(0) >= E_upper(0)) {
        C_diss(0,0) = 0.0;
    }
    else {
        C_diss(0,0) = C_diss_ls(0,0);
    }

    V_diss(0) = C_diss(0,0)*Fs(0);
    V_diss(1) = C_diss(1,1)*Fs(1);


    Vd = V_ve + V_diss;
    Vd(1) = 0.0; //lock y axis


    //Saturate velocity when E threshold triggered
    if (V_diss(0) != 0.0){
        Vd(0) = fmax(fmin(Vd(0), limit), -limit);
    }


    B_eff = EffDamp(M, dX, Fs, Vd, deltaT);


    E_obs_ls = E_obs;
    Fs_ls = Fs;
    C_diss_ls = C_diss;


    stateLogger.recordLogData();
    if(iterations()%10==1) {
        //robot->printStatus();
        std::cout << "Mass_X=[ " << M(0,0) << " ]\t" ;
        std::cout << "Damping_X=[ " << B(0,0) << " ]\t" ;
        //std::cout << "Upper_E_X=[ " << E_upper(0) << " ]\t" ;
        //std::cout << "Lower_E_X=[ " << E_lower(0) << " ]\t" ;
        std::cout << "Energy=[ " << E_obs_prediction.transpose() << " ]\t" ;
        //std::cout << "Damping_x=[ " << B(0,0) << " ]\t" ;
        //std::cout << "Damping_y=[ " << B(1,1) << " ]\t" <<std::endl;
        std::cout << "V_ve=[ " << V_ve.transpose() << " ]\t" ;
        std::cout << "V_diss=[ " << V_diss.transpose() << " ]\t" ;
        std::cout << "Damping_X=[ " << C_diss(0,0) << " ]\t" <<std::endl;
    }

    //apply velocity
    robot->setEndEffVelocity(Vd);
}
void M2Admittance3::exit(void) {
    robot->setJointVelocity(VM2::Zero());
}


// M2Admittance4: CORC-PID
void M2Admittance4::entry(void) {
    //Setup velocity control for position over velocity loop
    /*
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    */
    robot->initTorqueControl();

    //Virtual Environment
    M(0,0) = M(1,1) = 0.2;
    B(0,0) = B(1,1) = 0.1;
    //Damper for dissipation
    C_diss(0,0) = C_diss(1,1) = 0.0;
    C_diss_ls(0,0) = C_diss_ls(1,1) = 0.0;

    Obsv_T = 10000000; //an arbitrary large value
    i = 0;

    E_class(0) = E_class(1) = 0.0;
    E_diss(0) = E_diss(1) = 0.0;

    //E_obs(0) = E_obs(1) = 0.0;
    E_obs(0) = E_obs(1) = E_class(0) + 2.0;
    E_obs_ls(0) = E_obs_ls(1) = E_class(0) + 2.0;

    X(VM2::Zero());
    dX(VM2::Zero());
    Fs(VM2::Zero());
    Fs_ls(VM2::Zero());
    Vd(VM2::Zero());
    V_ve(VM2::Zero());
    V_diss(VM2::Zero());

    //Added for CORC-PID
    Fd(VM2::Zero());
    V_error(VM2::Zero());
    k_p = 200.0;
    Kp = k_p*Matrix2d::Identity();
    amplitude = 0.2;

    //Friction Compensation
    alpha = 1.68;
    beta = 70.0;
    threshold = 0.;

    stateLogger.initLogger("M2Admittance4State", "logs/M2Admittance4State.csv", LogFormat::CSV, true);
    stateLogger.add(running(), "%Time (s)");
    stateLogger.add(robot->getEndEffPosition(), "Position");
    stateLogger.add(robot->getEndEffVelocity(), "Velocity");
    stateLogger.add(robot->getInteractionForce(), "Interaction Force");
    stateLogger.add(B(0,0), "Virtual_damping_X");
    //stateLogger.add(B(1,1), "Virtual_damping_Y");
    stateLogger.add(M(0,0), "Virtual_mass_X");
    stateLogger.add(E_class, "Energy_threshold");
    stateLogger.add(E_obs, "Observed_Energy");
    stateLogger.add(E_diss, "Dissipation_Energy");
    stateLogger.add(V_ve(0), "VE_velocity_X");
    stateLogger.add(V_diss(0), "Diss_velocity_X");
    stateLogger.add(Vd(0), "Desired_velocity_X");
    stateLogger.add(C_diss(0,0), "1/b_X");
    stateLogger.add(E_obs_prediction, "Predicted_Energy");
    stateLogger.add(Fd(0), "Desired_force_X");
    stateLogger.add(V_error(0), "Velocity_error_X");
    stateLogger.add(k_p, "Gain");
    stateLogger.add(robot->getEndEffForce(), "Force");
    stateLogger.startLogger();
}
void M2Admittance4::during(void) {
    //VM2 X, dX, Fs, Vd;
    //get robot position and velocity and force mesaure
    X = robot->getEndEffPosition();
    dX = robot->getEndEffVelocity();
    Fs = robot->getInteractionForce();

    //Change P-Controller Gain
    if(robot->keyboard->getQ()) {
        k_p+=2.0;
        std::cout << k_p <<std::endl;
    }
    if(robot->keyboard->getA()) {
        k_p-=2.0;
        std::cout << k_p <<std::endl;
    }
    Kp = k_p*Matrix2d::Identity();

    /*
    //Change Sine Vd Amplitude
    if(robot->keyboard->getW()) {
        amplitude+=0.1;
        std::cout << amplitude <<std::endl;
    }
    if(robot->keyboard->getS()) {
        amplitude-=0.1;
        std::cout << amplitude <<std::endl;
    }
    */

    //Change beta
    if(robot->keyboard->getW()) {
        beta+=5;
        std::cout << beta <<std::endl;
    }
    if(robot->keyboard->getS()) {
        beta-=5;
        std::cout << beta <<std::endl;
    }


    //V_ve = myVE(X, dX, Fs, B, M, dt());
    //Vd = V_ve; //PO Only - Testing PO

    //Cst sine
    //Vd(0) = amplitude*sin(5.*running()); //sine wave

    //Chirp cmd
    //Vd(0) = amplitude*(1+0.01*running())*(sin((1+0.1*running())*4.*running()));

    //Square wave
    //Vd(0) = amplitude*(sin(4.*running())>=0.0?1.0:-1.0); //square wave

    if(running()<5.){
        Vd(0) = amplitude*(sin(4.*running())>=0.0?1.0:-1.0); //square wave
    } else{
        Vd(0) = amplitude*(1+0.01*running())*(sin((1+0.1*running())*4.*running()));
    }


    Vd(1) = 0.0; //lock y axis


    //Added for CORC-PID
    V_error = Vd - dX;
    Fd = Kp * V_error;


    //Friction Compensation
    tau_fc(VM2::Zero());
    dq = 0.;
    dq = dX(0);//Vd(0)
    if (abs(dq) > threshold) {
        tau_fc(0) = alpha * sign(dq) + beta * dq;
    } else {
        tau_fc(0) = .0;
    }
    Fd+=tau_fc;


    stateLogger.recordLogData();
    if(iterations()%100==1) {
        robot->printStatus();
        //std::cout << "i=[ " << i << " ]\t" ;
        std::cout << "K=[ " << k_p << " ]\t" ;
        std::cout << "beta=[ " << beta << " ]\t" ;
        std::cout << "V_error=[ " << V_error(0) << " ]\t" <<std::endl;
        //std::cout << "Mass_X=[ " << M(0,0) << " ]\t" ;
        //std::cout << "Damping_X=[ " << B(0,0) << " ]\t" ;
        //std::cout << "Damping_Y=[ " << B(1,1) << " ]\t" <<std::endl;
        //std::cout << "E_class=[ " << E_class.transpose() << " ]\t" ;
        //std::cout << "Energy=[ " << E_obs.transpose() << " ]\t" ;
        //std::cout << "Energy=[ " << E_obs_prediction.transpose() << " ]\t" ;
        //std::cout << "E_diss=[ " << E_diss.transpose() << " ]\t" ;
        //std::cout << "V_ve=[ " << V_ve.transpose() << " ]\t" ;
        //std::cout << "V_diss=[ " << V_diss.transpose() << " ]\t" ;
        //std::cout << "C_X=[ " << C_diss(0,0) << " ]\t" <<std::endl;
    }

    /*
    //apply velocity
    robot->setEndEffVelocity(Vd);
    */
    //Apply force
    robot->setEndEffForceWithCompensation(Fd, false);
}
void M2Admittance4::exit(void) {
    //robot->setJointVelocity(VM2::Zero());
    robot->setEndEffForce(VM2::Zero());
}


// M2Admittance5: Energy Tank
void M2Admittance5::entry(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    //Virtual Environment
    M(0,0) = M(1,1) = 0.24;
    B(0,0) = B(1,1) = 0.08;
    Rd = B(0,0)/M(0,0);
    //scale = 3.0; //For M2 Characterisation
    scale = 1.0; //For Energy Tank
    M(0,0) = M(0,0)*scale; M(1,1) = M(1,1)*scale;
    B(0,0) = B(0,0)*scale; B(1,1) = B(1,1)*scale;

    M_upd_ls(0,0) = M_upd_ls(1,1) = M(0,0);
    M_upd_2ls(0,0) = M_upd_2ls(1,1) = M(0,0);
    B_upd_ls(0,0) = B_upd_ls(1,1) = B(0,0);

    //B1(0,0) = B1(1,1) = 0.0;
    //B2(0,0) = B2(1,1) = 50.0;

    //Damper for dissipation
    //C_diss(0,0) = C_diss(1,1) = 0.0;
    //C_diss_ls(0,0) = C_diss_ls(1,1) = 0.0;

    E_obs(0) = E_obs(1) = 0.0 + 0.1;
    E_obs_ls(0) = E_obs_ls(1) = 0.0 + 0.1;

    Obsv_T = 10000000;
    i = 0;

    X(VM2::Zero());
    dX(VM2::Zero());
    Fs(VM2::Zero());
    Fs_ls(VM2::Zero());
    Vd(VM2::Zero());
    Vd_ls(VM2::Zero());
    V_ve(VM2::Zero());
    //V_diss(VM2::Zero());

    t_start = running();
    V_max = 1.76;
    A_max = 56.85;
    detect_threshold = 3.7;
    k = 0;
    p = 0;

    Tank_level(0) = Tank_level(1) = E_obs(0);
    Tank_lower(0) = Tank_lower(1) = 0.05;
    Tank_upper(0) = Tank_upper(1) = 0.2;

    M_deltamax = 0.5;
    forgetting = 0.1;

    limit = 0.1;

    stateLogger.initLogger("M2Admittance5State", "logs/M2Admittance5State.csv", LogFormat::CSV, true);
    stateLogger.add(elapsedT, "%Time (s)");
    stateLogger.add(X, "Position");
    stateLogger.add(dX, "Velocity");
    stateLogger.add(Fs, "Force");
    stateLogger.add(B_upd(0,0), "Virtual_damping_X");
    //stateLogger.add(B(1,1), "Virtual_damping_Y");
    stateLogger.add(M_upd(0,0), "Virtual_mass_X");
    stateLogger.add(Tank_upper, "Tank_threshold_upper");
    stateLogger.add(Tank_lower, "Tank_threshold_lower");
    stateLogger.add(E_obs, "Observed_Energy");
    //stateLogger.add(V_ve(0), "VE_velocity_X");
    //stateLogger.add(V_diss(0), "Diss_velocity_X");
    stateLogger.add(Vd(0), "Desired_velocity_X");
    //stateLogger.add(C_diss(0,0), "1/b_X");
    stateLogger.add(E_obs_prediction, "Predicted_Energy");
    stateLogger.add(B_eff(0), "Effective_damping_X");
    stateLogger.add(Tank_level, "Tank_level");
    stateLogger.add(detect_index, "Detect_index");
    stateLogger.add(k, "Num_osci");
    stateLogger.add(m2, "Incr_mass");
    stateLogger.add(m_residsum, "Sum_resid_mass");
    stateLogger.add(osciTimeStamp, "Time_osci");
    stateLogger.startLogger();
}
void M2Admittance5::during(void) {
    //get robot position and velocity and force mesaure
    X = robot->getEndEffPosition();
    dX = robot->getEndEffVelocity();
    Fs = robot->getInteractionForce();
    elapsedT = running();
    deltaT = dt();

    osciTimeStamp = 0.0;
    m1 = m2 = m_resid = m_residsum = 0.0;
    std::vector<double> M_resid;
    M_residsum(0,0) = M_residsum(1,1) = 0.0;

    //Calculate tank level
    M_upd_ls_dot = (M_upd_ls - M_upd_2ls) / deltaT;
    Pb = B_upd_ls * dX.cwiseProduct(dX);
    Pm = 1/2 * M_upd_ls_dot * dX.cwiseProduct(dX);
    if (Tank_level(0) <= Tank_upper(0)){
        phi = 1;
    }
    else phi = 0;
    if (M_upd_ls_dot(0,0) <= 0){
        gamma = phi;
    }
    else gamma = 1;
    T_evol = phi * Pb - gamma * Pm;
    Tank_level = Tank_level + T_evol*deltaT;


    //Calculate tracking error
    V_error = Vd_ls(0) - dX(0);
    A_error = V_error/deltaT;
    V_error_norm = V_error/V_max;
    A_error_norm = A_error/A_max;
    detect_index = abs(A_error_norm + V_error_norm*Rd);

    if (detect_index > detect_threshold){
        k = k + 1; //count number of oscillations
        OsciTimeStamp.push_back(elapsedT);
        osciTimeStamp = OsciTimeStamp[k-1];
        if (Tank_level(0) > Tank_lower(0)){
            m1 = 2*(Tank_level(0)-Tank_lower(0))/(V_max*V_max);
        }
        m2 = fmin(m1, M_deltamax);
        M1.push_back(m1);
        M2.push_back(m2);
    }


    //Forgetting effect
    if (k >= 1){
        for(p=0; p<k; p++){
            m_resid = M2[p] * pow(forgetting, elapsedT-OsciTimeStamp[p]);
            M_resid.push_back(m_resid);
            M_residsum(0,0) = M_residsum(0,0) + M_resid[p];
            test = p;
        }
    }

    m_residsum = M_residsum(0,0);
    M_upd = M + M_residsum;
    B_upd = Rd * M_upd;
    V_ve = myVE(X, dX, Fs, B_upd, M_upd, deltaT);
    //V_ve = myVE(X, dX, Fs, B, M, deltaT);


    // Passivity Observer
    if (i < Obsv_T) {
        E_obs = PsvObsvST(E_obs_ls, dX, Fs, deltaT);
        E_obs_prediction = PsvObsvSTprediction(E_obs, Fs, V_ve, deltaT);
        //E_obs = PsvObsv(E_obs_ls, Fs, Fs_ls, V_ve, C_diss_ls, deltaT);
        //E_obs = PsvObsvSimple(E_obs_ls, Fs, dX, deltaT);
        i += 1;
    }
    if (i >= Obsv_T) {
        i = 0;
        //E_obs(VM2::Zero());
        E_obs(0) = E_obs(1) = 0;
        std::cout << "PO is reseted" << std::endl;
    }


    Vd = V_ve;
    Vd(1) = 0.0; //lock y axis

    /*
    //x-axis step signal to test max acceleration
    if (elapsedT-t_start > 2.0){
        Vd(0) = 2.0;
    }
    if (X(0) > 0.4){
        robot->disable();
    }
    */

    B_eff = EffDamp(M_upd, dX, Fs, Vd, deltaT);


    E_obs_ls = E_obs;
    //Fs_ls = Fs;
    //C_diss_ls = C_diss;
    Vd_ls = Vd;
    M_upd_2ls = M_upd_ls;
    M_upd_ls = M_upd;
    B_upd_ls = B_upd;


    stateLogger.recordLogData();
    if(iterations()%100==1) {
        //robot->printStatus();
        std::cout << "Mass_X=[ " << M_upd(0,0) << " ]\t" ;
        std::cout << "Damping_X=[ " << B_upd(0,0) << " ]\t" ;
        //std::cout << "Upper_E_X=[ " << E_upper(0) << " ]\t" ;
        //std::cout << "Lower_E_X=[ " << E_lower(0) << " ]\t" ;
        //std::cout << "Energy=[ " << E_obs_prediction.transpose() << " ]\t" ;
        //std::cout << "Damping_x=[ " << B(0,0) << " ]\t" ;
        //std::cout << "Damping_y=[ " << B(1,1) << " ]\t" <<std::endl;
        //std::cout << "V_ve=[ " << V_ve.transpose() << " ]\t" ;
        //std::cout << "V_diss=[ " << V_diss.transpose() << " ]\t" ;
        std::cout << "Vd=[ " << Vd.transpose() << " ]\t" ;
        std::cout << "Index=[ " << detect_index << " ]\t" ;
        std::cout << "OsciNum=[ " << k << " ]\t" ;
        std::cout << "M_ResidSum=[ " << m_residsum << " ]\t" ;
        std::cout << "Tank=[ " << Tank_level.transpose() << " ]\t" <<std::endl;
        //std::cout << "Damping_X=[ " << C_diss(0,0) << " ]\t" <<std::endl;
    }

    //apply velocity
    robot->setEndEffVelocity(Vd);
}
void M2Admittance5::exit(void) {
    robot->setJointVelocity(VM2::Zero());
}


// M2Admittance6: Ultimate Passivity (New - with mass)
void M2Admittance6::entry(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    //Virtual Environment
    M(0,0) = M(1,1) = 0.24;
    B(0,0) = B(1,1) = 0.08;

    B1(0,0) = B1(1,1) = 1.2;
    //B1(0,0) = B1(1,1) = 0.0;
    B2(0,0) = B2(1,1) = 50.0;
    M2(0,0) = M2(1,1) = 0.1;

    //Damper for dissipation
    //C_diss(0,0) = C_diss(1,1) = 0.0;
    //C_diss_ls(0,0) = C_diss_ls(1,1) = 0.0;

    E_obs(0) = E_obs(1) = 0.0 + 0.1;
    E_obs_ls(0) = E_obs_ls(1) = 0.0 + 0.1;

    Obsv_T = 10000000;
    i = 0;

    X(VM2::Zero());
    dX(VM2::Zero());
    Fs(VM2::Zero());
    Fs_ls(VM2::Zero());
    Vd(VM2::Zero());
    V_ve(VM2::Zero());
    V_diss(VM2::Zero());

    E_lower(0) = E_lower(1) = -0.1;
    //E_lower(0) = E_lower(1) = -2.0;
    E_upper(0) = E_upper(1) = 0.1;

    M_upd_ls(0,0) = M_upd_ls(1,1) = M(0,0);
    B_upd_ls(0,0) = B_upd_ls(1,1) = B(0,0);

    limit = 0.1;

    stateLogger.initLogger("M2Admittance6State", "logs/M2Admittance6State.csv", LogFormat::CSV, true);
    stateLogger.add(elapsedT, "%Time (s)");
    stateLogger.add(X, "Position");
    stateLogger.add(dX, "Velocity");
    stateLogger.add(Fs, "Force");
    stateLogger.add(B_upd(0,0), "Virtual_damping_X");
    //stateLogger.add(B(1,1), "Virtual_damping_Y");
    stateLogger.add(M_upd(0,0), "Virtual_mass_X");
    stateLogger.add(E_upper, "Energy_threshold_upper");
    stateLogger.add(E_lower, "Energy_threshold_lower");
    stateLogger.add(E_obs, "Observed_Energy");
    stateLogger.add(V_ve(0), "VE_velocity_X");
    stateLogger.add(V_diss(0), "Diss_velocity_X");
    stateLogger.add(Vd(0), "Desired_velocity_X");
    stateLogger.add(C_diss(0,0), "1/b_X");
    stateLogger.add(E_obs_prediction, "Predicted_Energy");
    stateLogger.add(B_eff(0), "Effective_damping_X");
    stateLogger.startLogger();
}
void M2Admittance6::during(void) {
    //get robot position and velocity and force mesaure
    X = robot->getEndEffPosition();
    dX = robot->getEndEffVelocity();
    Fs = robot->getInteractionForce();
    elapsedT = running();
    deltaT = dt();

    //Change E_UltPsv upper threshold
    if(robot->keyboard->getQ()) {
        E_upper(0)+=0.1;
        E_upper(1)+=0.1;
        std::cout << E_upper <<std::endl;
    }
    if(robot->keyboard->getA()) {
        E_upper(0)-=0.1;
        E_upper(1)-=0.1;
        std::cout << E_upper <<std::endl;
    }
    //Change E_UltPsv lower threshold
    if(robot->keyboard->getW()) {
        E_lower(0)+=0.1;
        E_lower(1)+=0.1;
        std::cout << E_lower <<std::endl;
    }
    if(robot->keyboard->getS()) {
        E_lower(0)-=0.1;
        E_lower(1)-=0.1;
        std::cout << E_lower <<std::endl;
    }


    V_ve = myVE(X, dX, Fs, B, M, deltaT);


    // Passivity Observer
    if (i < Obsv_T) {
        E_obs = PsvObsvST(E_obs_ls, dX, Fs, deltaT);
        E_obs_prediction = PsvObsvSTprediction(E_obs, Fs, V_ve, deltaT);
        //E_obs = PsvObsv(E_obs_ls, Fs, Fs_ls, V_ve, C_diss_ls, deltaT);
        //E_obs = PsvObsvSimple(E_obs_ls, Fs, dX, deltaT);
        i += 1;
    }
    if (i >= Obsv_T) {
        i = 0;
        //E_obs(VM2::Zero());
        E_obs(0) = E_obs(1) = 0;
        std::cout << "PO is reseted" << std::endl;
    }


    //Switching Law
    /*
    if (E_obs_prediction(0) <= E_lower(0)) {
        B_diss(0,0) = B2(0,0);
        M_diss(0,0) = M2(0,0);
        Operator2(0,0) = 1/(M_diss(0,0) + B_diss(0,0)*deltaT);
        V_diss = Operator2*(Fs*deltaT + M_diss*dX);
    }
    else if (E_obs_prediction(0) >= E_upper(0)) {
        B_diss(0,0) = 0.0;
        M_diss(0,0) = 0.0;
        V_diss(0) = 0.0;
    }
    else if (E_obs_prediction(0) > E_lower(0) && E_obs_prediction(0) < E_upper(0) && B_diss_ls(0,0)==0) {
        B_diss(0,0) = 0.0;
        M_diss(0,0) = 0.0;
        V_diss(0) = 0.0;
    }
    else {
        B_diss(0,0) = B2(0,0);
        M_diss(0,0) = M2(0,0);
        Operator2(0,0) = 1/(M_diss(0,0) + B_diss(0,0)*deltaT);
        V_diss = Operator2*(Fs*deltaT + M_diss*dX);
    }
    */

    if (E_obs_prediction(0) <= E_lower(0)) {
        B_upd(0,0) = B(0,0) + B2(0,0);
        M_upd(0,0) = M(0,0) + M2(0,0);
        V_ve_upd = myVE(X, dX, Fs, B_upd, M_upd, deltaT);
    }
    else if (E_obs_prediction(0) >= E_upper(0)) {
        B_upd(0,0) = B(0,0);
        M_upd(0,0) = M(0,0);
        V_ve_upd = V_ve;
    }
    else if (E_obs_prediction(0) > E_lower(0) && E_obs_prediction(0) < E_upper(0) && B_upd_ls(0,0)==B(0,0)) {
        B_upd(0,0) = B(0,0);
        M_upd(0,0) = M(0,0);
        V_ve_upd = V_ve;
    }
    else {
        B_upd(0,0) = B(0,0) + B2(0,0);
        M_upd(0,0) = M(0,0) + M2(0,0);
        V_ve_upd = myVE(X, dX, Fs, B_upd, M_upd, deltaT);
    }

    //Vd = V_ve + V_diss;
    Vd = V_ve_upd;
    Vd(1) = 0.0; //lock y axis


    B_eff = EffDamp(M_upd, dX, Fs, Vd, deltaT);


    E_obs_ls = E_obs;
    Fs_ls = Fs;
    //C_diss_ls = C_diss;
    B_upd_ls = B_upd;
    M_upd_ls = M_upd;

    stateLogger.recordLogData();
    if(iterations()%10==1) {
        //robot->printStatus();
        std::cout << "Mass_X=[ " << M_upd(0,0) << " ]\t" ;
        std::cout << "Damping_X=[ " << B_upd(0,0) << " ]\t" ;
        //std::cout << "Upper_E_X=[ " << E_upper(0) << " ]\t" ;
        //std::cout << "Lower_E_X=[ " << E_lower(0) << " ]\t" ;
        std::cout << "Energy=[ " << E_obs_prediction.transpose() << " ]\t" ;
        //std::cout << "Damping_x=[ " << B(0,0) << " ]\t" ;
        //std::cout << "Damping_y=[ " << B(1,1) << " ]\t" <<std::endl;
        std::cout << "V_ve=[ " << V_ve.transpose() << " ]\t" ;
        std::cout << "V_diss=[ " << V_diss.transpose() << " ]\t" ;
        std::cout << "Damping_X=[ " << C_diss(0,0) << " ]\t" <<std::endl;
    }

    //apply velocity
    robot->setEndEffVelocity(Vd);
}
void M2Admittance6::exit(void) {
    robot->setJointVelocity(VM2::Zero());
}


