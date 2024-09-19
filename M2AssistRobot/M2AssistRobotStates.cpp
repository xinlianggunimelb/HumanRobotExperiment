#include "M2AssistRobotStates.h"
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <Eigen/Dense>


VM2 myVE(VM2 X, VM2 dX, VM2 Fm, Eigen::Matrix2d B, Eigen::Matrix2d M, double dt) {
    Eigen::Matrix2d Operator;
    Operator(0,0) = 1./(M(0,0) + B(0,0)*dt);
    Operator(1,1) = 1./(M(1,1) + B(1,1)*dt);
    return Operator*(Fm*dt + M*dX);
}


VM2 impedance(Eigen::Matrix2d K, Eigen::Matrix2d D, VM2 X0, VM2 X, VM2 dX, VM2 dXd=VM2::Zero()) {
    return K*(X0-X) + D*(dXd-dX);
}


double JerkIt(VM2 X0, VM2 Xf, double T, double t, VM2 &Xd, VM2 &dXd) {
    t = std::max(std::min(t, T), .0); //Bound time
    double tn=std::max(std::min(t/T, 1.0), .0); //Normalised time bounded 0-1
    double tn3=pow(tn,3.);
    double tn4=tn*tn3;
    double tn5=tn*tn4;
    Xd = X0 + ( (X0-Xf) * (15.*tn4-6.*tn5-10.*tn3) );
    dXd = (X0-Xf) * (4.*15.*tn4-5.*6.*tn5-10.*3*tn3)/t;
    return tn;
}


void M2Calib::entry(void) {
    calibDone=false;
    calibAttempts=0;
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
    double b = 200;
    for(int i=0; i<vel.size(); i++) {
        tau(i) = -std::min(std::max(50 + b * vel(i), .0), 50.);
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
    } else {
        //If all joints are calibrated
        if(at_stop[0] && at_stop[1]) {
            robot->applyCalibration();
            calibAttempts++;
            if(robot->isCalibrated())
                std::cout << "OK." << std::endl;
            else
                std::cout << "..." << std::endl;
        } else {
            robot->setJointTorque(tau);
            if(iterations()%100==1) {
                std::cout << "." << std::flush;
            }
        }
    }

    //Allow 5 attempts to calib sensors
    if(calibAttempts>3) {
        spdlog::critical("M2 calibration failed. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
}
void M2Calib::exit(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
    KTest->StateIndex=1.; //StateIndex: 1 - calibrated
}


void M2Transparent::entry(void) {
    robot->initVelocityControl();
    robot->setEndEffVelocity(VM2::Zero());
    //M(0,0)=1.5;//Admittance control
    //M(1,1)=1.5;
    //B(0,0)=10.0;
    //B(1,1)=10.0;
    M(0,0)=1.5;//Admittance control
    M(1,1)=1.5;
    B(0,0)=30.0;
    B(1,1)=30.0;
}
void M2Transparent::during(void) {

    X = robot->getEndEffPosition();
    dX = robot->getEndEffVelocity();
    Fm = robot->getInteractionForce();
    Vd = myVE(X, dX, Fm, B, M, dt());

    if(robot->setEndEffVelocity(Vd)!=SUCCESS) {
        if(KTest->StateIndex!=12. && KTest->StateIndex!=13. && KTest->StateIndex!=15.) { //StateIndex: 12 - max force/speed detected during trial, 13 - return to the starting point failed, 15 - exceed perturbation threshold
            KTest->StateIndex = 14.; //StateIndex: 14 - reset (restart the motor) needed to return to the standby state
        }
    }

    if(iterations()%100==1) {
        robot->printStatus();
        std::cout << "Vd is ["<< Vd.transpose() << "] \n";
    }

}
void M2Transparent::exit(void) {
    robot->setEndEffVelocity(VM2::Zero());
}


void M2MinJerkPosition::entry(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    trialDone=false;

    startTime=running();
    Xi = robot->getEndEffPosition();
    Xf = KTest->global_start_point;
    T=5; //Trajectory Time
    k_i=1.;
}
void M2MinJerkPosition::during(void) {
    VM2 Xd, dXd;
    //Compute current desired interpolated point
    double status=JerkIt(Xi, Xf, T, running()-startTime, Xd, dXd);
    //Apply position control
    if(robot->setEndEffVelocity(dXd+k_i*(Xd-robot->getEndEffPosition()))!=SUCCESS) {
        if(KTest->perturbation_number==0&&KTest->constF_number==0) {
            KTest->StateIndex = 11.; //StateIndex: 11 - max force/speed detected before trial
        }
        if(KTest->perturbation_number>0||KTest->constF_number>0) {
            KTest->StateIndex = 12.; //StateIndex: 12 - max force/speed detected during trial
        }
        KTest->goToTransparentFlag = true;
    }

    //distance to the starting point
    double threshold = 0.01;
    VM2 distanceStPt=KTest->global_start_point-robot->getEndEffPosition();

    //Have we reached a point?
    if (status>=1. && iterations()%100==1) {
        //check if we reach the starting point
        if(abs(distanceStPt[0])<=threshold && abs(distanceStPt[1])<=threshold) {
            std::cout << "OK. \n";
            if (KTest->perturbation_number==0 && KTest->StateIndex==3.) { //StateIndex: 3 - recording finished but not returned
                KTest->StateIndex=4.; //StateIndex: 4 - recording finished and returned
            }
            if (KTest->constF_number>=1 && KTest->constF_number<=KTest->total_constF-1) {
                KTest->StateIndex = 49.; //StateIndex: 49 - return to the starting point for the next target force
            }
            if (KTest->constF_number>=KTest->total_constF) {
                KTest->StateIndex = 50.; //StateIndex: 50 - all constant forces are done
                KTest->constF_number=0;
            }
            if (KTest->perturbation_number>=1 && KTest->perturbation_number<=KTest->total_perturbation-1) {
                KTest->StateIndex = 99.; //StateIndex: 99 - return to the starting point for the next perturbation
            }
            if (KTest->perturbation_number>=KTest->total_perturbation) {
                KTest->StateIndex = 100.; //StateIndex: 100 - all trials are done
                KTest->perturbation_number=0;
                trialDone=true;
            }
        } else {
            KTest->StateIndex = 13.; //StateIndex: 13 - return to the starting point failed
            KTest->goToTransparentFlag = true;
        }
    }
}
void M2MinJerkPosition::exit(void) {
    // std::cout << "Ready... \n";
    robot->setJointVelocity(VM2::Zero());
}


void M2Recording::entry(void) {
    KTest->StateIndex = 2.; //StateIndex: 2 - recording in progress
    recordingDone=false;
    recordingError=false;
    robot->initVelocityControl();
    robot->setEndEffVelocity(VM2::Zero());

    //ForceP(0,0) = 1.3; //use it for torque control
    //ForceP(1,1) = 1.4;
    //ForceP(0,0) = 0.005; //use if for velocity control
    //ForceP(1,1) = 0.005;
    M(0,0)=1.5;//Admittance control
    M(1,1)=1.5;
    B(0,0)=10.0;
    B(1,1)=10.0;
    //Define Variables
    RecordingPoint=0;
}
void M2Recording::during(void) {
    //Transparent force control

    //Apply corresponding force
    //VM2 f_m = robot->getInteractionForce();
    //robot->setEndEffVelocity(ForceP*f_m);

    X = robot->getEndEffPosition();
    dX = robot->getEndEffVelocity();
    Fm = robot->getInteractionForce();
    Vd = myVE(X, dX, Fm, B, M, dt());

    if(robot->setEndEffVelocity(Vd)!=SUCCESS) {
        KTest->StateIndex = 5.; //StateIndex: 5 - recording failed
        KTest->goToTransparentFlag = true;
    }

    //Record stuff...
    PositionNow=robot->getEndEffPosition();
    if(RecordingPoint<MaxRecordingPts) {
        PositionRecorded[RecordingPoint]=PositionNow;
        RecordingPoint++;
    }

    /*if(iterations()%100==1) {
        robot->printStatus();
    }*/

    // allow 10 seconds for recording
    double t = running();
    if(t>=10) {
        robot->setEndEffVelocity(VM2::Zero());

        ///Identify circle
        // Fit a circle on a data point cloud using Pratt method
        // V.Pratt, "Direct least-squares fitting of algebraic surfaces",
        // Computer Graphics, Vol. 21, pages 145-152 (1987)
        //
        // Inspired from Matlab function CircleFitByPratt by Nikolai Chernov

        n = RecordingPoint;// number of data points
        std::cout << n << " \n ";
        //Debug.Log("Number of points:" + n.ToString());
        //Find centroid of data set
        for (int i=0; i<n; i++) {
            centroid += PositionRecorded[i];
            //std::cout << PositionRecorded[i].transpose() << " \n ";
        }
        centroid = centroid/n;
        //std::cout << centroid.transpose() << "  ";

        //computing moments(note: all moments will be normalised, i.e.divided by n)
        Mxx = 0;
        Myy = 0;
        Mxy = 0;
        Mxz = 0;
        Myz = 0;
        Mzz = 0;
        for (int j = 0; j < n; j++) {
            // centering data
            Xi = PositionRecorded[j][0] - centroid[0];
            //std::cout << PositionRecorded[j][1] << " \n ";
            Yi = PositionRecorded[j][1] - centroid[1];
            Zi = Xi * Xi + Yi * Yi;
            Mxy = Mxy + Xi * Yi;
            Mxx = Mxx + Xi * Xi;
            Myy = Myy + Yi * Yi;
            Mxz = Mxz + Xi * Zi;
            Myz = Myz + Yi * Zi;
            Mzz = Mzz + Zi * Zi;
        }
        Mxx = Mxx / n;
        Myy = Myy / n;
        Mxy = Mxy / n;
        Mxz = Mxz / n;
        Myz = Myz / n;
        Mzz = Mzz / n;

        // computing the coefficients of the characteristic polynomial
        Mz = Mxx + Myy;
        Cov_xy = Mxx * Myy - Mxy * Mxy;
        Mxz2 = Mxz * Mxz;
        Myz2 = Myz * Myz;
        A2 = 4 * Cov_xy - 3 * Mz * Mz - Mzz;
        A1 = Mzz * Mz + 4 * Cov_xy * Mz - Mxz2 - Myz2 - Mz * Mz * Mz;
        A0 = Mxz2 * Myy + Myz2 * Mxx - Mzz * Cov_xy - 2 * Mxz * Myz * Mxy + Mz * Mz * Cov_xy;
        A22 = A2 + A2;
        epsilon = 1e-12;
        ynew = 1e+20;
        IterMax = 20;
        xnew = 0;

        // Newton's method starting at x=0
        for (int iter = 1; iter <= IterMax; iter++) {
            yold = ynew;
            ynew = A0 + xnew * (A1 + xnew * (A2 + 4 * xnew * xnew));
            if (abs(ynew) > abs(yold)) {
                //Debug.Log("Newton-Pratt goes wrong direction: |ynew| > |yold|");
                xnew = 0;
                break;
            }
            Dy = A1 + xnew * (A22 + 16 * xnew * xnew);
            xold = xnew;
            xnew = xold - ynew / Dy;
            if (abs((xnew - xold) / xnew) < epsilon)
                break;
            if (iter >= IterMax) {
                //Debug.Log("Newton-Pratt will not converge");
                xnew = 0;
            }
            if (xnew < 0) {
                //Debug.Log("Newton-Pratt negative root:  x=" + xnew.ToString());
                xnew = 0;
            }
        }

        //computing the circle parameters
        DET = xnew * xnew - xnew * Mz + Cov_xy;
        Center[0] = (Mxz * (Myy - xnew) - Myz * Mxy) / DET / 2 ;
        Center[1] = (Myz * (Mxx - xnew) - Mxz * Mxy) / DET / 2 ;
        radius = sqrt(Center[0]*Center[0] + Center[1]*Center[1] + Mz + 2 * xnew);
        std::cout << "Radius is " << radius << " meters \n";
        Center += centroid;//Shift to actual center
        std::cout << "Center point is ["<< Center.transpose() << "] \n";

        // Start angle defined with 0 along positive x axis, 180 along negative x
        start_angle = (atan2(PositionRecorded[0][1] - Center[1], PositionRecorded[0][0] - Center[0]) * 180.0 / M_PI);
        start_angle = (start_angle < -90)? 360 - abs(start_angle): start_angle;// Assume this is an indirect rotation(bottom left quadrant)
        std::cout << "Start angle is "<< start_angle << " degree \n";
        StartPt[0] = Center[0] + radius * cos(start_angle * M_PI / 180.0);
        StartPt[1] = Center[1] + radius * sin(start_angle * M_PI / 180.0);
        std::cout << "Start point is ["<< StartPt.transpose() << "] \n";


        /// resonable parameters
        if(radius>0.2 && radius<0.45 && StartPt[0]>=0 && StartPt[0]<=0.631 && StartPt[1]>=0 && StartPt[1]<=0.448 && abs(PositionRecorded[0][0]-PositionRecorded[n-1][0])>0.0 && abs(PositionRecorded[0][1]-PositionRecorded[n-1][1])>0.15) {
            //if parameters reasonable, give them to global variables
            KTest->global_center_point = Center;
            KTest->global_start_point = StartPt;
            KTest->global_radius = radius;
            KTest->global_start_angle = start_angle;
            KTest->StateIndex = 3.; //StateIndex: 3 - recording finished but not returned
            recordingDone=true;
        } else {
            KTest->StateIndex = 5.; //StateIndex: 5 - recording failed
            recordingError=true;
        }
    }
}
void M2Recording::exit(void) {
    robot->setEndEffVelocity(VM2::Zero());
    KTest->constF_number = 0;
    KTest->perturbation_number = 0; //for a new trial
}


void M2ArcCircle::entry(void) {
    KTest->StateIndex=6.; //StateIndex: 6 - circle testing in progress
    testingDone=false;
    testingError=false;
    movement_finished = false;
    robot->initVelocityControl();

    //Initialise values (from network command) and sanity check
    theta_s = KTest->global_start_angle;
    radius = KTest->global_radius;
    centerPt = KTest->global_center_point;
    startingPt = KTest->global_start_point;

    dTheta_t = 20; //testing velocity 20 degree/second
    std::cout << "Velocity is "<< dTheta_t << " degree/second \n";

    thetaRange=65;
    ddTheta=200;
    theta = theta_s;

    //Initialise profile timing
    t_init = 3.0; //waiting time before movement starts (need to be at least 0.8 because drives have a lag...)
    t_end_accel = t_init + dTheta_t/ddTheta; //acceleration phase to reach constant angular velociy
    t_end_cstt = t_end_accel + (thetaRange-(dTheta_t*dTheta_t)/ddTheta)/dTheta_t; //constant angular velocity phase: ensure total range is theta_range
    t_end_decel = t_end_cstt + dTheta_t/ddTheta; //decelaration phase

    //Define sign of movement based on starting angle
    sign=1;
    if(theta_s>90) {
        sign=-1;
    }
}
void M2ArcCircle::during(void) {
    //Define velocity profile phase based on timing
    double dTheta = 0;
    VM2 dXd, Xd, dX;
    double t = running();
    if(t<t_init) {
        dTheta=0;
    } else {
        if(t<t_end_accel) {
            //Acceleration phase
            dTheta=(t-t_init)*ddTheta;
        } else {
            if(t<=t_end_cstt) {
                //Constant phase
                dTheta=dTheta_t;
            } else {
                if(t<t_end_decel) {
                    //Deceleration phase
                    dTheta=dTheta_t-(t-t_end_cstt)*ddTheta;
                } else {
                    //Profile finished
                    dTheta=0;
                    movement_finished = true;
                }
            }
        }
    }
    dTheta*=sign;

    //Integrate to keep mobilisation angle
    theta += dTheta*dt();

    //Transform to end effector space
    //desired velocity
    dXd[0] = -radius*sin(theta*M_PI/180.)*dTheta*M_PI/180.;
    dXd[1] = radius*cos(theta*M_PI/180.)*dTheta*M_PI/180.;
    //desired position
    Xd[0] = centerPt[0]+radius*cos(theta*M_PI/180.);
    Xd[1] = centerPt[1]+radius*sin(theta*M_PI/180.);

    //desired position reaches bound
    if(Xd[0]<0 || Xd[0]>0.631 || Xd[1]<0 || Xd[1]>0.448) {
        KTest->StateIndex=5.; //StateIndex: 5 - recording failed (found by circle testing)
        testingError = true; //trigger event
    }

    //PI in velocity-position
    float K=5.0;
    dX = dXd + K*(Xd-robot->getEndEffPosition());

    //Apply
    if(robot->setEndEffVelocity(dX)!=SUCCESS) {
        KTest->StateIndex = 11.; //StateIndex: 11 - max force/speed detected before trial
        KTest->goToTransparentFlag = true;
    }

    /*if(iterations()%100==1) {
        std::cout << dXd.transpose() << "  ";
        robot->printStatus();
    }*/

    if(movement_finished && t>t_end_decel+1) { //wait one second
        KTest->StateIndex=7.; //StateIndex: 7 - circle testing finished but not returned
        testingDone = true; //trigger event
    }
}
void M2ArcCircle::exit(void) {
    robot->setEndEffVelocity(VM2::Zero());
    KTest->constF_number = 0;
    KTest->perturbation_number = 0; //for a new trial
}


void M2ArcCircleReturn::entry(void) {
    KTest->StateIndex=8.; //StateIndex: 8 - circle test return in progress
    testReturnDone=false;
    movement_finished = false;
    robot->initVelocityControl();

    theta_s = KTest->global_start_angle;
    radius = KTest->global_radius;
    centerPt = KTest->global_center_point;

    dTheta_t = 20; //Arc Return Velocity
    ddTheta=200;

    //Arc Return starting point
    startingReturnPt = robot->getEndEffPosition();
    //std::cout << startingReturnPt.transpose() << " \n";
    startReturnAngle = (atan2(startingReturnPt[1] - centerPt[1], startingReturnPt[0] - centerPt[0]) * 180.0 / M_PI);
    thetaReturnRange = abs(startReturnAngle-theta_s);
    thetaReturn = startReturnAngle;
    std::cout << "Current angle is " << thetaReturn << " degree \n";

    //Initialise profile timing
    t_init = 1.0; //waiting time before movement starts (need to be at least 0.8 because drives have a lag...)
    t_end_accel = t_init + dTheta_t/ddTheta; //acceleration phase to reach constant angular velociy
    t_end_cstt = t_end_accel + (thetaReturnRange-(dTheta_t*dTheta_t)/ddTheta)/dTheta_t; //constant angular velocity phase: ensure total range is theta_range
    t_end_decel = t_end_cstt + dTheta_t/ddTheta; //decelaration phase
    //std::cout << t_end_accel << " \n";
    //std::cout << t_end_cstt << " \n";
    //std::cout << t_end_decel << " \n";

    //Define sign of movement based on starting angle
    sign=-1;
    if(theta_s>90) {
        sign=1;
    }
}
void M2ArcCircleReturn::during(void) {
    //Define velocity profile phase based on timing
    double dThetaReturn = 0;
    VM2 dXd, Xd, dX;
    double t = running();
    if(t<t_init) {
        dThetaReturn=0;
    } else {
        if(t<t_end_accel) {
            //Acceleration phase
            dThetaReturn=(t-t_init)*ddTheta;
        } else {
            if(t<=t_end_cstt) {
                //Constant phase
                dThetaReturn=dTheta_t;
            } else {
                if(t<t_end_decel) {
                    //Deceleration phase
                    dThetaReturn=dTheta_t-(t-t_end_cstt)*ddTheta;
                } else {
                    //Profile finished
                    dThetaReturn=0;
                    movement_finished = true;
                }
            }
        }
    }
    dThetaReturn*=sign;
    //std::cout << dThetaReturn << " \n";

    //Integrate to keep mobilisation angle
    thetaReturn += dThetaReturn*dt();
    //std::cout << thetaReturn << " \n";

    //Transform to end effector space
    //desired velocity
    dXd[0] = -radius*sin(thetaReturn*M_PI/180.)*dThetaReturn*M_PI/180.;
    dXd[1] = radius*cos(thetaReturn*M_PI/180.)*dThetaReturn*M_PI/180.;
    //desired position
    Xd[0] = centerPt[0]+radius*cos(thetaReturn*M_PI/180.);
    Xd[1] = centerPt[1]+radius*sin(thetaReturn*M_PI/180.);
    //PI in velocity-position
    float K=5.0;
    dX = dXd + K*(Xd-robot->getEndEffPosition());

    //Apply
    if(robot->setEndEffVelocity(dX)!=SUCCESS) {
        KTest->StateIndex = 11.; //StateIndex: 11 - max force/speed detected before trial
        KTest->goToTransparentFlag = true;
    }

    /*if(iterations()%100==1) {
        std::cout << dXd.transpose() << "  ";
        robot->printStatus();
    }*/

    if(movement_finished && t>t_end_decel+1) { //wait one second
        KTest->StateIndex=9.; //StateIndex: 9 - circle test returned
        testReturnDone = true; //trigger event
    }
}
void M2ArcCircleReturn::exit(void) {
    robot->setEndEffVelocity(VM2::Zero());
}


void M2ConstForce::entry(void) {
    KTest->constF_number ++;
    KTest->StateIndex=20.+KTest->constF_number; //StateIndex:
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());

    constFDone = false;
    elapsedT = 0.0;
    duration = 10.0;
    Vd[0]=Vd[1]=0.;

    mvtDirAngle = KTest->global_start_angle * 2 * M_PI / 360 - M_PI / 2;
    mvtDirForce = mvtDirForceSum = 0.0;
}
void M2ConstForce::during(void) {
    Fs = robot->getInteractionForce();
    elapsedT = running();
    i = iterations();

    //Apply position control
    if(robot->setEndEffVelocity(Vd)!=SUCCESS) {
        KTest->StateIndex = 12.; //StateIndex: 12 - max force/speed detected during trial
        KTest->goToTransparentFlag = true;
    }

    mvtDirForce = Fs[0] * cos(mvtDirAngle) + Fs[1] * sin(mvtDirAngle);
    mvtDirForceSum = mvtDirForceSum + mvtDirForce;
    if(i%50==0) {
        KTest-> Feedback_F = mvtDirForceSum / 50;
        mvtDirForceSum = 0;
    }

    if(elapsedT>duration){
        KTest->StateIndex = 49.; //StateIndex: 49 - return to the starting point for the next target force
        constFDone = true;
    }

    if(iterations()%100==1) {
        std::cout << "num = [" << KTest->constF_number << "]";
        std::cout << "state = [" << KTest->StateIndex << "] \n";
        //robot->printStatus();
    }
}
void M2ConstForce::exit(void) {
    // std::cout << "Ready... \n";
    robot->setJointVelocity(VM2::Zero());
}


// Function to generate white noise
std::vector<double> generateWhiteNoise(int num_samples) {
    std::vector<double> white_noise(num_samples);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(0, 1);

    for (int i = 0; i < num_samples; ++i) {
        white_noise[i] = d(gen);
    }

    return white_noise;
}

// Butterworth Low-Pass Filter design
void butterworthLowpass(int order, double cutoff, double fs, Eigen::Vector3d& b, Eigen::Vector3d& a) {
//Only works on order two filter

    const double fn = 2*cutoff / fs;
    const double ita = 1.0 / tan(M_PI*fn);
    const double q = sqrt(2.0);

    b[0] = 1.0 / (1.0 + q*ita + ita*ita); //b[0]
    b[1] = 2.0*b[0]; //b[1]
    b[2] = b[0]; //b[2]
    a[0] = 1.; //a[0]
    a[1] = -2.0 * (ita*ita - 1.0) * b[0]; //a[1]
    a[2] = (1.0 - q*ita + ita*ita) * b[0]; //a[2]
}

void butterworthLowpass2(int order, double cutoff, double fs, Eigen::Vector3d& b, Eigen::Vector3d& a) {
//Only works on order two filter

    const double fn = 2*cutoff / fs;
    const double ita = 1.0 / tan(M_PI*fn);
    const double q = sqrt(2.0);

    b[0] = 1.0 / (1.0 + q*ita + ita*ita); //b[0]
    b[1] = 2.0*b[0]; //b[1]
    b[2] = b[0]; //b[2]
    a[0] = 1.; //a[0]
    a[1] = -2.0 * (ita*ita - 1.0) * b[0]; //a[1]
    a[2] = (1.0 - q*ita + ita*ita) * b[0]; //a[2]
}

// Apply filter to signal
std::vector<double> applyFilter(std::vector<double> signal, Eigen::Vector3d b, Eigen::Vector3d a) {

    //Initialise elements
    int order = 2;
    std::vector<double> x(order+1), y(order+1);
    for(unsigned int i=0; i<order+1; i++) {
        x[i] = 0.;
        y[i] = 0.;
    }

    std::vector<double> filtered_signal(signal.size());

    for (unsigned int n=0; n<signal.size(); n++) {

        //Shift elements and insert new one
        for(unsigned int k=0; k<order; k++) {
            x[k] = x[k+1];
            y[k] = y[k+1];
        }
        x[order] = signal[n];

        //Apply filter
        y[order] = 0;
        for(unsigned int i=0; i<order+1; i++) {
            y[order] += b[i] * x[order-i];
        }
        for(unsigned int i=1; i<order+1; i++) {
            y[order] -= a[i] * y[order-i];
        }
        y[order] /= a[0];

        filtered_signal[n] = y[order];
    }

    return filtered_signal;
}

// Apply filter to signal
double applyFilter2(double signal, Eigen::Vector3d b, Eigen::Vector3d a, Eigen::Vector3d& x, Eigen::Vector3d& y) {

    //Initialise elements
    int order = 2;
    double filtered_signal;

    //Shift elements and insert new one
    for(unsigned int k=0; k<order; k++) {
        x[k] = x[k+1];
        y[k] = y[k+1];
    }
    x[order] = signal;

    //Apply filter
    y[order] = 0;
    for(unsigned int i=0; i<order+1; i++) {
        y[order] += b[i] * x[order-i];
    }
    for(unsigned int i=1; i<order+1; i++) {
        y[order] -= a[i] * y[order-i];
    }
    y[order] /= a[0];

    filtered_signal = y[order];

    return filtered_signal;
}


void M2StochPert::entry(void) {
    KTest->perturbation_number ++;
    KTest->StateIndex=50.+KTest->perturbation_number; //StateIndex:
    pertDone = false;
    robot->initVelocityControl();
    robot->setEndEffVelocity(VM2::Zero());

    elapsedT=0; i=0; j=1;
    white_noise.clear(); perturbation.clear();
    DocWhiteNoise=0.; DocPerturbation=0.;
    //PertAmp(VM2::Zero()); PertDest(VM2::Zero());
    //dXd(VM2::Zero()); Vd(VM2::Zero());
    PertAmp[0]=PertAmp[1]=0.; PertDest[0]=PertDest[1]=0.;
    dXd[0]=dXd[1]=0.; Vd[0]=Vd[1]=0.;

    wait = 1.0; //waiting period
    duration = 40.0; //perturbation period
    fs = 100.0; //for perturbation
    fc = 3.0;
    fs2 = 500.0; //for F and dX
    fc2 = 5.0;
    filt_order = 2;

    num_samples = fs * duration;
    order_samples = 0;
    round = 0;

    Theta = KTest->global_start_angle;
    Xi = KTest->global_start_point;
    Xd = KTest->global_start_point;
    PertDest = KTest->global_start_point;

    mvtDirAngle = KTest->global_start_angle * 2 * M_PI / 360 - M_PI / 2;
    mvtDirForce = mvtDirForceAbs = mvtDirForceAbsSum = 0.0;
    mvtDirForceAbsSize = 500*2;
    mvtDirForceAbsVec.resize(mvtDirForceAbsSize);
    mvtDirForceAbsVec.clear();
    std::fill(mvtDirForceAbsVec.begin(), mvtDirForceAbsVec.end(), 0.0);

    // Generate white noise
    white_noise = generateWhiteNoise(num_samples);

    // Design the low-pass filter
    //Eigen::VectorXd b(filt_order + 1), a(filt_order + 1); //for perturbation
    butterworthLowpass(filt_order, fc, fs, b, a);
    //Eigen::VectorXd b2(filt_order + 1), a2(filt_order + 1); //for F and dX
    butterworthLowpass2(filt_order, fc2, fs2, b2, a2);

    // Apply the filter to the white noise
    for (int k = 0; k < num_samples; k++) {
        white_noise[k] = white_noise[k]/160; //Linux PC
    }
    perturbation = applyFilter(white_noise, b, a);

    std::string DocPertNum = std::to_string(KTest->perturbation_number);
    std::string loggerNameM2 = "M2StochPert" + DocPertNum;
    std::string fileNameM2 = "logs/M2StochPertState" + DocPertNum + ".csv";
    //stateLogger.initLogger("M2StochPert", "logs/M2StochPertState.csv", LogFormat::CSV, true);
    stateLogger.initLogger(loggerNameM2, fileNameM2, LogFormat::CSV, true);
    if(KTest->perturbation_number==1){
        stateLogger.add(elapsedT, "%Time (s)");
        stateLogger.add(i, "iterations");
        stateLogger.add(j, "Iterations");
        stateLogger.add(X, "Position");
        stateLogger.add(dX, "Velocity");
        stateLogger.add(Fs, "Force");
        stateLogger.add(DocWhiteNoise, "White_Noise");
        stateLogger.add(DocPerturbation, "Perturbation");
        stateLogger.add(PertAmp, "Desired_amp");
        stateLogger.add(PertDest, "Desired_dest");
        stateLogger.add(Xd, "Desired_pos");
        stateLogger.add(Vd, "Desired_vel");
    }
    stateLogger.startLogger();

}
void M2StochPert::during(void) {
    X = robot->getEndEffPosition();
    dX = robot->getEndEffVelocity();
    Fs = robot->getInteractionForce();
    elapsedT = running();
    deltaT = dt();
    i = iterations();

    /*
    dX_filt[0] = applyFilter2(dX[0], b2, a2, x_dX0, y_dX0);
    dX_filt[1] = applyFilter2(dX[1], b2, a2, x_dX1, y_dX1);
    Fs_filt[0] = applyFilter2(Fs[0], b2, a2, x_Fs0, y_Fs0);
    Fs_filt[1] = applyFilter2(Fs[1], b2, a2, x_Fs1, y_Fs1);
    */

    if(elapsedT<wait) {
        Vd(VM2::Zero());
    }
    else{
        if(elapsedT<wait+duration){
            if(j%5==1) { //Linux PC
                if(order_samples > num_samples-1) {
                    round = round + 1;
                    order_samples = 0;
                }
                //white noise and perturbation values for documentation
                DocWhiteNoise = white_noise[order_samples];
                DocPerturbation = perturbation[order_samples];

                //calculate perturbation amplitude and destination in x and y
                PertAmp[0] = perturbation[order_samples] * sin(Theta * M_PI / 180.0);
                PertAmp[1] = perturbation[order_samples] * -1 * cos(Theta * M_PI / 180.0);
                X_orgn = PertDest;
                PertDest = Xi + PertAmp;

                //calculate desired position and velocity for each control period
                //X_orgn = X;
                stepDistance = (PertDest-X_orgn)/5; //Linux PC
                dXd = stepDistance/deltaT;

                step = 1;
                order_samples = order_samples + 1;
                }

            Xd = X_orgn + stepDistance*step;
            step = step + 1;
            j++;

            //PI in velocity-position
            float K = 5.;
            Vd = dXd + K*(Xd-X);

            //Compute force as a reflection of stiffness for virtual feedback
            mvtDirForce = Fs[0] * cos(mvtDirAngle) + Fs[1] * sin(mvtDirAngle);
            mvtDirVelocity = dX[0] * cos(mvtDirAngle) + dX[1] * sin(mvtDirAngle);
            mvtDirForce_filt = applyFilter2(mvtDirForce, b2, a2, x_MDF, y_MDF);
            mvtDirVelocity_filt = applyFilter2(mvtDirVelocity, b2, a2, x_MDV, y_MDV);

            mvtDirAcc = (mvtDirVelocity_filt - mvtDirVelocity_filt_ls) / (1/fs);
            mvtDirForceRmI = - mvtDirForce_filt - fixedI * mvtDirAcc;
            mvtDirForceAbs = abs(mvtDirForceRmI);
            /*
            mvtDirForceAbsSum = mvtDirForceAbsSum + mvtDirForceAbs;
            if(j%50==0) {
                //KTest-> Feedback_K = mvtDirForceAbsSum / 50;
                mvtDirForceAbsSum = 0;
            }
            */
            for(int n=0; n<mvtDirForceAbsSize-1; n++) {
                mvtDirForceAbsVec[n] = mvtDirForceAbsVec[n+1];
            }
            mvtDirForceAbsVec[mvtDirForceAbsSize-1] = mvtDirForceAbs;
            mvtDirForceAbsSum = 0.0;
            for(int n=0; n<mvtDirForceAbsSize; n++) {
                mvtDirForceAbsSum = mvtDirForceAbsSum + mvtDirForceAbsVec[n];
            }

            KTest-> Feedback_K = mvtDirForceAbsSum / mvtDirForceAbsSize;
            mvtDirVelocity_filt_ls = mvtDirVelocity_filt;
        }
        else{
            KTest->StateIndex = 99.; //StateIndex: 99 - return to the starting point for the next perturbation
            pertDone = true;
        }
    }

    //Apply
    //distance to the starting point
    double threshold = 0.01; //Linux PC
    VM2 distance = Xi - X;
    if(abs(distance[0])>=threshold || abs(distance[1])>=threshold) {
        std::cout << "distance = [" << distance.transpose() << "] \n";
        KTest->StateIndex = 15.; //StateIndex: 15 - exceed perturbation threshold
        KTest->goToTransparentFlag = true;
    }

    //if(robot->setEndEffVelocity(VM2::Zero())!=SUCCESS) {
    if(robot->setEndEffVelocity(Vd)!=SUCCESS) {
        KTest->StateIndex = 12.; //StateIndex: 12 - max force/speed detected during trial
        KTest->goToTransparentFlag = true;
    }

    stateLogger.recordLogData();

    if(iterations()%10==1) {
        //std::cout << "Vel_d = [" << Vd.transpose() << "] \n";
        //robot->printStatus();
        //std::cout << "F = [" << mvtDirForceAbs << "] ";
        //std::cout << "Flastest = [" << mvtDirForceAbsVec[mvtDirForceAbsSize-1] << "] ";
        std::cout << "F = [" << KTest-> Feedback_K << "] \n";
    }


}
void M2StochPert::exit(void) {
    robot->setEndEffVelocity(VM2::Zero());
    stateLogger.endLog();
}


