#include "unitree_arm_sdk/control/unitreeArm.h"
#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <ctime>
#include <iomanip>

using namespace UNITREE_ARM;


Vec6 to_vec6(Vec6 var0, double value, double index) {
    Vec6 var = var0;
    var(index) = value;
    return var;
};


void simple_logger(const std::string& msg) {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm buf;
    localtime_r(&in_time_t, &buf);
    std::cout << std::put_time(&buf, "%Y-%m-%d %H:%M:%S");
    std::cout << '.' << std::setw(3) << std::setfill('0') << ms.count();
    std::cout << " - " << msg << std::endl;
}


int main(int argc, char *argv[]) {
    simple_logger("Starting...");
    std::ofstream log_file;

    std::cout << std::fixed << std::setprecision(3);
    bool hasGripper = false;
    unitreeArm arm(hasGripper);

    arm.sendRecvThread->start();
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.setFsm(ArmFSMState::LOWCMD);
    std::vector<double> KP, KW;
    KP = arm._ctrlComp->lowcmd->kp;
    KW = arm._ctrlComp->lowcmd->kd;
    arm._ctrlComp->lowcmd->setControlGain(KP, KW);
    arm.sendRecvThread->shutdown();

    simple_logger("Control loop [Hz]: " + std::to_string(1/arm._ctrlComp->dt));

    Vec6 vec_zero; vec_zero.setZero();
    Vec6 q_0;
    Vec6 q_des;
    Timer timer(arm._ctrlComp->dt);

    ////////////////////////////////
    // 0. Move to initial pose
    simple_logger("Move to initial pose.");
    q_0 = arm.lowstate->getQ();
    q_des << 0.0, 1.57079633, -2.7925268, -0.34906585, 0.0, 0.0; // `candle` position
    for(int i(0); i<1000; i++){
        arm.q = q_0 * (1-i/1000) + q_des * (i/1000);
        arm.qd = (q_des - q_0) / (1000 * arm._ctrlComp->dt);
        arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.sendRecv();
        timer.sleep();
    }


    ////////////////////////////////
    // 1. Do experiments for joint indexes
    
    std::vector<int> joint_ids = {0, 5};

    for (auto k: joint_ids) {

        simple_logger("Start experiments for joint: " + std::to_string(k) +".");
        ////////////////////////////////
        // Sinusoidal experiment
        {

            // for kp = kd = 0
            // tau = kp e + kd de + delta_tau   -->> tau = delta_tau
            // delta_tau = ID(.)                -->> delta_tau = ID(q,0,0) + Asin(wt)
            
            // Make kp = kd = 0
            arm.sendRecvThread->start();
            arm._ctrlComp->lowcmd->setPassive();
            arm._ctrlComp->lowcmd->setZeroKd();
            arm.sendRecvThread->shutdown();
    
            // sin-signal parameters
            std::vector<std::pair<double, double>> params = {{0.5, 2.0}, {1.0, 6.0}}; // where pair is <A, omega>
            
            for (int j(0); j < params.size(); ++j) {
                simple_logger("Experiment " + std::to_string(j) + ": sinusoidal");

                // save data separately for each sin-signal parameters
                log_file.open("data_sinusoidal_exp" + std::to_string(j) + "_joint_" + std::to_string(k) + "_torque.txt");

                // sin-signal parameters
                double A = params[j].first;
                double omega = params[j].second;
                double T = 2 * M_PI / omega;
                
                double t = 0.0;

                for(int i(0); i < 5000; i++){ // 10 sec for dt=0.002

                    arm.q = vec_zero;
                    arm.qd = vec_zero;
                    
                    double disturbance = A * std::sin(omega * t);
                    Vec6 tau_sin = to_vec6(vec_zero, disturbance, k);
                    
                    // compensate gravity only
                    Vec6 tau_gravity = arm._ctrlComp->armModel->inverseDynamics(arm.q, Vec6::Zero(), Vec6::Zero(), Vec6::Zero());
                    
                    arm.tau = tau_sin + tau_gravity;
                    
                    arm.setArmCmd(arm.q, arm.qd, arm.tau);
                    arm.sendRecv();
                    timer.sleep();

                    // q_des, dq_des, calc_ID_tau, q, dq, ddq, tau, time
                    log_file << tau_sin.transpose() << " ";
                    log_file << tau_gravity.transpose() << " ";
                    log_file << arm.lowstate->getQ().transpose() << " ";
                    log_file << arm.lowstate->getQd().transpose() << " ";
                    log_file << arm.lowstate->getQdd().transpose() << " ";
                    log_file << arm.lowstate->getTau().transpose() << " ";
                    log_file << i * arm._ctrlComp->dt << "\n";

                    t += arm._ctrlComp->dt;
                }
                log_file.close();       
            }
        }
    }

    // move to home
    arm.sendRecvThread->start();
    arm.setFsm(ArmFSMState::JOINTCTRL);
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    simple_logger("Finish.");
    return 0;
}