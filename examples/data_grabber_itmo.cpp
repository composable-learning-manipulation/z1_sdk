#include "unitree_arm_sdk/control/unitreeArm.h"
#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>

using namespace UNITREE_ARM;


class TrapezoidalTrajectory {

public:
    double q, dq, ddq;

    TrapezoidalTrajectory() : q(0), dq(0), ddq(0), 
                    t_acc(0), t_const(0), t_total(0), 
                    q0(0), q_des(0), 
                    dq_max(0), ddq_max(0), 
                    direction(1), current_time(0) {}

    double get_time_total() {
        return this->t_total;
    }

    void setup(double q0, double q_des,  double dq_max, double ddq_max) {
        this->q0 = q0;
        this->q_des = q_des;
        this->ddq_max = ddq_max;
        this->direction = (q_des > q0) ? 1 : -1;

        double delta_q = std::abs(q_des - q0);
        this->dq_max = std::min(std::sqrt(ddq_max * delta_q), dq_max);
        this->t_acc = dq_max / ddq_max;
        this->t_const = (delta_q - dq_max * dq_max / ddq_max) / dq_max;
        this->t_total = 2 * t_acc + t_const;

        this->q = q0;
        this->dq = 0;
        this->ddq = 0;

        this->current_time = 0;
    }

    void update(double dt) {
        current_time += dt;
        if (q0 == q_des) {
            q = q0;
            dq = 0;
            ddq = 0;
        } else {
            if (current_time < t_acc) {
                ddq = direction * ddq_max;
                dq = ddq * current_time;
                q = q0 + 0.5 * ddq * current_time * current_time;
            } else if (current_time < (t_acc + t_const)) {
                ddq = 0;
                dq = direction * dq_max;
                double t = current_time - t_acc;
                q = q0 + direction * (0.5 * dq_max * t_acc + dq_max * t);
            } else if (current_time < t_total) {
                ddq = -direction * ddq_max;
                double t_dec = current_time - t_acc - t_const;
                dq = direction * (dq_max - ddq_max * t_dec);
                q = q0 + direction * (0.5 * dq_max * t_acc + dq_max * t_const + dq_max * t_dec - 0.5 * ddq_max * t_dec * t_dec);
            } else {
                ddq = 0;
                dq = 0;
                q = q_des;
            }
        }
    }

private:
    double t_acc, t_const, t_total;
    double q0, q_des, dq_max, ddq_max;
    double current_time;
    int direction;
};


class SinusoidalTrajectory {
public:
    double q, dq, ddq;

    SinusoidalTrajectory() : q(0), dq(0), ddq(0), q0(0), A(0), omega(0), current_time(0) {}

    void setup(double q0, double A, double omega) {
        this->q0 = q0;
        this->A = A;
        this->omega = omega;

        q = q0 + A * std::cos(omega * current_time); // Set initial position
        dq = A * omega * std::sin(omega * current_time); // Initial velocity
        ddq = -A * omega * omega * std::cos(omega * current_time); // Initial acceleration
        current_time = 0;
    }

    void update(double dt) {
        current_time += dt;
        q = q0 - A + A * std::cos(omega * current_time);
        dq = -A * omega * std::sin(omega * current_time);
        ddq = A * omega * omega * std::cos(omega * current_time);
    }

private:
    double q0, A, omega;
    double current_time;
};

Vec6 to_vec6(Vec6 var0, double value, double index) {
    Vec6 var = var0;
    var(index) = value;
    return var;
};


int main(int argc, char *argv[]) {
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

    Vec6 vec_zero; vec_zero.setZero();
    Vec6 q_0;
    Vec6 q_des;
    Timer timer(arm._ctrlComp->dt);

    ////////////////////////////////
    // 0. Move to initial pose
    q_0 = arm.lowstate->getQ();
    q_des << 0.0, 1.09955743, -1.09955743, 1.57079633, 0.0, 0.0;

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

        ////////////////////////////////
        // 1.1 Trapezoidal experiment: `q_00 \pm delta_des`
        {
            log_file.open("data_trapezoidal_joint_" + std::to_string(k) +".txt");

            TrapezoidalTrajectory ramp;
            q_0 = arm.lowstate->getQ();

            // prepare initials for k-th joint
            double q_00 = q_0(k);
            double q_0des = q_0(k);

            double delta_des = M_PI; // desired delta offset

            for (int j(0); j < 10; ++j) { // change setpoint \times 10 
                
                // go to zero position at the end
                if (j  < 9) {
                    q_0des += delta_des;
                    delta_des *= -1;
                } else {
                    q_0des = 0;
                }

                // setup desired motion
                ramp.setup(q_00, q_0des, M_PI, 2*M_PI); // q0, q_des, dq_max, ddq_max

                // moving + waiting time
                double duration = 1.5 * ramp.get_time_total() / arm._ctrlComp->dt;

                for(int i(0); i < duration; i++){
                    // update desired motion
                    ramp.update(arm._ctrlComp->dt);

                    arm.q = to_vec6(q_0, ramp.q, k);
                    arm.qd = to_vec6(vec_zero, ramp.dq, k);
                    
                    // compensate gravity only
                    arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, Vec6::Zero(), Vec6::Zero(), Vec6::Zero());
                    
                    arm.setArmCmd(arm.q, arm.qd, arm.tau);
                    arm.sendRecv();
                    timer.sleep();
                }
            }
            log_file.close();
        }

        ////////////////////////////////
        // 1.2 Sinusoidal experiment: `q_00 + Acos(wt)`
        {
            // 1.2.1 0.2*cos(0.2t)
    
            std::vector<std::pair<double, double>> params = {{0.2, 0.2}, {0.1, 2}}; // where pair is <A, omega>
            
            for (int j(0); j < params.size(); ++j) {
                // save data separately for each sin-signal parameters
                log_file.open("data_sinusoidal_" + std::to_string(j) + "_joint_" + std::to_string(k) + ".txt");

                SinusoidalTrajectory sinusoid;
                q_0 = arm.lowstate->getQ();
                double q_00 = q_0(k);

                // new sinus parameters
                double A = params[j].first;
                double omega = params[j].second;

                double T = 2 * M_PI / omega;

                sinusoid.setup(q_00, A, omega);

                for(int i(0); i < 1000 * T; i++){ // 60 second for dt=0.01
                    sinusoid.update(arm._ctrlComp->dt);

                    arm.q = to_vec6(q_0, sinusoid.q, k);
                    arm.qd = to_vec6(vec_zero, sinusoid.dq, k);
                    
                    // compensate gravity only
                    arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, Vec6::Zero(), Vec6::Zero(), Vec6::Zero());
                    
                    arm.setArmCmd(arm.q, arm.qd, arm.tau);
                    arm.sendRecv();
                    timer.sleep();
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
    return 0;
}