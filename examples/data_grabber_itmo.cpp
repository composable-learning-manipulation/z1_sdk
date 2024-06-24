#include "unitree_arm_sdk/control/unitreeArm.h"
#include <fstream>
#include <iostream>
#include <string>


using namespace UNITREE_ARM;

int main(int argc, char *argv[]) {
    std::cout << std::fixed << std::setprecision(3);
    bool hasGripper = false;
    unitreeArm arm(hasGripper);

    ///////////////////////////////
    // initial setup
    arm.sendRecvThread->start();

    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.setFsm(ArmFSMState::LOWCMD);

    std::vector<double> KP, KW;
    KP = arm._ctrlComp->lowcmd->kp;
    KW = arm._ctrlComp->lowcmd->kd;
    arm._ctrlComp->lowcmd->setControlGain(KP, KW);

    arm.sendRecvThread->shutdown();

    ///////////////////////////////
    //////// there are standard poses: 
    //////// - stow 0, 63, -63, 90, 0, 0
    //////// - pounce 0 90 -41 -52 0 0
    // 0. Move to stow initial position
    Vec6 q_0 = arm.lowstate->getQ();
    double duration = 1000;
    Vec6 q_stow;
    q_stow << 0.0, 1.09955743, -1.09955743, 1.57079633, 0.0, 0.0;
    Timer timer(arm._ctrlComp->dt);
    for(int i(0); i<duration; i++){
        arm.q = q_0 * (1-i/duration) + q_stow * (i/duration);
        arm.qd = (q_stow - q_0) / (duration * arm._ctrlComp->dt);
        arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
        
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.sendRecv();
        timer.sleep();
    }

    std::vector<int> joint_ids = {0, 5};

    for (auto k: joint_ids) {

        ///////////////////////////////
        // 1. move 1st joint by trapezoidal profile \times 10
        std::ofstream output_trapezoidal_joint_1("data_trapezoidal_joint_" + std::to_string(k) +".txt");

        q_0 = arm.lowstate->getQ();
        TrapezoidalTrajectory ramp;

        double q_0_0 = q_0(k);
        double q_0_des = q_0(k);
        double delta_des = M_PI;

        for (int j(0); j < 10; ++j) {
            q_0_des = q_0_des + delta_des;
            delta_des *= -1;

            ramp.setup(q_0_0, q_0_des, M_PI, 2*M_PI); // q0, q_des, dq_max, ddq_max
            duration = 1.1 * ramp.get_time_total() / arm._ctrlComp->dt;

            for(int i(0); i < duration; i++){
                double dt = arm._ctrlComp->dt;
                ramp.update(dt);

                arm.q = ramp.q;
                arm.qd = ramp.dq;
                arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
                
                arm.setArmCmd(arm.q, arm.qd, arm.tau);
                arm.sendRecv();
                timer.sleep();
            }
        }
        output_trapezoidal_joint_1.close();

        ///////////////////////////////
        // move 1st joint by sinusoidal profile \times 10

        // 2.1 0.2*sin(0.2t)
        std::ofstream output_sinusoidal_0_joint_1("data_sinusoidal_0_joint_" + std::to_string(k) + ".txt");
        q_0 = arm.lowstate->getQ();

        SinusoidalTrajectory sinusoid;

        q_0_0 = q_0(k);
        double A = 0.2;
        double omega = 0.2;
        double T = 2 * M_PI / omega;

        sinusoid.setup(q_0_0, A, omega);

        for(int i(0); i < 10 * T * 100; i++){
            double dt = arm._ctrlComp->dt;
            sinusoid.update(dt);

            arm.q = sinusoid.q;
            arm.qd = sinusoid.dq;
            arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
            
            arm.setArmCmd(arm.q, arm.qd, arm.tau);
            arm.sendRecv();
            timer.sleep();
        }
        output_sinusoidal_0_joint_1.close();

        // 2.2 0.1*sin(2t)
        std::ofstream output_sinusoidal_1_joint_1("data_sinusoidal_1_joint_" + std::to_string(k) + ".txt");
        q_0 = arm.lowstate->getQ();

        q_0_0 = q_0(k);
        double A = 0.2;
        double omega = 0.2;
        double T = 2 * M_PI / omega;

        sinusoid.setup(q_0_0, A, omega);

        for(int i(0); i < 10 * T * 100; i++){
            double dt = arm._ctrlComp->dt;
            sinusoid.update(dt);

            arm.q = sinusoid.q;
            arm.qd = sinusoid.dq;
            arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
            
            arm.setArmCmd(arm.q, arm.qd, arm.tau);
            arm.sendRecv();
            timer.sleep();
        }
        output_sinusoidal_1_joint_1.close();        
    }



    // finish him
    arm.sendRecvThread->start();
    arm.setFsm(ArmFSMState::JOINTCTRL);
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    return 0;
}