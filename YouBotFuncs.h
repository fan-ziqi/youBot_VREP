#ifndef YOUBOT_YOUBOTFUNCS_H
#define YOUBOT_YOUBOTFUNCS_H
#include "Eigen/Dense"
#include "modern_robotics.h"
#include"extApi.h"

void trajectoryGenerator(const Eigen::MatrixXd& T_se_Start,
                         const Eigen::MatrixXd& T_sc_Start,
                         const Eigen::MatrixXd& T_sc_End,
                         const Eigen::MatrixXd& T_ce_Down,
                         const Eigen::MatrixXd& T_ce_Up,
                         float dt,
                         std::vector<Eigen::MatrixXd>& T_se_vector,
                         Eigen::VectorXd& gripper_state);

void feedbackControl(Eigen::MatrixXd config,
                     const Eigen::MatrixXd& T_sed,
                     const Eigen::MatrixXd& T_sed_next,
                     float Kp, float Ki, float dt,
                     float joint_limit,
                     Eigen::MatrixXd& V_t,
                     Eigen::MatrixXd& speeds,
                     Eigen::MatrixXd& V_err);

Eigen::MatrixXd nextState(Eigen::MatrixXd& config,
                          Eigen::MatrixXd& speeds,
                          float delta_t,
                          float speed_max);

void VREP_armControl(simxInt clientID,
                     const simxInt* arm_joints_handle,
                     const double* desired_arm_joint_angles);

void VREP_wheelsControl(simxInt clientID,
                        const simxInt* wheel_joints_handle,
                        const double* desired_wheel_positions);

void VREP_WorldFrameControl(simxInt clientID,
                            const simxInt* world_joints_handle,
                            const double* desired_world_positions);

void jointLimits(const Eigen::MatrixXd& config,
                 Eigen::MatrixXd J_arm);

#endif //YOUBOT_YOUBOTFUNCS_H
