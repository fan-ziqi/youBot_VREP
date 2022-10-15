#include "YouBotFuncs.h"
#include <iostream>

using namespace std;
using namespace mr;

//VREP control function definition

//Arm control function of youBot
void VREP_armControl(const simxInt clientID,
                     const simxInt* arm_joints_handle,
                     const double* desired_arm_joint_angles)
{
	for(int i = 0; i < 5; i++)
	{
		simxSetJointPosition(clientID,
		                     arm_joints_handle[i],
							 (simxFloat)desired_arm_joint_angles[i],
							 simx_opmode_oneshot);
	}
}
//Wheels control function of youBot
void VREP_wheelsControl(const simxInt clientID,
                        const simxInt* wheel_joints_handle,
                        const double* desired_wheel_positions)
{
	for(int i = 0; i < 4; i++)
	{
		simxSetJointPosition(clientID,
		                     wheel_joints_handle[i],
		                     (simxFloat)desired_wheel_positions[i],
		                     simx_opmode_oneshot);
	}
}
//World frame control function of youBot
void VREP_WorldFrameControl(const simxInt clientID,
                            const simxInt* world_joints_handle,
                            const double* desired_world_positions)
{
	for(int i = 0; i < 3; i++)
	{
		simxSetJointPosition(clientID,
		                     world_joints_handle[i],
		                     (simxFloat)desired_world_positions[i],
		                     simx_opmode_oneshot);
	}
}



void trajectoryGenerator(const Eigen::MatrixXd& T_se_Start,
						 const Eigen::MatrixXd& T_sc_Start,
						 const Eigen::MatrixXd& T_sc_End,
						 const Eigen::MatrixXd& T_ce_Down,
						 const Eigen::MatrixXd& T_ce_Up,
						 float dt,
						 std::vector<Eigen::MatrixXd>& T_se_vector,
						 Eigen::VectorXd& gripper_state)
{
	//时间设置
	double time_Start_to_StartUp = 5;
	double time_StartUp_to_StartDown = 2;
	double time_StartDown = 1;
	double time_StartDown_to_StartUp = 2;
	double time_StartUp_to_EndUp = 8;
	double time_EndUp_to_EndDown = 2;
	double time_EndDown = 1;
	double time_EndDown_to_EndUp = 2;

	//每秒钟轨迹规划的点数
	int points_per_sec = (int)(1/dt);

	/*第1段：将夹具从当前位置移动到物块初始位置上方几cm处的Up位置，共5秒*/
	//在空间坐标系中表示的夹具在物块初始Up位置的齐次矩阵
	Eigen::MatrixXd T_se_Up_Start = T_sc_Start * T_ce_Up;
	//生成到达物块初始Up位置的轨迹
	std::vector<Eigen::MatrixXd> T_se_seg1 = mr::CartesianTrajectory(T_se_Start,
																	 T_se_Up_Start,
																	 time_Start_to_StartUp,
																	 (int)(time_Start_to_StartUp * points_per_sec),
																	 3);
	//加入第1段轨迹
	T_se_vector.insert(T_se_vector.end(), T_se_seg1.begin(), T_se_seg1.end());

	/*第2段：将夹具向下移动到物块初始Down位置，共2秒*/
	//在空间坐标系中表示的夹具在物块初始Down位置的齐次矩阵
	Eigen::MatrixXd T_se_Down = T_sc_Start * T_ce_Down;
	//生成到达物块初始Down位置的轨迹
	std::vector<Eigen::MatrixXd> T_se_seg2 = mr::CartesianTrajectory(T_se_seg1.back(),
																	 T_se_Down,
																	 time_StartUp_to_StartDown,
																	 (int)(time_StartUp_to_StartDown * points_per_sec),
																	 3);
	//加入第2段轨迹
	T_se_vector.insert(T_se_vector.end(), T_se_seg2.begin(), T_se_seg2.end());

	/*第3段：闭合夹具，共1秒*/
	//把轨迹的最后一个位置重复100遍（1秒）
	T_se_vector.insert(T_se_vector.end(), (int)(time_StartDown * points_per_sec), T_se_vector.back());

	/*第4段：向上移动夹具，回到物块初始Up位置，共2秒*/
	//生成到达物块初始Up位置的轨迹
	std::vector<Eigen::MatrixXd> T_se_segment_4 = mr::CartesianTrajectory(T_se_Down,
																		  T_se_Up_Start,
																		  time_StartDown_to_StartUp,
																		  (int)(time_StartDown_to_StartUp * points_per_sec),
																		  3);
	//加入第4段轨迹
	T_se_vector.insert(T_se_vector.end(), T_se_segment_4.begin(), T_se_segment_4.end());

	/*第5段：将夹具从当前位置移动到物块目标位置上方几cm处的Up位置，共8秒*/
	//在空间坐标系中表示的夹具在物块目标Up位置的齐次矩阵
	Eigen::MatrixXd T_se_Up_End = T_sc_End * T_ce_Up;
	//生成到达物块目标Up位置的齐次矩阵
	std::vector<Eigen::MatrixXd> T_se_segment_5 = mr::CartesianTrajectory(T_se_Up_Start,
																		  T_se_Up_End,
																		  time_StartUp_to_EndUp,
																		  (int)(time_StartUp_to_EndUp * points_per_sec),
																		  3);
	//加入第5段轨迹
	T_se_vector.insert(T_se_vector.end(), T_se_segment_5.begin(), T_se_segment_5.end());

	/*第6段：将夹具向下移动到物块目标Down位置，共2秒*/
	//在空间坐标系中表示的夹具在物块目标Down位置的齐次矩阵
	Eigen::MatrixXd T_se_lose = T_sc_End * T_ce_Down;
	//生成到达物块目标Down位置的轨迹
	std::vector<Eigen::MatrixXd> T_se_segment_6 = mr::CartesianTrajectory(T_se_Up_End,
																		  T_se_lose,
																		  time_EndUp_to_EndDown,
																		  (int)(time_EndUp_to_EndDown * points_per_sec),
																		  3);
	//加入第6段轨迹
	T_se_vector.insert(T_se_vector.end(), T_se_segment_6.begin(), T_se_segment_6.end());

	/*第7段：打开夹具，共1秒*/
	//把轨迹的最后一个位置重复100遍（1秒）
	T_se_vector.insert(T_se_vector.end(), (int)(time_EndDown * points_per_sec), T_se_vector.back());

	/*第8段：向上移动夹具，回到物块目标Up位置，共2秒*/
	//生成到达物块目标Up位置的轨迹
	std::vector<Eigen::MatrixXd> T_se_segment_8 = mr::CartesianTrajectory(T_se_vector.back(),
	                                                                      T_se_Up_End,
	                                                                      time_EndDown_to_EndUp,
	                                                                      (int)(time_EndDown_to_EndUp * points_per_sec),
	                                                                      3);
	//加入第8段轨迹
	T_se_vector.insert(T_se_vector.end(), T_se_segment_8.begin(), T_se_segment_8.end());

	//轨迹总点数
	int num_points =(int)((
			time_Start_to_StartUp +
			time_StartUp_to_StartDown +
			time_StartDown +
			time_StartDown_to_StartUp +
			time_StartUp_to_EndUp +
			time_EndUp_to_EndDown +
			time_EndDown +
			time_EndDown_to_EndUp) * points_per_sec);

	//夹具状态，0为打开，1为闭合
	Eigen::VectorXd _gripper_state = Eigen::VectorXd::Zero(num_points);
	//在第3~6段闭合夹具
	double time_close_gripper = time_Start_to_StartUp + time_StartUp_to_StartDown;
	double time_open_gripper = time_close_gripper + time_StartDown + time_StartDown_to_StartUp + time_StartUp_to_EndUp + time_EndUp_to_EndDown + time_EndDown;
	for(int i = (int)(time_close_gripper * points_per_sec); i < (int)((time_open_gripper) * points_per_sec); i++)
	{
		_gripper_state(i) = 1;
	}
	gripper_state = _gripper_state;
}

void feedbackControl(Eigen::MatrixXd config, //1x13
					 const Eigen::MatrixXd& T_sed,
					 const Eigen::MatrixXd& T_sed_next,
					 float Kp, float Ki, float dt,
					 float joint_limit,
					 Eigen::MatrixXd& V_t,//6x1
					 Eigen::MatrixXd& speeds, //1x9
					 Eigen::MatrixXd& V_err)
{
	//构造齐次变换矩阵的临时变量R和p
	Eigen::Matrix3d T_R;
	Eigen::Vector3d T_p;

	Eigen::MatrixXd Blist(6,5);
	Blist <<
		0,       0,       0,       0, 0,
		0,      -1,      -1,      -1, 0,
		1,       0,       0,       0, 1,
		0, -0.5076, -0.3526, -0.2176, 0,
		0.033,   0,       0,       0, 0,
		0,       0,       0,       0, 0;
	Eigen::VectorXd thetalist(5);
	thetalist << config.block(0,3,1,5).transpose();

	/*1 计算当前末端位姿*/
	//1.1 计算底盘相对固定坐标系的位姿 T_sb ，与phi、x、y有关。z项为底盘高度，为定值
	Eigen::MatrixXd T_sb(4, 4);
	T_R = mr::rotz(config(0,0));
	T_p << config(0,1), config(0,2), 0.0963;
	T_sb = RpToTrans(T_R, T_p);
	//1.2 计算机械臂末端到底盘的变换矩阵 T_b0 ，为固定值
	Eigen::MatrixXd T_b0(4, 4);
	T_R = Eigen::MatrixXd::Identity(3, 3);
	T_p << 0.1662, 0, 0.0026;
	T_b0 = RpToTrans(T_R, T_p);
	//1.3 机械臂正运动学 T_0e ，与5个关节角有关
	//末端初始位形
	Eigen::MatrixXd M(4, 4);
	T_R = Eigen::MatrixXd::Identity(3, 3);
	T_p << 0.033, 0, 0.6546;
	M = RpToTrans(T_R, T_p);
	Eigen::MatrixXd T_0e = mr::FKinBody(M, Blist, thetalist);
	//1.4 连乘，计算当前末端位姿
	Eigen::MatrixXd T_se = T_sb * T_b0 * T_0e;

	//计算目标运动旋量 V_ed (计算当前位置与下一目标点之间的矩阵对数，再从中提取出运动旋量)
	Eigen::MatrixXd V_ed = mr::se3ToVec((1/dt)*mr::MatrixLog6(T_sed.inverse() * T_sed_next));

	//计算运动旋量 V_e = Ad[inv(T_se) * T_sed] * V_ed
	Eigen::MatrixXd V_e = mr::Adjoint(T_se.inverse() * T_sed) * V_ed;//6x6 * 6x1 = 6x1

	//计算 V_err = T_eed
	V_err = mr::se3ToVec(mr::MatrixLog6(T_se.inverse() * T_sed));

	//计算用于控制的旋量函数 V(t)
//	V_t = V_e + Kp * V_err + Ki * (V_err + V_err * dt);//6x1
	V_t = V_e + Kp * V_err;//6x1 暂时只使用P控制

	/*计算J_e*/
	//首先算出J_arm
	Eigen::MatrixXd J_arm = mr::JacobianBody(Blist, thetalist);//6x5
	//然后算出J_base
	double wheel_r = 0.0475;//轮子半径
	double wheel_l = 0.47/2;
	double wheel_w = 0.3/2;
	double wheel_gamma[4] = {-M_PI/4, M_PI/4, -M_PI/4, M_PI/4};//辊子角度
	double wheel_beta[4] = {0, 0, 0, 0};//轮子驱动方向
	double wheel_x[4] = {wheel_l, wheel_l, -wheel_l, -wheel_l};
	double wheel_y[4] = {wheel_w, -wheel_w, -wheel_w, wheel_w};

	/*计算底盘雅可比*/
	//计算四个轮子的h0，拼成H0
	Eigen::MatrixXd hi_trans1(1,2);
	Eigen::MatrixXd hi_trans2(2,2);
	Eigen::MatrixXd hi_trans3(2,3);
	std::vector<Eigen::Matrix<double,1,3>> h0(4);
	for(int i = 0; i < 4; i++)
	{
		hi_trans1 << 1/wheel_r, tan(wheel_gamma[i])/wheel_r;
		hi_trans2 <<
		                cos(wheel_beta[i]), sin(wheel_beta[i]),
				-sin(wheel_beta[i]), cos(wheel_beta[i]);
		hi_trans3 <<
		                -wheel_y[i], 1, 0,
				wheel_x[i], 0, 1;
		h0.at(i) = hi_trans1 * hi_trans2 * hi_trans3;
	}
	Eigen::MatrixXd H0(4, 3);
	H0 <<
		h0.at(0),
		h0.at(1),
		h0.at(2),
		h0.at(3);

	//计算F6
	Eigen::MatrixXd F = pinv(H0);//3x4
	Eigen::MatrixXd F6 = Eigen::MatrixXd::Zero(6, 4);
	F6.row(2) = F.block(0,0,1,4);
	F6.row(3) = F.block(1,0,1,4);
	F6.row(4) = F.block(2,0,1,4);

	//计算J_base
	Eigen::MatrixXd T_eb = T_se.inverse() * T_sb;
	Eigen::MatrixXd J_base = mr::Adjoint(T_eb) * F6;

	//关节限制
	if(joint_limit == 1) jointLimits(config,J_arm);

	//最后计算J_e
	Eigen::MatrixXd J_e(J_base.rows(), J_base.cols()+J_arm.cols());
	J_e << J_arm, J_base;//6x9

	//计算速度speeds(1x9)[5arm,4wheel]
	speeds = (pinv(J_e, 1e-2) * V_t).transpose();//(9x6 * 6x1)' = 1x9
}

Eigen::MatrixXd nextState(Eigen::MatrixXd& config, //1x13
						  Eigen::MatrixXd& speeds, //1x9
						  const float delta_t,
						  const float speed_max)
{
	//速度限制
	for(int i = 0; i < speeds.cols(); i++)
	{
		if(speeds(0, i) > speed_max) speeds(0, i) = speed_max;
		else if(speeds(0, i) < -speed_max) speeds(0, i) = -speed_max;
	}

	//获得新的机械臂和轮子的config
	Eigen::MatrixXd new_angle_config = config.block(0,3,1,9) + speeds * delta_t;//1x9

	//使用里程计计算底盘的config
	//第1步：计算delta_theta，即车轮角度的变化
	Eigen::MatrixXd delta_theta = speeds.block(0, 5, 1, 4).transpose() * delta_t;//4x1

	//第2步：计算轮速
	Eigen::MatrixXd theta_dot = delta_theta;//4x1

	//第3步：计算底盘旋量Vb
	float r = 0.0475;
	float l = 0.47/2;
	float w = 0.3/2;
	Eigen::MatrixXd _Vb(3,4);
	_Vb <<
		-1/(l+w),1/(l+w),1/(l+w),-1/(l+w),
		1,1,1,1,
		-1,1,-1,1;
	Eigen::MatrixXd Vb = (r/4) * _Vb * theta_dot;//3x4 * 4x1 = 3x1

	//step4: calculate new chassis config
	//here we write Down dqb
	Eigen::MatrixXd dqb(3,1);
	if(Vb(0,0) == 0)
	{
		dqb <<
			0,
			Vb(1,0),
			Vb(2,0);

	}
	else if(Vb(0,0) != 0)
	{
		dqb <<
		Vb(0,0),
		(Vb(1,0)*sin(Vb(0,0))+Vb(2,0)*(cos(Vb(0,0))-1))/Vb(0,0),
		(Vb(2,0)*sin(Vb(0,0))+Vb(1,0)*(1-cos(Vb(0,0))))/Vb(0,0);
	}
	//here we write Down dq
	Eigen::MatrixXd _dq(3,3);
	_dq <<
		1,0,0,
		0,cos(config(0,0)),-sin(config(0,0)),
		0,sin(config(0,0)),cos(config(0,0));
	Eigen::MatrixXd dq = _dq * dqb;//3x1
	//here we write Down new chassis config
	Eigen::MatrixXd new_chassis_config = config.block(0,0,1,3) + dq.transpose();//1x3

	//here we put the angle and chassis configuration together
	Eigen::MatrixXd new_config(1, 12);//1x12
	new_config << new_chassis_config, new_angle_config;

	return new_config;
}

void jointLimits(const Eigen::MatrixXd& config,
				 Eigen::MatrixXd J_arm)
{
	Eigen::VectorXd Limit(6);
	Limit << 0,0,0,0,0,0;
	if(config(0,5) > -0.1)
	{
		J_arm.col(2) = Limit;
	}
	if(config(0,6) > -0.1)
	{
		J_arm.col(3) = Limit;
	}
}



