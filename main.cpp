#include <iostream>
#include "extApi.h"
#include "Eigen/Dense"
#include "modern_robotics.h"
#include "YouBotFuncs.h"
#include "windows.h"

using namespace std;
using namespace mr;

int main()
{
	//构造齐次变换矩阵的临时变量R和p
	Eigen::Matrix3d T_R;
	Eigen::Vector3d T_p;

	cout << "程序开始" << endl;

	//先关闭所有连接
	simxFinish(-1);

	//连接VREP
	simxInt clientID = simxStart("127.0.0.1", 19999, 1, 1, 5000, 5);

	simxInt return_code;

	if (clientID != -1)
	{
		cout << "成功连接VREP" << endl;

		simxInt youBot_handle;
		return_code = simxGetObjectHandle(clientID, "youBot", &youBot_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object youBot ok" << endl;

		simxInt youBot_dummy_handle;
		return_code = simxGetObjectHandle(clientID, "youBotDummy", &youBot_dummy_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object youBotDummy ok" << endl;


		//为四个轮子准备初始值
		simxInt wheel_joints_handle[4] = {-1,-1,-1,-1};
		return_code = simxGetObjectHandle(clientID, "rollingJoint_fl", &wheel_joints_handle[0], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object rollingJoint_rl ok" << endl;
		return_code = simxGetObjectHandle(clientID, "rollingJoint_fr", &wheel_joints_handle[1], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object rollingJoint_fl ok" << endl;
		return_code = simxGetObjectHandle(clientID, "rollingJoint_rr", &wheel_joints_handle[2], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object rollingJoint_fr ok" << endl;
		return_code = simxGetObjectHandle(clientID, "rollingJoint_rl", &wheel_joints_handle[3], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object rollingJoint_rr ok" << endl;

		//准备五个手臂关节的初始值
		simxInt arm_joints_handle[5] = {-1,-1,-1,-1,-1};
		for(int i = 0; i < 5; i++)
		{
			string str = "Joint" + to_string(i+1);
			return_code = simxGetObjectHandle(clientID, str.c_str(), &wheel_joints_handle[i], simx_opmode_blocking);
			if(return_code == simx_return_ok) cout << "get object arm joint " << i+1 << " ok" << endl;
		}

		simxInt gripper_joint_1_handle;
		return_code = simxGetObjectHandle(clientID, "youBotGripperJoint1", &gripper_joint_1_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object gripper joint 1 ok" << endl;

		simxInt gripper_joint_2_handle;
		return_code = simxGetObjectHandle(clientID, "youBotGripperJoint2", &gripper_joint_2_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object gripper joint 2 ok" << endl;

		simxInt gripper_tip_handle;
		return_code = simxGetObjectHandle(clientID, "youBot_positionTip", &gripper_tip_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object gripper position tip ok" << endl;

		//初始关节角
		double desired_arm_joint_angles[5] = {0, 0, 0, 0, 0};

		//初始化关节
		for(int i = 0; i < 5; i++)
		{
			simxSetJointPosition(clientID, arm_joints_handle[i], (simxFloat)desired_arm_joint_angles[i], simx_opmode_blocking);
		}

		//获取底盘handles
		simxInt world_joints_handle[3] = {-1, -1, -1};

		return_code = simxGetObjectHandle(clientID, "World_X_Joint", &world_joints_handle[0], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object world joint X ok" << endl;

		return_code = simxGetObjectHandle(clientID, "World_Y_Joint", &world_joints_handle[1], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object world joint Y ok" << endl;

		return_code = simxGetObjectHandle(clientID, "World_Th_Joint", &world_joints_handle[2], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object world joint Th ok" << endl;

		//获取初始和目标物块handles
		simxInt cube_1_handle;
		return_code = simxGetObjectHandle(clientID, "Cube_1", &cube_1_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object cube_1 ok" << endl;

		simxInt cube_2_handle;
		return_code = simxGetObjectHandle(clientID, "Cube_2", &cube_2_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object cube_2 ok" << endl;

		simxInt cube_goal_handle;
		return_code = simxGetObjectHandle(clientID, "Cube_goal", &cube_goal_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object cube_goal_1 ok" << endl;

		/*生成轨迹并控制机器人*/

		//获得立方体的初始位置和目标位置
		simxFloat cube_1_position[3];
		simxGetObjectPosition(clientID, cube_1_handle, -1, cube_1_position, simx_opmode_blocking);
		simxFloat cube_2_position[3];
		simxGetObjectPosition(clientID, cube_2_handle, -1, cube_2_position, simx_opmode_blocking);
		simxFloat cube_goal_position[3];
		simxGetObjectPosition(clientID, cube_goal_handle, -1, cube_goal_position, simx_opmode_blocking);

		Eigen::MatrixXd T_sc_initial(4, 4);
		Eigen::MatrixXd T_sc_final(4, 4);

		//两个物块
		for(int i = 0; i < 2; i++)
		{
			//获取 youBot 位置
			simxFloat youBot_position[3];
			simxGetObjectPosition(clientID, youBot_handle, -1, youBot_position, simx_opmode_blocking);
			simxFloat youBot_x;
			simxGetJointPosition(clientID, world_joints_handle[0], &youBot_x, simx_opmode_blocking);
			simxFloat youBot_y;
			simxGetJointPosition(clientID, world_joints_handle[1], &youBot_y, simx_opmode_blocking);
			simxFloat youBot_theta;
			simxGetJointPosition(clientID, world_joints_handle[2], &youBot_theta, simx_opmode_blocking);

			//获取 youBot 手臂关节角度
			simxFloat youBot_arm_q[5] = {0, 0, 0, 0, 0};
			for(int j = 0; j < 5; j++)
			{
				simxGetJointPosition(clientID, arm_joints_handle[j], &youBot_arm_q[j], simx_opmode_blocking);
			}

			//获取 youBot 轮子角度
			simxFloat youBot_wheel_angles[4] = {0, 0, 0, 0};
			for(int j = 0; j < 4; j++)
			{
				simxGetJointPosition(clientID, wheel_joints_handle[j], &youBot_wheel_angles[j], simx_opmode_blocking);
			}

			//获取 youBot 夹具位置
			simxFloat youBot_gripper_position[3];
			simxGetObjectPosition(clientID, gripper_tip_handle, -1, youBot_gripper_position, simx_opmode_blocking);

			//初始化机器人配置
			//theta x  y  J1 J2 J3 J4 J5 w1 w2 w3 w4 gripper
			Eigen::MatrixXd config_initial(1, 13);//1x13
			config_initial << youBot_theta, youBot_x, youBot_y, youBot_arm_q[0], youBot_arm_q[1], youBot_arm_q[2], youBot_arm_q[3], youBot_arm_q[4], youBot_wheel_angles[0], youBot_wheel_angles[1], youBot_wheel_angles[2], youBot_wheel_angles[3], 0;

			//生成轨迹
			T_R = mr::roty(M_PI/2);
			T_p << youBot_gripper_position[0], youBot_gripper_position[1], youBot_gripper_position[2];
			Eigen::MatrixXd T_se_initial = RpToTrans(T_R, T_p);

			//两个物块的始末位姿
			if(i == 0)
			{
				T_R = Eigen::MatrixXd::Identity(3, 3);
				T_p << cube_1_position[0], cube_1_position[1], cube_1_position[2];
				T_sc_initial = RpToTrans(T_R, T_p);

				T_R = mr::rotz(3*M_PI/2);
				T_p << cube_goal_position[0], cube_goal_position[1], cube_goal_position[2];
				T_sc_final = RpToTrans(T_R, T_p);
			}
			else if(i == 1)
			{
				T_R = mr::rotz(0);
				T_p << cube_2_position[0], cube_2_position[1], cube_2_position[2];
				T_sc_initial = RpToTrans(T_R, T_p);

				T_R = mr::rotz(M_PI);
				T_p << cube_goal_position[0], cube_goal_position[1], cube_goal_position[2] + 0.05;
				T_sc_final = RpToTrans(T_R, T_p);
			}

			//夹具默认初始先绕y轴旋转90°到达水平位姿，再继续旋转60度。
			float gripper_pose = M_PI/3;
			//到达物块上方
			T_R = mr::roty(M_PI / 2) * mr::roty(gripper_pose);
			T_p << 0, 0, 0;
			Eigen::MatrixXd T_ce_grasp = RpToTrans(T_R, T_p);

			//下落0.1m
			T_p << 0, 0, 0.1;
			Eigen::MatrixXd T_ce_standoff = RpToTrans(T_R, T_p);

			std::vector<Eigen::MatrixXd> T_se_vector;

			//设置 dt
			float dt = 0.01;

			//夹具状态，0为打开，1为闭合
			Eigen::VectorXd gripper_state;
			trajectoryGenerator(T_se_initial,
			                    T_sc_initial,
			                    T_sc_final,
			                    T_ce_grasp,
			                    T_ce_standoff,
			                    dt,
			                    T_se_vector,
			                    gripper_state);

			//设置关节限制
			float joint_limit = 0;

			//设置 Kp, Ki
			float Kp = 2.2;
			float Ki = 0;

			/*主循环*/
			double desired_wheel_positions[4] = {0, 0, 0, 0};
			double desired_world_positions[3] = {0, 0, 0};
			Eigen::MatrixXd new_config = config_initial;//1x13

			cout << "正在移动第" << i+1 << "个物块" << endl;
			for(int j = 0; j < T_se_vector.size() - 1; j++)//最后一个没有next，故需要-1
			{
				//当前机时刻械手的参考Config
				Eigen::MatrixXd T_sed = T_se_vector.at(j);

				//下一机时刻械手的参考Config
				Eigen::MatrixXd T_sed_next = T_se_vector.at(j + 1);

				//计算速度speeds和X误差X_err
				Eigen::MatrixXd V(6,1);//6x1
				Eigen::MatrixXd speeds(1,9);//1x9
				Eigen::MatrixXd V_err;
				feedbackControl(new_config, T_sed, T_sed_next,
				                Kp, Ki, dt, joint_limit,
				                V, speeds, V_err);

				float speeds_max = 1000;

				//计算新配置config
				Eigen::MatrixXd new_config_12 = nextState(new_config,
														  speeds,
														  dt,
														  speeds_max);//1x12
				new_config << new_config_12, gripper_state(j);//1x13


				//准备要发送的命令
				desired_world_positions[0] = new_config(0, 1);
				desired_world_positions[1] = new_config(0, 2);
				desired_world_positions[2] = new_config(0, 0);

				desired_arm_joint_angles[0] = new_config(0, 3);
				desired_arm_joint_angles[1] = new_config(0, 4);
				desired_arm_joint_angles[2] = new_config(0, 5);
				desired_arm_joint_angles[3] = new_config(0, 6);
				desired_arm_joint_angles[4] = new_config(0, 7);

				desired_wheel_positions[0] = new_config(0, 8);
				desired_wheel_positions[1] = new_config(0, 9);
				desired_wheel_positions[2] = new_config(0, 10);
				desired_wheel_positions[3] = new_config(0, 11);

				//向VREP中的youBot模型发送命令
				VREP_armControl(clientID, arm_joints_handle, desired_arm_joint_angles);
				VREP_wheelsControl(clientID, wheel_joints_handle, desired_wheel_positions);
				VREP_WorldFrameControl(clientID, world_joints_handle, desired_world_positions);

				//设置夹具状态
				if(new_config(0,12) == 0)
				{
					//开
					simxSetJointTargetPosition(clientID, gripper_joint_1_handle, 0.015, simx_opmode_oneshot);
					simxSetJointTargetPosition(clientID, gripper_joint_2_handle, -0.015, simx_opmode_oneshot);
				}
				else
				{
					//合
					simxSetJointTargetPosition(clientID, gripper_joint_1_handle, 0.005, simx_opmode_oneshot);
					simxSetJointTargetPosition(clientID, gripper_joint_2_handle, -0.005, simx_opmode_oneshot);
				}
				Sleep(2);
			}
		}

		//断开与VREP的连接
		simxFinish(clientID);
		cout << "程序结束" << endl;
	}
	else
	{
		cout << "连接VREP失败";
	}

	return 0;
}