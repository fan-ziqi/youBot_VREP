#include <iostream>
#include <iomanip>
#include "extApi.h"
#include "Eigen/Dense"
#include "modern_robotics.h"
#include "YouBotFuncs.h"
#include "windows.h"

using namespace std;
using namespace mr;

int main()
{

	//������α任�������ʱ����R��p
	Eigen::Matrix3d T_R;
	Eigen::Vector3d T_p;

	cout << "����ʼ" << endl;

	//�ȹر���������
	simxFinish(-1);

	//����VREP
	simxInt clientID = simxStart("127.0.0.1", 19999, 1, 1, 5000, 5);

	simxInt return_code;

	if (clientID != -1)
	{
		cout << "�ɹ�����VREP" << endl;

		simxInt youBot_handle;
		return_code = simxGetObjectHandle(clientID, "youBot", &youBot_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object youBot ok" << endl;

		simxInt youBot_dummy_handle;
		return_code = simxGetObjectHandle(clientID, "youBotDummy", &youBot_dummy_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object youBotDummy ok" << endl;


		//Ϊ�ĸ�����׼����ʼֵ
		simxInt wheel_joints_handle[4] = {-1,-1,-1,-1};
		return_code = simxGetObjectHandle(clientID, "rollingJoint_fl", &wheel_joints_handle[0], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object rollingJoint_rl ok" << endl;
		return_code = simxGetObjectHandle(clientID, "rollingJoint_fr", &wheel_joints_handle[1], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object rollingJoint_fl ok" << endl;
		return_code = simxGetObjectHandle(clientID, "rollingJoint_rr", &wheel_joints_handle[2], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object rollingJoint_fr ok" << endl;
		return_code = simxGetObjectHandle(clientID, "rollingJoint_rl", &wheel_joints_handle[3], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object rollingJoint_rr ok" << endl;

		simxInt arm_joints_handle[5] = {-1,-1,-1,-1,-1};
		for(int i = 0; i < 5; i++)
		{
			string str = "Joint" + to_string(i+1);
			return_code = simxGetObjectHandle(clientID, str.c_str(), &arm_joints_handle[i], simx_opmode_blocking);
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

		//��ʼ�ؽڽ�
		double desired_arm_joint_angles[5] = {0, 0, 0, 0, 0};

		//��ʼ���ؽ�
		for(int i = 0; i < 5; i++)
		{
			simxSetJointPosition(clientID, arm_joints_handle[i], (simxFloat)desired_arm_joint_angles[i], simx_opmode_blocking);
		}

		//��ȡ����handles
		simxInt world_joints_handle[3] = {-1, -1, -1};

		return_code = simxGetObjectHandle(clientID, "World_X_Joint", &world_joints_handle[0], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object world joint X ok" << endl;

		return_code = simxGetObjectHandle(clientID, "World_Y_Joint", &world_joints_handle[1], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object world joint Y ok" << endl;

		return_code = simxGetObjectHandle(clientID, "World_Th_Joint", &world_joints_handle[2], simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object world joint Th ok" << endl;

		//��ȡ��ʼ��Ŀ�����handles
		simxInt cube_1_handle;
		return_code = simxGetObjectHandle(clientID, "Cube_1", &cube_1_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object cube_1 ok" << endl;

		simxInt cube_2_handle;
		return_code = simxGetObjectHandle(clientID, "Cube_2", &cube_2_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object cube_2 ok" << endl;

		simxInt cube_goal_handle;
		return_code = simxGetObjectHandle(clientID, "Cube_goal", &cube_goal_handle, simx_opmode_blocking);
		if(return_code == simx_return_ok) cout << "get object cube_goal_1 ok" << endl;

		/*���ɹ켣�����ƻ�����*/

		//���������ĳ�ʼλ�ú�Ŀ��λ��
		simxFloat cube_1_position[3];
		simxGetObjectPosition(clientID, cube_1_handle, -1, cube_1_position, simx_opmode_blocking);
		simxFloat cube_2_position[3];
		simxGetObjectPosition(clientID, cube_2_handle, -1, cube_2_position, simx_opmode_blocking);
		simxFloat cube_goal_position[3];
		simxGetObjectPosition(clientID, cube_goal_handle, -1, cube_goal_position, simx_opmode_blocking);
		simxFloat cube_1_orientation[3];
		simxSetObjectOrientation(clientID, cube_1_handle, -1, cube_1_orientation, simx_opmode_blocking);
		simxFloat cube_2_orientation[3];
		simxSetObjectOrientation(clientID, cube_2_handle, -1, cube_2_orientation, simx_opmode_blocking);
		simxFloat cube_goal_orientation[3];
		simxSetObjectOrientation(clientID, cube_goal_handle, -1, cube_goal_orientation, simx_opmode_blocking);

		Eigen::MatrixXd T_sc_initial(4, 4);
		Eigen::MatrixXd T_sc_final(4, 4);

		//�������
		for(int i = 0; i < 2; i++)
		{
			//��ȡ youBot λ��
			simxFloat youBot_position[3];
			simxGetObjectPosition(clientID, youBot_handle, -1, youBot_position, simx_opmode_blocking);
			simxFloat youBot_x;
			simxGetJointPosition(clientID, world_joints_handle[0], &youBot_x, simx_opmode_blocking);
			simxFloat youBot_y;
			simxGetJointPosition(clientID, world_joints_handle[1], &youBot_y, simx_opmode_blocking);
			simxFloat youBot_phi;
			simxGetJointPosition(clientID, world_joints_handle[2], &youBot_phi, simx_opmode_blocking);

			//��ȡ youBot �ֱ۹ؽڽǶ�
			simxFloat youBot_arm_q[5] = {0, 0, 0, 0, 0};
			for(int j = 0; j < 5; j++)
			{
				simxGetJointPosition(clientID, arm_joints_handle[j], &youBot_arm_q[j], simx_opmode_blocking);
			}

			//��ȡ youBot ���ӽǶ�
			simxFloat youBot_wheel_angles[4] = {0, 0, 0, 0};
			for(int j = 0; j < 4; j++)
			{
				simxGetJointPosition(clientID, wheel_joints_handle[j], &youBot_wheel_angles[j], simx_opmode_blocking);
			}

			//��ȡ youBot �о�λ��
			simxFloat youBot_gripper_position[3];
			simxGetObjectPosition(clientID, gripper_tip_handle, -1, youBot_gripper_position, simx_opmode_blocking);

			//��ʼ������������
			//phi x  y  J1 J2 J3 J4 J5 w1 w2 w3 w4 gripper
			Eigen::VectorXd config_initial(13);//1x13
			config_initial << youBot_phi, youBot_x, youBot_y, youBot_arm_q[0], youBot_arm_q[1], youBot_arm_q[2], youBot_arm_q[3], youBot_arm_q[4], youBot_wheel_angles[0], youBot_wheel_angles[1], youBot_wheel_angles[2], youBot_wheel_angles[3], 0;

			//���ɹ켣
			T_R = mr::roty(M_PI/2);
			T_p << youBot_gripper_position[0], youBot_gripper_position[1], youBot_gripper_position[2];
			Eigen::MatrixXd T_se_initial = RpToTrans(T_R, T_p);

			//��������ʼĩλ��
			if(i == 0)
			{
				T_R = mr::rotz(cube_1_orientation[2]);
				T_p << cube_1_position[0], cube_1_position[1], cube_1_position[2];
				T_sc_initial = RpToTrans(T_R, T_p);

				T_R = mr::rotz(cube_goal_orientation[2]);
				T_p << cube_goal_position[0], cube_goal_position[1], cube_goal_position[2];
				T_sc_final = RpToTrans(T_R, T_p);
			}
			else if(i == 1)
			{
				T_R = mr::rotz(cube_2_orientation[2]);
				T_p << cube_2_position[0], cube_2_position[1], cube_2_position[2];
				T_sc_initial = RpToTrans(T_R, T_p);

				T_R = mr::rotz(cube_goal_orientation[2]);
				T_p << cube_goal_position[0], cube_goal_position[1], cube_goal_position[2] + 0.05;
				T_sc_final = RpToTrans(T_R, T_p);
			}

			//�о�Ĭ�ϳ�ʼ����y����ת90�㵽��ˮƽλ�ˣ��ټ�����ת60�ȡ�
			float gripper_pose = M_PI/3;
			//��������Ϸ�
			T_R = mr::roty(M_PI / 2) * mr::roty(gripper_pose);
			T_p << 0, 0, 0;
			Eigen::MatrixXd T_ce_grasp = RpToTrans(T_R, T_p);

			//����0.1m
			T_p << 0, 0, 0.1;
			Eigen::MatrixXd T_ce_standoff = RpToTrans(T_R, T_p);

			std::vector<Eigen::MatrixXd> T_se_vector;

			//����ʱ���� dt
			float dt = 0.01;

			//�о�״̬��0Ϊ�򿪣�1Ϊ�պ�
			Eigen::VectorXd gripper_state;
			trajectoryGenerator(T_se_initial,
			                    T_sc_initial,
			                    T_sc_final,
			                    T_ce_grasp,
			                    T_ce_standoff,
			                    dt,
			                    T_se_vector,
			                    gripper_state);

			//���ùؽ����ٶ�����
			float joint_limit = 1;
			float speeds_max = 1000;
			//���� Kp, Ki
			float Kp = 2.2;
			float Ki = 0;

			/*��ѭ��*/
			double desired_wheel_positions[4] = {0, 0, 0, 0};
			double desired_world_positions[3] = {0, 0, 0};
			Eigen::VectorXd new_config = config_initial;//13x1

			cout << "�����ƶ���" << i+1 << "�����" << endl;
			for(int j = 0; j < T_se_vector.size() - 1; j++)//���һ��û��next������Ҫ-1
			{
				//��ǰ��ʱ��е�ֵĲο�Config
				Eigen::MatrixXd T_sed = T_se_vector.at(j);

				//��һ��ʱ��е�ֵĲο�Config
				Eigen::MatrixXd T_sed_next = T_se_vector.at(j + 1);

				//�����ٶ�speeds
				Eigen::VectorXd speeds = FeedbackControl(new_config, T_sed, T_sed_next, Kp, Ki, dt, joint_limit);//9x1

				//����������config
				Eigen::VectorXd new_config_12 = CountOdom(new_config, speeds, dt, speeds_max);//12x1
				new_config << new_config_12, gripper_state(j);//13x1

				//׼��Ҫ���͵�����
				desired_world_positions[0] = new_config(1);
				desired_world_positions[1] = new_config(2);
				desired_world_positions[2] = new_config(0);

				desired_arm_joint_angles[0] = new_config(3);
				desired_arm_joint_angles[1] = new_config(4);
				desired_arm_joint_angles[2] = new_config(5);
				desired_arm_joint_angles[3] = new_config(6);
				desired_arm_joint_angles[4] = new_config(7);

				desired_wheel_positions[0] = new_config(8);
				desired_wheel_positions[1] = new_config(9);
				desired_wheel_positions[2] = new_config(10);
				desired_wheel_positions[3] = new_config(11);

				//��VREP�е�youBotģ�ͷ�������
				VREP_armControl(clientID, arm_joints_handle, desired_arm_joint_angles);
				VREP_wheelsControl(clientID, wheel_joints_handle, desired_wheel_positions);
				VREP_WorldFrameControl(clientID, world_joints_handle, desired_world_positions);

				//���üо�״̬
				if(new_config(12) == 0)
				{
					//��
					simxSetJointTargetPosition(clientID, gripper_joint_1_handle, 0.015, simx_opmode_oneshot);
					simxSetJointTargetPosition(clientID, gripper_joint_2_handle, -0.015, simx_opmode_oneshot);
				}
				else
				{
					//��
					simxSetJointTargetPosition(clientID, gripper_joint_1_handle, 0.005, simx_opmode_oneshot);
					simxSetJointTargetPosition(clientID, gripper_joint_2_handle, -0.005, simx_opmode_oneshot);
				}
				Sleep(2);
			}
		}
		Sleep(100);
		//�Ͽ���VREP������
		simxFinish(clientID);
		cout << "�������" << endl;
	}
	else
	{
		cout << "����VREPʧ��";
	}

	return 0;
}