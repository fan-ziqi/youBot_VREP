#include "MRTest.h"

#include <iostream>
#include "Eigen/Dense"
#include "modern_robotics.h"

using namespace mr;
using namespace std;

/*--------------------��3�� �����˶� P69--------------------*/

void RotInvTest()
{
	//��ת����R
	Eigen::MatrixXd R(3, 3);
	R <<
		0, 0, 1,
		1, 0, 0,
		0, 1, 0;

	//�����������
	Eigen::MatrixXd invR = mr::RotInv(R);

	//��ȷ���
	Eigen::MatrixXd result(3, 3);
	result <<
		0, 1, 0,
		0, 0, 1,
		1, 0, 0;

	cout << "RotInvTest" << endl
	     << "The result is:" << endl << invR << endl
	     << "It should be:" << endl << result << endl;
}

void VecToso3Test()
{
	//��ά����(���ٶ�ʸ��)
	Eigen::Vector3d omg(1, 2, 3);

	//���ɷ��Գƾ���
	Eigen::Matrix3d so3mat = mr::VecToso3(omg);

	//��ȷ���
	Eigen::Matrix3d result(3, 3);
	result <<
		 0, -3,  2,
		 3,  0, -1,
		-2,  1,  0;

	cout << "VecToso3Test" << endl
	     << "The result is:" << endl << so3mat << endl
	     << "It should be:" << endl << result << endl;
}

void so3ToVecTest()
{
	//���Գƾ���
	Eigen::Matrix3d so3mat(3, 3);
	so3mat <<
		 0, -3,  2,
		 3,  0, -1,
		-2,  1,  0;

	//������ά����(���ٶ�ʸ��)
	Eigen::Vector3d omg = mr::so3ToVec(so3mat);

	//��ȷ���
	Eigen::Vector3d result(1, 2, 3);

	cout << "so3ToVecTest" << endl
	     << "The result is:" << endl << omg << endl
	     << "It should be:" << endl << result << endl;
}

void AxisAng3Test()
{
	//��ת��ָ������
	Eigen::Vector3d expc3(1, 2, 3);

	//��ת�����ת�Ƕ� [x, y, z, theta]
	Eigen::Vector4d omega_and_theta = mr::AxisAng3(expc3);

	//��ȷ���
	Eigen::Vector4d result(0.2673, 0.5345, 0.8018, 3.7417);

	cout << "AxisAng3Test" << endl
	     << "The result is:" << endl << omega_and_theta << endl
	     << "It should be:" << endl << result << endl;
}

void MatrixExp3Test()
{
	//��ת�ľ���ָ��
	Eigen::Matrix3d so3mat(3, 3);
	so3mat <<
		 0, -3,  2,
		 3,  0, -1,
		-2,  1,  0;

	//��ת����R
	Eigen::MatrixXd R = mr::MatrixExp3(so3mat);

	//��ȷ���
	Eigen::MatrixXd result(3, 3);
	result <<
		-0.6949,  0.7135, 0.0893,
		-0.1920, -0.3038, 0.9332,
		 0.6930,  0.6313, 0.3481;

	cout << "MatrixExp3Test" << endl
	     << "The result is:" << endl << R << endl
	     << "It should be:" << endl << result << endl;
}

void MatrixLog3Test()
{
	//��ת����R
	Eigen::Matrix3d R(3, 3);
	R <<
		0, 0, 1,
		1, 0, 0,
		0, 1, 0;

	//��ת�ľ������
	Eigen::Matrix3d so3mat = mr::MatrixLog3(R);

	//��ȷ���
	Eigen::MatrixXd result(3, 3);
	result <<
		      0, -1.2092,  1.2092,
		 1.2092,       0, -1.2092,
		-1.2092,  1.2092,       0;

	cout << "MatrixLog3Test" << endl
	     << "The result is:" << endl << so3mat << endl
	     << "It should be:" << endl << result << endl;
}

void DistanceToSO3Test()
{
	//Ҫ����ľ���mat
	Eigen::Matrix3d mat;
	mat <<
		1.0, 0.0,   0.0,
		0.0, 0.1, -0.95,
		0.0, 1.0,   0.1;

	//����mat��SO(3)�ľ���
	double d = mr::DistanceToSO3(mat);

	//��ȷ���
	double result = 0.0884;

	cout << "DistanceToSO3Test" << endl
	     << "The result is:" << endl << d << endl
	     << "It should be:" << endl << result << endl;
}

void TestIfSO3Test()
{
	//Ҫ�жϵľ���mat
	Eigen::Matrix3d mat;
	mat <<
		1.0, 0.0,   0.0,
		0.0, 0.1, -0.95,
		0.0, 1.0,   0.1;

	//���mat�Ƿ�ӽ���λ��SO(3)��
	bool judge = mr::TestIfSO3(mat);

	//��ȷ���
	bool result = false;

	cout << "TestIfSO3Test" << endl
	     << "The result is:" << endl << judge << endl
	     << "It should be:" << endl << result << endl;
}

void ProjectToSO3Test()
{
	//Ҫ����ľ���mat
	Eigen::Matrix3d mat;
	mat <<
		 0.675, 0.150,  0.720,
		 0.370, 0.771, -0.511,
		-0.630, 0.619,  0.472;

	//����mat��SO(3)�е�ͶӰ
	Eigen::MatrixXd R = mr::ProjectToSO3(mat);

	//��ȷ���
	Eigen::MatrixXd result(3, 3);
	result <<
		 0.6790, 0.1489,  0.7189,
		 0.3732, 0.7732, -0.5127,
		-0.6322, 0.6164,  0.4694;

	cout << "ProjectToSO3Test" << endl
	     << "The result is:" << endl << R << endl
	     << "It should be:" << endl << result << endl;
}

void RpToTransTest()
{
	//��ת����
	Eigen::Matrix3d R(3, 3);
	R <<
		1, 0,  0,
		0, 0, -1,
		0, 1,  0;

	//λ������
	Eigen::Vector3d p(1, 2, 5);

	//������α任����
	Eigen::MatrixXd T = mr::RpToTrans(R, p);

	//��ȷ���
	Eigen::MatrixXd result(4, 4);
	result <<
		1, 0,  0, 1,
		0, 0, -1, 2,
		0, 1,  0, 5,
		0, 0,  0, 1;

	cout << "RpToTransTest" << endl
	     << "The result is:" << endl << T << endl
	     << "It should be:" << endl << result << endl;
}

void TransToRpTest()
{
	//��α任����
	Eigen::MatrixXd T(4, 4);
	T <<
		1, 0,  0, 0,
		0, 0, -1, 0,
		0, 1,  0, 3,
		0, 0,  0, 1;

	//��ת����R, λ������p
	std::vector<Eigen::MatrixXd> R_and_p = mr::TransToRp(T);
	Eigen::Matrix3d R = R_and_p.at(0);
	Eigen::Vector3d p = R_and_p.at(1);

	//��ȷ���
	Eigen::Matrix3d R_result;
	R_result <<
		1, 0,  0,
		0, 0, -1,
		0, 1,  0;
	Eigen::Vector3d p_result;
	p_result <<
		0,
		0,
		3;

	cout << "TransToRpTest" << endl
	     << "The result is:" << endl << R << endl
	     << "It should be:" << endl << R_result << endl
	 	 << "The result is:" << endl << p << endl
		 << "It should be:" << endl << p_result << endl;
}

void TransInvTest()
{
	//��α任����T
	Eigen::MatrixXd T(4, 4);
	T <<
		1, 0,  0, 0,
		0, 0, -1, 0,
		0, 1,  0, 3,
		0, 0,  0, 1;

	//�����������
	Eigen::MatrixXd invT = mr::TransInv(T);

	//��ȷ���
	Eigen::MatrixXd result(4, 4);
	result <<
		1,  0, 0,  0,
		0,  0, 1, -3,
		0, -1, 0,  0,
		0,  0, 0,  1;

	cout << "TransInvTest" << endl
	     << "The result is:" << endl << invT << endl
	     << "It should be:" << endl << result << endl;
}

void VecTose3Test()
{
	//�˶�����
	Eigen::VectorXd V(6);
	V <<
		1,
		2,
		3,
		4,
		5,
		6;

	//V��se(3)�����ʾ
	Eigen::MatrixXd se3mat = mr::VecTose3(V);

	//��ȷ���
	Eigen::MatrixXd result(4, 4);
	result <<
		 0, -3,  2, 4,
		 3,  0, -1, 5,
		-2,  1,  0, 6,
		 0,  0,  0, 0;

	cout << "VecTose3Test" << endl
	     << "The result is:" << endl << se3mat << endl
	     << "It should be:" << endl << result << endl;
}

void se3ToVecTest()
{
	//se(3)����
	Eigen::MatrixXd se3mat(4, 4);
	se3mat <<
		 0, -3,  2, 4,
		 3,  0, -1, 5,
		-2,  1,  0, 6,
		 0,  0,  0, 0;

	//�˶�����
	Eigen::MatrixXd V = mr::se3ToVec(se3mat);

	//��ȷ���
	Eigen::VectorXd result(6);
	result <<
		1,
		2,
		3,
		4,
		5,
		6;

	cout << "se3ToVecTest" << endl
	     << "The result is:" << endl << V << endl
	     << "It should be:" << endl << result << endl;
}

void AdjointTest()
{
	//��α任����
	Eigen::Matrix4d T;
	T <<
		1, 0,  0, 0,
		0, 0, -1, 0,
		0, 1,  0, 3,
		0, 0,  0, 1;

	//�������
	Eigen::MatrixXd AdT = mr::Adjoint(T);

	//��ȷ���
	Eigen::MatrixXd result(6, 6);
	result <<
		1, 0,  0, 0, 0,  0,
		0, 0, -1, 0, 0,  0,
		0, 1,  0, 0, 0,  0,
		0, 0,  3, 1, 0,  0,
		3, 0,  0, 0, 0, -1,
		0, 0,  0, 0, 1,  0;

	cout << "AdjointTest" << endl
	     << "The result is:" << endl << AdT << endl
	     << "It should be:" << endl << result << endl;
}

void ScrewToAxisTest()
{
	//��������һ��q
	Eigen::Vector3d q;
	q <<
		3,
		0,
		1;

	//��ʾ�����᷽��ĵ�λ����
	Eigen::Vector3d s;
	s <<
		0,
		0,
		1;

	//�����˵Ľھ�
	double h = 2;

	//�����׼����������S�����ʽ
	Eigen::VectorXd S = mr::ScrewToAxis(q, s, h);

	//��ȷ���
	Eigen::VectorXd result(6);
	result <<
		 0,
		 0,
		 1,
		 0,
		-3,
		 2;

	cout << "ScrewToAxisTest" << endl
	     << "The result is:" << endl << S << endl
	     << "It should be:" << endl << result << endl;
}

void AxisAng6Test()
{
	//��ά������ʽ��ָ������S0
	Eigen::VectorXd expc6(6);
	expc6 <<
		1.0,
		0.0,
		0.0,
		1.0,
		2.0,
		3.0;

	//�����׼����������S, �������ƶ��ľ���0 [S, theta]
	Eigen::VectorXd S_and_theta = mr::AxisAng6(expc6);

	//��ȷ���
	Eigen::VectorXd result(7);
	result <<
		1.0,
		0.0,
		0.0,
		1.0,
		2.0,
		3.0,
		1.0;

	cout << "AxisAng6Test" << endl
	     << "The result is:" << endl << S_and_theta << endl
	     << "It should be:" << endl << result << endl;
}

void MatrixExp6Test()
{
	//ָ�������se(3)�����ʾ(�任����)
	Eigen::MatrixXd se3mat(4, 4);
	se3mat <<
		0,      0,       0,      0,
		0,      0, -1.5708, 2.3562,
		0, 1.5708,       0, 2.3562,
		0,      0,       0,      0;

	//��α任����
	Eigen::MatrixXd T = mr::MatrixExp6(se3mat);

	Eigen::MatrixXd result(4, 4);
	result <<
		1, 0,  0, 0,
		0, 0, -1, 0,
		0, 1,  0, 3,
		0, 0,  0, 1;

	cout << "MatrixExp6Test" << endl
	     << "The result is:" << endl << T << endl
	     << "It should be:" << endl << result << endl;
}

void MatrixLog6Test()
{
	//��α任����T
	Eigen::MatrixXd T(4, 4);
	T <<
		1, 0,  0, 0,
		0, 0, -1, 0,
		0, 1,  0, 3,
		0, 0,  0, 1;

	//T�ľ������semat3
	Eigen::MatrixXd expmat = mr::MatrixLog6(T);

	//��ȷ���
	Eigen::MatrixXd result(4, 4);
	result <<
		0,      0,       0,      0,
		0,      0, -1.5708, 2.3562,
		0, 1.5708,       0, 2.3562,
		0,      0,       0,      0;

	cout << "MatrixLog6Test" << endl
	     << "The result is:" << endl << expmat << endl
	     << "It should be:" << endl << result << endl;
}

void DistanceToSE3Test()
{
	//Ҫ����ľ���mat
	Eigen::Matrix4d mat;
	mat <<
		1.0, 0.0,   0.0,  1.2,
		0.0, 0.1, -0.95,  1.5,
		0.0, 1.0,   0.1, -0.9,
		0.0, 0.0,   0.1, 0.98;

	//����mat��SE(3)�ľ���
	double d = mr::DistanceToSE3(mat);

	//��ȷ���
	double result = 0.1349;

	cout << "DistanceToSE3Test" << endl
	     << "The result is:" << endl << d << endl
	     << "It should be:" << endl << result << endl;
}

void TestIfSE3Test()
{
	//Ҫ�жϵľ���mat
	Eigen::Matrix4d mat;
	mat <<
		1.0, 0.0,   0.0,  1.2,
		0.0, 0.1, -0.95,  1.5,
		0.0, 1.0,   0.1, -0.9,
		0.0, 0.0,   0.1, 0.98;

	//���mat�Ƿ�ӽ���λ��SE(3)��
	bool judge = mr::TestIfSE3(mat);

	//��ȷ���
	bool result = false;

	cout << "TestIfSE3Test" << endl
	     << "The result is:" << endl << judge << endl
	     << "It should be:" << endl << result << endl;
}

void ProjectToSE3Test()
{
	//Ҫ����ľ���mat
	Eigen::Matrix4d mat;
	mat <<
		 0.675, 0.150,  0.720, 1.2,
		 0.370, 0.771, -0.511, 5.4,
		-0.630, 0.619,  0.472, 3.6,
		 0.003, 0.002,  0.010, 0.9;

	//����mat��SO(3)�е�ͶӰ
	Eigen::MatrixXd T = mr::ProjectToSE3(mat);

	//��ȷ���
	Eigen::MatrixXd result(4, 4);
	result <<
		 0.6790, 0.1489,  0.7189, 1.2000,
		 0.3732, 0.7732, -0.5127, 5.4000,
		-0.6322, 0.6164,  0.4694, 3.6000,
		      0,      0,       0, 1.0000;

	cout << "ProjectToSE3Test" << endl
	     << "The result is:" << endl << T << endl
	     << "It should be:" << endl << result << endl;
}

/*--------------------��4�� �����˶�ѧ P99--------------------*/

void FKinBodyTest()
{
	//ĩ�˵ĳ�ʼλ��M
	Eigen::MatrixXd M(4, 4);
	M <<
		-1, 0,  0, 0,
		 0, 1,  0, 6,
		 0, 0, -1, 2,
		 0, 0,  0, 1;

	//ĩ������ϵ�µĹؽ�����Blist
	Eigen::MatrixXd Blist(6, 3);
	Blist <<
		 0, 0,   0,
		 0, 0,   0,
		-1, 0,   1,
		 2, 0,   0,
		 0, 1,   0,
		 0, 0, 0.1;

	//�ؽ�ֵthetalist
	Eigen::VectorXd thetaList(3);
	thetaList << M_PI / 2.0, 3, M_PI;

	//�����˶�ѧ
	Eigen::MatrixXd T = mr::FKinBody(M, Blist, thetaList);

	//��ȷ���
	Eigen::MatrixXd result(4, 4);
	result <<
		-0.0000, 1.0000,       0, -5.0000,
		 1.0000, 0.0000,       0,  4.0000,
		      0,      0, -1.0000,  1.6858,
		      0,      0,       0,  1.0000;

	cout << "FKinBodyTest" << endl
	     << "The result is:" << endl << T << endl
	     << "It should be:" << endl << result << endl;
}

void FKinSpaceTest()
{
	//ĩ�˵ĳ�ʼλ��M
	Eigen::MatrixXd M(4, 4);
	M <<
		-1, 0,  0, 0,
		 0, 1,  0, 6,
		 0, 0, -1, 2,
		 0, 0,  0, 1;

	//�ռ�����ϵ�µĹؽ�����Slist
	Eigen::MatrixXd Slist(6, 3);
	Slist <<
		0, 0,    0,
		0, 0,    0,
		1, 0,   -1,
		4, 0,   -6,
		0, 1,    0,
		0, 0, -0.1;

	//�ؽ�ֵthetalist
	Eigen::VectorXd thetaList(3);
	thetaList << M_PI / 2.0, 3, M_PI;

	//�����˶�ѧ
	Eigen::MatrixXd T = mr::FKinSpace(M, Slist, thetaList);

	//��ȷ���
	Eigen::MatrixXd result(4, 4);
	result <<
	       -0.0000, 1.0000,       0, -5.0000,
			1.0000, 0.0000,       0,  4.0000,
			0,           0, -1.0000,  1.6858,
			0,           0,       0,  1.0000;

	cout << "FKinSpaceTest" << endl
	     << "The result is:" << endl << T << endl
	     << "It should be:" << endl << result << endl;
}

/*--------------------��5�� һ���˶�ѧ�뾲��ѧ P125--------------------*/

void JacobianBodyTest()
{
	//ĩ������ϵ�µĹؽ�����Blist
	Eigen::MatrixXd Blist(6, 4);
	Blist <<
		  0, 1, 0,   1,
		  0, 0, 1,   0,
		  1, 0, 0,   0,
		  0, 2, 0, 0.2,
		0.2, 0, 2, 0.3,
		0.2, 3, 1, 0.4;

	//�ؽ�ֵthetalist
	Eigen::VectorXd thetalist(4);
	thetalist << 0.2, 1.1, 0.1, 1.2;

	//��ȷ���
	Eigen::MatrixXd result(6, 4);
	result <<
		-0.0453, 0.9950,       0, 1.0000,
		 0.7436, 0.0930,  0.3624,      0,
		-0.6671, 0.0362, -0.9320,      0,
		 2.3259, 1.6681,  0.5641, 0.2000,
		-1.4432, 2.9456,  1.4331, 0.3000,
		-2.0664, 1.8288, -1.5887, 0.4000;

	//�����ſɱ�
	Eigen::MatrixXd Jb = mr::JacobianBody(Blist, thetalist);

	cout << "JacobianBodyTest" << endl
	     << "The result is:" << endl << Jb << endl
	     << "It should be:" << endl << result << endl;
}

void JacobianSpaceTest()
{
	//�ռ�����ϵ�µĹؽ�����Slist
	Eigen::MatrixXd Slist(6, 4);
	Slist <<
		  0, 1, 0,   1,
		  0, 0, 1,   0,
		  1, 0, 0,   0,
		  0, 2, 0, 0.2,
		0.2, 0, 2, 0.3,
		0.2, 3, 1, 0.4;

	//�ؽ�ֵthetalist
	Eigen::VectorXd thetalist(4);
	thetalist << 0.2, 1.1, 0.1, 1.2;

	//��ȷ���
	Eigen::MatrixXd result(6, 4);
	result <<
			 0, 0.9801, -0.0901,  0.9575,
		     0, 0.1987,  0.4446,  0.2849,
		1.0000,      0,  0.8912, -0.0453,
		     0, 1.9522, -2.2164, -0.5116,
		0.2000, 0.4365, -2.4371,  2.7754,
		0.2000, 2.9603,  3.2357,  2.2251;

	//�ռ��ſɱ�
	Eigen::MatrixXd Js = mr::JacobianSpace(Slist, thetalist);

	cout << "JacobianSpaceTest" << endl
	     << "The result is:" << endl << Js << endl
	     << "It should be:" << endl << result << endl;
}

/*--------------------��6�� ���˶�ѧ P144--------------------*/

void IKinBodyTest()
{
	//ĩ������ϵ�µĹؽ�����Blist
	Eigen::MatrixXd Blist(6, 3);
	Blist <<
		 0, 0,   0,
		 0, 0,   0,
		-1, 0,   1,
		 2, 0,   0,
		 0, 1,   0,
		 0, 0, 0.1;

	//ĩ�˳�ʼλ��
	Eigen::Matrix4d M;
	M <<
		-1, 0,  0, 0,
		 0, 1,  0, 6,
		 0, 0, -1, 2,
		 0, 0,  0, 1;

	//Ԥ�ڵ�ĩ��λ��
	Eigen::Matrix4d T;
	T <<
		0, 1,  0,     -5,
		1, 0,  0,      4,
		0, 0, -1, 1.6858,
		0, 0,  0,      1;

	//�ؽڽǵĳ�ʼ����ֵ
	Eigen::VectorXd thetalist(3);
	thetalist << 1.5, 2.5, 3;

	//ĩ������ϵ��ĩ�˷����С�����
	double eomg = 0.01;

	//ĩ������ϵ��ĩ��λ�õ�С�����
	double ev = 0.001;

	//��ȷ���
	bool success_result = true;
	Eigen::VectorXd thetalist_result(3);
	thetalist_result << 1.5707, 2.9997, 3.1415;

	//���˶�ѧ
	bool success = mr::IKinBody(Blist, M, T, thetalist, eomg, ev);

	cout << "IKinBodyTest" << endl
	     << "The result is:" << endl << success << endl
	     << "It should be:" << endl << success_result << endl
	     << "The result is:" << endl << thetalist << endl
	     << "It should be:" << endl << thetalist_result << endl;
}

void IKinSpaceTest()
{
	//�ռ�����ϵ�µĹؽ�����
	Eigen::MatrixXd Slist(6, 3);
	Slist <<
		0, 0,    0,
		0, 0,    0,
		1, 0,   -1,
		4, 0,   -6,
		0, 1,    0,
		0, 0, -0.1;

	//ĩ�˳�ʼλ��
	Eigen::Matrix4d M;
	M <<
		-1, 0,  0, 0,
		 0, 1,  0, 6,
		 0, 0, -1, 2,
		 0, 0,  0, 1;

	//Ԥ�ڵ�ĩ��λ��
	Eigen::Matrix4d T;
	T <<
		0, 1,  0,     -5,
		1, 0,  0,      4,
		0, 0, -1, 1.6858,
		0, 0,  0,      1;

	//�ؽڽǵĳ�ʼ����ֵ
	Eigen::VectorXd thetalist(3);
	thetalist << 1.5, 2.5, 3;

	//�ռ�����ϵ��ĩ�˷����С�����
	double eomg = 0.01;

	//�ռ�����ϵ��ĩ��λ�õ�С�����
	double ev = 0.001;

	//��ȷ���
	bool success_result = true;
	Eigen::VectorXd thetalist_result(3);
	thetalist_result << 1.5707, 2.9997, 3.1415;

	//���˶�ѧ
	bool success = mr::IKinSpace(Slist, M, T, thetalist, eomg, ev);

	cout << "IKinSpaceTest" << endl
	     << "The result is:" << endl << success << endl
	     << "It should be:" << endl << success_result << endl
	     << "The result is:" << endl << thetalist << endl
	     << "It should be:" << endl << thetalist_result << endl;
}

/*--------------------��8�� ��������ѧ P197--------------------*/

void adTest()
{
	Eigen::VectorXd V(6);
	V << 1, 2, 3, 4, 5, 6;

	//��ȷ���
	Eigen::MatrixXd result(6, 6);
	result <<
		 0, -3,  2,  0,  0,  0,
		 3,  0, -1,  0,  0,  0,
		-2,  1,  0,  0,  0,  0,
		 0, -6,  5,  0, -3,  2,
		 6,  0, -4,  3,  0, -1,
		-5,  4,  0, -2,  1,  0;

	Eigen::MatrixXd adV = mr::ad(V);

	cout << "adTest" << endl
	     << "The result is:" << endl << adV << endl
	     << "It should be:" << endl << result << endl;
}

void InverseDynamicsTest()
{
	//�Ƕȡ��ٶȡ����ٶ�
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;
	Eigen::VectorXd ddthetalist(3);
	ddthetalist << 2, 1.5, 1;

	//�ڻ�������ϵ�е���������
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;

	//��ĩ��ִ��������ϵ{n+1}�б�ʾ����ĩ��ִ���������ڻ�����������
	Eigen::VectorXd Ftip(6);
	Ftip << 1, 1, 1, 1, 1, 1;

	//Mlist�е�ÿһ��MΪ����ϵ{i-1}��{i}�е�λ��
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0.089159,
		0, 0, 0, 1;
	Eigen::Matrix4d M12;
	M12 <<
		 0, 0, 1,    0.28,
		 0, 1, 0, 0.13585,
		-1, 0, 0,       0,
		 0, 0, 0,       1;
	Eigen::Matrix4d M23;
	M23 <<
		1, 0, 0,       0,
		0, 1, 0, -0.1197,
		0, 0, 1,   0.395,
		0, 0, 0,       1;
	Eigen::Matrix4d M34;
	M34 <<
		1, 0, 0,       0,
		0, 1, 0,       0,
		0, 0, 1, 0.14225,
		0, 0, 0,       1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist�е�ÿһ��GΪ�ռ��������Ϊ�Խ���ʹ��asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist�е�ÿһ��SiΪ�������ڻ�������ϵ{0}�еı�ʾ
	Eigen::MatrixXd Slist(6, 3);
	Slist <<
		1,      0,      0,
		0,      1,      1,
		1,      0,      0,
		0, -0.089, -0.089,
		1,      0,      0,
		0,      0,  0.425;

	//��������ؽ���-���ص�nά������
	Eigen::VectorXd taulist = mr::InverseDynamics(thetalist, dthetalist, ddthetalist,
												  g, Ftip, Mlist, Glist, Slist);

	//��ȷ���
	Eigen::VectorXd result(3);
	result << 74.6962, -33.0677, -3.2306;

	cout << "InverseDynamicsTest" << endl
	     << "The result is:" << endl << taulist << endl
	     << "It should be:" << endl << result << endl;
}

void MassMatrixTest()
{
	//�Ƕ�
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;

	//Mlist�е�ÿһ��MΪ����ϵ{i-1}��{i}�е�λ��
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 <<
	    1, 0, 0,        0,
		0, 1, 0,        0,
		0, 0, 1, 0.089159,
		0, 0, 0,        1;
	Eigen::Matrix4d M12;
	M12 <<
	     0, 0, 1,    0.28,
		 0, 1, 0, 0.13585,
		-1, 0, 0,       0,
		 0, 0, 0,       1;
	Eigen::Matrix4d M23;
	M23 <<
	    1, 0, 0,       0,
		0, 1, 0, -0.1197,
		0, 0, 1,   0.395,
		0, 0, 0,       1;
	Eigen::Matrix4d M34;
	M34 <<
	    1, 0, 0,       0,
		0, 1, 0,       0,
		0, 0, 1, 0.14225,
		0, 0, 0,       1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist�е�ÿһ��GΪ�ռ��������Ϊ�Խ���ʹ��asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist�е�ÿһ��SiΪ�������ڻ�������ϵ{0}�еı�ʾ
	Eigen::MatrixXd Slist(6, 3);
	Slist <<
		1,      0,      0,
		0,      1,      1,
		1,      0,      0,
		0, -0.089, -0.089,
		1,      0,      0,
		0,      0,  0.425;

	//�������thetalist�����µ�n�ؽڴ��������Ծ���M(��)
	Eigen::MatrixXd M = mr::MassMatrix(thetalist, Mlist, Glist, Slist);

	//��ȷ���
	Eigen::MatrixXd result(3, 3);
	result <<
		22.5433, -0.3071, -0.0072,
		-0.3071,  1.9685,  0.4322,
		-0.0072,  0.4322,  0.1916;

	cout << "MassMatrixTest" << endl
	     << "The result is:" << endl << M << endl
	     << "It should be:" << endl << result << endl;
}

void VelQuadraticForcesTest()
{
	//�Ƕȡ��ٶ�
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;

	//Mlist�е�ÿһ��MΪ����ϵ{i-1}��{i}�е�λ��
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 <<
	    1, 0, 0,        0,
		0, 1, 0,        0,
		0, 0, 1, 0.089159,
		0, 0, 0,        1;
	Eigen::Matrix4d M12;
	M12 <<
	     0, 0, 1,    0.28,
		 0, 1, 0, 0.13585,
		-1, 0, 0,       0,
		 0, 0, 0,       1;
	Eigen::Matrix4d M23;
	M23 <<
	    1, 0, 0,       0,
		0, 1, 0, -0.1197,
		0, 0, 1,   0.395,
		0, 0, 0,       1;
	Eigen::Matrix4d M34;
	M34 <<
	    1, 0, 0,       0,
		0, 1, 0,       0,
		0, 0, 1, 0.14225,
		0, 0, 0,       1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist�е�ÿһ��GΪ�ռ��������Ϊ�Խ���ʹ��asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist�е�ÿһ��SiΪ�������ڻ�������ϵ{0}�еı�ʾ
	Eigen::MatrixXd Slist(6, 3);
	Slist <<
		1,      0,      0,
		0,      1,      1,
		1,      0,      0,
		0, -0.089, -0.089,
		1,      0,      0,
		0,      0,  0.425;

	//�������thetalist��dtetalist�µĿ���������������c(0,0')
	Eigen::VectorXd c = mr::VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist);

	//��ȷ���
	Eigen::VectorXd result(3);
	result << 0.2645, -0.0551, -0.0069;

	cout << "VelQuadraticForcesTest" << endl
	     << "The result is:" << endl << c << endl
	     << "It should be:" << endl << result << endl;
}

void GravityForcesTest()
{
	//�Ƕ�
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;

	//gΪ�ڻ�������ϵ�е���������
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;

	//Mlist�е�ÿһ��MΪ����ϵ{i-1}��{i}�е�λ��
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 <<
	    1, 0, 0,        0,
		0, 1, 0,        0,
		0, 0, 1, 0.089159,
		0, 0, 0,        1;
	Eigen::Matrix4d M12;
	M12 <<
	     0, 0, 1,    0.28,
		 0, 1, 0, 0.13585,
		-1, 0, 0,       0,
		 0, 0, 0,       1;
	Eigen::Matrix4d M23;
	M23 <<
	    1, 0, 0,       0,
		0, 1, 0, -0.1197,
		0, 0, 1,   0.395,
		0, 0, 0,       1;
	Eigen::Matrix4d M34;
	M34 <<
	    1, 0, 0,       0,
		0, 1, 0,       0,
		0, 0, 1, 0.14225,
		0, 0, 0,       1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist�е�ÿһ��GΪ�ռ��������Ϊ�Խ���ʹ��asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist�е�ÿһ��SiΪ�������ڻ�������ϵ{0}�еı�ʾ
	Eigen::MatrixXd Slist(6, 3);
	Slist <<
		1,      0,      0,
		0,      1,      1,
		1,      0,      0,
		0, -0.089, -0.089,
		1,      0,      0,
		0,      0,  0.425;

	//����������g(0)
	Eigen::VectorXd grav = mr::GravityForces(thetalist, g, Mlist, Glist, Slist);

	//��ȷ���
	Eigen::VectorXd result(3);
	result << 28.4033, -37.6409, -5.4416;

	cout << "GravityForcesTest" << endl
	     << "The result is:" << endl << grav << endl
	     << "It should be:" << endl << result << endl;
}

void EndEffectorForcesTest()
{
	//�Ƕ�
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;

	//��ĩ��ִ��������ϵ{n+1}�б�ʾ����ĩ��ִ���������ڻ�����������
	Eigen::VectorXd Ftip(6);
	Ftip << 1, 1, 1, 1, 1, 1;

	//Mlist�е�ÿһ��MΪ����ϵ{i-1}��{i}�е�λ��
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 <<
	    1, 0, 0,        0,
		0, 1, 0,        0,
		0, 0, 1, 0.089159,
		0, 0, 0,        1;
	Eigen::Matrix4d M12;
	M12 <<
	     0, 0, 1,    0.28,
		 0, 1, 0, 0.13585,
		-1, 0, 0,       0,
		 0, 0, 0,       1;
	Eigen::Matrix4d M23;
	M23 <<
	    1, 0, 0,       0,
		0, 1, 0, -0.1197,
		0, 0, 1,   0.395,
		0, 0, 0,       1;
	Eigen::Matrix4d M34;
	M34 <<
	    1, 0, 0,       0,
		0, 1, 0,       0,
		0, 0, 1, 0.14225,
		0, 0, 0,       1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist�е�ÿһ��GΪ�ռ��������Ϊ�Խ���ʹ��asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist�е�ÿһ��SiΪ�������ڻ�������ϵ{0}�еı�ʾ
	Eigen::MatrixXd Slist(6, 3);
	Slist <<
		1,      0,      0,
		0,      1,      1,
		1,      0,      0,
		0, -0.089, -0.089,
		1,      0,      0,
		0,      0,  0.425;

	//���������ĩ��ִ������Ftip����Ĺؽ���������
	Eigen::VectorXd JTFtip = mr::EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);

	//��ȷ���
	Eigen::VectorXd result(3);
	result << 1.4095, 1.8577, 1.3924;

	cout << "EndEffectorForcesTest" << endl
	     << "The result is:" << endl << JTFtip << endl
	     << "It should be:" << endl << result << endl;
}

void ForwardDynamicsTest()
{
	//�Ƕȡ��ٶ�
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;

	//����ؽ���-���ص�nά������
	Eigen::VectorXd taulist(3);
	taulist << 0.5, 0.6, 0.7;

	//gΪ�ڻ�������ϵ�е���������
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;

	//��ĩ��ִ��������ϵ{n+1}�б�ʾ����ĩ��ִ���������ڻ�����������
	Eigen::VectorXd Ftip(6);
	Ftip << 1, 1, 1, 1, 1, 1;

	//Mlist�е�ÿһ��MΪ����ϵ{i-1}��{i}�е�λ��
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 <<
	    1, 0, 0,        0,
		0, 1, 0,        0,
		0, 0, 1, 0.089159,
		0, 0, 0,        1;
	Eigen::Matrix4d M12;
	M12 <<
	     0, 0, 1,    0.28,
		 0, 1, 0, 0.13585,
		-1, 0, 0,       0,
		 0, 0, 0,       1;
	Eigen::Matrix4d M23;
	M23 <<
	    1, 0, 0,       0,
		0, 1, 0, -0.1197,
		0, 0, 1,   0.395,
		0, 0, 0,       1;
	Eigen::Matrix4d M34;
	M34 <<
	    1, 0, 0,       0,
		0, 1, 0,       0,
		0, 0, 1, 0.14225,
		0, 0, 0,       1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist�е�ÿһ��GΪ�ռ��������Ϊ�Խ���ʹ��asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist�е�ÿһ��SiΪ�������ڻ�������ϵ{0}�еı�ʾ
	Eigen::MatrixXd Slist(6, 3);
	Slist <<
		1,      0,      0,
		0,      1,      1,
		1,      0,      0,
		0, -0.089, -0.089,
		1,      0,      0,
		0,      0,  0.425;

	//����nά�ؽڼ��ٶ�����0''
	Eigen::VectorXd ddthetalist = mr::ForwardDynamics(thetalist, dthetalist, taulist,
													  g, Ftip, Mlist, Glist, Slist);

	//��ȷ���
	Eigen::VectorXd result(3);
	result << -0.9739, 25.5847, -32.9150;

	cout << "ForwardDynamicsTest" << endl
	     << "The result is:" << endl << ddthetalist << endl
	     << "It should be:" << endl << result << endl;
}

void EulerStepTest()
{
	//�Ƕȡ��ٶȡ����ٶ�
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;
	Eigen::VectorXd ddthetalist(3);
	ddthetalist << 2, 1.5, 1;

	//ʱ�䲽����t
	double dt = 0.1;

	//ʹ��һ��ŷ�����ּ�����һʱ�䲽�ĹؽڽǶȺ��ٶ�
	mr::EulerStep(thetalist, dthetalist, ddthetalist, dt);

	//��ȷ���
	Eigen::VectorXd next_thetalist_result(3);
	next_thetalist_result << 0.11, 0.12, 0.13;
	Eigen::VectorXd next_dthetalist_result(3);
	next_dthetalist_result << 0.3, 0.35, 0.4;

	cout << "EulerStepTest" << endl
	     << "The result is:" << endl << thetalist << endl
	     << "It should be:" << endl << next_thetalist_result << endl
	     << "The result is:" << endl << dthetalist << endl
	     << "It should be:" << endl << next_dthetalist_result << endl;
}

void InverseDynamicsTrajectoryTest()
{
	int dof = 3;
	Eigen::VectorXd thetastart(dof);
	thetastart << 0, 0, 0;
	Eigen::VectorXd thetaend(dof);
	thetaend << M_PI / 2, M_PI / 2, M_PI / 2;
	double Tf = 3.0;
	int N = 1000;
	int method = 5;

	Eigen::MatrixXd traj = mr::JointTrajectory(thetastart, thetaend, Tf, N, method);
	Eigen::MatrixXd thetamat = traj;
	Eigen::MatrixXd dthetamat = Eigen::MatrixXd::Zero(N, dof);
	Eigen::MatrixXd ddthetamat = Eigen::MatrixXd::Zero(N, dof);
	double dt = Tf / (N - 1.0);
	for (int i = 0; i < N - 1; ++i)
	{
		dthetamat.row(i + 1) = (thetamat.row(i + 1) - thetamat.row(i)) / dt;
		ddthetamat.row(i + 1) = (dthetamat.row(i + 1) - dthetamat.row(i)) / dt;
	}

	//gΪ�ڻ�������ϵ�е���������
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;
	Eigen::MatrixXd Ftipmat = Eigen::MatrixXd::Zero(N, 6);

	//Mlist�е�ÿһ��MΪ����ϵ{i-1}��{i}�е�λ��
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 <<
	    1, 0, 0,        0,
		0, 1, 0,        0,
		0, 0, 1, 0.089159,
		0, 0, 0,        1;
	Eigen::Matrix4d M12;
	M12 <<
	     0, 0, 1,    0.28,
		 0, 1, 0, 0.13585,
		-1, 0, 0,       0,
		 0, 0, 0,       1;
	Eigen::Matrix4d M23;
	M23 <<
	    1, 0, 0,       0,
		0, 1, 0, -0.1197,
		0, 0, 1,   0.395,
		0, 0, 0,       1;
	Eigen::Matrix4d M34;
	M34 <<
	    1, 0, 0,       0,
		0, 1, 0,       0,
		0, 0, 1, 0.14225,
		0, 0, 0,       1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist�е�ÿһ��GΪ�ռ��������Ϊ�Խ���ʹ��asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist�е�ÿһ��SiΪ�������ڻ�������ϵ{0}�еı�ʾ
	Eigen::MatrixXd Slist(6, 3);
	Slist <<
		1,      0,      0,
		0,      1,      1,
		1,      0,      0,
		0, -0.089, -0.089,
		1,      0,      0,
		0,      0,  0.425;

	int numTest = 3;

	//��ȷ���
	Eigen::MatrixXd result(numTest, dof);
	Eigen::VectorXd tau_timestep_beg(3);
	tau_timestep_beg << 13.22970794, -36.262108, -4.181341;
	Eigen::VectorXd tau_timestep_mid(3);
	tau_timestep_mid << 115.55863434, -22.05129215, 1.00916115;
	Eigen::VectorXd tau_timestep_end(3);
	tau_timestep_end << 81.12700926, -23.20753925, 2.48432708;
	result << tau_timestep_beg.transpose(),
			tau_timestep_mid.transpose(),
			tau_timestep_end.transpose();

	Eigen::MatrixXd taumat = mr::InverseDynamicsTrajectory(thetamat, dthetamat, ddthetamat, g, Ftipmat, Mlist, Glist, Slist);
	Eigen::MatrixXd taumat_timestep(numTest, dof);
	taumat_timestep << taumat.row(0),
			taumat.row(int(N / 2) - 1),
			taumat.row(N - 1);
	cout << "InverseDynamicsTrajectoryTest" << endl
	     << "The result is:" << endl << taumat_timestep << endl
	     << "It should be:" << endl << result << endl;
}

void ForwardDynamicsTrajectoryTest()
{
	//3���˻�е������
	//�Ƕȡ��ٶ�
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;
	int N = 10, dof = 3;
	Eigen::MatrixXd taumat(N, 3);
	taumat << 3.63, -6.58, -5.57,
			3.74, -5.55, -5.5,
			4.31, -0.68, -5.19,
			5.18, 5.63, -4.31,
			5.85, 8.17, -2.59,
			5.78, 2.79, -1.7,
			4.99, -5.3, -1.19,
			4.08, -9.41, 0.07,
			3.56, -10.1, 0.97,
			3.49, -9.41, 1.23;

	//gΪ�ڻ�������ϵ�е���������
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;
	Eigen::MatrixXd Ftipmat = Eigen::MatrixXd::Zero(N, 6);

	//Mlist�е�ÿһ��MΪ����ϵ{i-1}��{i}�е�λ��
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 <<
	    1, 0, 0,        0,
		0, 1, 0,        0,
		0, 0, 1, 0.089159,
		0, 0, 0,        1;
	Eigen::Matrix4d M12;
	M12 <<
	     0, 0, 1,    0.28,
		 0, 1, 0, 0.13585,
		-1, 0, 0,       0,
		 0, 0, 0,       1;
	Eigen::Matrix4d M23;
	M23 <<
	    1, 0, 0,       0,
		0, 1, 0, -0.1197,
		0, 0, 1,   0.395,
		0, 0, 0,       1;
	Eigen::Matrix4d M34;
	M34 <<
	    1, 0, 0,       0,
		0, 1, 0,       0,
		0, 0, 1, 0.14225,
		0, 0, 0,       1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist�е�ÿһ��GΪ�ռ��������Ϊ�Խ���ʹ��asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist�е�ÿһ��SiΪ�������ڻ�������ϵ{0}�еı�ʾ
	Eigen::MatrixXd Slist(6, 3);
	Slist <<
		1,      0,      0,
		0,      1,      1,
		1,      0,      0,
		0, -0.089, -0.089,
		1,      0,      0,
		0,      0,  0.425;
	double dt = 0.1;
	int intRes = 8;

	//��ȷ���
	Eigen::MatrixXd result_thetamat(N, dof);
	Eigen::MatrixXd result_dthetamat(N, dof);
	result_thetamat << 0.1, 0.1, 0.1,
			0.10643138, 0.2625997, -0.22664947,
			0.10197954, 0.71581297, -1.22521632,
			0.0801044, 1.33930884, -2.28074132,
			0.0282165, 2.11957376, -3.07544297,
			-0.07123855, 2.87726666, -3.83289684,
			-0.20136466, 3.397858, -4.83821609,
			-0.32380092, 3.73338535, -5.98695747,
			-0.41523262, 3.85883317, -7.01130559,
			-0.4638099, 3.63178793, -7.63190052;
	result_dthetamat << 0.1, 0.2, 0.3,
			0.01212502, 3.42975773, -7.74792602,
			-0.13052771, 5.55997471, -11.22722784,
			-0.35521041, 7.11775879, -9.18173035,
			-0.77358795, 8.17307573, -7.05744594,
			-1.2350231, 6.35907497, -8.99784746,
			-1.31426299, 4.07685875, -11.18480509,
			-1.06794821, 2.49227786, -11.69748583,
			-0.70264871, -0.55925705, -8.16067131,
			-0.1455669, -4.57149985, -3.43135114;

	std::vector<Eigen::MatrixXd> traj = mr::ForwardDynamicsTrajectory(thetalist, dthetalist, taumat, g, Ftipmat, Mlist, Glist, Slist, dt, intRes);
	Eigen::MatrixXd traj_theta = traj.at(0);
	Eigen::MatrixXd traj_dtheta = traj.at(1);

	cout << "ForwardDynamicsTrajectoryTest" << endl
	     << "The result is:" << endl << traj_theta << endl
	     << "It should be:" << endl << result_thetamat << endl
	     << "The result is:" << endl << traj_dtheta << endl
	     << "It should be:" << endl << result_dthetamat << endl;
}

/*--------------------��9�� �켣���� P216--------------------*/

void CubicTimeScalingTest()
{
	double Tf = 2.0;
	double t = 0.6;

	//��ȷ���
	double result = 0.216;

	double tmp_result = CubicTimeScaling(Tf, t);

	cout << "CubicTimeScalingTest" << endl
	     << "The result is:" << endl << tmp_result << endl
	     << "It should be:" << endl << result << endl;
}

void QuinticTimeScalingTest()
{
	double Tf = 2.0;
	double t = 0.6;

	//��ȷ���
	double result = 0.16308;

	double tmp_result = QuinticTimeScaling(Tf, t);

	cout << "QuinticTimeScalingTest" << endl
	     << "The result is:" << endl << tmp_result << endl
	     << "It should be:" << endl << result << endl;
}

void JointTrajectoryTest()
{
	int dof = 8;
	Eigen::VectorXd thetastart(dof);
	thetastart << 1, 0, 0, 1, 1, 0.2, 0, 1;
	Eigen::VectorXd thetaend(dof);
	thetaend << 1.2, 0.5, 0.6, 1.1, 2, 2, 0.9, 1;
	double Tf = 4.0;
	int N = 6;
	int method = 3;

	//��ȷ���
	Eigen::MatrixXd result(N, dof);
	result << 1, 0, 0, 1, 1, 0.2, 0, 1,
			1.0208, 0.052, 0.0624, 1.0104, 1.104, 0.3872, 0.0936, 1,
			1.0704, 0.176, 0.2112, 1.0352, 1.352, 0.8336, 0.3168, 1,
			1.1296, 0.324, 0.3888, 1.0648, 1.648, 1.3664, 0.5832, 1,
			1.1792, 0.448, 0.5376, 1.0896, 1.896, 1.8128, 0.8064, 1,
			1.2, 0.5, 0.6, 1.1, 2, 2, 0.9, 1;

	Eigen::MatrixXd traj = mr::JointTrajectory(thetastart, thetaend, Tf, N, method);
	cout << "JointTrajectoryTest" << endl
	     << "The result is:" << endl << traj << endl
	     << "It should be:" << endl << result << endl;
}

void ScrewTrajectoryTest()
{
	Eigen::MatrixXd Xstart(4, 4);
	Xstart << 1, 0, 0, 1,
			0, 1, 0, 0,
			0, 0, 1, 1,
			0, 0, 0, 1;
	Eigen::MatrixXd Xend(4, 4);
	Xend << 0, 0, 1, 0.1,
			1, 0, 0, 0,
			0, 1, 0, 4.1,
			0, 0, 0, 1;
	double Tf = 5.0;
	int N = 4;
	int method = 3;

	//��ȷ���
	std::vector<Eigen::MatrixXd> result(N);
	result[0] = Xstart;
	Eigen::Matrix4d X12;
	X12 << 0.904, -0.25, 0.346, 0.441,
			0.346, 0.904, -0.25, 0.529,
			-0.25, 0.346, 0.904, 1.601,
			0, 0, 0, 1;
	Eigen::Matrix4d X23;
	X23 << 0.346, -0.25, 0.904, -0.117,
			0.904, 0.346, -0.25, 0.473,
			-0.25, 0.904, 0.346, 3.274,
			0, 0, 0, 1;
	result[1] = X12;
	result[2] = X23;
	result[3] = Xend;

	std::vector<Eigen::MatrixXd> traj = mr::ScrewTrajectory(Xstart, Xend, Tf, N, method);

	cout << "ScrewTrajectoryTest" << endl;
	for (int i = 0; i < N; ++i)
	{
		cout << "The result is:" << endl << traj[i] << endl
		     << "It should be:" << endl << result[i] << endl;
	}
}

void CartesianTrajectoryTest()
{
	Eigen::MatrixXd Xstart(4, 4);
	Xstart << 1, 0, 0, 1,
			0, 1, 0, 0,
			0, 0, 1, 1,
			0, 0, 0, 1;
	Eigen::MatrixXd Xend(4, 4);
	Xend << 0, 0, 1, 0.1,
			1, 0, 0, 0,
			0, 1, 0, 4.1,
			0, 0, 0, 1;
	double Tf = 5.0;
	int N = 4;
	int method = 5;

	//��ȷ���
	std::vector<Eigen::MatrixXd> result(N);
	result[0] = Xstart;
	Eigen::Matrix4d X12;
	X12 << 0.937, -0.214, 0.277, 0.811,
			0.277, 0.937, -0.214, 0,
			-0.214, 0.277, 0.937, 1.651,
			0, 0, 0, 1;
	Eigen::Matrix4d X23;
	X23 << 0.277, -0.214, 0.937, 0.289,
			0.937, 0.277, -0.214, 0,
			-0.214, 0.937, 0.277, 3.449,
			0, 0, 0, 1;
	result[1] = X12;
	result[2] = X23;
	result[3] = Xend;

	std::vector<Eigen::MatrixXd> traj = mr::CartesianTrajectory(Xstart, Xend, Tf, N, method);

	cout << "CartesianTrajectoryTest" << endl;
	for (int i = 0; i < N; ++i)
	{
		cout << "The result is:" << endl << traj[i] << endl
		     << "It should be:" << endl << result[i] << endl;
	}
}

/*--------------------��11�� �����˿��� P287--------------------*/

void ComputedTorqueTest()
{
	//3���˻�е������
	//�Ƕȡ��ٶ�
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;
	Eigen::VectorXd eint(3);
	eint << 0.2, 0.2, 0.2;

	//gΪ�ڻ�������ϵ�е���������
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;

	//Mlist�е�ÿһ��MΪ����ϵ{i-1}��{i}�е�λ��
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 <<
	    1, 0, 0,        0,
		0, 1, 0,        0,
		0, 0, 1, 0.089159,
		0, 0, 0,        1;
	Eigen::Matrix4d M12;
	M12 <<
	     0, 0, 1,    0.28,
		 0, 1, 0, 0.13585,
		-1, 0, 0,       0,
		 0, 0, 0,       1;
	Eigen::Matrix4d M23;
	M23 <<
	    1, 0, 0,       0,
		0, 1, 0, -0.1197,
		0, 0, 1,   0.395,
		0, 0, 0,       1;
	Eigen::Matrix4d M34;
	M34 <<
	    1, 0, 0,       0,
		0, 1, 0,       0,
		0, 0, 1, 0.14225,
		0, 0, 0,       1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist�е�ÿһ��GΪ�ռ��������Ϊ�Խ���ʹ��asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist�е�ÿһ��SiΪ�������ڻ�������ϵ{0}�еı�ʾ
	Eigen::MatrixXd Slist(6, 3);
	Slist <<
		1,      0,      0,
		0,      1,      1,
		1,      0,      0,
		0, -0.089, -0.089,
		1,      0,      0,
		0,      0,  0.425;

	Eigen::VectorXd thetalistd(3);
	thetalistd << 1.0, 1.0, 1.0;
	Eigen::VectorXd dthetalistd(3);
	dthetalistd << 2, 1.2, 2;
	Eigen::VectorXd ddthetalistd(3);
	ddthetalistd << 0.1, 0.1, 0.1;
	double Kp = 1.3;
	double Ki = 1.2;
	double Kd = 1.1;

	Eigen::VectorXd taulist = mr::ComputedTorque(thetalist, dthetalist, eint, g,
	                                             Mlist, Glist, Slist, thetalistd, dthetalistd, ddthetalistd, Kp, Ki, Kd);

	//��ȷ���
	Eigen::VectorXd result(3);
	result << 133.00525246, -29.94223324, -3.03276856;

	cout << "ComputedTorqueTest" << endl
	     << "The result is:" << endl << taulist << endl
	     << "It should be:" << endl << result << endl;
}

void SimulateControlTest()
{
	//3���˻�е������
	//�Ƕȡ��ٶ�
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;

	//gΪ�ڻ�������ϵ�е���������
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;

	//Mlist�е�ÿһ��MΪ����ϵ{i-1}��{i}�е�λ��
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 <<
	    1, 0, 0,        0,
		0, 1, 0,        0,
		0, 0, 1, 0.089159,
		0, 0, 0,        1;
	Eigen::Matrix4d M12;
	M12 <<
	     0, 0, 1,    0.28,
		 0, 1, 0, 0.13585,
		-1, 0, 0,       0,
		 0, 0, 0,       1;
	Eigen::Matrix4d M23;
	M23 <<
	    1, 0, 0,       0,
		0, 1, 0, -0.1197,
		0, 0, 1,   0.395,
		0, 0, 0,       1;
	Eigen::Matrix4d M34;
	M34 <<
	    1, 0, 0,       0,
		0, 1, 0,       0,
		0, 0, 1, 0.14225,
		0, 0, 0,       1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist�е�ÿһ��GΪ�ռ��������Ϊ�Խ���ʹ��asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist�е�ÿһ��SiΪ�������ڻ�������ϵ{0}�еı�ʾ
	Eigen::MatrixXd Slist(6, 3);
	Slist <<
		1,      0,      0,
		0,      1,      1,
		1,      0,      0,
		0, -0.089, -0.089,
		1,      0,      0,
		0,      0,  0.425;
	double dt = 0.01;
	Eigen::VectorXd thetaend(3);
	thetaend << M_PI / 2, M_PI / 2, M_PI / 2;
	double Tf = 1.0;
	int N = int(1.0*Tf / dt);
	int method = 5;

	Eigen::MatrixXd traj = mr::JointTrajectory(thetalist, thetaend, Tf, N, method);
	Eigen::MatrixXd thetamatd = traj;
	Eigen::MatrixXd dthetamatd = Eigen::MatrixXd::Zero(N, 3);
	Eigen::MatrixXd ddthetamatd = Eigen::MatrixXd::Zero(N, 3);
	dt = Tf / (N - 1.0);
	for (int i = 0; i < N - 1; ++i)
	{
		dthetamatd.row(i + 1) = (thetamatd.row(i + 1) - thetamatd.row(i)) / dt;
		ddthetamatd.row(i + 1) = (dthetamatd.row(i + 1) - dthetamatd.row(i)) / dt;
	}

	Eigen::VectorXd gtilde(3);
	gtilde << 0.8, 0.2, -8.8;

	std::vector<Eigen::MatrixXd> Mtildelist;
	std::vector<Eigen::MatrixXd> Gtildelist;
	Eigen::Matrix4d Mhat01;
	Mhat01 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.1,
			0, 0, 0, 1;
	Eigen::Matrix4d Mhat12;
	Mhat12 << 0, 0, 1, 0.3,
			0, 1, 0, 0.2,
			-1, 0, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4d Mhat23;
	Mhat23 << 1, 0, 0, 0,
			0, 1, 0, -0.2,
			0, 0, 1, 0.4,
			0, 0, 0, 1;
	Eigen::Matrix4d Mhat34;
	Mhat34 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.2,
			0, 0, 0, 1;
	Mtildelist.push_back(Mhat01);
	Mtildelist.push_back(Mhat12);
	Mtildelist.push_back(Mhat23);
	Mtildelist.push_back(Mhat34);

	Eigen::VectorXd Ghat1(6);
	Ghat1 << 0.1, 0.1, 0.1, 4, 4, 4;
	Eigen::VectorXd Ghat2(6);
	Ghat2 << 0.3, 0.3, 0.1, 9, 9, 9;
	Eigen::VectorXd Ghat3(6);
	Ghat3 << 0.1, 0.1, 0.1, 3, 3, 3;
	Gtildelist.push_back(Ghat1.asDiagonal());
	Gtildelist.push_back(Ghat2.asDiagonal());
	Gtildelist.push_back(Ghat3.asDiagonal());
	Eigen::MatrixXd Ftipmat = Eigen::MatrixXd::Ones(N, 6);
	double Kp = 20.0;
	double Ki = 10.0;
	double Kd = 18.0;
	int intRes = 8;

	int numTest = 3;  // test 0, N/2-1, N-1 indices of results

	//��ȷ���
	Eigen::MatrixXd result_taumat(numTest, 3);
	Eigen::MatrixXd result_thetamat(numTest, 3);

	Eigen::VectorXd tau_timestep_beg(3);
	tau_timestep_beg << -14.2640765, -54.06797429, -11.265448;
	Eigen::VectorXd tau_timestep_mid(3);
	tau_timestep_mid << 31.98269367, 9.89625811, 1.47810165;
	Eigen::VectorXd tau_timestep_end(3);
	tau_timestep_end << 57.04391384, 4.75360586, -1.66561523;
	result_taumat << tau_timestep_beg.transpose(),
			tau_timestep_mid.transpose(),
			tau_timestep_end.transpose();

	Eigen::VectorXd theta_timestep_beg(3);
	theta_timestep_beg << 0.10092029, 0.10190511, 0.10160667;
	Eigen::VectorXd theta_timestep_mid(3);
	theta_timestep_mid << 0.85794085, 1.55124503, 2.80130978;
	Eigen::VectorXd theta_timestep_end(3);
	theta_timestep_end << 1.56344023, 3.07994906, 4.52269971;
	result_thetamat << theta_timestep_beg.transpose(),
			theta_timestep_mid.transpose(),
			theta_timestep_end.transpose();

	std::vector<Eigen::MatrixXd> controlTraj = mr::SimulateControl(thetalist, dthetalist, g, Ftipmat, Mlist, Glist, Slist, thetamatd, dthetamatd,
	                                                               ddthetamatd, gtilde, Mtildelist, Gtildelist, Kp, Ki, Kd, dt, intRes);
	Eigen::MatrixXd traj_tau = controlTraj.at(0);
	Eigen::MatrixXd traj_theta = controlTraj.at(1);
	Eigen::MatrixXd traj_tau_timestep(numTest, 3);
	traj_tau_timestep << traj_tau.row(0),
			traj_tau.row(int(N / 2) - 1),
			traj_tau.row(N - 1);
	Eigen::MatrixXd traj_theta_timestep(numTest, 3);
	traj_theta_timestep << traj_theta.row(0),
			traj_theta.row(int(N / 2) - 1),
			traj_theta.row(N - 1);

	cout << "SimulateControlTest" << endl
	     << "The result is:" << endl << traj_tau_timestep << endl
	     << "It should be:" << endl << result_taumat << endl
	     << "The result is:" << endl << traj_theta_timestep << endl
	     << "It should be:" << endl << result_thetamat << endl;
}

/*--------------------����--------------------*/

void UR5Test()
{
	//P90 UR5-6R��е��
	//�����˲���
	float L1 = 0.425;
	float L2 = 0.392;
	float W1 = 0.109;
	float W2 = 0.082;
	float H1 = 0.089;
	float H2 = 0.095;

	//��ʼֵ
	Eigen::Matrix3d M_R(rotx(-M_PI/2)*rotz(M_PI));
	Eigen::Vector3d M_p(L1+L2,W1+W2,H1-H2);
	Eigen::MatrixXd M = RpToTrans(M_R, M_p);
	cout << "��ʼֵ:" << endl << "M = " << endl << M << endl;

	//�ռ�����ϵ�µĹؽ�����
	//���ٶȷ���
	Eigen::Vector3d
		w1(0, 0, 1),
		w2(0, 1, 0),
		w3(0, 1, 0),
		w4(0, 1, 0),
		w5(0, 0, -1),
		w6(0, 1, 0);
	//������λ��
	Eigen::Vector3d
		q1(0, 0, 0),
		q2(0, 0, H1),
		q3(L1, 0, H1),
		q4(L1+L2, 0, H1),
		q5(L1+L2, W1, 0),
		q6(L1+L2, 0, H1-H2);

	Eigen::Vector3d
		v1(-w1.cross(q1)),
		v2(-w2.cross(q2)),
		v3(-w3.cross(q3)),
		v4(-w4.cross(q4)),
		v5(-w5.cross(q5)),
		v6(-w6.cross(q6));
	Eigen::MatrixXd Slist(6, 6);
	Slist <<
		w1, w2, w3, w4, w5, w6,
		v1, v2, v3, v4, v5, v6;
	cout << "�ռ�����ϵ�µĹؽ�����:" << endl << "Slist = " << endl << Slist << endl;

	//�ؽڽǶ�ֵ
	Eigen::VectorXd thetalist(6, 1);
	thetalist << 0,
			-M_PI/4,
			M_PI/4,
			0,
			0,
			0;
	cout << "�ؽڽǶ�ֵ:" << endl << "thetalist = " << endl << thetalist << endl;

	//���˶�ѧ
	Eigen::MatrixXd T(4, 4); //Ŀ��λ�˾���
	T = FKinSpace(M, Slist, thetalist);
	cout << "���˶�ѧ-ĩ����α任����:" << endl << "T = " << endl << T << endl;

	//�ſɱ�
	Eigen::MatrixXd Js(6, 6);
	Js = JacobianSpace(Slist, thetalist);
	cout << "�ռ��ſɱȾ���:" << endl << "Js = " << endl << Js << endl;

	double eomg = 0.0;
	double ev = 0.0;
	Eigen::VectorXd thetalist0(6, 1);
	thetalist0 << 0,
			-M_PI/5,
			M_PI/5,
			0,
			0,
			0;
	IKinSpace(Slist, M, T, thetalist0, eomg, ev);
	cout << "���˶�ѧ-�ؽڽǶ�ֵ:" << endl << "thetalist = " << endl << thetalist0 << endl;
}