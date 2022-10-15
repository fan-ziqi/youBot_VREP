#include "modern_robotics.h"

#include <Eigen/Dense>
#include <cmath>
#include <vector>

#define M_PI           3.14159265358979323846  /* pi */

namespace mr
{
	/*--------------------第3章 刚体运动 P69--------------------*/

	Eigen::MatrixXd RotInv(const Eigen::MatrixXd& R)
	{
		return R.transpose();
	}

	Eigen::Matrix3d VecToso3(const Eigen::Vector3d& omg)
	{
		Eigen::Matrix3d m_ret;
		m_ret << 0, -omg(2), omg(1),
				 omg(2), 0, -omg(0),
				 -omg(1), omg(0), 0;
		return m_ret;
	}

	Eigen::Vector3d so3ToVec(const Eigen::MatrixXd& so3mat)
	{
		Eigen::Vector3d v_ret;
		v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
		return v_ret;
	}

	Eigen::Vector4d AxisAng3(const Eigen::Vector3d& expc3)
	{
		Eigen::Vector4d v_ret;
		v_ret << Normalize(expc3), expc3.norm();
		return v_ret;
	}

	Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d& so3mat)
	{
		Eigen::Vector3d omgtheta = so3ToVec(so3mat);

		Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
		if (NearZero(so3mat.norm()))
		{
			return m_ret;
		}
		else
		{
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = so3mat * (1 / theta);
			return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
		}
	}

	Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d& R)
	{
		double acosinput = (R.trace() - 1) / 2.0;
		Eigen::MatrixXd m_ret = Eigen::MatrixXd::Zero(3, 3);
		if (acosinput >= 1)
		{
			return m_ret;
		}
		else if (acosinput <= -1)
		{
			Eigen::Vector3d omg;
			if (!NearZero(1 + R(2, 2)))
				omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if (!NearZero(1 + R(1, 1)))
				omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = VecToso3(M_PI * omg);
			return m_ret;
		}
		else
		{
			double theta = std::acos(acosinput);
			m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
			return m_ret;
		}
	}

	double DistanceToSO3(const Eigen::Matrix3d& M)
	{
		if (M.determinant() > 0)
		{
			return (M.transpose() * M - Eigen::Matrix3d::Identity()).norm();
		}
		else
		{
			return 1.0e9;
		}
	}

	bool TestIfSO3(const Eigen::Matrix3d& M)
	{
		return std::abs(DistanceToSO3(M)) < 1e-3;
	}

	Eigen::MatrixXd ProjectToSO3(const Eigen::MatrixXd& M)
	{
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::MatrixXd R = svd.matrixU() * svd.matrixV().transpose();
		if (R.determinant() < 0)
		{
			// 这种情况下结果会远离M; 反转第三列的符号
			R.col(2) *= -1;
		}
		return R;
	}

	Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d& R,
							  const Eigen::Vector3d& p)
	{
		Eigen::MatrixXd m_ret(4, 4);
		m_ret << R,       p,
				 0, 0, 0, 1;
		return m_ret;
	}

	std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd& T)
	{
		std::vector<Eigen::MatrixXd> Rp_ret;
		Eigen::Matrix3d R_ret;
		// Get top left 3x3 corner
		R_ret = T.block<3, 3>(0, 0);

		Eigen::Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));

		Rp_ret.push_back(R_ret);
		Rp_ret.push_back(p_ret);

		return Rp_ret;
	}

	Eigen::MatrixXd TransInv(const Eigen::MatrixXd& T)
	{
		auto rp = mr::TransToRp(T);
		auto Rt = rp.at(0).transpose();
		auto t = -(Rt * rp.at(1));
		Eigen::MatrixXd inv(4, 4);
		inv = Eigen::MatrixXd::Zero(4,4);
		inv.block(0, 0, 3, 3) = Rt;
		inv.block(0, 3, 3, 1) = t;
		inv(3, 3) = 1;
		return inv;
	}

	Eigen::MatrixXd VecTose3(const Eigen::VectorXd& V)
	{
		// 分离角速度(指数表示)和线速度
		Eigen::Vector3d exp(V(0), V(1), V(2));
		Eigen::Vector3d linear(V(3), V(4), V(5));

		// 将值填入变换矩阵的适当部分
		Eigen::MatrixXd m_ret(4, 4);
		m_ret << VecToso3(exp), linear,
				 0, 0, 0, 0;

		return m_ret;
	}

	Eigen::VectorXd se3ToVec(const Eigen::MatrixXd& se3mat)
	{
		Eigen::VectorXd m_ret(6);
		m_ret << se3mat(2, 1), se3mat(0, 2), se3mat(1, 0), se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);

		return m_ret;
	}

	Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& T)
	{
		std::vector<Eigen::MatrixXd> R = TransToRp(T);
		Eigen::MatrixXd ad_ret(6, 6);
		ad_ret = Eigen::MatrixXd::Zero(6, 6);
		Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3, 3);
		ad_ret << R[0], zeroes,
				  VecToso3(R[1]) * R[0], R[0];
		return ad_ret;
	}

	Eigen::VectorXd ScrewToAxis(Eigen::Vector3d q,
	                            Eigen::Vector3d s,
	                            double h)
	{
		Eigen::VectorXd axis(6);
		axis.segment(0, 3) = s;
		axis.segment(3, 3) = q.cross(s) + (h * s);
		return axis;
	}

	Eigen::VectorXd AxisAng6(const Eigen::VectorXd& expc6)
	{
		Eigen::VectorXd v_ret(7);
		double theta = Eigen::Vector3d(expc6(0), expc6(1), expc6(2)).norm();
		if (NearZero(theta))
		{
			theta = Eigen::Vector3d(expc6(3), expc6(4), expc6(5)).norm();
		}
		v_ret << expc6 / theta, theta;
		return v_ret;
	}

	Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd& se3mat)
	{
		// Extract the angular velocity vector from the transformation matrix
		Eigen::Matrix3d se3mat_cut = se3mat.block<3, 3>(0, 0);
		Eigen::Vector3d omgtheta = so3ToVec(se3mat_cut);

		Eigen::MatrixXd m_ret(4, 4);

		// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
		//									[	0	 ,		1		   ]]
		if (NearZero(omgtheta.norm()))
		{
			// Reuse previous variables that have our required size
			se3mat_cut = Eigen::MatrixXd::Identity(3, 3);
			omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
			m_ret << se3mat_cut, omgtheta,
				0, 0, 0, 1;
			return m_ret;
		}
		// If not negligible, MR page 105
		else
		{
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
			Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
			Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
			Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
			m_ret << MatrixExp3(se3mat_cut), GThetaV,
				0, 0, 0, 1;
			return m_ret;
		}

	}

	Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd& T)
	{
		Eigen::MatrixXd m_ret(4, 4);
		auto rp = mr::TransToRp(T);
		Eigen::Matrix3d omgmat = MatrixLog3(rp.at(0));
		Eigen::Matrix3d zeros3d = Eigen::Matrix3d::Zero(3, 3);
		if (NearZero(omgmat.norm()))
		{
			m_ret << zeros3d, rp.at(1),
				0, 0, 0, 0;
		}
		else
		{
			double theta = std::acos((rp.at(0).trace() - 1) / 2.0);
			Eigen::Matrix3d logExpand1 = Eigen::MatrixXd::Identity(3, 3) - omgmat / 2.0;
			Eigen::Matrix3d logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
			Eigen::Matrix3d logExpand = logExpand1 + logExpand2;
			m_ret << omgmat, logExpand*rp.at(1),
				0, 0, 0, 0;
		}
		return m_ret;
	}

	double DistanceToSE3(const Eigen::Matrix4d& T)
	{
		Eigen::Matrix3d matR = T.block<3, 3>(0, 0);
		if (matR.determinant() > 0)
		{
			Eigen::Matrix4d m_ret;
			m_ret << matR.transpose()*matR, Eigen::Vector3d::Zero(3),
					T.row(3);
			m_ret = m_ret - Eigen::Matrix4d::Identity();
			return m_ret.norm();
		}
		else
		{
			return 1.0e9;
		}
	}

	bool TestIfSE3(const Eigen::Matrix4d& T)
	{
		return std::abs(DistanceToSE3(T)) < 1e-3;
	}

	Eigen::MatrixXd ProjectToSE3(const Eigen::MatrixXd& M)
	{
		Eigen::Matrix3d R = M.block<3, 3>(0, 0);
		Eigen::Vector3d t = M.block<3, 1>(0, 3);
		Eigen::MatrixXd T = RpToTrans(ProjectToSO3(R), t);
		return T;
	}

	/*--------------------第4章 正向运动学 P99--------------------*/

	Eigen::MatrixXd FKinBody(const Eigen::MatrixXd& M,
	                         const Eigen::MatrixXd& Blist,
	                         const Eigen::VectorXd& thetaList)
	{
		Eigen::MatrixXd T = M;
		for (int i = 0; i < thetaList.size(); i++)
		{
			T = T * MatrixExp6(VecTose3(Blist.col(i)*thetaList(i)));
		}
		return T;
	}

	Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd& M,
							  const Eigen::MatrixXd& Slist,
							  const Eigen::VectorXd& thetaList)
	{
		Eigen::MatrixXd T = M;
		for (int i = (thetaList.size() - 1); i > -1; i--)
		{
			T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
		}
		return T;
	}

	/*--------------------第5章 一阶运动学与静力学 P125--------------------*/

	Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd& Blist,
	                             const Eigen::MatrixXd& thetaList)
	{
		Eigen::MatrixXd Jb = Blist;
		Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
		Eigen::VectorXd bListTemp(Blist.col(0).size());
		for (int i = thetaList.size() - 2; i >= 0; i--)
		{
			bListTemp << Blist.col(i + 1) * thetaList(i + 1);
			T = T * MatrixExp6(VecTose3(-1 * bListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Jb.col(i) = Adjoint(T) * Blist.col(i);
		}
		return Jb;
	}

	Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd& Slist,
								  const Eigen::MatrixXd& thetaList)
	{
		Eigen::MatrixXd Js = Slist;
		Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
		Eigen::VectorXd sListTemp(Slist.col(0).size());
		for (int i = 1; i < thetaList.size(); i++)
		{
			sListTemp << Slist.col(i - 1) * thetaList(i - 1);
			T = T * MatrixExp6(VecTose3(sListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Js.col(i) = Adjoint(T) * Slist.col(i);
		}

		return Js;
	}

	/*--------------------第6章 逆运动学 P144--------------------*/

	bool IKinBody(const Eigen::MatrixXd& Blist,
	              const Eigen::MatrixXd& M,
	              const Eigen::MatrixXd& T,
	              Eigen::VectorXd& thetalist,
	              double eomg, double ev)
	{
		int i = 0;
		int maxiterations = 20;
		Eigen::MatrixXd Tfk = FKinBody(M, Blist, thetalist);
		Eigen::MatrixXd Tdiff = TransInv(Tfk)*T;
		Eigen::VectorXd Vb = se3ToVec(MatrixLog6(Tdiff));
		Eigen::Vector3d angular(Vb(0), Vb(1), Vb(2));
		Eigen::Vector3d linear(Vb(3), Vb(4), Vb(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Eigen::MatrixXd Jb;
		while (err && i < maxiterations)
		{
			Jb = JacobianBody(Blist, thetalist);
			thetalist += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
			i += 1;
			// iterate
			Tfk = FKinBody(M, Blist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vb = se3ToVec(MatrixLog6(Tdiff));
			angular = Eigen::Vector3d(Vb(0), Vb(1), Vb(2));
			linear = Eigen::Vector3d(Vb(3), Vb(4), Vb(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}

	bool IKinSpace(const Eigen::MatrixXd& Slist,
	               const Eigen::MatrixXd& M,
	               const Eigen::MatrixXd& T,
	               Eigen::VectorXd& thetalist,
	               double eomg, double ev)
	{
		int i = 0;
		int maxiterations = 20;
		Eigen::MatrixXd Tfk = FKinSpace(M, Slist, thetalist);
		Eigen::MatrixXd Tdiff = TransInv(Tfk)*T;
		Eigen::VectorXd Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
		Eigen::Vector3d angular(Vs(0), Vs(1), Vs(2));
		Eigen::Vector3d linear(Vs(3), Vs(4), Vs(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Eigen::MatrixXd Js;
		while (err && i < maxiterations)
		{
			Js = JacobianSpace(Slist, thetalist);
			thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
			i += 1;
			// iterate
			Tfk = FKinSpace(M, Slist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
			angular = Eigen::Vector3d(Vs(0), Vs(1), Vs(2));
			linear = Eigen::Vector3d(Vs(3), Vs(4), Vs(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}

	/*--------------------第8章 开链动力学 P197--------------------*/

	Eigen::MatrixXd ad(Eigen::VectorXd V)
	{
		Eigen::Matrix3d omgmat = VecToso3(Eigen::Vector3d(V(0), V(1), V(2)));

		Eigen::MatrixXd result(6, 6);
		result.topLeftCorner<3, 3>() = omgmat;
		result.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero(3, 3);
		result.bottomLeftCorner<3, 3>() = VecToso3(Eigen::Vector3d(V(3), V(4), V(5)));
		result.bottomRightCorner<3, 3>() = omgmat;
		return result;
	}

	Eigen::VectorXd InverseDynamics(const Eigen::VectorXd& thetalist,
									const Eigen::VectorXd& dthetalist,
									const Eigen::VectorXd& ddthetalist,
									const Eigen::VectorXd& g,
									const Eigen::VectorXd& Ftip,
									const std::vector<Eigen::MatrixXd>& Mlist,
									const std::vector<Eigen::MatrixXd>& Glist,
									const Eigen::MatrixXd& Slist)
	{
	    // 列表的纬度(连杆数量)
		int n = thetalist.size();

		Eigen::MatrixXd Mi = Eigen::MatrixXd::Identity(4, 4);
		Eigen::MatrixXd Ai = Eigen::MatrixXd::Zero(6,n);
		std::vector<Eigen::MatrixXd> AdTi;
		for (int i = 0; i < n+1; i++)
		{
			AdTi.push_back(Eigen::MatrixXd::Zero(6,6));
		}
		Eigen::MatrixXd Vi = Eigen::MatrixXd::Zero(6,n+1);    // velocity
		Eigen::MatrixXd Vdi = Eigen::MatrixXd::Zero(6,n+1);   // acceleration

		Vdi.block(3, 0, 3, 1) = - g;
		AdTi[n] = mr::Adjoint(mr::TransInv(Mlist[n]));
		Eigen::VectorXd Fi = Ftip;

		Eigen::VectorXd taulist = Eigen::VectorXd::Zero(n);

		// 正向迭代
		for (int i = 0; i < n; i++)
		{
			Mi = Mi * Mlist[i];
			Ai.col(i) = mr::Adjoint(mr::TransInv(Mi))*Slist.col(i);

			AdTi[i] = mr::Adjoint(mr::MatrixExp6(mr::VecTose3(Ai.col(i)*-thetalist(i)))
			          * mr::TransInv(Mlist[i]));

			Vi.col(i+1) = AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
			Vdi.col(i+1) = AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i)
						   + ad(Vi.col(i+1)) * Ai.col(i) * dthetalist(i); // this index is different from book!
		}

		// 逆向迭代
		for (int i = n-1; i >= 0; i--)
		{
			Fi = AdTi[i+1].transpose() * Fi + Glist[i] * Vdi.col(i+1)
			     - ad(Vi.col(i+1)).transpose() * (Glist[i] * Vi.col(i+1));
			taulist(i) = Fi.transpose() * Ai.col(i);
		}
		return taulist;
	}

	Eigen::MatrixXd MassMatrix(const Eigen::VectorXd& thetalist,
							   const std::vector<Eigen::MatrixXd>& Mlist,
							   const std::vector<Eigen::MatrixXd>& Glist,
							   const Eigen::MatrixXd& Slist)
	{
		int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
		Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
		Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n,n);
		for (int i = 0; i < n; i++)
		{
			Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(n);
			ddthetalist(i) = 1;
			M.col(i) = mr::InverseDynamics(thetalist, dummylist, ddthetalist,
                             dummyg, dummyforce, Mlist, Glist, Slist);
		}
		return M;
	}

	Eigen::VectorXd VelQuadraticForces(const Eigen::VectorXd& thetalist,
									   const Eigen::VectorXd& dthetalist,
									   const std::vector<Eigen::MatrixXd>& Mlist,
									   const std::vector<Eigen::MatrixXd>& Glist,
									   const Eigen::MatrixXd& Slist)
    {
		int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
		Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd c = mr::InverseDynamics(thetalist, dthetalist, dummylist,
                             dummyg, dummyforce, Mlist, Glist, Slist);
		return c;
	}

	Eigen::VectorXd GravityForces(const Eigen::VectorXd& thetalist,
	                              const Eigen::VectorXd& g,
	                              const std::vector<Eigen::MatrixXd>& Mlist,
	                              const std::vector<Eigen::MatrixXd>& Glist,
	                              const Eigen::MatrixXd& Slist)
	{
		int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyForce = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd grav = mr::InverseDynamics(thetalist, dummylist, dummylist, g,
		                                           dummyForce, Mlist, Glist, Slist);
		return grav;
	}

	Eigen::VectorXd EndEffectorForces(const Eigen::VectorXd& thetalist,
									  const Eigen::VectorXd& Ftip,
									  const std::vector<Eigen::MatrixXd>& Mlist,
									  const std::vector<Eigen::MatrixXd>& Glist,
									  const Eigen::MatrixXd& Slist)
	{
		int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);

		Eigen::VectorXd JTFtip = mr::InverseDynamics(thetalist, dummylist, dummylist,
                             dummyg, Ftip, Mlist, Glist, Slist);
		return JTFtip;
	}

	Eigen::VectorXd ForwardDynamics(const Eigen::VectorXd& thetalist,
									const Eigen::VectorXd& dthetalist,
									const Eigen::VectorXd& taulist,
									const Eigen::VectorXd& g,
									const Eigen::VectorXd& Ftip,
									const std::vector<Eigen::MatrixXd>& Mlist,
									const std::vector<Eigen::MatrixXd>& Glist,
									const Eigen::MatrixXd& Slist)
	{
		Eigen::VectorXd totalForce = taulist - mr::VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)
                 							 - mr::GravityForces(thetalist, g, Mlist, Glist, Slist)
                                             - mr::EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);

		Eigen::MatrixXd M = mr::MassMatrix(thetalist, Mlist, Glist, Slist);

		// Use LDLT since M is positive definite
        Eigen::VectorXd ddthetalist = M.ldlt().solve(totalForce);

		return ddthetalist;
	}

	void EulerStep(Eigen::VectorXd& thetalist,
				   Eigen::VectorXd& dthetalist,
				   const Eigen::VectorXd& ddthetalist,
				   double dt)
	{
		thetalist += dthetalist * dt;
		dthetalist += ddthetalist * dt;
		return;
	}

	Eigen::MatrixXd InverseDynamicsTrajectory(const Eigen::MatrixXd& thetamat,
											  const Eigen::MatrixXd& dthetamat,
											  const Eigen::MatrixXd& ddthetamat,
											  const Eigen::VectorXd& g,
											  const Eigen::MatrixXd& Ftipmat,
											  const std::vector<Eigen::MatrixXd>& Mlist,
											  const std::vector<Eigen::MatrixXd>& Glist,
											  const Eigen::MatrixXd& Slist)
	{
		Eigen::MatrixXd thetamatT = thetamat.transpose();
		Eigen::MatrixXd dthetamatT = dthetamat.transpose();
		Eigen::MatrixXd ddthetamatT = ddthetamat.transpose();
		Eigen::MatrixXd FtipmatT = Ftipmat.transpose();

		int N = thetamat.rows();  // trajectory points
		int dof = thetamat.cols();
		Eigen::MatrixXd taumatT = Eigen::MatrixXd::Zero(dof, N);
		for (int i = 0; i < N; ++i)
		{
			taumatT.col(i) = InverseDynamics(thetamatT.col(i), dthetamatT.col(i), ddthetamatT.col(i), g, FtipmatT.col(i), Mlist, Glist, Slist);
		}
		Eigen::MatrixXd taumat = taumatT.transpose();
		return taumat;
	}

	std::vector<Eigen::MatrixXd> ForwardDynamicsTrajectory(const Eigen::VectorXd& thetalist,
														   const Eigen::VectorXd& dthetalist,
														   const Eigen::MatrixXd& taumat,
														   const Eigen::VectorXd& g,
														   const Eigen::MatrixXd& Ftipmat,
														   const std::vector<Eigen::MatrixXd>& Mlist,
														   const std::vector<Eigen::MatrixXd>& Glist,
														   const Eigen::MatrixXd& Slist,
														   double dt, int intRes)
	{
		Eigen::MatrixXd taumatT = taumat.transpose();
		Eigen::MatrixXd FtipmatT = Ftipmat.transpose();
		int N = taumat.rows();  // force/torque points
		int dof = taumat.cols();
		Eigen::MatrixXd thetamatT = Eigen::MatrixXd::Zero(dof, N);
		Eigen::MatrixXd dthetamatT = Eigen::MatrixXd::Zero(dof, N);
		thetamatT.col(0) = thetalist;
		dthetamatT.col(0) = dthetalist;
		Eigen::VectorXd thetacurrent = thetalist;
		Eigen::VectorXd dthetacurrent = dthetalist;
		Eigen::VectorXd ddthetalist;
		for (int i = 0; i < N - 1; ++i)
		{
			for (int j = 0; j < intRes; ++j)
			{
				ddthetalist = ForwardDynamics(thetacurrent, dthetacurrent, taumatT.col(i), g, FtipmatT.col(i), Mlist, Glist, Slist);
				EulerStep(thetacurrent, dthetacurrent, ddthetalist, 1.0*dt / intRes);
			}
			thetamatT.col(i + 1) = thetacurrent;
			dthetamatT.col(i + 1) = dthetacurrent;
		}
		std::vector<Eigen::MatrixXd> JointTraj_ret;
		JointTraj_ret.push_back(thetamatT.transpose());
		JointTraj_ret.push_back(dthetamatT.transpose());
		return JointTraj_ret;
	}

	/*--------------------第9章 轨迹生成 P216--------------------*/

	double CubicTimeScaling(double Tf, double t)
	{
		double timeratio = 1.0*t / Tf;
		double st = 3 * pow(timeratio, 2) - 2 * pow(timeratio, 3);
		return st;
	}

	double QuinticTimeScaling(double Tf, double t)
	{
		double timeratio = 1.0*t / Tf;
		double st = 10 * pow(timeratio, 3) - 15 * pow(timeratio, 4) + 6 * pow(timeratio, 5);
		return st;
	}

	Eigen::MatrixXd JointTrajectory(const Eigen::VectorXd& thetastart,
									const Eigen::VectorXd& thetaend,
									double Tf, int N, int method)
	{
		double timegap = Tf / (N - 1);
		Eigen::MatrixXd trajT = Eigen::MatrixXd::Zero(thetastart.size(), N);
		double st;
		for (int i = 0; i < N; ++i)
		{
			if (method == 3)
			{
				st = CubicTimeScaling(Tf, timegap*i);
			}
			else
			{
				st = QuinticTimeScaling(Tf, timegap*i);
			}
			trajT.col(i) = st * thetaend + (1 - st)*thetastart;
		}
		Eigen::MatrixXd traj = trajT.transpose();
		return traj;
	}
	std::vector<Eigen::MatrixXd> ScrewTrajectory(const Eigen::MatrixXd& Xstart,
												 const Eigen::MatrixXd& Xend,
												 double Tf, int N, int method)
	{
		double timegap = Tf / (N - 1);
		std::vector<Eigen::MatrixXd> traj(N);
		double st;
		for (int i = 0; i < N; ++i)
		{
			if (method == 3)
			{
				st = CubicTimeScaling(Tf, timegap*i);
			}
			else
			{
				st = QuinticTimeScaling(Tf, timegap*i);
			}
			Eigen::MatrixXd Ttemp = MatrixLog6(TransInv(Xstart)*Xend);
			traj.at(i) = Xstart * MatrixExp6(Ttemp*st);
		}
		return traj;
	}

	std::vector<Eigen::MatrixXd> CartesianTrajectory(const Eigen::MatrixXd& Xstart,
													 const Eigen::MatrixXd& Xend,
													 double Tf, int N, int method)
	{
		double timegap = Tf / (N - 1);
		std::vector<Eigen::MatrixXd> traj(N);
		std::vector<Eigen::MatrixXd> Rpstart = TransToRp(Xstart);
		std::vector<Eigen::MatrixXd> Rpend = TransToRp(Xend);
		Eigen::Matrix3d Rstart = Rpstart[0]; Eigen::Vector3d pstart = Rpstart[1];
		Eigen::Matrix3d Rend = Rpend[0]; Eigen::Vector3d pend = Rpend[1];
		double st;
		for (int i = 0; i < N; ++i)
		{
			if (method == 3)
			{
				st = CubicTimeScaling(Tf, timegap*i);
			}
			else
			{
				st = QuinticTimeScaling(Tf, timegap*i);
			}
			Eigen::Matrix3d Ri = Rstart * MatrixExp3(MatrixLog3(Rstart.transpose() * Rend)*st);
			Eigen::Vector3d pi = st*pend + (1 - st)*pstart;
			Eigen::MatrixXd traji(4, 4);
			traji << Ri, pi,
					 0, 0, 0, 1;
			traj.at(i) = traji;
		}
		return traj;
	}

	/*--------------------第11章 机器人控制 P287--------------------*/

	Eigen::VectorXd ComputedTorque(const Eigen::VectorXd& thetalist,
	                               const Eigen::VectorXd& dthetalist,
	                               const Eigen::VectorXd& eint,
	                               const Eigen::VectorXd& g,
	                               const std::vector<Eigen::MatrixXd>& Mlist,
	                               const std::vector<Eigen::MatrixXd>& Glist,
	                               const Eigen::MatrixXd& Slist,
	                               const Eigen::VectorXd& thetalistd,
	                               const Eigen::VectorXd& dthetalistd,
	                               const Eigen::VectorXd& ddthetalistd,
	                               double Kp, double Ki, double Kd)
	{

		Eigen::VectorXd e = thetalistd - thetalist;  // position err
		Eigen::VectorXd tau_feedforward = MassMatrix(thetalist, Mlist, Glist, Slist)*(Kp*e + Ki * (eint + e) + Kd * (dthetalistd - dthetalist));

		Eigen::VectorXd Ftip = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd tau_inversedyn = InverseDynamics(thetalist, dthetalist, ddthetalistd, g, Ftip, Mlist, Glist, Slist);

		Eigen::VectorXd tau_computed = tau_feedforward + tau_inversedyn;
		return tau_computed;
	}

	std::vector<Eigen::MatrixXd> SimulateControl(const Eigen::VectorXd& thetalist,
												 const Eigen::VectorXd& dthetalist,
												 const Eigen::VectorXd& g,
												 const Eigen::MatrixXd& Ftipmat,
												 const std::vector<Eigen::MatrixXd>& Mlist,
												 const std::vector<Eigen::MatrixXd>& Glist,
												 const Eigen::MatrixXd& Slist,
												 const Eigen::MatrixXd& thetamatd,
												 const Eigen::MatrixXd& dthetamatd,
												 const Eigen::MatrixXd& ddthetamatd,
												 const Eigen::VectorXd& gtilde,
												 const std::vector<Eigen::MatrixXd>& Mtildelist,
												 const std::vector<Eigen::MatrixXd>& Gtildelist,
												 double Kp, double Ki, double Kd, double dt, int intRes)
	{
		Eigen::MatrixXd FtipmatT = Ftipmat.transpose();
		Eigen::MatrixXd thetamatdT = thetamatd.transpose();
		Eigen::MatrixXd dthetamatdT = dthetamatd.transpose();
		Eigen::MatrixXd ddthetamatdT = ddthetamatd.transpose();
		int m = thetamatdT.rows(); int n = thetamatdT.cols();
		Eigen::VectorXd thetacurrent = thetalist;
		Eigen::VectorXd dthetacurrent = dthetalist;
		Eigen::VectorXd eint = Eigen::VectorXd::Zero(m);
		Eigen::MatrixXd taumatT = Eigen::MatrixXd::Zero(m, n);
		Eigen::MatrixXd thetamatT = Eigen::MatrixXd::Zero(m, n);
		Eigen::VectorXd taulist;
		Eigen::VectorXd ddthetalist;
		for (int i = 0; i < n; ++i)
		{
			taulist = ComputedTorque(thetacurrent, dthetacurrent, eint, gtilde, Mtildelist, Gtildelist, Slist, thetamatdT.col(i),
				dthetamatdT.col(i), ddthetamatdT.col(i), Kp, Ki, Kd);
			for (int j = 0; j < intRes; ++j)
			{
				ddthetalist = ForwardDynamics(thetacurrent, dthetacurrent, taulist, g, FtipmatT.col(i), Mlist, Glist, Slist);
				EulerStep(thetacurrent, dthetacurrent, ddthetalist, dt / intRes);
			}
			taumatT.col(i) = taulist;
			thetamatT.col(i) = thetacurrent;
			eint += dt * (thetamatdT.col(i) - thetacurrent);
		}
		std::vector<Eigen::MatrixXd> ControlTauTraj_ret;
		ControlTauTraj_ret.push_back(taumatT.transpose());
		ControlTauTraj_ret.push_back(thetamatT.transpose());
		return ControlTauTraj_ret;
	}

	/*--------------------其他--------------------*/

	bool NearZero(const double val)
	{
		return (std::abs(val) < .000001);
	}


	Eigen::MatrixXd Normalize(Eigen::MatrixXd V)
	{
		V.normalize();
		return V;
	}

	Eigen::Matrix3d rotx(const double t)
	{
		Eigen::Matrix3d R;
		double ct = cos(t); if(NearZero(ct)) ct = 0;
		double st = sin(t); if(NearZero(st)) st = 0;
		R << 1,  0,   0,
			 0,  ct, -st,
			 0,  st,  ct;
		return R;
	}

	Eigen::Matrix3d roty(const double t)
	{
		Eigen::Matrix3d R;
		double ct = cos(t); if(NearZero(ct)) ct = 0;
		double st = sin(t); if(NearZero(st)) st = 0;
		R << ct,  0,  st,
			 0,   1,  0,
			 -st, 0,  ct;
		return R;
	}

	Eigen::Matrix3d rotz(const double t)
	{
		Eigen::Matrix3d R;
		double ct = cos(t); if(NearZero(ct)) ct = 0;
		double st = sin(t); if(NearZero(st)) st = 0;
		R << ct, -st,  0,
			 st,  ct,  0,
			 0,   0,   1;
		return R;
	}

	Eigen::MatrixXd pinv(const Eigen::MatrixXd& origin, const float er)
	{
		// 进行svd分解
		Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin, Eigen::ComputeThinU | Eigen::ComputeThinV);
		// 构建SVD分解结果
		Eigen::MatrixXd U = svd_holder.matrixU();
		Eigen::MatrixXd V = svd_holder.matrixV();
		Eigen::MatrixXd D = svd_holder.singularValues();

		// 构建S矩阵
		Eigen::MatrixXd S(V.cols(), U.cols());
		S.setZero();

		for (unsigned int i = 0; i < D.size(); i++)
		{
			if(D(i, 0) > er)
			{
				S(i, i) = 1 / D(i, 0);
			}
			else
			{
				S(i, i) = 0;
			}
		}
		return V * S * U.transpose();
	}
}
