//实现一个标准线性KF
//一定是最基本的线性KF，没有任何变化，作为所有变化的基础

#include "kalman_filter.h"
#include "tools.h"

KalmanFilter::KalmanFilter() 
{
	;
}

KalmanFilter::~KalmanFilter() 
{
	;
}

void KalmanFilter::Predict() 
{
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &innovation)
{
	//虚观测z_pred
	//	std::cout << z << std::endl;
	//	VectorXd z_pred = H_ * x_;

	//新息
	VectorXd y = innovation;
	//观测方程的转置
	MatrixXd Ht = H_.transpose();
	//可逆块及其逆
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	//增益的两个组成
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//生成新的观测
	x_ = x_ + (K * y);
	P_ = P_ - K * H_ * P_;
}

