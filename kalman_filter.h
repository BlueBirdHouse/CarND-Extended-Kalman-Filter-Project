
//实现一个标准线性KF

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter 
{
	public:
		///* 状态
		VectorXd x_;

		///* 状态协方差阵
		MatrixXd P_;

		///* 系统方程
		MatrixXd F_;

		///* 系统噪声
		MatrixXd Q_;

		///* 观测方程
		MatrixXd H_;

		///* 观测噪声
		MatrixXd R_;

		//构成和析构函数
		KalmanFilter();
		virtual ~KalmanFilter();

		//基本KF预测方程
		void Predict();

		//基本KF更新方程(虚观测需要外部输入)
		void Update(const VectorXd &innovation);

};

#endif /* KALMAN_FILTER_H_ */
