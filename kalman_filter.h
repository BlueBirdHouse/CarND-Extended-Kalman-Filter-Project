
//ʵ��һ����׼����KF

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter 
{
	public:
		///* ״̬
		VectorXd x_;

		///* ״̬Э������
		MatrixXd P_;

		///* ϵͳ����
		MatrixXd F_;

		///* ϵͳ����
		MatrixXd Q_;

		///* �۲ⷽ��
		MatrixXd H_;

		///* �۲�����
		MatrixXd R_;

		//���ɺ���������
		KalmanFilter();
		virtual ~KalmanFilter();

		//����KFԤ�ⷽ��
		void Predict();

		//����KF���·���(��۲���Ҫ�ⲿ����)
		void Update(const VectorXd &innovation);

};

#endif /* KALMAN_FILTER_H_ */
