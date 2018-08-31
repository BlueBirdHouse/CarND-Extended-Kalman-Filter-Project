//ʵ��һ����׼����KF
//һ���������������KF��û���κα仯����Ϊ���б仯�Ļ���

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
	//��۲�z_pred
	//	std::cout << z << std::endl;
	//	VectorXd z_pred = H_ * x_;

	//��Ϣ
	VectorXd y = innovation;
	//�۲ⷽ�̵�ת��
	MatrixXd Ht = H_.transpose();
	//����鼰����
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	//������������
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//�����µĹ۲�
	x_ = x_ + (K * y);
	P_ = P_ - K * H_ * P_;
}

