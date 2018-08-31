
//����ļ���ʵ������KFִ���˲��Ĺ��ܺ���

#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "kalman_filter.h"

#include <Eigen/Dense>


class FusionEKF 
{
	public:
		//���ɺ�����
		FusionEKF();
		virtual ~FusionEKF();
		
		//ʵ�����һ���˲��ĺ���
		void ProcessMeasurement(MeasurementPackage measurement_pack);

		//���õĻ����˲�������
		KalmanFilter ekf_;

//	private:

		//�˲�����ʼ����־λ
		bool is_initialized_;
		
		//��һ�ε�ʱ���
		long long previous_timestamp_;
		
		//ϵͳ�����ļ��ٶȸ���
		float noise_ax;
		float noise_ay;

		//����������������
		Eigen::MatrixXd R_laser_;
		Eigen::MatrixXd R_radar_;

		//���㵱ǰϵͳ���̵ĺ���
		Eigen::MatrixXd Get_F(double dt);
		//���㵱����Q�ĺ���
		Eigen::MatrixXd Get_Q(double dt);
		//����α�۲�ĺ���
		Eigen::VectorXd Get_z_pred(MeasurementPackage::SensorType type);

		//���⴫�����Ĺ۲ⷽ�����ڲ������仯���ҵ�һ��ͳһ�Ķ���λ��
		MatrixXd Get_LASER_H();
		//��ȡ�۲���󣬱�Ҫ��ʱ���ȡ�ſɱ���ʽ�Ĺ۲����
		MatrixXd Get_H(MeasurementPackage::SensorType type);
};

#endif /* FusionEKF_H_ */
