
//这个文件是实际驱动KF执行滤波的功能函数

#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "kalman_filter.h"

#include <Eigen/Dense>


class FusionEKF 
{
	public:
		//构成和析构
		FusionEKF();
		virtual ~FusionEKF();
		
		//实际完成一次滤波的函数
		void ProcessMeasurement(MeasurementPackage measurement_pack);

		//内置的基本滤波器对象
		KalmanFilter ekf_;

//	private:

		//滤波器初始化标志位
		bool is_initialized_;
		
		//上一次的时间戳
		long long previous_timestamp_;
		
		//系统噪声的加速度干扰
		float noise_ax;
		float noise_ay;

		//传感器噪声在这里
		Eigen::MatrixXd R_laser_;
		Eigen::MatrixXd R_radar_;

		//计算当前系统方程的函数
		Eigen::MatrixXd Get_F(double dt);
		//计算当噪声Q的函数
		Eigen::MatrixXd Get_Q(double dt);
		//计算伪观测的函数
		Eigen::VectorXd Get_z_pred(MeasurementPackage::SensorType type);

		//激光传感器的观测方程由于不发生变化，找到一个统一的定义位置
		MatrixXd Get_LASER_H();
		//获取观测矩阵，必要的时候获取雅可比形式的观测矩阵
		MatrixXd Get_H(MeasurementPackage::SensorType type);
};

#endif /* FusionEKF_H_ */
