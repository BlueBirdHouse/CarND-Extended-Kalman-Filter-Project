#include "FusionEKF.h"
#include "tools.h"

#include <iostream>
#include <cmath>


using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() 
{
	is_initialized_ = false;
	previous_timestamp_ = 0;
	
	// 设定传感器噪声初始化
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	R_laser_ << 0.0225, 0,
        0, 0.0225;
	R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

	//初始化系统加速度噪声
	noise_ax = 9;
	noise_ay = 9;

}

MatrixXd FusionEKF::Get_LASER_H()
{
	//让激光传感器的观测方程有一个唯一的定义位置。
	MatrixXd LASER_H = MatrixXd(2, 4);
	LASER_H << 1, 0, 0, 0,
		0, 1, 0, 0;
	return LASER_H;

}

MatrixXd FusionEKF::Get_F(double dt)
{
	//计算系统方程的函数
	MatrixXd F = MatrixXd(4, 4);

	F << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;
	F(0, 2) = dt;
	F(1, 3) = dt;
	return F;
}

MatrixXd FusionEKF::Get_Q(double dt)
{
	float dt_2 = dt * dt;
	MatrixXd G = MatrixXd(4, 2);
	G << dt_2 / 2.0, 0,
		0, dt_2 / 2.0,
		dt, 0,
		0, dt;
	MatrixXd EAcc = MatrixXd(2, 2);
	EAcc << noise_ax, 0,
		0, noise_ay;
	MatrixXd Q = G * EAcc * G.transpose();
	return Q;
}

VectorXd FusionEKF::Get_z_pred(MeasurementPackage::SensorType type)
{
	//产生伪观测
	if (type == MeasurementPackage::LASER)
	{
		VectorXd z_pred = VectorXd(2);
		MatrixXd H = MatrixXd(2, 4);
		H = Get_LASER_H();
		z_pred = H * ekf_.x_;
		return z_pred;
	}
	else if (type == MeasurementPackage::RADAR)
	{
		//展开状态
		float px = ekf_.x_(0);
		float py = ekf_.x_(1);
		float vx = ekf_.x_(2);
		float vy = ekf_.x_(3);

		float Part1 = sqrt(px*px + py*py);
		float Part2 = Tools::fixAngle(atan2(py, px));
		float Part3 = (px*vx + py * vy) / Part1;

		VectorXd z_pred = VectorXd(3);
		z_pred << Part1, Part2, Part3;
		return z_pred;
	}
	cout << "试图为未知的传感器生成伪观测。" << endl;
	assert(false);
	return VectorXd(0);
}

MatrixXd FusionEKF::Get_H(MeasurementPackage::SensorType type)
{
	//输出观测方程，或者输出观测方程的导数
	MatrixXd H;
	if (type == MeasurementPackage::LASER)
	{
		return Get_LASER_H();
	}
	else if (type == MeasurementPackage::RADAR)
	{
		MatrixXd H(3, 4);
		//展开状态
		float px = ekf_.x_(0);
		float py = ekf_.x_(1);
		float vx = ekf_.x_(2);
		float vy = ekf_.x_(3);

		float rho_2 = pow(px, 2) + pow(py, 2);
		float rho = pow(rho_2, 0.5);
		float rho_3_2 = pow(rho_2, 3);

		if (rho_2 < 1e-32)
		{
			cout << "Error! Divide by zero!" << endl;
			assert(false);
			return VectorXd(0);
		}

		H << px / rho, py / rho, 0, 0,
			-py / rho_2, px / rho_2, 0, 0,
			(py*vx*py - py * vy*px) / rho_3_2, (px*vy*px - px * vx*py) / rho_3_2, px / rho, py / rho;

		return H;
	}
	cout << "试图为未知的传感器生成观测方程的导数。" << endl;
	assert(false);
	return VectorXd(0);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(MeasurementPackage measurement_pack) 
{
  if (!is_initialized_) 
  {
	  cout << "最近的一次观测会被用于初始化滤波器" << endl;
	  ekf_.x_ = VectorXd(4);
	  ekf_.x_ << 1, 1, 1, 1;
	  ekf_.P_ = MatrixXd(4, 4);
	  ekf_.P_ << 1, 0, 0, 0,
		  0, 1, 0, 0,
		  0, 0, 1000, 0,
		  0, 0, 0, 1000;
	  previous_timestamp_ = measurement_pack.timestamp_;

	//利用观测来初始化状态
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
	{
		VectorXd a_raw_measurements = measurement_pack.Get_raw_measurements();
		float rho = a_raw_measurements[0];
		float angle = a_raw_measurements[1];
		float x = rho * cos(angle);
		float y = rho * sin(angle);

		ekf_.x_ << x, y, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
	{
		VectorXd a_raw_measurements = measurement_pack.Get_raw_measurements();
		ekf_.x_ << a_raw_measurements[0], a_raw_measurements[1], 0, 0;
    }
	else
	{
		cout << "试图为未知的传感器初始化。" << endl;
		assert(false);
		return;
	}
    is_initialized_ = true;
    return;
  }

//执行预测过程

  //计算时间流逝
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  if (dt > 0)
  {
	  previous_timestamp_ = measurement_pack.timestamp_;
  }
  else
  {
	  cout << "时间流逝计算出错" << endl;
	  assert(false);
	  return;
  }

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ = Get_F(dt);

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ = Get_Q(dt);

  ekf_.Predict();

// 执行更新过程

  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
  {
//	  return;
	  ekf_.R_ = MatrixXd(2, 2);
	  ekf_.R_ = R_laser_;

	  ekf_.H_ = MatrixXd(2, 4);
	  ekf_.H_ = Get_H(measurement_pack.sensor_type_);

	  VectorXd z_pred = VectorXd(2);
	  z_pred = Get_z_pred(measurement_pack.sensor_type_);

	  VectorXd z = VectorXd(2);
	  z = measurement_pack.Get_raw_measurements();

	  VectorXd innovation = VectorXd(2);
	  innovation = z - z_pred;
	  ekf_.Update(innovation);

  } 
  else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
//	  return;
	  ekf_.R_ = MatrixXd(3, 3);
	  ekf_.R_ = R_radar_;

	  ekf_.H_ = MatrixXd(3, 4);
	  ekf_.H_ = Get_H(measurement_pack.sensor_type_);

	  VectorXd z_pred = VectorXd(3);
	  z_pred = Get_z_pred(measurement_pack.sensor_type_);

	  VectorXd z = VectorXd(3);
	  z = measurement_pack.Get_raw_measurements();

	  VectorXd innovation = VectorXd(3);
	  innovation = z - z_pred;

	  float angle = Tools::angleAddition(z(1), -z_pred(1));
	  innovation(1) = angle;

	  ekf_.Update(innovation);

//	  cout << z[1] << endl;
//	  cout << z_pred[1] << endl;
//	  cout << "----------------------------" << endl;

  }
  else
  {
	  cout << "试图为未知滤波器做状态更新。" << endl;
	  assert(false);
	  return;
  }

  // 输出结果
  cout << "最新的滤波结果：" << endl;
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
