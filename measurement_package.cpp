
#include "measurement_package.h"
#include <iostream>
#include "tools.h"

using namespace std;

void MeasurementPackage::Printer()
{
	cout << "下面打印的是当前的测量信息：" << endl;
	if (this->sensor_type_ == 0)
	{
		cout << "激光雷达" << endl;
	}
	else if (this->sensor_type_ == 1)
	{
		cout << "毫米波雷达" << endl;
	}
	else
	{
		cout << "没有注册的雷达" << endl;
	}
	
	printf("时间戳：%lld\n", this->timestamp_);
	cout << this->raw_measurements_ << endl;
}

Eigen::VectorXd MeasurementPackage::Get_raw_measurements()
{
	if (sensor_type_ == this->LASER)
	{
		return this->raw_measurements_;
	}
	else if (sensor_type_ == this->RADAR)
	{
		VectorXd outer = raw_measurements_;
		float fixedAngle = Tools::fixAngle(outer(1));
		outer(1) = fixedAngle;
		return outer;
	}
	cout << "试图获取未知传感器的观测" << endl;
	assert(false);
	return VectorXd(0);
}