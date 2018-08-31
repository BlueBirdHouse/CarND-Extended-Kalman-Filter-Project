
#include "measurement_package.h"
#include <iostream>
#include "tools.h"

using namespace std;

void MeasurementPackage::Printer()
{
	cout << "�����ӡ���ǵ�ǰ�Ĳ�����Ϣ��" << endl;
	if (this->sensor_type_ == 0)
	{
		cout << "�����״�" << endl;
	}
	else if (this->sensor_type_ == 1)
	{
		cout << "���ײ��״�" << endl;
	}
	else
	{
		cout << "û��ע����״�" << endl;
	}
	
	printf("ʱ�����%lld\n", this->timestamp_);
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
	cout << "��ͼ��ȡδ֪�������Ĺ۲�" << endl;
	assert(false);
	return VectorXd(0);
}