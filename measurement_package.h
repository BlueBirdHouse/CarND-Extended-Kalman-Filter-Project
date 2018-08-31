#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include <Eigen/Dense>

class MeasurementPackage {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

  //Ҫ��ȡ�۲⣬ֻ��ʹ�������������Ϊ���еĽǶ��Ǿ���������
  Eigen::VectorXd Get_raw_measurements();
  //���д�ӡ�����Ĺ���
  void Printer();
};

#endif /* MEASUREMENT_PACKAGE_H_ */
